//
//  OperatorGuidanceEngine.swift
//  Rangefinder
//
//  IMU-based operator guidance engine.
//
//  Analyzes device motion (gyroscope, accelerometer) to provide
//  real-time coaching hints that improve ranging accuracy:
//
//  - Stability level: how steady the device is held
//  - Breathing detection: natural respiratory pause window
//  - Brace suggestion: when the operator should seek support
//  - GPS quality warnings: when DEM/terrain ranging is degraded
//  - Light level warnings: when neural depth accuracy drops
//  - Calibration status: when LiDAR calibration is stale
//  - Steep angle warnings: when cosine correction is large
//
//  Military doctrine (FM 23-10, TC 3-22.10):
//  - Best readings during natural respiratory pause (bottom of exhale)
//  - Stability hierarchy: prone > sitting > kneeling > standing
//  - Multiple readings averaged for precision
//  - Environmental awareness: mirage, wind, light
//
//  The engine outputs discrete guidance hints that the HUD overlay
//  displays as brief, terse, military-style status messages.
//

import Foundation
import Combine

// MARK: - Guidance Hint Types

enum GuidanceHint: Equatable, Hashable {
    // Stability hints
    case holdSteady           // High motion detected — operator should stabilize
    case stabilized           // Good stability — optimal for reading
    case braceDevice          // Prolonged instability — suggest bracing/support
    case excessiveMotion      // Very high motion — reading unreliable

    // Environmental hints
    case lowLight             // Poor lighting degrades neural depth
    case gpsAcquiring         // GPS not ready for DEM ranging
    case gpsLowAccuracy       // GPS accuracy poor — DEM degraded
    case compassInterference  // Heading accuracy poor — DEM bearing unreliable

    // Calibration hints
    case calibrating          // LiDAR calibrating neural depth (good — walk near objects)
    case calibrationStale     // Calibration aging — walk near objects to refresh
    case calibrationNeeded    // No calibration — neural depth uncalibrated

    // Range quality hints
    case rangeUncorroborated  // Neural-only extrapolation with no DEM/object backup
    case demUnavailable       // DEM not producing estimates — long range degraded
    case demDownloading       // SRTM terrain data is being downloaded
    case terrainRangeLimited  // LST mode, range suspiciously short, DEM needs tiles

    // Angle hints
    case steepAngle           // Pitch > 45° — large cosine correction applied
    case ballisticsAngleWarning // Pitch > 60° with ballistics enabled — solution unreliable

    // Technique hints
    case respiratoryPause     // Breathing pause detected — optimal capture window
    case readingLocked        // Reading stable for 2+ seconds — good capture
    case multipleReadings     // Suggest taking multiple readings for long range

    var message: String {
        switch self {
        case .holdSteady:              return "HOLD STEADY"
        case .stabilized:              return "STABILIZED"
        case .braceDevice:             return "BRACE DEVICE"
        case .excessiveMotion:         return "EXCESSIVE MOTION"
        case .lowLight:                return "LOW LIGHT"
        case .gpsAcquiring:            return "GPS ACQUIRING"
        case .gpsLowAccuracy:          return "GPS LOW ACCURACY"
        case .compassInterference:     return "COMPASS INTERFERENCE"
        case .rangeUncorroborated:     return "RANGE UNCORROBORATED"
        case .demUnavailable:          return "NO TERRAIN DATA"
        case .demDownloading:          return "DOWNLOADING TERRAIN"
        case .terrainRangeLimited:     return "DOWNLOAD TERRAIN"
        case .calibrating:             return "CALIBRATING"
        case .calibrationStale:        return "CAL AGING — WALK CLOSE"
        case .calibrationNeeded:       return "CAL NEEDED — WALK CLOSE"
        case .steepAngle:              return "STEEP ANGLE"
        case .ballisticsAngleWarning:  return "ANGLE — VERIFY SOLUTION"
        case .respiratoryPause:        return "STEADY — TAKE READING"
        case .readingLocked:           return "READING LOCKED"
        case .multipleReadings:        return "MULTIPLE READINGS REC"
        }
    }

    var icon: String {
        switch self {
        case .holdSteady:              return "hand.raised"
        case .stabilized:              return "checkmark.circle"
        case .braceDevice:             return "rectangle.and.hand.point.up.left"
        case .excessiveMotion:         return "exclamationmark.triangle"
        case .lowLight:                return "sun.min"
        case .gpsAcquiring:            return "location.slash"
        case .gpsLowAccuracy:          return "location.circle"
        case .rangeUncorroborated:     return "exclamationmark.triangle.fill"
        case .demUnavailable:          return "mountain.2"
        case .demDownloading:          return "icloud.and.arrow.down"
        case .terrainRangeLimited:     return "square.and.arrow.down"
        case .compassInterference:     return "location.north.line"
        case .calibrating:             return "sensor.fill"
        case .calibrationStale:        return "clock.arrow.circlepath"
        case .calibrationNeeded:       return "exclamationmark.circle"
        case .steepAngle:              return "angle"
        case .ballisticsAngleWarning:  return "scope"
        case .respiratoryPause:        return "circle.circle"
        case .readingLocked:           return "lock.fill"
        case .multipleReadings:        return "number.circle"
        }
    }

    var priority: Int {
        switch self {
        case .rangeUncorroborated:     return 105
        case .excessiveMotion:         return 100
        case .demUnavailable:          return 95
        case .terrainRangeLimited:     return 97
        case .ballisticsAngleWarning:  return 88
        case .steepAngle:              return 85
        case .demDownloading:          return 45
        case .calibrationNeeded:       return 90
        case .holdSteady:              return 80
        case .braceDevice:             return 75
        case .lowLight:                return 70
        case .gpsAcquiring:            return 65
        case .gpsLowAccuracy:          return 60
        case .compassInterference:     return 55
        case .calibrationStale:        return 50
        case .calibrating:             return 40
        case .multipleReadings:        return 30
        case .respiratoryPause:        return 25
        case .readingLocked:           return 20
        case .stabilized:              return 10
        }
    }

    var severity: HintSeverity {
        switch self {
        case .stabilized, .readingLocked, .respiratoryPause, .calibrating:
            return .positive
        case .holdSteady, .calibrationStale, .multipleReadings,
             .gpsLowAccuracy, .compassInterference, .demUnavailable,
             .demDownloading, .steepAngle:
            return .caution
        case .excessiveMotion, .braceDevice, .lowLight,
             .gpsAcquiring, .calibrationNeeded, .rangeUncorroborated,
             .terrainRangeLimited, .ballisticsAngleWarning:
            return .warning
        }
    }
}

enum HintSeverity {
    case positive  // Green — good condition
    case caution   // Amber — advisory
    case warning   // Red/orange — action needed
}

// MARK: - Stability Level

enum StabilityLevel: Int, Comparable {
    case unstable = 0     // Angular velocity > threshold — panning/shaking
    case marginal = 1     // Moving but usable
    case adequate = 2     // Tracking, moderate stability
    case good = 3         // Held steady, some drift
    case excellent = 4    // Braced/tripod quality

    static func < (lhs: StabilityLevel, rhs: StabilityLevel) -> Bool {
        lhs.rawValue < rhs.rawValue
    }

    var description: String {
        switch self {
        case .unstable:  return "UNSTABLE"
        case .marginal:  return "MARGINAL"
        case .adequate:  return "ADEQUATE"
        case .good:      return "GOOD"
        case .excellent: return "EXCELLENT"
        }
    }
}

// MARK: - Guidance Engine

@MainActor
class OperatorGuidanceEngine: ObservableObject {

    // MARK: - Published State

    @Published var activeHints: [GuidanceHint] = []
    @Published var stabilityLevel: StabilityLevel = .adequate
    @Published var stabilityPercent: Float = 0.5    // 0.0 = shaking, 1.0 = rock steady
    @Published var isCapturePrimed: Bool = false     // Respiratory pause detected
    @Published var isReadingLocked: Bool = false     // Range stable for 2+ seconds

    // MARK: - Configuration

    /// Minimum time a hint stays visible (prevents flicker)
    private let hintMinDuration: TimeInterval = 2.0
    /// Maximum hints shown simultaneously
    private let maxVisibleHints: Int = 2
    /// Stability history window (seconds)
    private let stabilityWindowSize: Int = 30  // 30 samples at ~1 Hz

    // MARK: - State

    private var angularVelocityHistory: [Double] = []
    private var rangeHistory: [(range: Double, timestamp: TimeInterval)] = []
    private var lastHintTimes: [GuidanceHint: TimeInterval] = [:]
    private var unstableDuration: TimeInterval = 0
    private var stableDuration: TimeInterval = 0
    private var lastUpdateTime: TimeInterval = 0
    private var lastAngularVelocity: Double = 0

    // External state (set by AppState)
    var gpsAccuracy: Float = 100        // Horizontal accuracy in meters
    var headingAccuracy: Float = 20     // Heading accuracy in degrees
    var hasGPSFix: Bool = false
    var isGPSAuthorized: Bool = false
    var calibrationAge: TimeInterval = 0
    var calibrationConfidence: Float = 0
    var calibrationSampleCount: Int = 0
    var currentRangeM: Double = 0
    var currentConfidence: Float = 0
    var ambientLightLevel: Float = 1.0  // 0 = dark, 1 = bright (from camera exposure)
    var isNeuralUncorroborated: Bool = false  // Neural in extrapolation zone with no DEM/object
    var isDEMAvailable: Bool = false           // Whether DEM is producing estimates
    var isDEMDownloading: Bool = false         // SRTM tile download in progress
    var isDEMFarField: Bool = false            // DEM producing estimates >200m
    var isLSTMode: Bool = false                // Far-target priority (LAST TARGET)
    var cameraPitchDegrees: Double = 0         // Camera pitch (negative = looking down)
    var isBallisticsEnabled: Bool = false       // Whether ballistics solver is active

    // MARK: - Adaptive Stability Thresholds

    /// At steep angles, holding the phone steady is physically harder.
    /// Relax stability thresholds proportionally to acknowledge ergonomic reality.
    ///
    /// 0–30°: 1.0× (normal thresholds)
    /// 30–60°: up to 1.5× (moderate relaxation)
    /// 60–85°: up to 2.0× (significant relaxation — arm extended overhead/below)
    private var stabilityRelaxationFactor: Double {
        let absPitch = abs(cameraPitchDegrees)
        if absPitch < 30 { return 1.0 }
        if absPitch < 60 { return 1.0 + (absPitch - 30.0) / 60.0 }  // 1.0 → 1.5
        return 1.5 + (min(absPitch, 85.0) - 60.0) / 50.0            // 1.5 → 2.0
    }

    // MARK: - Update

    /// Call at ~10-30 Hz with current angular velocity from InclinationManager.
    func update(angularVelocity: Double, timestamp: TimeInterval) {
        let dt = timestamp - lastUpdateTime
        guard dt > 0.03 else { return }  // Throttle to ~30 Hz max
        lastUpdateTime = timestamp
        lastAngularVelocity = angularVelocity

        // Update stability tracking
        updateStability(angularVelocity: angularVelocity, dt: dt)

        // Detect respiratory pause (period of minimal motion after motion)
        detectRespiratoryPause(angularVelocity: angularVelocity)

        // Update reading lock (range stable for 2+ seconds)
        updateReadingLock(timestamp: timestamp)

        // Compute hints
        let newHints = computeHints(timestamp: timestamp)
        updateActiveHints(newHints, timestamp: timestamp)
    }

    /// Feed range reading for lock detection.
    func feedRange(rangeM: Double, timestamp: TimeInterval) {
        rangeHistory.append((rangeM, timestamp))
        // Keep last 60 readings (~2-4 seconds)
        if rangeHistory.count > 60 {
            rangeHistory.removeFirst()
        }
        currentRangeM = rangeM
    }

    // MARK: - Stability Analysis

    private func updateStability(angularVelocity: Double, dt: TimeInterval) {
        angularVelocityHistory.append(angularVelocity)
        if angularVelocityHistory.count > stabilityWindowSize {
            angularVelocityHistory.removeFirst()
        }

        // Classify stability level from smoothed angular velocity
        let smoothedAV = angularVelocityHistory.suffix(5).reduce(0, +)
            / Double(min(5, angularVelocityHistory.count))

        // Apply adaptive relaxation for steep angles.
        // At 75°, thresholds expand by ~2×, so the operator isn't punished
        // for the physical difficulty of holding a phone at extreme pitch.
        let relax = stabilityRelaxationFactor

        let newLevel: StabilityLevel
        if smoothedAV > 0.3 * relax {
            newLevel = .unstable
        } else if smoothedAV > 0.1 * relax {
            newLevel = .marginal
        } else if smoothedAV > 0.05 * relax {
            newLevel = .adequate
        } else if smoothedAV > 0.02 * relax {
            newLevel = .good
        } else {
            newLevel = .excellent
        }

        stabilityLevel = newLevel

        // Stability percent: map angular velocity to 0-1, with relaxed ceiling
        stabilityPercent = Float(max(0, min(1, 1.0 - smoothedAV / (0.3 * relax))))

        // Track durations
        if newLevel <= .marginal {
            unstableDuration += dt
            stableDuration = 0
        } else {
            stableDuration += dt
            unstableDuration = 0
        }
    }

    // MARK: - Respiratory Pause Detection

    /// Detects the natural respiratory pause: a brief period of very low motion
    /// that occurs at the bottom of each exhale cycle. In shooting, this 2-3
    /// second window is optimal for taking a reading.
    ///
    /// Pattern: motion → stillness transition with angular velocity < threshold
    private func detectRespiratoryPause(angularVelocity: Double) {
        // Need at least 10 samples of history
        guard angularVelocityHistory.count >= 10 else {
            isCapturePrimed = false
            return
        }

        let relax = stabilityRelaxationFactor

        // Current motion must be very low (relaxed at steep angles)
        let recent3 = Array(angularVelocityHistory.suffix(3))
        let currentAvg = recent3.reduce(0, +) / Double(recent3.count)

        // Previous motion (3-8 samples ago) should have been higher
        let earlier = Array(angularVelocityHistory.suffix(8).prefix(5))
        let earlierAvg = earlier.reduce(0, +) / Double(earlier.count)

        // Pattern: was moving, now still (transition to stillness)
        if currentAvg < 0.02 * relax && earlierAvg > 0.05 * relax {
            isCapturePrimed = true
        } else if currentAvg > 0.05 * relax {
            isCapturePrimed = false
        }
        // If already primed and still steady, keep primed
    }

    // MARK: - Reading Lock Detection

    /// A "locked" reading means the displayed range has been stable
    /// (within 5% relative change) for at least 2 seconds.
    private func updateReadingLock(timestamp: TimeInterval) {
        guard rangeHistory.count >= 10 else {
            isReadingLocked = false
            return
        }

        // Get readings from the last 2 seconds
        let cutoff = timestamp - 2.0
        let recentReadings = rangeHistory.filter { $0.timestamp >= cutoff }
        guard recentReadings.count >= 5 else {
            isReadingLocked = false
            return
        }

        let ranges = recentReadings.map { $0.range }
        let median = ranges.sorted()[ranges.count / 2]
        guard median > 0.5 else {
            isReadingLocked = false
            return
        }

        // Check if all readings are within 5% of median
        let maxDeviation = ranges.map { abs($0 - median) / median }.max() ?? 1.0
        isReadingLocked = maxDeviation < 0.05
    }

    // MARK: - Hint Computation

    private func computeHints(timestamp: TimeInterval) -> [GuidanceHint] {
        var hints: [GuidanceHint] = []

        // --- Stability hints ---
        if stabilityLevel == .unstable {
            hints.append(.excessiveMotion)
        } else if stabilityLevel == .marginal {
            if unstableDuration > 5.0 {
                hints.append(.braceDevice)
            } else {
                hints.append(.holdSteady)
            }
        } else if stabilityLevel >= .good && stableDuration > 1.0 {
            // Only show "stabilized" briefly after transition from unstable
            if stableDuration < 4.0 {
                hints.append(.stabilized)
            }
        }

        // --- Capture window ---
        if isCapturePrimed && stabilityLevel >= .good {
            hints.append(.respiratoryPause)
        }

        // --- Reading locked ---
        if isReadingLocked && stabilityLevel >= .adequate {
            hints.append(.readingLocked)
        }

        // --- Steep angle warning ---
        // At > 45°, cosine correction reduces displayed range by > 29%.
        // Operator needs to know the correction is significant.
        let absPitch = abs(cameraPitchDegrees)
        if absPitch > 45 {
            hints.append(.steepAngle)

            // Additional warning when ballistics is enabled at extreme angles.
            // The simplified Rifleman's Rule becomes unreliable above 60°.
            if isBallisticsEnabled && absPitch > 60 {
                hints.append(.ballisticsAngleWarning)
            }
        }

        // --- Calibration hints ---
        if calibrationSampleCount < 5 {
            hints.append(.calibrationNeeded)
        } else if calibrationAge > 120 && calibrationConfidence < 0.5 {
            hints.append(.calibrationStale)
        } else if calibrationSampleCount > 0 && calibrationSampleCount < 20 && calibrationAge < 30 {
            hints.append(.calibrating)
        }

        // --- GPS/DEM hints ---
        if !isGPSAuthorized || !hasGPSFix {
            hints.append(.gpsAcquiring)
        } else if gpsAccuracy > 20 {
            hints.append(.gpsLowAccuracy)
        }

        if hasGPSFix && headingAccuracy > 15 {
            hints.append(.compassInterference)
        }

        // --- Range quality hints ---
        // Highest priority: neural extrapolation with no corroboration means
        // the displayed range is almost certainly wrong for distant targets.
        if isNeuralUncorroborated {
            hints.append(.rangeUncorroborated)
        }

        // DEM unavailable: when GPS is valid but DEM still isn't producing
        // estimates (SRTM data missing, ray-cast failing, etc.), warn the user
        // that long-range terrain ranging is degraded.
        if hasGPSFix && !isDEMAvailable && currentRangeM > 50 {
            if isDEMDownloading {
                hints.append(.demDownloading)
            } else {
                hints.append(.demUnavailable)
            }
        }

        // Terrain range limited: in far-target (LST) mode, the displayed range
        // is suspiciously short (<300m) without far-field DEM coverage.
        // This means the DEM either has no tiles or can't reach the terrain
        // the user is pointing at. Nudge them to download SRTM tiles.
        // Only fires when pitch is shallow (looking toward horizon, not down).
        if isLSTMode && hasGPSFix && currentRangeM > 20 && currentRangeM < 300
            && !isDEMFarField && cameraPitchDegrees > -15 {
            hints.append(.terrainRangeLimited)
        }

        // --- Multiple readings suggestion (long range) ---
        if currentRangeM > 200 && !isReadingLocked && stabilityLevel >= .adequate {
            hints.append(.multipleReadings)
        }

        // --- Low light (placeholder — set by camera exposure analysis) ---
        if ambientLightLevel < 0.15 {
            hints.append(.lowLight)
        }

        return hints
    }

    // MARK: - Hint Display Management

    private func updateActiveHints(_ newHints: [GuidanceHint], timestamp: TimeInterval) {
        // Sort by priority (highest first)
        let sorted = newHints.sorted { $0.priority > $1.priority }

        // Take top N hints
        var candidates = Array(sorted.prefix(maxVisibleHints))

        // Enforce minimum display duration: keep existing hints that haven't
        // met their minimum duration, even if they'd otherwise be replaced
        for existing in activeHints {
            if let lastShown = lastHintTimes[existing],
               timestamp - lastShown < hintMinDuration,
               !candidates.contains(existing) {
                candidates.append(existing)
            }
        }

        // Deduplicate and sort by priority
        let unique = Array(Set(candidates)).sorted { $0.priority > $1.priority }
        let final = Array(unique.prefix(maxVisibleHints))

        // Record show times for new hints
        for hint in final where !activeHints.contains(hint) {
            lastHintTimes[hint] = timestamp
        }

        activeHints = final
    }

    // MARK: - Reset

    func reset() {
        activeHints = []
        angularVelocityHistory = []
        rangeHistory = []
        lastHintTimes = [:]
        unstableDuration = 0
        stableDuration = 0
        stabilityLevel = .adequate
        stabilityPercent = 0.5
        isCapturePrimed = false
        isReadingLocked = false
    }
}
