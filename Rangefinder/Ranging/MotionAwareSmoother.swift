//
//  MotionAwareSmoother.swift
//  Rangefinder
//
//  Context-sensitive temporal smoothing that adapts based on WHY
//  the range is changing:
//
//  - Stationary (gyro quiet): Heavy smoothing for stability
//  - Panning (gyro active): Light smoothing — scene is changing
//  - Tracking: Medium smoothing for smooth tracking
//
//  Also detects depth discontinuities (object edges) using a
//  3-frame confirmation buffer. If 3 consecutive frames agree on
//  a new depth that differs >30% from current, snap to the new
//  value instead of slowly interpolating.
//

import Foundation

struct MotionAwareSmoother {

    // MARK: - Configuration

    // Base alphas (used at short range; reduced at longer ranges)
    private let stationaryAlphaBase: Double = 0.12  // Heavy smoothing
    private let panningAlphaBase: Double = 0.6      // Light smoothing
    private let trackingAlphaBase: Double = 0.30    // Medium smoothing

    // Depth discontinuity detection
    private let discontinuityThresholdBase: Double = 0.30  // 30% change at short range
    private let confirmationFrames = 4  // Was 3 — require one more frame for confirmation

    // MARK: - State

    private var smoothedValue: Double = 0
    private var isInitialized = false

    // Discontinuity detection buffer
    private var recentValues: [Double] = []

    // Confidence hysteresis
    private var smoothedConfidence: Float = 0
    private let confidenceAlpha: Float = 0.08  // Even slower confidence changes

    // MARK: - Smooth

    /// Apply motion-aware, distance-dependent smoothing to a new depth value.
    ///
    /// At longer ranges, neural depth noise increases quadratically (due to
    /// inverse-depth calibration). The smoother compensates by using a smaller
    /// alpha (heavier smoothing) for distant targets when stationary.
    ///
    /// - Parameters:
    ///   - newValue: New depth measurement in meters.
    ///   - motionState: Current device motion state.
    /// - Returns: Smoothed depth value.
    mutating func smooth(newValue: Double, motionState: MotionState) -> Double {
        guard newValue > 0 else { return smoothedValue }

        // First value: initialize immediately
        guard isInitialized else {
            smoothedValue = newValue
            isInitialized = true
            recentValues = [newValue]
            return newValue
        }

        // Update discontinuity buffer
        recentValues.append(newValue)
        if recentValues.count > confirmationFrames {
            recentValues.removeFirst()
        }

        // Check for depth discontinuity (object edge crossing)
        if detectDiscontinuity(newValue: newValue) {
            // Snap to new value — the crosshair moved to a different object
            smoothedValue = newValue
            return newValue
        }

        // Choose alpha based on motion state AND distance
        let alpha = computeAlpha(motionState: motionState, depth: smoothedValue)

        // Exponential moving average
        smoothedValue = alpha * newValue + (1.0 - alpha) * smoothedValue

        return smoothedValue
    }

    /// Smooth confidence with hysteresis to prevent color flickering.
    ///
    /// Uses a very slow alpha so confidence indicator doesn't
    /// rapidly change color.
    mutating func smoothConfidence(_ newConfidence: Float) -> Float {
        if smoothedConfidence == 0 {
            smoothedConfidence = newConfidence
            return newConfidence
        }

        smoothedConfidence = confidenceAlpha * newConfidence + (1.0 - confidenceAlpha) * smoothedConfidence
        return smoothedConfidence
    }

    /// Reset the smoother state (e.g., on scene change).
    mutating func reset() {
        smoothedValue = 0
        isInitialized = false
        recentValues.removeAll()
        smoothedConfidence = 0
    }

    // MARK: - Distance-Dependent Alpha

    /// Compute the EMA alpha for the given motion state and depth.
    ///
    /// At long range (100m+), stationary alpha drops to as low as 0.03,
    /// meaning only 3% of each new noisy reading passes through. This
    /// prevents the display from oscillating when the crosshair is steady.
    ///
    /// Panning alpha stays relatively high — if the user is actively
    /// moving the device, they expect the reading to change.
    private func computeAlpha(motionState: MotionState, depth: Double) -> Double {
        let baseAlpha: Double
        switch motionState {
        case .stationary:
            baseAlpha = stationaryAlphaBase
        case .panning:
            baseAlpha = panningAlphaBase
        case .tracking:
            baseAlpha = trackingAlphaBase
        }

        // Distance scaling: reduce alpha at longer ranges
        // Only affects stationary and tracking — panning stays responsive
        guard motionState != .panning else { return baseAlpha }

        let distanceScale: Double
        if depth < 20 {
            distanceScale = 1.0       // Short range: full alpha
        } else if depth < 50 {
            distanceScale = 0.8       // Mid range: slightly heavier
        } else if depth < 100 {
            distanceScale = 0.5       // Far: half alpha (0.06 for stationary)
        } else if depth < 200 {
            distanceScale = 0.3       // Very far: aggressive smoothing (0.036)
        } else {
            distanceScale = 0.2       // Extreme: ultra-heavy (0.024)
        }

        // Floor: never go below 0.02 (would take ~50 frames to converge)
        return max(0.02, baseAlpha * distanceScale)
    }

    // MARK: - Discontinuity Detection

    /// Detects if the crosshair has crossed an object edge.
    ///
    /// If the last N frames all agree on a new depth that differs
    /// significantly from the smoothed value, it's not noise —
    /// it's a genuine depth edge. Snap to the new value.
    ///
    /// The threshold scales with distance: at long range, neural noise
    /// can easily produce 30% swings, so we require a larger relative
    /// change before declaring a discontinuity.
    private func detectDiscontinuity(newValue: Double) -> Bool {
        guard recentValues.count >= confirmationFrames,
              smoothedValue > 0 else { return false }

        // Distance-scaled threshold: at 200m, require 60% change instead of 30%
        let threshold = discontinuityThreshold(forDepth: smoothedValue)

        // Check if relative change is above threshold
        let relativeChange = abs(newValue - smoothedValue) / smoothedValue
        guard relativeChange > threshold else { return false }

        // Check if all recent values agree (they're all on the "new" side)
        // AND they all agree with each other (low spread)
        let newIsLarger = newValue > smoothedValue
        let halfThreshold = threshold * 0.5
        let allAgree = recentValues.allSatisfy { value in
            let change = abs(value - smoothedValue) / smoothedValue
            let isLarger = value > smoothedValue
            return change > halfThreshold && isLarger == newIsLarger
        }

        guard allAgree else { return false }

        // Additional check: the recent values must be consistent with each other
        // (not just all "different from smoothed" — they must agree on WHERE they are)
        let recentMedian = sortedMedian(recentValues)
        let spread = recentValues.map { abs($0 - recentMedian) / max(recentMedian, 1.0) }.max() ?? 0
        // At long range, require tighter agreement among recent values
        let maxSpread = smoothedValue > 100 ? 0.10 : 0.15
        return spread < maxSpread
    }

    /// Discontinuity threshold scales with distance to prevent false triggers
    /// from neural depth noise at long range.
    private func discontinuityThreshold(forDepth depth: Double) -> Double {
        if depth < 20 {
            return discontinuityThresholdBase       // 30% at short range
        } else if depth < 50 {
            return discontinuityThresholdBase * 1.2 // 36%
        } else if depth < 100 {
            return discontinuityThresholdBase * 1.5 // 45%
        } else if depth < 200 {
            return discontinuityThresholdBase * 2.0 // 60%
        } else {
            return discontinuityThresholdBase * 2.5 // 75% at extreme range
        }
    }

    private func sortedMedian(_ values: [Double]) -> Double {
        let sorted = values.sorted()
        return sorted[sorted.count / 2]
    }
}
