//
//  RangingEngine.swift
//  Rangefinder
//
//  Reads the UnifiedDepthField at the crosshair, applies inclination
//  correction and motion-aware smoothing to produce the final range output.
//
//  Performance-critical: object detection runs on its own cadence
//  (not blocking the depth pipeline). Geometric ranging is pure math
//  and adds negligible cost.
//
//  Prediction pipeline (added for transition smoothing):
//  1. IMUDepthPredictor tracks camera motion between inference frames
//  2. DepthKalmanFilter provides temporal prediction + smooth transitions
//  3. When a new depth measurement arrives, the Kalman filter is updated
//  4. Between measurements, the Kalman filter predicts using IMU velocity
//

import Foundation
import Combine
import CoreVideo
import simd
import os

@MainActor
class RangingEngine: ObservableObject {

    // MARK: - Output

    @Published var currentRange: RangeOutput = .none

    // MARK: - Dependencies

    let depthField: UnifiedDepthField
    let inclinationManager: InclinationManager
    let objectDetector = ObjectDetector()
    weak var performanceMonitor: PerformanceMonitor?
    weak var locationManager: LocationManager?

    // MARK: - Multi-Hypothesis Tracking

    @Published var backgroundRange: RangeOutput = .none
    private var fgKalmanFilter = DepthKalmanFilter()
    private var bgKalmanFilter = DepthKalmanFilter()
    private var imuPredictor = IMUDepthPredictor()
    private var previousSemanticDecision: SemanticSourceDecision = .none

    // MARK: - State

    private var smoother = MotionAwareSmoother()
    private var cancellables = Set<AnyCancellable>()

    /// Rolling buffer of recent fused depth readings for outlier rejection.
    /// At long range, neural depth can produce occasional wild outliers
    /// (e.g., 200m→50m→210m). The median pre-filter rejects these before
    /// they pollute the Kalman filter's state estimate.
    private var depthRingBuffer: [Double] = []
    private let ringBufferSize = 5

    // Object detection runs on its own slower cadence to avoid blocking depth
    private var objectDetectionTask: Task<Void, Never>?
    private var lastObjectDetectionTime: CFAbsoluteTime = 0
    private let objectDetectionInterval: CFAbsoluteTime = 0.2 // 5 FPS max

    // DEM ray-casting runs at 2 Hz (I/O bound, not per-frame)
    private var demQueryTask: Task<Void, Never>?
    private var lastDEMQueryTime: CFAbsoluteTime = 0

    // MARK: - Init

    init(depthField: UnifiedDepthField, inclinationManager: InclinationManager) {
        self.depthField = depthField
        self.inclinationManager = inclinationManager
    }

    // MARK: - Model Loading

    func loadModels() async {
        await depthField.loadModels()

        // Object detector is supplementary — don't block on it
        Task.detached { [objectDetector] in
            do {
                try await objectDetector.loadModel()
            } catch {
                Logger.detection.warning("Object detector not available: \(error.localizedDescription)")
            }
        }
    }

    // MARK: - Process Frame

    /// Full pipeline: depth + object detection → fusion → Kalman → smoothing → output.
    /// Object detection runs asynchronously on its own cadence, never blocking depth.
    func processFrame(_ frameData: FrameData) async {
        // Check for task cancellation early
        guard !Task.isCancelled else { return }

        // 0. Always feed camera pose to IMU predictor (cheap, ~0 cost)
        imuPredictor.updateCameraPose(
            cameraTransform: frameData.cameraPose,
            timestamp: frameData.timestamp
        )

        // 1. Kick off object detection in background (non-blocking, own cadence)
        scheduleObjectDetection(frameData: frameData)

        // 1b. Kick off DEM ray-cast in background (2 Hz, I/O bound)
        scheduleDEMQuery()

        // 2. Feed current pitch + heading to depth field for geometric + DEM ranging
        depthField.currentPitchRadians = Double(inclinationManager.pitchRadians)
        depthField.currentHeadingDegrees = inclinationManager.headingDegrees

        // 3. Process depth pipeline (LiDAR + neural + geometric + DEM)
        await depthField.processFrame(frameData)

        // Check cancellation after the heavy work
        guard !Task.isCancelled else { return }

        // 4. Read fused depth at crosshair
        var depthEstimate = depthField.crosshairDepth

        // 4a. Plausibility check: reject obviously impossible readings.
        // ARKit's depth pipeline is unaffected by visual zoom (it always
        // processes the full wide-angle FOV), so depth data is correct at
        // any zoom level. This check catches rare calibration failures or
        // sensor glitches that produce sub-0.1m readings.
        if depthEstimate.isValid && depthEstimate.distanceMeters < 0.1 {
            Logger.ranging.info("Plausibility reject: \(String(format: "%.3f", depthEstimate.distanceMeters))m is below minimum plausible range, src=\(depthEstimate.source.shortName)")
            depthEstimate = .none
        }

        guard depthEstimate.isValid else {
            // Even with no valid measurement, try Kalman prediction
            if let predicted = fgKalmanFilter.predict(at: frameData.timestamp),
               fgKalmanFilter.isTracking {
                emitPredictedRange(
                    depth: predicted,
                    confidence: max(0.15, smoother.smoothConfidence(0.3)),
                    uncertainty: fgKalmanFilter.uncertainty,
                    timestamp: frameData.timestamp
                )
            } else {
                currentRange = .none
            }
            return
        }

        // 5. Semantic source switch detection: if the semantic decision changed,
        //    reset the foreground Kalman filter to avoid stale state contaminating
        //    the new source's readings. MUST happen BEFORE outlier rejection so the
        //    ring buffer is clean when the new source's reading arrives — otherwise
        //    old readings from the previous source cause false outlier rejection.
        let currentDecision = depthField.semanticDecision
        if currentDecision != previousSemanticDecision {
            if previousSemanticDecision != .none {
                Logger.ranging.info("Semantic switch: \(self.previousSemanticDecision.rawValue) -> \(currentDecision.rawValue) — resetting fg Kalman")
                fgKalmanFilter.reset()
                depthRingBuffer.removeAll()
            }
            previousSemanticDecision = currentDecision
        }

        // 6. Outlier rejection: check if this measurement is an outlier
        //    relative to recent readings. At long range, occasional wild
        //    spikes can occur due to disparity noise amplification.
        let motionState = inclinationManager.motionState
        let rawDepth = depthEstimate.distanceMeters
        let filteredDepth = rejectOutlier(rawDepth, motionState: motionState)

        // 7. Update Kalman filter with filtered measurement
        let kalmanDepth = fgKalmanFilter.update(
            measurement: filteredDepth,
            confidence: depthEstimate.confidence,
            motionState: motionState,
            timestamp: frameData.timestamp
        )

        // Tell IMU predictor that we consumed a measurement
        imuPredictor.onNewMeasurement()

        // 8. Apply motion-aware smoothing on Kalman-filtered depth
        let smoothedDistance = smoother.smooth(
            newValue: kalmanDepth,
            motionState: motionState
        )
        let smoothedConfidence = smoother.smoothConfidence(depthEstimate.confidence)

        // 9. Apply inclination correction
        let pitchRad = inclinationManager.pitchRadians
        let (adjustedRange, correctionFactor) = InclinationCorrector.correct(
            lineOfSightRange: smoothedDistance,
            pitchRadians: pitchRad
        )

        // 10. Build foreground output
        currentRange = RangeOutput(
            lineOfSightRange: Measurement(value: smoothedDistance, unit: .meters),
            adjustedRange: Measurement(value: adjustedRange, unit: .meters),
            confidence: smoothedConfidence,
            uncertainty: Measurement(value: min(depthEstimate.uncertainty, fgKalmanFilter.uncertainty), unit: .meters),
            inclinationDegrees: inclinationManager.pitchDegrees,
            inclinationCorrectionFactor: correctionFactor,
            primarySource: depthEstimate.source,
            sourceWeights: depthEstimate.sourceWeights,
            timestamp: Date()
        )

        // 11. Background hypothesis processing
        let bgEstimate = depthField.backgroundDepth
        if bgEstimate.isValid {
            let bgKalmanDepth = bgKalmanFilter.update(
                measurement: bgEstimate.distanceMeters,
                confidence: bgEstimate.confidence,
                motionState: motionState,
                timestamp: frameData.timestamp
            )
            let bgSmoothed = smoother.smooth(newValue: bgKalmanDepth, motionState: motionState)
            let bgSmoothedConf = smoother.smoothConfidence(bgEstimate.confidence)
            let (bgAdjusted, bgCorrFactor) = InclinationCorrector.correct(
                lineOfSightRange: bgSmoothed,
                pitchRadians: pitchRad
            )
            backgroundRange = RangeOutput(
                lineOfSightRange: Measurement(value: bgSmoothed, unit: .meters),
                adjustedRange: Measurement(value: bgAdjusted, unit: .meters),
                confidence: bgSmoothedConf,
                uncertainty: Measurement(value: bgEstimate.uncertainty, unit: .meters),
                inclinationDegrees: inclinationManager.pitchDegrees,
                inclinationCorrectionFactor: bgCorrFactor,
                primarySource: bgEstimate.source,
                sourceWeights: bgEstimate.sourceWeights,
                timestamp: Date()
            )
        } else {
            backgroundRange = .none
        }
    }

    /// Emit a range output from Kalman prediction (no fresh measurement available).
    private func emitPredictedRange(
        depth: Double,
        confidence: Float,
        uncertainty: Double,
        timestamp: TimeInterval
    ) {
        let motionState = inclinationManager.motionState
        let smoothedDistance = smoother.smooth(newValue: depth, motionState: motionState)
        let smoothedConfidence = smoother.smoothConfidence(confidence)

        let pitchRad = inclinationManager.pitchRadians
        let (adjustedRange, correctionFactor) = InclinationCorrector.correct(
            lineOfSightRange: smoothedDistance,
            pitchRadians: pitchRad
        )

        currentRange = RangeOutput(
            lineOfSightRange: Measurement(value: smoothedDistance, unit: .meters),
            adjustedRange: Measurement(value: adjustedRange, unit: .meters),
            confidence: smoothedConfidence,
            uncertainty: Measurement(value: uncertainty, unit: .meters),
            inclinationDegrees: inclinationManager.pitchDegrees,
            inclinationCorrectionFactor: correctionFactor,
            primarySource: .semantic,
            sourceWeights: [:],
            timestamp: Date()
        )
    }

    // MARK: - Outlier Rejection

    /// Pre-filter that rejects depth measurements that are wild outliers
    /// relative to recent history. Returns the raw measurement if it's
    /// reasonable, or the median of recent values if it's an outlier.
    ///
    /// This prevents single-frame spikes from corrupting Kalman state,
    /// which is especially important at long range where inverse-depth
    /// calibration can amplify noise into 2-3× jumps.
    ///
    /// When bimodal far-target mode triggers a legitimate large jump (e.g.,
    /// 42m → 1600m), the ring buffer is cleared to prevent the old near
    /// readings from rejecting the new far reading.
    private func rejectOutlier(_ rawDepth: Double, motionState: MotionState) -> Double {
        // DEM-primary terrain routing: when the depth field returned DEM as
        // the primary source, accept the reading directly. The ring buffer
        // may contain near-range readings from neural — those should not
        // block a legitimate terrain distance from DEM ray-cast.
        if depthField.crosshairDepth.source == .demRaycast {
            if depthRingBuffer.count >= 3 {
                let sorted = depthRingBuffer.sorted()
                let median = sorted[sorted.count / 2]
                if median > 0.5 {
                    let ratio = rawDepth / median
                    if ratio > 2.0 || ratio < 0.5 {
                        // Large transition from old readings → reset tracking
                        Logger.ranging.info("DEM-primary transition: \(String(format: "%.0f", median))m → \(String(format: "%.0f", rawDepth))m")
                        depthRingBuffer.removeAll()
                        fgKalmanFilter.reset()
                    }
                }
            }
            depthRingBuffer.append(rawDepth)
            if depthRingBuffer.count > ringBufferSize {
                depthRingBuffer.removeFirst()
            }
            return rawDepth
        }

        // When bimodal far-target override is active, allow large jumps.
        // Detect a legitimate transition: the fused depth jumped >3× from
        // the median of recent readings. If the depth field is bimodal AND
        // far-target mode is active, this is intentional — clear the buffer
        // and accept the new reading.
        if depthField.targetPriority == .far && depthField.isBimodal
           && depthRingBuffer.count >= 3 {
            let sorted = depthRingBuffer.sorted()
            let median = sorted[sorted.count / 2]
            if median > 0.5 {
                let jump = rawDepth / median
                if jump > 3.0 || jump < 0.33 {
                    // Legitimate far-target transition — reset tracking
                    Logger.ranging.info("Far-target transition: \(String(format: "%.0f", median))m → \(String(format: "%.0f", rawDepth))m (bimodal override)")
                    depthRingBuffer.removeAll()
                    depthRingBuffer.append(rawDepth)
                    fgKalmanFilter.reset()
                    return rawDepth
                }
            }
        }

        // Always add to ring buffer
        depthRingBuffer.append(rawDepth)
        if depthRingBuffer.count > ringBufferSize {
            depthRingBuffer.removeFirst()
        }

        // Need at least 3 readings for meaningful outlier detection
        guard depthRingBuffer.count >= 3 else { return rawDepth }

        // Don't reject outliers while panning — scene is genuinely changing
        guard motionState != .panning else { return rawDepth }

        let sorted = depthRingBuffer.sorted()
        let median = sorted[sorted.count / 2]
        guard median > 0.5 else { return rawDepth }

        // Relative deviation from median
        let deviation = abs(rawDepth - median) / median

        // Distance-scaled outlier threshold:
        // At short range (< 20m), allow 40% deviation
        // At long range (200m+), only allow 25% — noise produces larger relative swings
        let threshold: Double
        if median < 20 {
            threshold = 0.40
        } else if median < 50 {
            threshold = 0.35
        } else if median < 100 {
            threshold = 0.30
        } else {
            threshold = 0.25
        }

        if deviation > threshold {
            // This measurement is an outlier — replace with median
            Logger.ranging.debug("Outlier rejected: raw=\(String(format: "%.1f", rawDepth))m median=\(String(format: "%.1f", median))m dev=\(String(format: "%.0f%%", deviation * 100))")
            return median
        }

        return rawDepth
    }

    // MARK: - IMU Feed (called on EVERY frame, including dropped ones)

    /// Feed camera pose for IMU prediction — called from AppState on every
    /// ARFrame, even when depth inference is still running on the previous frame.
    /// This keeps the motion model current so Kalman prediction stays accurate.
    func feedIMU(cameraTransform: simd_float4x4, timestamp: TimeInterval) {
        imuPredictor.updateCameraPose(
            cameraTransform: cameraTransform,
            timestamp: timestamp
        )
    }

    // MARK: - Object Detection (decoupled cadence)

    /// Fires object detection at ~5 FPS max, never blocking the depth pipeline.
    private func scheduleObjectDetection(frameData: FrameData) {
        guard objectDetector.isLoaded else { return }

        let now = CFAbsoluteTimeGetCurrent()
        guard now - lastObjectDetectionTime >= objectDetectionInterval else { return }

        // Don't stack detections
        guard objectDetectionTask == nil else { return }

        lastObjectDetectionTime = now
        objectDetectionTask = Task { [weak self] in
            guard let self = self else { return }
            defer { self.objectDetectionTask = nil }

            do {
                let detections = try await self.objectDetector.detect(in: frameData.capturedImage)
                guard !Task.isCancelled else { return }
                let results = ObjectRangeEstimator.estimateRanges(
                    detections: detections,
                    intrinsics: frameData.intrinsics,
                    imageWidth: CVPixelBufferGetWidth(frameData.capturedImage),
                    imageHeight: CVPixelBufferGetHeight(frameData.capturedImage)
                )
                self.depthField.updateObjectDetections(results)
            } catch {
                Logger.detection.error("Detection failed: \(error.localizedDescription)")
            }
        }
    }

    // MARK: - DEM Ray-Casting (decoupled cadence)

    /// Fires DEM ray-cast at ~2 Hz max, non-blocking.
    private func scheduleDEMQuery() {
        guard let demEstimator = depthField.demEstimator,
              let locationManager = locationManager,
              locationManager.hasValidFix else { return }

        let now = CFAbsoluteTimeGetCurrent()
        guard now - lastDEMQueryTime >= AppConfiguration.demQueryInterval else { return }
        guard demQueryTask == nil else { return }

        lastDEMQueryTime = now
        let coordinate = locationManager.coordinate
        let altitude = locationManager.bestAltitude           // Prefers barometric when more accurate
        let horizontalAccuracy = locationManager.horizontalAccuracy
        let verticalAccuracy = locationManager.bestVerticalAccuracy
        let pitchRadians = inclinationManager.pitchRadians
        let headingDegrees = inclinationManager.headingDegrees

        demQueryTask = Task { [weak self] in
            guard let self = self else { return }
            defer { self.demQueryTask = nil }

            let estimate = await demEstimator.estimate(
                coordinate: coordinate,
                altitude: altitude,
                pitchRadians: pitchRadians,
                headingDegrees: headingDegrees,
                horizontalAccuracy: horizontalAccuracy,
                verticalAccuracy: verticalAccuracy
            )

            guard !Task.isCancelled else { return }
            self.depthField.latestDEMEstimate = estimate
        }
    }

    // MARK: - Target Mode Support

    /// Clear the outlier rejection buffer to allow immediate transition
    /// when the user switches between near/far target mode.
    func clearOutlierBuffer() {
        depthRingBuffer.removeAll()
        fgKalmanFilter.reset()
        bgKalmanFilter.reset()
    }

    // MARK: - Reset

    func reset() {
        smoother.reset()
        fgKalmanFilter.reset()
        bgKalmanFilter.reset()
        imuPredictor.reset()
        depthRingBuffer.removeAll()
        objectDetectionTask?.cancel()
        objectDetectionTask = nil
        demQueryTask?.cancel()
        demQueryTask = nil
        depthField.latestDEMEstimate = nil
        depthField.demEstimator?.reset()
        currentRange = .none
        backgroundRange = .none
        previousSemanticDecision = .none
    }
}
