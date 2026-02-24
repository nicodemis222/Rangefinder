//
//  UnifiedDepthField.swift
//  Rangefinder
//
//  Semantic source selection: each frame, gather all source readings, then
//  a priority state machine picks ONE authoritative source. No weighted
//  fusion — the best source for the current scene wins outright.
//
//  Priority order:
//  1. Stadiametric (manual bracket overlay — user intent is explicit)
//  2. LiDAR (< 8m, high confidence — gold standard at close range)
//  3. Object detection (known-size pinhole ranging at crosshair)
//  4. DEM ray-cast (terrain target, no object detected)
//  5. Neural depth (calibrated, < 50m hard cap)
//  6. Geometric ground-plane (fallback)
//
//  A secondary "background" hypothesis is also emitted for the UI
//  (e.g., if DEM is primary, background = neural/geo foreground reading).
//

import Foundation
import Combine
import CoreVideo
import os

@MainActor
class UnifiedDepthField: ObservableObject {

    // MARK: - Output

    @Published var crosshairDepth: DepthEstimate = .none
    @Published var backgroundDepth: DepthEstimate = .none
    @Published var semanticDecision: SemanticSourceDecision = .none
    var stadiametricInput: StadiametricInput?

    // MARK: - Dependencies

    let metricEstimator = MetricDepthEstimator()
    // Force inverse model type: DepthAnythingV2 outputs inverse/relative depth
    // (high values = close, low values = far). Auto-detection over the narrow
    // LiDAR calibration range (0.2-8m) is unreliable.
    let calibrator = ContinuousCalibrator(forceModelType: .inverse)

    /// Geometric ground-plane ranging using IMU pitch + camera height.
    /// No ML model needed — pure trigonometry on sensor data.
    var geometricEstimator = GeometricRangeEstimator()

    /// Scene classifier: analyzes the neural depth map to detect sky, ground,
    /// and structure regions at the crosshair. Zero model cost — uses depth
    /// map statistics that we're already computing.
    let sceneClassifier = SceneClassifier()

    /// DEM ray-casting estimator for terrain-aware ranging.
    /// Set by AppState once GPS is available.
    var demEstimator: DEMRaycastEstimator?

    /// Latest DEM estimate (updated at 2Hz by RangingEngine).
    var latestDEMEstimate: DEMRaycastEstimate?

    /// Current device pitch in radians (0=level, negative=down).
    /// Updated every frame from InclinationManager via RangingEngine.
    var currentPitchRadians: Double = 0

    /// Current true-north heading in degrees (0-360).
    /// Updated every frame from InclinationManager via RangingEngine.
    var currentHeadingDegrees: Double = 0

    /// Target priority mode: near (first target) or far (last target).
    /// When far-target mode is active and the depth field shows bimodal
    /// distribution (foreground occluder + background terrain), the semantic
    /// selector prefers the DEM/far-distance reading.
    var targetPriority: TargetPriority = .far

    /// Whether the current frame detected a bimodal depth distribution.
    /// Published so the UI can show an indicator when occluders are detected.
    @Published var isBimodal: Bool = false

    /// Latest bimodal analysis result — exposed for the reticle depth zone overlay.
    /// Updated every frame; contains near/far peak distances and cluster fractions.
    @Published var latestBimodalAnalysis: BimodalAnalysis = .notBimodal

    // MARK: - Latest Depth Maps (updated by pipeline)

    private var latestLiDARDepthMap: CVPixelBuffer?
    private var latestLiDARConfidenceMap: CVPixelBuffer?
    private var latestNeuralDepthMap: CVPixelBuffer?
    private var latestObjectDetections: [ObjectRangeResult] = []

    // MARK: - Frame Processing

    private var cancellables = Set<AnyCancellable>()
    private var isProcessing = false

    // MARK: - Init

    init() {}

    // MARK: - Model Loading

    func loadModels() async {
        // Load standard model (critical path)
        do {
            try await metricEstimator.loadModel()
            Logger.depth.info("Metric depth model ready")
        } catch {
            Logger.depth.error("Failed to load metric depth model: \(error.localizedDescription)")
        }
    }

    // MARK: - Source Updates

    /// Called with each ARFrame's LiDAR depth data.
    func updateLiDAR(depthMap: CVPixelBuffer, confidenceMap: CVPixelBuffer?) {
        latestLiDARDepthMap = depthMap
        latestLiDARConfidenceMap = confidenceMap
    }

    /// Called when neural depth estimation completes.
    func updateNeuralDepth(depthMap: CVPixelBuffer) {
        latestNeuralDepthMap = depthMap
    }

    /// Called when object detection + ranging completes.
    func updateObjectDetections(_ detections: [ObjectRangeResult]) {
        latestObjectDetections = detections
    }

    // MARK: - Process Frame

    /// Process a camera frame through the depth pipeline.
    ///
    /// Critical performance design:
    /// - LiDAR update is instant (just stores the buffer pointer)
    /// - Neural depth is the primary `await` — this is the one we wait for
    /// - Geometric ranging is pure math — completes in microseconds
    func processFrame(_ frameData: FrameData) async {
        guard !isProcessing else { return }
        isProcessing = true
        defer { isProcessing = false }

        // 1. Update LiDAR data (instant — just stores the buffer pointer)
        if let lidarDepth = frameData.lidarDepthMap {
            updateLiDAR(depthMap: lidarDepth, confidenceMap: frameData.lidarConfidenceMap)
        }

        // 2. Run neural depth (this is the only await we block on)
        await runNeuralDepth(frameData: frameData)

        // 2b. Classify scene at crosshair using the neural depth map (~0.1ms)
        if let neuralMap = latestNeuralDepthMap {
            sceneClassifier.classify(
                depthMap: neuralMap,
                pitchDegrees: currentPitchRadians * 180.0 / .pi
            )
        }

        // 2c. Bimodal depth analysis for far-target mode
        //     Detect whether the crosshair ROI contains both near foreground
        //     and far background objects (e.g., rocks at 40m + mountains at 1600m)
        var bimodalResult: BimodalAnalysis = .notBimodal
        if let neuralMap = latestNeuralDepthMap {
            bimodalResult = analyzeBimodalDepth(neuralMap: neuralMap)
            isBimodal = bimodalResult.isBimodal
            latestBimodalAnalysis = bimodalResult
        }

        // Check for cancellation before publishing
        guard !Task.isCancelled else { return }

        // 3. Semantic source selection at crosshair (screen center = 0.5, 0.5)
        let crosshairPoint = CGPoint(x: 0.5, y: 0.5)
        let (primary, background) = semanticSelect(
            screenPoint: crosshairPoint,
            timestamp: frameData.timestamp,
            bimodal: bimodalResult
        )
        crosshairDepth = primary
        backgroundDepth = background ?? .none
    }

    // MARK: - Concurrent Depth Tasks

    // Track frames for periodic logging
    private var frameCount: Int = 0

    private func runNeuralDepth(frameData: FrameData) async {
        guard metricEstimator.isLoaded else { return }
        do {
            let result = try await metricEstimator.estimateDepth(
                from: frameData.capturedImage,
                intrinsics: frameData.intrinsics
            )
            updateNeuralDepth(depthMap: result.depthMap)

            frameCount += 1

            // Log neural depth map stats periodically
            if frameCount % 60 == 1 {
                let centerSample = metricEstimator.sampleDepth(from: result.depthMap, at: CGPoint(x: 0.5, y: 0.5))
                let format = CVPixelBufferGetPixelFormatType(result.depthMap)
                let w = CVPixelBufferGetWidth(result.depthMap)
                let h = CVPixelBufferGetHeight(result.depthMap)
                Logger.depth.info("Neural depth map: \(w)x\(h) fmt=\(format) center=\(String(describing: centerSample)) range=[\(String(format: "%.3f", result.minDepth)),\(String(format: "%.3f", result.maxDepth))]")
            }

            // Continuous calibration: where LiDAR is valid, feed pairs
            if let lidarMap = latestLiDARDepthMap {
                feedCalibration(
                    neuralMap: result.depthMap,
                    lidarMap: lidarMap,
                    confidenceMap: frameData.lidarConfidenceMap,
                    timestamp: frameData.timestamp
                )
            }
        } catch {
            Logger.depth.error("Neural depth failed: \(error.localizedDescription)")
        }
    }

    // MARK: - Semantic Source Selection

    /// A single source entry: tracks source identity, depth, and weight.
    private struct SourceEntry {
        let source: DepthSource
        var depth: Float
        var weight: Float
    }

    /// Semantic source selection: gather all source readings, then apply
    /// a priority state machine to pick ONE authoritative primary source.
    /// Returns (primary, background?) where background is an alternate
    /// hypothesis for the UI.
    func semanticSelect(
        screenPoint: CGPoint,
        timestamp: TimeInterval,
        bimodal: BimodalAnalysis = .notBimodal
    ) -> (primary: DepthEstimate, background: DepthEstimate?) {

        var entries: [SourceEntry] = []
        var sourceWeights: [DepthSource: Float] = [:]

        // --- Gather All Source Readings ---

        // --- LiDAR ---
        if let lidarMap = latestLiDARDepthMap {
            if let lidarDepth = LiDARDepthProvider.sampleDepth(from: lidarMap, at: screenPoint) {
                let lidarConfidence: Float
                if let confMap = latestLiDARConfidenceMap {
                    lidarConfidence = LiDARDepthProvider.sampleConfidence(from: confMap, at: screenPoint)
                } else {
                    lidarConfidence = 0.8
                }

                let distanceWeight = DepthSourceConfidence.lidar(distanceM: lidarDepth)
                let weight = distanceWeight * lidarConfidence

                if weight > 0.01 {
                    entries.append(SourceEntry(source: .lidar, depth: lidarDepth, weight: weight))
                }
            }
        }

        // --- Neural Depth (calibrated) with hard cap ---
        if let neuralMap = latestNeuralDepthMap {
            if let rawNeural = metricEstimator.sampleDepth(from: neuralMap, at: screenPoint) {
                let calibratedDepth = calibrator.calibrate(rawNeural)
                let calConf = calibrator.confidence

                // Log periodically
                if frameCount % 60 == 1 {
                    let calAge = calibrator.calibrationAge(currentTimestamp: timestamp)
                    Logger.depth.info("Neural semantic: raw=\(String(format: "%.3f", rawNeural)) calibrated=\(String(format: "%.3f", calibratedDepth)) calConf=\(String(format: "%.2f", calConf)) calAge=\(String(format: "%.1f", calAge))s model=\(String(describing: self.calibrator.modelType))")
                }

                // Hard cap: skip neural beyond 50m
                if calibratedDepth > 0.1 && calibratedDepth <= AppConfiguration.neuralHardCapMeters {
                    let calAge = calibrator.calibrationAge(currentTimestamp: timestamp)
                    let calQuality = DepthSourceConfidence.calibrationQuality(
                        calibrationAge: calAge,
                        calibrationConfidence: calConf
                    )
                    let distanceWeight = DepthSourceConfidence.neural(distanceM: calibratedDepth)
                    let weight = distanceWeight * calQuality

                    if weight > 0.01 {
                        entries.append(SourceEntry(source: .neural, depth: calibratedDepth, weight: weight))
                    }
                }
            }
        }

        // --- Geometric (Ground-Plane) ---
        if let geoEstimate = geometricEstimator.estimate(pitchRadians: currentPitchRadians) {
            let geoDepth = geoEstimate.distanceMeters
            let distanceWeight = DepthSourceConfidence.geometric(distanceM: geoDepth)
            let weight = distanceWeight * geoEstimate.confidence

            if frameCount % 60 == 1 {
                Logger.depth.info("Geometric: D=\(String(format: "%.1f", geoDepth))m pitch=\(String(format: "%.2f", geoEstimate.pitchBelowHorizontalDeg))deg conf=\(String(format: "%.2f", geoEstimate.confidence)) weight=\(String(format: "%.3f", weight))")
            }

            if weight > 0.01 {
                entries.append(SourceEntry(source: .geometric, depth: geoDepth, weight: weight))
            }
        }

        // --- DEM Ray-Cast (Terrain) ---
        if let demEstimate = latestDEMEstimate {
            let demDepth = demEstimate.distanceMeters
            let distanceWeight = DepthSourceConfidence.demRaycast(
                distanceM: demDepth,
                gpsAccuracy: demEstimate.gpsAccuracy,
                headingAccuracy: 5.0
            )
            let weight = distanceWeight * demEstimate.confidence

            if frameCount % 60 == 1 {
                Logger.depth.info("DEM: D=\(String(format: "%.1f", demDepth))m heading=\(String(format: "%.1f", demEstimate.headingDeg))° terrainElev=\(String(format: "%.1f", demEstimate.terrainElevation))m gpsAcc=\(String(format: "%.1f", demEstimate.gpsAccuracy))m conf=\(String(format: "%.2f", demEstimate.confidence)) weight=\(String(format: "%.3f", weight))")
            }

            if weight > 0.01 {
                entries.append(SourceEntry(source: .demRaycast, depth: demDepth, weight: weight))
            }
        }

        // --- Object Detection ---
        if !latestObjectDetections.isEmpty {
            let centerDetection = latestObjectDetections.min { a, b in
                let da = hypot(a.screenCenter.x - screenPoint.x, a.screenCenter.y - screenPoint.y)
                let db = hypot(b.screenCenter.x - screenPoint.x, b.screenCenter.y - screenPoint.y)
                return da < db
            }

            if let det = centerDetection {
                let dist = hypot(det.screenCenter.x - screenPoint.x, det.screenCenter.y - screenPoint.y)
                if dist < 0.15 {
                    let objectDepth = Float(det.distanceMeters)
                    let weight = DepthSourceConfidence.object(
                        distanceM: objectDepth,
                        detectionConfidence: det.confidence
                    )

                    if weight > 0.01 {
                        entries.append(SourceEntry(source: .objectSize, depth: objectDepth, weight: weight))
                    }
                }
            }
        }

        // Record all source weights for HUD display
        for entry in entries where entry.weight > 0 {
            sourceWeights[entry.source] = entry.weight
        }

        // --- Semantic Priority State Machine ---

        let hasObjectAtCrosshair = entries.contains { $0.source == .objectSize && $0.weight > 0.05 }
        var decision: SemanticSourceDecision = .none
        var primaryEntry: SourceEntry?
        var backgroundEntry: SourceEntry?

        // 0. Stadiametric input (highest priority — user-explicit)
        if let stadia = stadiametricInput, stadia.computedRange > 0 {
            decision = .stadiametric
            let stadiaRange = Float(stadia.computedRange)
            primaryEntry = SourceEntry(source: .stadiametric, depth: stadiaRange, weight: 1.0)
            // Background: best automated source
            backgroundEntry = entries.max(by: { $0.weight < $1.weight })
        }

        // 1. LiDAR (< 8m, authoritative)
        if primaryEntry == nil,
           let lidarEntry = entries.first(where: { $0.source == .lidar }),
           lidarEntry.depth < 8.0, lidarEntry.weight > 0.3 {
            decision = .lidarPrimary
            primaryEntry = lidarEntry
            // Background: DEM or neural
            backgroundEntry = entries.first(where: { $0.source == .demRaycast && $0.weight > 0.01 })
                ?? entries.first(where: { $0.source == .neural && $0.weight > 0.01 })
        }

        // 2. Object detection (known-size at crosshair, authoritative)
        if primaryEntry == nil, hasObjectAtCrosshair,
           let objEntry = entries.first(where: { $0.source == .objectSize }) {
            decision = .objectPrimary
            primaryEntry = objEntry
            // Background: DEM if available
            backgroundEntry = entries.first(where: { $0.source == .demRaycast && $0.weight > 0.01 })
        }

        // 3. DEM (terrain target, far-target priority, no object)
        if primaryEntry == nil,
           let demEntry = entries.first(where: { $0.source == .demRaycast }),
           demEntry.weight > 0.15,
           !hasObjectAtCrosshair,
           targetPriority == .far {
            decision = .demPrimary
            primaryEntry = demEntry
            // Background: neural or geometric (foreground context)
            backgroundEntry = entries.first(where: { $0.source == .neural && $0.weight > 0.01 })
                ?? entries.first(where: { $0.source == .geometric && $0.weight > 0.01 })
        }

        // 3b. DEM even in near mode if no object and DEM is available
        if primaryEntry == nil,
           let demEntry = entries.first(where: { $0.source == .demRaycast }),
           demEntry.weight > 0.15,
           !hasObjectAtCrosshair {
            decision = .demPrimary
            primaryEntry = demEntry
            backgroundEntry = entries.first(where: { $0.source == .neural && $0.weight > 0.01 })
                ?? entries.first(where: { $0.source == .geometric && $0.weight > 0.01 })
        }

        // 4. Neural (< 50m, calibrated, mid-range)
        if primaryEntry == nil,
           let neuralEntry = entries.first(where: { $0.source == .neural }),
           neuralEntry.weight > 0.05 {
            decision = .neuralPrimary
            primaryEntry = neuralEntry
            // Background: DEM if available
            backgroundEntry = entries.first(where: { $0.source == .demRaycast && $0.weight > 0.01 })
        }

        // 5. Geometric (fallback)
        if primaryEntry == nil,
           let geoEntry = entries.first(where: { $0.source == .geometric }),
           geoEntry.weight > 0.05 {
            decision = .geometricPrimary
            primaryEntry = geoEntry
            backgroundEntry = entries.first(where: { $0.source == .demRaycast && $0.weight > 0.01 })
        }

        // Update published decision
        semanticDecision = decision

        // --- Build Primary Estimate ---
        guard let primary = primaryEntry else {
            if frameCount % 60 == 1 {
                Logger.depth.warning("Semantic: NO valid sources")
            }
            return (.none, nil)
        }

        let primaryConfidence = computeSemanticConfidence(entry: primary, decision: decision)
        let primaryUncertainty = computeSemanticUncertainty(entry: primary, decision: decision)

        if frameCount % 60 == 1 {
            let sourceList = sourceWeights.map { "\($0.key.shortName):\(String(format: "%.2f", $0.value))" }.joined(separator: " ")
            Logger.depth.info("Semantic: decision=\(decision.rawValue) primary=\(primary.source.shortName) \(String(format: "%.1f", primary.depth))m conf=\(String(format: "%.2f", primaryConfidence)) sources=[\(sourceList)]")
        }

        let primaryEstimate = DepthEstimate(
            distanceMeters: Double(primary.depth),
            confidence: primaryConfidence,
            uncertainty: primaryUncertainty,
            source: primary.source,
            sourceWeights: sourceWeights,
            timestamp: Date()
        )

        // --- Build Background Estimate ---
        let bgEstimate: DepthEstimate?
        if let bg = backgroundEntry, bg.source != primary.source {
            let bgConf = computeSemanticConfidence(entry: bg, decision: decision)
            let bgUnc = computeSemanticUncertainty(entry: bg, decision: decision)
            bgEstimate = DepthEstimate(
                distanceMeters: Double(bg.depth),
                confidence: bgConf,
                uncertainty: bgUnc,
                source: bg.source,
                sourceWeights: sourceWeights,
                timestamp: Date()
            )
        } else {
            bgEstimate = nil
        }

        return (primaryEstimate, bgEstimate)
    }

    // MARK: - Semantic Confidence

    /// Compute confidence for the selected source based on the semantic decision.
    private func computeSemanticConfidence(entry: SourceEntry, decision: SemanticSourceDecision) -> Float {
        switch decision {
        case .lidarPrimary:
            // LiDAR is authoritative at close range
            return min(1.0, entry.weight * 1.1)
        case .objectPrimary:
            // Object detection confidence is well-calibrated
            return min(1.0, entry.weight)
        case .demPrimary:
            // DEM: scale by the source weight; single-source penalty if no corroboration
            let base = min(1.0, entry.weight)
            return max(0.15, base)
        case .neuralPrimary:
            // Neural within hard cap is reliable
            return min(0.85, entry.weight * 1.05)
        case .geometricPrimary:
            // Geometric is the weakest source
            return min(0.70, entry.weight)
        case .stadiametric:
            // User-manual ranging: confidence depends on how precise the bracket is
            return 0.80
        case .none:
            return 0.0
        }
    }

    // MARK: - Semantic Uncertainty

    /// Compute uncertainty for the selected source based on the semantic decision.
    private func computeSemanticUncertainty(entry: SourceEntry, decision: SemanticSourceDecision) -> Double {
        let depth = Double(entry.depth)
        switch decision {
        case .lidarPrimary:
            return depth * 0.02  // 2% for LiDAR
        case .objectPrimary:
            return depth * 0.12  // 12% for object-size ranging
        case .demPrimary:
            // DEM uncertainty from GPS + heading error
            let gpsAcc = Double(latestDEMEstimate?.gpsAccuracy ?? 10.0)
            let headingErrorM = 5.0 * depth * Double.pi / 180.0
            return sqrt(gpsAcc * gpsAcc + headingErrorM * headingErrorM)
        case .neuralPrimary:
            if depth < 15 {
                return depth * 0.05  // 5% near calibration range
            } else {
                return depth * 0.15  // 15% extrapolation zone
            }
        case .geometricPrimary:
            return depth * 0.15  // 15% for geometric
        case .stadiametric:
            return depth * 0.08  // 8% for manual bracket
        case .none:
            return depth * 0.25
        }
    }

    // MARK: - Bimodal Depth Detection (Far-Target Mode)

    /// Result of bimodal analysis on the depth ROI around the crosshair.
    struct BimodalAnalysis {
        let isBimodal: Bool
        let nearPeakM: Float          // Distance of the near cluster (meters)
        let farPeakM: Float           // Distance of the far cluster (meters)
        let nearFraction: Float       // Fraction of ROI pixels in near cluster (0-1)
        let farFraction: Float        // Fraction of ROI pixels in far cluster (0-1)
        let demAgreesWithFar: Bool    // Whether DEM ray-cast confirms the far peak

        static let notBimodal = BimodalAnalysis(
            isBimodal: false, nearPeakM: 0, farPeakM: 0,
            nearFraction: 0, farFraction: 0, demAgreesWithFar: false
        )
    }

    /// Analyze the neural depth map ROI around center for bimodal distribution.
    /// When the scene has both near (rocks at 40m) and far (mountains at 1600m)
    /// objects, the depth histogram will show two distinct peaks. This is the
    /// key signal for occluder detection.
    ///
    /// Uses log-scale histogram binning because depth spans 3+ orders of magnitude.
    /// Bimodal detection requires:
    /// 1. Two peaks each covering >10% of ROI pixels
    /// 2. A valley between them dropping to <60% of the smaller peak
    /// 3. Peaks separated by at least 2x in distance
    func analyzeBimodalDepth(neuralMap: CVPixelBuffer) -> BimodalAnalysis {
        CVPixelBufferLockBaseAddress(neuralMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(neuralMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(neuralMap) else {
            return .notBimodal
        }

        let width = CVPixelBufferGetWidth(neuralMap)
        let height = CVPixelBufferGetHeight(neuralMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(neuralMap)
        let formatType = CVPixelBufferGetPixelFormatType(neuralMap)

        // Sample center 30% of the frame (wider than the 5x5 patch)
        let roiXStart = Int(Float(width) * 0.35)
        let roiXEnd = Int(Float(width) * 0.65)
        let roiYStart = Int(Float(height) * 0.35)
        let roiYEnd = Int(Float(height) * 0.65)

        // Collect depth samples from ROI
        // Step by 2 for performance (still get ~225 samples from a 30x30 region of 256x192 map)
        var rawValues: [Float] = []
        rawValues.reserveCapacity(400)

        for py in stride(from: roiYStart, to: roiYEnd, by: 2) {
            let rowPtr = baseAddress + py * bytesPerRow
            for px in stride(from: roiXStart, to: roiXEnd, by: 2) {
                let val: Float
                switch formatType {
                case kCVPixelFormatType_DepthFloat16,
                     kCVPixelFormatType_OneComponent16Half:
                    let ptr = rowPtr.assumingMemoryBound(to: UInt16.self)
                    val = float16ToFloat(ptr[px])
                default:
                    let ptr = rowPtr.assumingMemoryBound(to: Float.self)
                    val = ptr[px]
                }
                if !val.isNaN && !val.isInfinite && val > 0.1 {
                    rawValues.append(val)
                }
            }
        }

        guard rawValues.count >= 20 else { return .notBimodal }

        let validValues = rawValues.filter { $0 > 0.001 && $0 < 100000 }
        guard validValues.count >= 15 else { return .notBimodal }

        // Build log-scale histogram on RAW values
        let sortedVals = validValues.sorted()
        let rawMin = sortedVals.first!
        let rawMax = sortedVals.last!
        guard rawMax > rawMin * 1.5 else { return .notBimodal }

        let logMin = log10(rawMin) - 0.1
        let logMax = log10(rawMax) + 0.1
        let numBins = 30
        let binWidth = (logMax - logMin) / Float(numBins)
        var histogram = [Int](repeating: 0, count: numBins)
        var binDepths = [[Float]](repeating: [], count: numBins)

        for depth in validValues {
            let logD = log10(depth)
            let binIdx = min(numBins - 1, max(0, Int((logD - logMin) / binWidth)))
            histogram[binIdx] += 1
            binDepths[binIdx].append(depth)
        }

        // Smooth histogram with a 3-bin moving average
        var smoothed = [Float](repeating: 0, count: numBins)
        for i in 0..<numBins {
            var sum: Float = 0
            var count: Float = 0
            for j in max(0, i-1)...min(numBins-1, i+1) {
                sum += Float(histogram[j])
                count += 1
            }
            smoothed[i] = sum / count
        }

        // Find peaks (local maxima with minimum height of 5% of total samples)
        let minPeakHeight = Float(validValues.count) * 0.05
        var peaks: [(bin: Int, height: Float)] = []

        for i in 1..<(numBins - 1) {
            if smoothed[i] > smoothed[i-1] && smoothed[i] >= smoothed[i+1]
               && smoothed[i] >= minPeakHeight {
                peaks.append((bin: i, height: smoothed[i]))
            }
        }
        // Also check edges
        if smoothed[0] > smoothed[1] && smoothed[0] >= minPeakHeight {
            peaks.insert((bin: 0, height: smoothed[0]), at: 0)
        }
        if smoothed[numBins-1] > smoothed[numBins-2] && smoothed[numBins-1] >= minPeakHeight {
            peaks.append((bin: numBins-1, height: smoothed[numBins-1]))
        }

        guard peaks.count >= 2 else { return .notBimodal }

        let sortedPeaks = peaks.sorted { $0.height > $1.height }
        let peak1 = sortedPeaks[0]
        let peak2 = sortedPeaks[1]
        let nearPeakBin = min(peak1.bin, peak2.bin)
        let farPeakBin = max(peak1.bin, peak2.bin)

        guard farPeakBin - nearPeakBin >= 3 else { return .notBimodal }

        let smallerPeakHeight = min(peak1.height, peak2.height)
        var valleyMin: Float = .infinity
        for i in nearPeakBin...farPeakBin {
            valleyMin = min(valleyMin, smoothed[i])
        }
        guard valleyMin < smallerPeakHeight * 0.6 else { return .notBimodal }

        let midBin = nearPeakBin + (farPeakBin - nearPeakBin) / 2
        var nearDepths: [Float] = []
        var farDepths: [Float] = []
        for i in 0..<numBins {
            if i <= midBin {
                nearDepths.append(contentsOf: binDepths[i])
            } else {
                farDepths.append(contentsOf: binDepths[i])
            }
        }

        guard !nearDepths.isEmpty && !farDepths.isEmpty else { return .notBimodal }

        let nearRawMedian = nearDepths.sorted()[nearDepths.count / 2]
        let farRawMedian = farDepths.sorted()[farDepths.count / 2]

        let rawRatio = max(nearRawMedian, farRawMedian) / max(min(nearRawMedian, farRawMedian), 0.0001)
        guard rawRatio > 2.0 else { return .notBimodal }

        let total = Float(validValues.count)
        let nearFraction = Float(nearDepths.count) / total
        let farFraction = Float(farDepths.count) / total

        guard nearFraction > 0.10 && farFraction > 0.10 else { return .notBimodal }

        let nearPeakM: Float
        let farPeakM: Float

        if calibrator.confidence > 0.01 {
            let nearMedianM = calibrator.calibrate(nearRawMedian)
            let farMedianM = calibrator.calibrate(farRawMedian)
            nearPeakM = min(nearMedianM, farMedianM)
            farPeakM = max(nearMedianM, farMedianM)
        } else {
            nearPeakM = min(nearRawMedian, farRawMedian)
            farPeakM = max(nearRawMedian, farRawMedian)
        }

        guard farPeakM > nearPeakM * 2.0, nearPeakM > 0.01 else { return .notBimodal }

        let demAgreesWithFar: Bool
        if let demEstimate = latestDEMEstimate {
            let demD = demEstimate.distanceMeters
            let farError = abs(demD - farPeakM) / max(demD, 1)
            demAgreesWithFar = farError < 0.30
        } else {
            demAgreesWithFar = false
        }

        if frameCount % 60 == 1 {
            Logger.depth.info("Bimodal: near=\(String(format: "%.0f", nearPeakM))m (\(String(format: "%.0f%%", nearFraction * 100))) far=\(String(format: "%.0f", farPeakM))m (\(String(format: "%.0f%%", farFraction * 100))) demAgrees=\(demAgreesWithFar)")
        }

        return BimodalAnalysis(
            isBimodal: true,
            nearPeakM: nearPeakM,
            farPeakM: farPeakM,
            nearFraction: nearFraction,
            farFraction: farFraction,
            demAgreesWithFar: demAgreesWithFar
        )
    }

    // MARK: - Calibration Feeding

    private var calibrationLogCount: Int = 0

    private func feedCalibration(
        neuralMap: CVPixelBuffer,
        lidarMap: CVPixelBuffer,
        confidenceMap: CVPixelBuffer?,
        timestamp: TimeInterval
    ) {
        // Sample at multiple points for robust calibration
        let samplePoints: [CGPoint] = [
            CGPoint(x: 0.5, y: 0.5),   // Center
            CGPoint(x: 0.3, y: 0.3),
            CGPoint(x: 0.7, y: 0.3),
            CGPoint(x: 0.3, y: 0.7),
            CGPoint(x: 0.7, y: 0.7),
        ]

        var fedCount = 0
        calibrationLogCount += 1

        // Use metricEstimator.sampleDepth for neural map (same method used in selection)
        for point in samplePoints {
            let lidarDepth = LiDARDepthProvider.sampleDepth(from: lidarMap, at: point)
            let neuralDepth = metricEstimator.sampleDepth(from: neuralMap, at: point)

            // Log the raw values periodically to diagnose issues
            if calibrationLogCount % 60 == 1 && point.x == 0.5 {
                Logger.calibration.info("Feed raw: neural=\(String(describing: neuralDepth)) lidar=\(String(describing: lidarDepth)) calModel=\(String(describing: self.calibrator.modelType)) calConf=\(String(format: "%.2f", self.calibrator.confidence))")
            }

            guard let lidarD = lidarDepth,
                  let neuralD = neuralDepth,
                  !neuralD.isNaN, abs(neuralD) > 0.001,
                  lidarD > 0.2, lidarD < 8.0 else { continue }

            let confidence: Float
            if let confMap = confidenceMap {
                confidence = LiDARDepthProvider.sampleConfidence(from: confMap, at: point)
            } else {
                confidence = 0.7
            }

            calibrator.ingestSample(
                neuralDepth: neuralD,
                lidarDepth: lidarD,
                confidence: confidence,
                timestamp: timestamp
            )
            fedCount += 1
        }

        if calibrationLogCount % 60 == 1 {
            Logger.calibration.info("Calibration fed \(fedCount)/5 samples, total conf=\(String(format: "%.2f", self.calibrator.confidence))")
        }
    }

    // MARK: - Utility

    /// Sample a pixel buffer using median-filtered patch sampling for noise robustness.
    /// Uses a 5x5 patch (radius 2) and returns the median valid value.
    private func samplePixelBuffer(_ buffer: CVPixelBuffer, at point: CGPoint, patchRadius: Int = 2) -> Float? {
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(buffer) else { return nil }

        let width = CVPixelBufferGetWidth(buffer)
        let height = CVPixelBufferGetHeight(buffer)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
        let formatType = CVPixelBufferGetPixelFormatType(buffer)

        let cx = min(width - 1, max(0, Int(point.x * CGFloat(width))))
        let cy = min(height - 1, max(0, Int(point.y * CGFloat(height))))

        var values: [Float] = []
        values.reserveCapacity((2 * patchRadius + 1) * (2 * patchRadius + 1))

        for dy in -patchRadius...patchRadius {
            for dx in -patchRadius...patchRadius {
                let px = cx + dx
                let py = cy + dy
                guard px >= 0, px < width, py >= 0, py < height else { continue }

                let rowPtr = baseAddress + py * bytesPerRow
                let val: Float

                switch formatType {
                case kCVPixelFormatType_DepthFloat16,
                     kCVPixelFormatType_OneComponent16Half:
                    let ptr = rowPtr.assumingMemoryBound(to: UInt16.self)
                    val = float16ToFloat(ptr[px])
                default:
                    let ptr = rowPtr.assumingMemoryBound(to: Float.self)
                    val = ptr[px]
                }

                if !val.isNaN && !val.isInfinite && val > 0 {
                    values.append(val)
                }
            }
        }

        guard !values.isEmpty else { return nil }

        // Median for noise robustness
        values.sort()
        return values[values.count / 2]
    }

    /// Convert Float16 (as UInt16) to Float32.
    private func float16ToFloat(_ value: UInt16) -> Float {
        let sign = (value & 0x8000) >> 15
        let exponent = (value & 0x7C00) >> 10
        let mantissa = value & 0x03FF
        if exponent == 0 {
            if mantissa == 0 { return sign == 0 ? 0.0 : -0.0 }
            var m = Float(mantissa) / 1024.0
            m *= pow(2.0, -14.0)
            return sign == 0 ? m : -m
        }
        if exponent == 0x1F {
            return mantissa == 0 ? (sign == 0 ? .infinity : -.infinity) : .nan
        }
        let f32Exponent = Int(exponent) - 15 + 127
        let f32Bits = UInt32(sign) << 31 | UInt32(f32Exponent) << 23 | UInt32(mantissa) << 13
        return Float(bitPattern: f32Bits)
    }

    /// Weighted median: sort by depth, find the depth where cumulative weight >= 50%.
    /// More robust than weighted mean against outliers.
    static func weightedMedian(estimates: [(Float, Float)]) -> Float {
        let sorted = estimates.sorted { $0.0 < $1.0 }
        let totalW = sorted.map { $0.1 }.reduce(0, +)
        guard totalW > 0 else { return sorted.first?.0 ?? 0 }

        var cumulative: Float = 0
        for (depth, weight) in sorted {
            cumulative += weight
            if cumulative >= totalW * 0.5 {
                return depth
            }
        }
        return sorted.last?.0 ?? 0
    }
}

// MARK: - Object Range Result (for object detection integration)

/// Result from object detection + pinhole ranging.
struct ObjectRangeResult {
    let label: String
    let distanceMeters: Double
    let confidence: Float
    let screenCenter: CGPoint   // Normalized 0-1
    let boundingBox: CGRect     // Normalized 0-1
}
