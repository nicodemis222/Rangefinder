//
//  ContinuousCalibrator.swift
//  Rangefinder
//
//  Always-on LiDAR → neural depth calibration.
//
//  Handles BOTH metric and inverse/relative depth models:
//  - Metric models: lidar ≈ scale * neural + shift  (positive correlation)
//  - Inverse depth models (e.g. DepthAnythingV2): lidar ≈ scale / neural + shift
//
//  Auto-detects which model type is active by checking correlation sign
//  on the first few samples. Then continuously fits the appropriate
//  transform using a rolling window with exponential recency weighting.
//

import Foundation
import os

class ContinuousCalibrator: @unchecked Sendable {

    // MARK: - Configuration

    private let maxSamples: Int
    private let decayFactor: Float  // Per-sample exponential weight decay

    // MARK: - State

    enum DepthModelType {
        case unknown
        case metric       // lidar = scale * neural + shift
        case inverse      // lidar = scale / neural + shift
    }

    private var samples: [CalibrationSample] = []
    private(set) var calibration = AffineCalibration.identity
    private(set) var modelType: DepthModelType = .unknown
    private let lock = NSLock()

    // MARK: - Init

    init(
        maxSamples: Int = 50,
        decayFactor: Float = 0.95,
        forceModelType: DepthModelType = .unknown
    ) {
        self.maxSamples = maxSamples
        self.decayFactor = decayFactor
        // DepthAnythingV2 is an inverse depth model. The auto-detection
        // can misclassify it as metric when calibration samples are all
        // within the narrow LiDAR range (0.2–8m), because within that
        // band the correlation can appear positive. Force inverse to
        // prevent catastrophic long-range underestimation.
        if forceModelType != .unknown {
            self.modelType = forceModelType
            Logger.calibration.info("Forced model type: \(String(describing: forceModelType))")
        }
    }

    // MARK: - Ingest Sample

    func ingestSample(neuralDepth: Float, lidarDepth: Float, confidence: Float, timestamp: TimeInterval) {
        guard neuralDepth > 0.001, lidarDepth > 0.1,
              !neuralDepth.isNaN, !lidarDepth.isNaN,
              confidence > 0.3 else { return }

        lock.lock()
        defer { lock.unlock() }

        let sample = CalibrationSample(
            neuralDepth: neuralDepth,
            lidarDepth: lidarDepth,
            confidence: confidence,
            timestamp: timestamp
        )

        samples.append(sample)

        if samples.count > maxSamples {
            samples.removeFirst(samples.count - maxSamples)
        }

        // Auto-detect model type once we have enough samples
        // (skipped when model type was forced via init)
        if modelType == .unknown && samples.count >= 5 {
            detectModelType()
        }

        refit(currentTimestamp: timestamp)
    }

    // MARK: - Apply Calibration

    func calibrate(_ neuralDepth: Float) -> Float {
        lock.lock()
        let cal = calibration
        let mType = modelType
        lock.unlock()

        guard neuralDepth > 0.001 else { return 0 }

        switch mType {
        case .inverse:
            // lidar = scale / neural + shift
            let raw = cal.scale / neuralDepth + cal.shift
            // Guard against extreme extrapolation: the inverse transform
            // amplifies noise at very small neural values (far objects).
            // Calibration trained on 0.2-8m LiDAR; reliable to ~80m,
            // progressively unreliable beyond.
            //
            // Raised cap from 200m → 500m so that neural depth can extend
            // further when the calibration fit supports it (e.g. when the
            // inverse-depth curve has a gentle slope at the far end).
            // Beyond 150m we apply soft compression to avoid wild jumps:
            // the extrapolation penalty in UnifiedDepthField handles the
            // confidence reduction, but we still need a plausible distance.
            if raw <= 150.0 {
                return max(0.1, raw)
            } else {
                // Soft compression: asymptotically approach 500m
                // At 150m: returns 150m
                // At 300m: returns ~280m
                // At 1000m: returns ~460m
                let excess = raw - 150.0
                let compressed = 150.0 + 350.0 * (1.0 - exp(-excess / 350.0))
                return min(compressed, 500.0)
            }
        case .metric, .unknown:
            // lidar = scale * neural + shift
            let raw = cal.apply(to: neuralDepth)
            if raw <= 150.0 {
                return max(0.1, raw)
            } else {
                let excess = raw - 150.0
                let compressed = 150.0 + 350.0 * (1.0 - exp(-excess / 350.0))
                return min(compressed, 500.0)
            }
        }
    }

    func calibrationAge(currentTimestamp: TimeInterval) -> TimeInterval {
        lock.lock()
        let lastUpdate = calibration.lastUpdate
        lock.unlock()
        return currentTimestamp - lastUpdate
    }

    var confidence: Float {
        lock.lock()
        let c = calibration.confidence
        lock.unlock()
        return c
    }

    // MARK: - Reset

    func reset() {
        lock.lock()
        samples.removeAll()
        calibration = .identity
        modelType = .unknown
        lock.unlock()
        Logger.calibration.info("Calibration reset")
    }

    // MARK: - Model Type Detection

    /// Detect whether the neural model outputs metric depth or inverse depth.
    /// Inverse depth: close objects have HIGH neural values, far objects have LOW values.
    /// Metric depth: close objects have LOW values, far objects have HIGH values.
    private func detectModelType() {
        guard samples.count >= 5 else { return }

        // Compute Pearson correlation between neural and lidar
        let n = Float(samples.count)
        var sumN: Float = 0, sumL: Float = 0
        var sumNL: Float = 0, sumNN: Float = 0, sumLL: Float = 0

        for s in samples {
            sumN += s.neuralDepth
            sumL += s.lidarDepth
            sumNL += s.neuralDepth * s.lidarDepth
            sumNN += s.neuralDepth * s.neuralDepth
            sumLL += s.lidarDepth * s.lidarDepth
        }

        let numerator = n * sumNL - sumN * sumL
        let denomN = n * sumNN - sumN * sumN
        let denomL = n * sumLL - sumL * sumL
        let denom = sqrt(denomN * denomL)

        guard denom > 0.001 else {
            modelType = .metric // Default
            return
        }

        let correlation = numerator / denom

        if correlation < -0.3 {
            modelType = .inverse
            Logger.calibration.info("Detected INVERSE depth model (r=\(String(format: "%.2f", correlation)))")
        } else {
            modelType = .metric
            Logger.calibration.info("Detected METRIC depth model (r=\(String(format: "%.2f", correlation)))")
        }
    }

    // MARK: - Weighted Least-Squares Fit

    private func refit(currentTimestamp: TimeInterval) {
        switch modelType {
        case .inverse:
            refitInverse(currentTimestamp: currentTimestamp)
        case .metric, .unknown:
            refitMetric(currentTimestamp: currentTimestamp)
        }
    }

    /// Fit: lidar = scale * neural + shift
    private func refitMetric(currentTimestamp: TimeInterval) {
        guard samples.count >= 3 else {
            if let s = samples.last, s.neuralDepth > 0.01 {
                calibration.scale = s.lidarDepth / s.neuralDepth
                calibration.shift = 0
                calibration.confidence = 0.3 * Float(samples.count)
                calibration.sampleCount = samples.count
                calibration.lastUpdate = currentTimestamp
            }
            return
        }

        var weights = computeWeights(currentTimestamp: currentTimestamp)

        var sumW: Float = 0, sumWx: Float = 0, sumWy: Float = 0
        var sumWxx: Float = 0, sumWxy: Float = 0

        for i in 0..<samples.count {
            let w = weights[i]
            let x = samples[i].neuralDepth
            let y = samples[i].lidarDepth
            sumW += w; sumWx += w * x; sumWy += w * y
            sumWxx += w * x * x; sumWxy += w * x * y
        }

        let denom = sumW * sumWxx - sumWx * sumWx
        guard abs(denom) > 1e-6 else { return }

        let scale = (sumW * sumWxy - sumWx * sumWy) / denom
        let shift = (sumWy - scale * sumWx) / sumW

        guard scale > 0.01 && scale < 1000.0, abs(shift) < 100.0 else {
            Logger.calibration.warning("Metric cal out of range: scale=\(scale), shift=\(shift)")
            return
        }

        let rSquared = computeRSquared(predictedValues: samples.map { scale * $0.neuralDepth + shift }, weights: weights)
        let sampleConfidence = min(1.0, Float(samples.count) / Float(maxSamples))

        calibration = AffineCalibration(
            scale: scale, shift: shift,
            confidence: rSquared * sampleConfidence,
            sampleCount: samples.count,
            lastUpdate: currentTimestamp
        )

        logCalibration(scale: scale, shift: shift, rSquared: rSquared, type: "metric")
    }

    /// Fit: lidar = scale / neural + shift
    private func refitInverse(currentTimestamp: TimeInterval) {
        guard samples.count >= 3 else {
            if let s = samples.last, s.neuralDepth > 0.001 {
                calibration.scale = s.lidarDepth * s.neuralDepth
                calibration.shift = 0
                calibration.confidence = 0.3 * Float(samples.count)
                calibration.sampleCount = samples.count
                calibration.lastUpdate = currentTimestamp
            }
            return
        }

        let weights = computeWeights(currentTimestamp: currentTimestamp)

        // Transform: let x' = 1/neural, then lidar = scale * x' + shift
        var sumW: Float = 0, sumWx: Float = 0, sumWy: Float = 0
        var sumWxx: Float = 0, sumWxy: Float = 0

        for i in 0..<samples.count {
            let w = weights[i]
            let x = 1.0 / max(samples[i].neuralDepth, 0.001) // 1/neural
            let y = samples[i].lidarDepth
            sumW += w; sumWx += w * x; sumWy += w * y
            sumWxx += w * x * x; sumWxy += w * x * y
        }

        let denom = sumW * sumWxx - sumWx * sumWx
        guard abs(denom) > 1e-6 else { return }

        let scale = (sumW * sumWxy - sumWx * sumWy) / denom
        let shift = (sumWy - scale * sumWx) / sumW

        guard scale > 0.001 && scale < 100000.0, abs(shift) < 100.0 else {
            Logger.calibration.warning("Inverse cal out of range: scale=\(scale), shift=\(shift)")
            return
        }

        let rSquared = computeRSquared(
            predictedValues: samples.map { scale / max($0.neuralDepth, 0.001) + shift },
            weights: weights
        )
        let sampleConfidence = min(1.0, Float(samples.count) / Float(maxSamples))

        calibration = AffineCalibration(
            scale: scale, shift: shift,
            confidence: rSquared * sampleConfidence,
            sampleCount: samples.count,
            lastUpdate: currentTimestamp
        )

        logCalibration(scale: scale, shift: shift, rSquared: rSquared, type: "inverse")
    }

    // MARK: - Utility

    private func computeWeights(currentTimestamp: TimeInterval) -> [Float] {
        samples.map { sample in
            let age = currentTimestamp - sample.timestamp
            return pow(decayFactor, Float(age)) * sample.confidence
        }
    }

    private func computeRSquared(predictedValues: [Float], weights: [Float]) -> Float {
        let sumW = weights.reduce(0, +)
        guard sumW > 0 else { return 0 }
        let meanY = zip(samples, weights).reduce(Float(0)) { $0 + $1.0.lidarDepth * $1.1 } / sumW

        var ssRes: Float = 0, ssTot: Float = 0
        for i in 0..<samples.count {
            let residual = samples[i].lidarDepth - predictedValues[i]
            ssRes += weights[i] * residual * residual
            let diff = samples[i].lidarDepth - meanY
            ssTot += weights[i] * diff * diff
        }
        return ssTot > 0 ? max(0, 1.0 - ssRes / ssTot) : 0
    }

    private func logCalibration(scale: Float, shift: Float, rSquared: Float, type: String) {
        let n = samples.count
        if n <= 10 || n % 10 == 0 {
            Logger.calibration.info("Cal(\(type)): scale=\(String(format: "%.3f", scale)) shift=\(String(format: "%.3f", shift)) R²=\(String(format: "%.3f", rSquared)) n=\(n)")
        }
    }
}
