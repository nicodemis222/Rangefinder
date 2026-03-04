//
//  SceneRangeAnalyzer.swift
//  Rangefinder
//
//  Spatial coherence validation for scene-aware multi-point ranging.
//  Compares anchor readings against the center reading to detect
//  inconsistencies that indicate unreliable depth estimates.
//

import Foundation

@MainActor
class SceneRangeAnalyzer {

    // MARK: - Coherence Validation

    /// Validate spatial coherence: do the anchor readings form a consistent
    /// spatial picture with the center reading?
    ///
    /// If center reads 47m (neural) but three anchors from DEM/objects
    /// read 200+m, the center is likely wrong. If anchors agree with center
    /// (+/- ratio threshold), the scene is spatially coherent.
    ///
    /// Special case: when ALL samples are neural-only (no DEM, no object,
    /// no LiDAR) and all are in the extrapolation zone (>15m), the
    /// "coherence" is meaningless — they all agree because they're all
    /// equally wrong. Return .insufficient to signal this.
    func validateCoherence(
        center: SceneRangeSample,
        anchors: [SceneRangeSample]
    ) -> SpatialCoherence {

        let validAnchors = anchors.filter { $0.estimate.isValid }
        guard validAnchors.count >= 2 else { return .insufficient }

        let centerM = center.estimate.distanceMeters
        guard centerM > 0 else { return .unknown }

        // Detect neural-only extrapolation: when every sample comes from
        // neural or geometric (both unreliable in extrapolation) and all are
        // beyond 15m, the readings are likely artifacts. No point calling
        // them "coherent" — they're all extrapolation noise agreeing with
        // each other.
        let allSamples = [center] + validAnchors
        let extrapolationLimit: Double = 15.0
        let allNeuralExtrapolation = allSamples.allSatisfy { sample in
            let src = sample.estimate.source
            let dist = sample.estimate.distanceMeters
            return (src == .neural || src == .geometric) && dist > extrapolationLimit
        }
        if allNeuralExtrapolation {
            return .insufficient
        }

        let threshold = Double(AppConfiguration.coherenceRatioThreshold)
        var agreeCount = 0
        var disagreeCount = 0

        for anchor in validAnchors {
            let anchorM = anchor.estimate.distanceMeters
            guard anchorM > 0 else { continue }

            let ratio = max(centerM, anchorM) / min(centerM, anchorM)

            if ratio < threshold {
                agreeCount += 1
            } else {
                disagreeCount += 1
            }
        }

        if disagreeCount == 0 && agreeCount >= 2 {
            return .allCoherent
        } else if disagreeCount > agreeCount {
            return .centerSuspect
        } else if disagreeCount > 0 {
            return .mixed
        } else {
            return .insufficient
        }
    }

    /// Update individual sample coherence status relative to the center reading.
    func assignSampleCoherence(
        samples: inout [SceneRangeSample],
        centerDistance: Double
    ) {
        guard centerDistance > 0 else {
            for i in samples.indices where !samples[i].isCenter {
                samples[i].coherenceStatus = .noData
            }
            return
        }

        let threshold = Double(AppConfiguration.coherenceRatioThreshold)

        for i in samples.indices where !samples[i].isCenter {
            let anchorM = samples[i].estimate.distanceMeters
            guard anchorM > 0, samples[i].estimate.isValid else {
                samples[i].coherenceStatus = .noData
                continue
            }

            let ratio = max(centerDistance, anchorM) / min(centerDistance, anchorM)
            samples[i].coherenceStatus = ratio < threshold ? .coherent : .inconsistent
        }
    }
}
