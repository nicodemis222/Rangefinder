//
//  SceneRangeSample.swift
//  Rangefinder
//
//  Data types for scene-aware multi-point ranging.
//  Replaces single-crosshair architecture with 5 ML-selected
//  range sample points distributed across the scene.
//

import Foundation

// MARK: - Anchor Selection Reason

/// Why an anchor point was selected by the AnchorPointSelector.
enum AnchorSelectionReason: String, Sendable {
    case center = "CENTER"
    case depthEdge = "EDGE"
    case objectDetection = "OBJ"
    case highConfidence = "CONF"
    case histogramPeak = "PEAK"
}

// MARK: - Coherence Status

/// Whether a sample's range is coherent with the center reading.
enum CoherenceStatus: Equatable, Sendable {
    case coherent
    case inconsistent
    case noData
}

// MARK: - Spatial Coherence

/// Overall spatial coherence assessment across all scene samples.
enum SpatialCoherence: String, Sendable {
    case allCoherent = "COHERENT"
    case centerSuspect = "CENTER_SUSPECT"
    case mixed = "MIXED"
    case insufficient = "INSUFFICIENT"
    case unknown = "UNKNOWN"
}

// MARK: - Scene Range Sample

/// A single range sample point in the scene.
/// One of up to 5 multi-point range readings displayed on the camera overlay.
struct SceneRangeSample: Identifiable, Equatable {
    let id: UUID

    /// Normalized screen position (0-1) where this sample was taken.
    let screenPoint: CGPoint

    /// The depth estimate at this point from semantic source selection.
    let estimate: DepthEstimate

    /// Whether this is the center (aim point) sample.
    let isCenter: Bool

    /// The reason this point was selected as a feature anchor.
    let selectionReason: AnchorSelectionReason

    /// Spatial coherence status relative to center reading.
    var coherenceStatus: CoherenceStatus

    static func == (lhs: SceneRangeSample, rhs: SceneRangeSample) -> Bool {
        lhs.id == rhs.id
            && lhs.screenPoint == rhs.screenPoint
            && lhs.estimate.distanceMeters == rhs.estimate.distanceMeters
            && lhs.isCenter == rhs.isCenter
            && lhs.coherenceStatus == rhs.coherenceStatus
    }
}

// MARK: - Scene Range Result

/// Complete result from scene-aware multi-point analysis.
/// Contains up to 5 samples and the spatial coherence assessment.
struct SceneRangeResult {
    /// All scene samples (0-5). The center sample has isCenter == true.
    let samples: [SceneRangeSample]

    /// Overall spatial coherence assessment.
    let spatialCoherence: SpatialCoherence

    /// Timestamp of this result.
    let timestamp: Date

    /// The center (primary) sample, if valid.
    var centerSample: SceneRangeSample? {
        samples.first { $0.isCenter }
    }

    /// The anchor samples (non-center).
    var anchorSamples: [SceneRangeSample] {
        samples.filter { !$0.isCenter }
    }

    /// Median distance of valid anchor samples (meters).
    /// Used as an alternative estimate when center is suspect.
    var anchorMedianMeters: Double? {
        let validDistances = anchorSamples
            .filter { $0.estimate.isValid }
            .map { $0.estimate.distanceMeters }
            .sorted()
        guard !validDistances.isEmpty else { return nil }
        let mid = validDistances.count / 2
        if validDistances.count.isMultiple(of: 2) {
            return (validDistances[mid - 1] + validDistances[mid]) / 2.0
        } else {
            return validDistances[mid]
        }
    }

    static let empty = SceneRangeResult(
        samples: [],
        spatialCoherence: .unknown,
        timestamp: Date()
    )
}
