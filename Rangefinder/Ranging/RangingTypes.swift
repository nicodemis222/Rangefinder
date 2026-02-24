//
//  RangingTypes.swift
//  Rangefinder
//
//  Core types for the ranging pipeline.
//

import Foundation

// MARK: - Depth Source

/// Identifies which depth estimation method produced a reading.
enum DepthSource: String, CaseIterable {
    case lidar = "LiDAR"
    case neural = "Neural"
    case geometric = "Geometric"
    case demRaycast = "DEM"
    case objectSize = "Object"
    case semantic = "Semantic"
    case stadiametric = "Stadia"

    var icon: String {
        switch self {
        case .lidar: return "sensor.fill"
        case .neural: return "brain.head.profile"
        case .geometric: return "location.north.line"
        case .demRaycast: return "mountain.2.fill"
        case .objectSize: return "viewfinder"
        case .semantic: return "target"
        case .stadiametric: return "ruler"
        }
    }

    var shortName: String {
        switch self {
        case .lidar: return "LIDAR"
        case .neural: return "AI"
        case .geometric: return "GEO"
        case .demRaycast: return "DEM"
        case .objectSize: return "OBJ"
        case .semantic: return "SEM"
        case .stadiametric: return "STADIA"
        }
    }
}

// MARK: - Depth Estimate

/// A single depth estimate from any source.
struct DepthEstimate {
    let distanceMeters: Double
    let confidence: Float
    let uncertainty: Double  // ± meters
    let source: DepthSource
    let sourceWeights: [DepthSource: Float]  // How much each source contributed
    let timestamp: Date

    var isValid: Bool {
        confidence > AppConfiguration.minDisplayConfidence && distanceMeters > 0
    }

    var distanceMeasurement: Measurement<UnitLength> {
        Measurement(value: distanceMeters, unit: .meters)
    }

    var uncertaintyMeasurement: Measurement<UnitLength> {
        Measurement(value: uncertainty, unit: .meters)
    }

    /// Uncertainty as percentage of distance
    var uncertaintyPercent: Double {
        guard distanceMeters > 0 else { return 100 }
        return (uncertainty / distanceMeters) * 100
    }

    static let none = DepthEstimate(
        distanceMeters: 0,
        confidence: 0,
        uncertainty: 0,
        source: .semantic,
        sourceWeights: [:],
        timestamp: Date()
    )
}

// MARK: - Range Output

/// Final range output after inclination correction and smoothing.
struct RangeOutput {
    let lineOfSightRange: Measurement<UnitLength>
    let adjustedRange: Measurement<UnitLength>  // Inclination-corrected
    let confidence: Float
    let uncertainty: Measurement<UnitLength>
    let inclinationDegrees: Double
    let inclinationCorrectionFactor: Double  // cos(angle)
    let primarySource: DepthSource
    let sourceWeights: [DepthSource: Float]
    let timestamp: Date

    var isValid: Bool {
        confidence > AppConfiguration.minDisplayConfidence
            && adjustedRange.converted(to: .meters).value > 0
    }

    static let none = RangeOutput(
        lineOfSightRange: .init(value: 0, unit: .meters),
        adjustedRange: .init(value: 0, unit: .meters),
        confidence: 0,
        uncertainty: .init(value: 0, unit: .meters),
        inclinationDegrees: 0,
        inclinationCorrectionFactor: 1.0,
        primarySource: .semantic,
        sourceWeights: [:],
        timestamp: Date()
    )
}

// MARK: - Target Priority Mode

/// Target priority mode, inspired by laser rangefinder first/last target modes.
///
/// Laser rangefinders face the same fundamental problem: the beam can hit foreground
/// objects (brush, fences, rocks) when the user intends to range the distant target
/// behind them. Hardware LRFs solve this with "first target" and "last target" modes.
///
/// We implement the same concept for our multi-source depth fusion:
/// - **near**: Always prefer the closest valid depth reading (default, legacy behavior)
/// - **far**: When the depth field is bimodal (foreground + background), prefer the
///   far-distance cluster, cross-validated against DEM ray-cast when available
enum TargetPriority: String, CaseIterable, Identifiable {
    case near = "NEAR"
    case far = "FAR"

    var id: String { rawValue }

    var description: String {
        switch self {
        case .near: return "First target — ranges closest object"
        case .far: return "Last target — ranges through occluders to far target"
        }
    }

    var shortLabel: String {
        switch self {
        case .near: return "1ST"
        case .far: return "LST"
        }
    }
}

// MARK: - Motion State

/// Device motion state for adaptive smoothing.
enum MotionState {
    case stationary    // Device is held still
    case panning       // Device is rotating/sweeping
    case tracking      // Slow deliberate movement

    init(angularVelocity: Double) {
        if angularVelocity < 0.05 {
            self = .stationary
        } else if angularVelocity > 0.3 {
            self = .panning
        } else {
            self = .tracking
        }
    }
}

// MARK: - Semantic Source Decision

/// Records which source won the semantic selection state machine and why.
enum SemanticSourceDecision: String, CaseIterable {
    case lidarPrimary = "LIDAR_PRIMARY"
    case objectPrimary = "OBJECT_PRIMARY"
    case demPrimary = "DEM_PRIMARY"
    case neuralPrimary = "NEURAL_PRIMARY"
    case geometricPrimary = "GEO_PRIMARY"
    case stadiametric = "STADIAMETRIC"
    case none = "NONE"
}

// MARK: - Stadiametric Input

/// Input for manual stadiametric (pinhole-formula) ranging.
/// The user brackets a known-size target with the overlay, and the
/// pinhole equation computes range: R = (knownSize * focalLength) / pixelSize
struct StadiametricInput {
    let knownSizeMeters: Double
    let pixelSize: Double
    let focalLengthPixels: Double

    /// Computed range in meters via pinhole formula.
    var computedRange: Double {
        guard pixelSize > 0 else { return 0 }
        return (knownSizeMeters * focalLengthPixels) / pixelSize
    }
}
