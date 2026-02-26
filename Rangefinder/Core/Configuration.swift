//
//  Configuration.swift
//  Rangefinder
//
//  Centralized app configuration.
//

import Foundation

struct AppConfiguration {
    // MARK: - Depth Pipeline

    /// Neural depth inference target rate (Hz)
    static let neuralDepthFPS: Double = 15.0
    static let neuralDepthInterval: CFAbsoluteTime = 1.0 / neuralDepthFPS

    /// Object detection target rate (Hz)
    static let objectDetectionFPS: Double = 5.0
    static let objectDetectionInterval: CFAbsoluteTime = 1.0 / objectDetectionFPS

    // MARK: - LiDAR

    /// Maximum reliable LiDAR range (meters)
    static let lidarMaxRange: Float = 5.0

    /// Minimum LiDAR range (meters)
    static let lidarMinRange: Float = 0.3

    // MARK: - Calibration

    /// Rolling window size for continuous calibration
    static let calibrationWindowSize: Int = 50

    /// Minimum samples before calibration is usable
    static let calibrationMinSamples: Int = 5

    /// Confidence decay rate when no LiDAR data (per second)
    static let calibrationDecayRate: Float = 0.02

    // MARK: - Ranging

    /// Maximum display range (meters) ~2000m via geometric + object detection
    static let maxDisplayRange: Double = 2000.0

    // MARK: - Geometric Ranging

    /// Default camera height above ground (meters). 1.5m = handheld at eye level.
    static let defaultCameraHeight: Float = 1.5

    /// Minimum allowed camera height (meters)
    static let minCameraHeight: Float = 0.3

    /// Maximum allowed camera height (meters)
    static let maxCameraHeight: Float = 5.0

    // MARK: - DEM Terrain Ranging

    /// DEM ray-cast query interval (seconds). 2 Hz max.
    static let demQueryInterval: CFAbsoluteTime = 0.5

    /// Maximum DEM ray-cast range (meters)
    static let demMaxRange: Float = 2000.0

    /// Minimum DEM range (meters) — GPS noise dominates below this
    static let demMinRange: Float = 20.0

    /// Minimum confidence to show range
    static let minDisplayConfidence: Float = 0.15

    // MARK: - Smoothing

    /// Depth discontinuity threshold (relative change)
    static let depthDiscontinuityThreshold: Double = 0.30

    /// Frames to confirm depth discontinuity before snapping
    static let discontinuityConfirmFrames: Int = 3

    // MARK: - Camera

    /// iPhone 17 Pro Max main camera HFOV (degrees) at 1x
    static let mainCameraHFOV: Double = 78.0

    /// Zoom breakpoints for lens switching
    static let ultrawideFactor: CGFloat = 0.5
    static let mainFactor: CGFloat = 1.0
    static let telephoto5xFactor: CGFloat = 5.0
    static let maxOpticalZoom: CGFloat = 8.0
    static let maxDigitalZoom: CGFloat = 25.0

    // MARK: - Reticle

    /// Number of mil dots per axis
    static let milDotsPerAxis: Int = 5

    /// Dot diameter in mils
    static let dotDiameterMils: CGFloat = 0.25

    /// Center crosshair gap in mils
    static let centerGapMils: CGFloat = 0.5

    /// Crosshair line width in points (base, before scaling)
    static let crosshairLineWidth: CGFloat = 1.0

    /// Thick outer line width in points
    static let outerLineWidth: CGFloat = 2.5

    // MARK: - Semantic Selection

    /// Neural depth hard cap: readings beyond this distance are discarded.
    /// Neural (DAv2 calibrated via LiDAR at 0.2-8m) extrapolates reasonably
    /// to ~150m with progressively declining confidence. Beyond 150m the
    /// inverse-depth amplification makes it unreliable — DEM and object
    /// detection take over.
    static let neuralHardCapMeters: Float = 150.0

    // MARK: - Stadiametric Target Presets

    /// Known target sizes for stadiametric (pinhole) ranging.
    /// Each entry: (label, height in meters).
    static let stadiametricTargetPresets: [(label: String, heightMeters: Double)] = [
        ("PERSON", 1.8),
        ("VEHICLE", 1.5),
        ("DEER", 1.0),
        ("DOOR", 2.0),
        ("FENCE POST", 1.2),
        ("POWER POLE", 10.0),
        ("WINDOW", 1.0),
        ("GOLF PIN", 2.13),
    ]
}
