//
//  DepthTypes.swift
//  Rangefinder
//
//  Data types for depth estimation pipeline.
//

import Foundation
import CoreVideo

/// Result from a depth model inference.
struct MetricDepthResult {
    let depthMap: CVPixelBuffer
    let confidenceMap: CVPixelBuffer?
    let minDepth: Float
    let maxDepth: Float
    let intrinsics: CameraIntrinsics
    let timestamp: Date
    let inferenceTime: TimeInterval

    var depthRange: Float { maxDepth - minDepth }
}

/// A calibration sample pairing neural and LiDAR depth.
struct CalibrationSample {
    let neuralDepth: Float
    let lidarDepth: Float
    let confidence: Float
    let timestamp: TimeInterval
}

/// Affine calibration parameters: metric = scale * neural + shift.
struct AffineCalibration {
    var scale: Float = 1.0
    var shift: Float = 0.0
    var confidence: Float = 0.0
    var sampleCount: Int = 0
    var lastUpdate: TimeInterval = 0

    static let identity = AffineCalibration()

    func apply(to neuralDepth: Float) -> Float {
        scale * neuralDepth + shift
    }
}
