//
//  Errors.swift
//  Rangefinder
//
//  Typed error enum for the app.
//

import Foundation

enum MilDotError: LocalizedError {
    case cameraUnavailable
    case arSessionFailed(String)
    case lidarUnavailable
    case depthModelLoadFailed(String)
    case depthModelFailed(String)
    case depthEstimationFailed
    case calibrationFailed(String)
    case invalidDepthMap
    case objectDetectionFailed(String)

    var errorDescription: String? {
        switch self {
        case .cameraUnavailable:
            return "Camera is not available"
        case .arSessionFailed(let reason):
            return "AR session failed: \(reason)"
        case .lidarUnavailable:
            return "LiDAR scanner not available on this device"
        case .depthModelLoadFailed(let model):
            return "Failed to load depth model: \(model)"
        case .depthModelFailed(let reason):
            return "Depth model error: \(reason)"
        case .depthEstimationFailed:
            return "Depth estimation failed"
        case .calibrationFailed(let reason):
            return "Calibration failed: \(reason)"
        case .invalidDepthMap:
            return "Invalid depth map data"
        case .objectDetectionFailed(let reason):
            return "Object detection failed: \(reason)"
        }
    }
}
