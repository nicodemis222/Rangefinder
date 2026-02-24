//
//  CameraIntrinsics.swift
//  Rangefinder
//
//  Camera intrinsics extraction and management.
//

import Foundation
import simd
import CoreVideo

struct CameraIntrinsics: Sendable {
    /// Focal length in pixels (fx, fy)
    let focalLength: SIMD2<Float>

    /// Principal point in pixels (cx, cy)
    let principalPoint: SIMD2<Float>

    /// Image dimensions
    let imageWidth: Int
    let imageHeight: Int

    /// Horizontal field of view in degrees
    var horizontalFOVDegrees: Double {
        2.0 * atan(Double(imageWidth) / (2.0 * Double(focalLength.x))) * 180.0 / .pi
    }

    /// Vertical field of view in degrees
    var verticalFOVDegrees: Double {
        2.0 * atan(Double(imageHeight) / (2.0 * Double(focalLength.y))) * 180.0 / .pi
    }

    /// Horizontal field of view in milliradians
    var horizontalFOVMils: Double {
        horizontalFOVDegrees * 17.453292519943
    }

    /// Creates from ARKit 3x3 intrinsics matrix
    init(from matrix: simd_float3x3, width: Int, height: Int) {
        self.focalLength = SIMD2(matrix[0][0], matrix[1][1])
        self.principalPoint = SIMD2(matrix[2][0], matrix[2][1])
        self.imageWidth = width
        self.imageHeight = height
    }

    /// Manual construction
    init(focalLength: SIMD2<Float>, principalPoint: SIMD2<Float>, width: Int, height: Int) {
        self.focalLength = focalLength
        self.principalPoint = principalPoint
        self.imageWidth = width
        self.imageHeight = height
    }

    /// Scales intrinsics for a different resolution (e.g., depth map vs camera image)
    func scaled(toWidth newWidth: Int, toHeight newHeight: Int) -> CameraIntrinsics {
        let scaleX = Float(newWidth) / Float(imageWidth)
        let scaleY = Float(newHeight) / Float(imageHeight)
        return CameraIntrinsics(
            focalLength: SIMD2(focalLength.x * scaleX, focalLength.y * scaleY),
            principalPoint: SIMD2(principalPoint.x * scaleX, principalPoint.y * scaleY),
            width: newWidth,
            height: newHeight
        )
    }

    /// Returns pixels per mil at given image width
    func pixelsPerMil(atImageWidth screenWidth: CGFloat) -> CGFloat {
        let fovMils = horizontalFOVMils
        guard fovMils > 0 else { return 1.0 }
        return screenWidth / CGFloat(fovMils)
    }
}
