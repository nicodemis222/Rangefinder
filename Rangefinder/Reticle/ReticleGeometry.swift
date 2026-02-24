//
//  ReticleGeometry.swift
//  Rangefinder
//
//  FFP reticle geometry calculations.
//
//  In an FFP (First Focal Plane) scope, the reticle is in the same
//  focal plane as the image, so it magnifies with the target. Digitally,
//  this means reticle elements defined in mil-space scale linearly with
//  the camera's zoom factor.
//
//  Key relationship:
//    pixelsPerMil = screenWidth / currentFOVMils
//    currentFOVMils = baseFOVMils / zoomFactor
//  Therefore:
//    pixelsPerMil = screenWidth * zoomFactor / baseFOVMils
//  The reticle grows on-screen as you zoom in.
//

import Foundation
import CoreGraphics

struct ReticleGeometry {
    /// Base FOV at 1x zoom in milliradians
    /// iPhone 17 Pro Max main camera: ~78 deg HFOV = ~1361 mils
    static let baseFOVMils: Double = AppConfiguration.mainCameraHFOV * 17.453292519943

    /// Compute pixels per milliradian for current zoom and screen width
    static func pixelsPerMil(screenWidth: CGFloat, zoomFactor: CGFloat) -> CGFloat {
        let currentFOVMils = baseFOVMils / Double(zoomFactor)
        guard currentFOVMils > 0 else { return 1.0 }
        return screenWidth / CGFloat(currentFOVMils)
    }

    /// Mil-dot positions along each axis (in mils from center)
    /// Standard: dots at 1, 2, 3, 4, 5 mils in each direction
    static let dotPositionsMils: [CGFloat] = [1, 2, 3, 4, 5]

    /// Half-mil hash mark positions
    static let halfMilPositionsMils: [CGFloat] = [0.5, 1.5, 2.5, 3.5, 4.5]

    /// Dot diameter in mils (standard is 0.25 mil)
    static let dotDiameterMils: CGFloat = AppConfiguration.dotDiameterMils

    /// Center crosshair gap in mils
    static let centerGapMils: CGFloat = AppConfiguration.centerGapMils

    /// How far the fine crosshair extends (mils from center)
    static let fineExtentMils: CGFloat = 6.0

    /// How far the thick outer crosshair extends (mils from fine end)
    static let outerExtentMils: CGFloat = 15.0

    /// Hash mark length perpendicular to axis (mils)
    static let hashLengthMils: CGFloat = 0.15

    /// Convert a mil value to pixel offset from center
    static func milToPixels(_ mils: CGFloat, screenWidth: CGFloat, zoomFactor: CGFloat) -> CGFloat {
        return mils * pixelsPerMil(screenWidth: screenWidth, zoomFactor: zoomFactor)
    }
}
