//
//  InclinationCorrector.swift
//  Rangefinder
//
//  Applies cosine correction for inclined shots.
//  adjustedRange = lineOfSightRange × cos(angle)
//

import Foundation

struct InclinationCorrector {

    /// Applies inclination correction to a line-of-sight range.
    ///
    /// When shooting at an angle (uphill or downhill), the horizontal
    /// distance to the target is shorter than the line-of-sight distance.
    /// The correction is the cosine of the inclination angle.
    ///
    /// - Parameters:
    ///   - lineOfSightRange: Measured distance in meters.
    ///   - pitchRadians: Device pitch angle in radians.
    /// - Returns: Corrected horizontal range and correction factor.
    static func correct(
        lineOfSightRange: Double,
        pitchRadians: Double
    ) -> (adjustedRange: Double, correctionFactor: Double) {
        // Use absolute value of pitch (correction is same for up/down)
        let absPitch = abs(pitchRadians)

        // At very small angles (<2°), skip correction
        if absPitch < 0.035 { // ~2 degrees
            return (lineOfSightRange, 1.0)
        }

        let correctionFactor = cos(absPitch)
        let adjustedRange = lineOfSightRange * correctionFactor

        return (adjustedRange, correctionFactor)
    }

    /// Format the inclination angle for display.
    static func formatAngle(_ pitchDegrees: Double) -> String {
        let sign = pitchDegrees >= 0 ? "+" : ""
        return "\(sign)\(String(format: "%.1f", pitchDegrees))°"
    }

    /// Format the correction factor for display.
    static func formatCorrectionFactor(_ factor: Double) -> String {
        String(format: "×%.3f", factor)
    }
}
