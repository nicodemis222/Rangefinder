//
//  GeometricRangeEstimator.swift
//  Rangefinder
//
//  Ground-plane geometric ranging using IMU pitch angle.
//
//  When the camera points slightly downward at a target on the ground,
//  distance = cameraHeight / tan(pitchBelowHorizontal)
//
//  This is accurate to ~100m with iPhone IMU (0.1-0.5 degree error),
//  degrades gracefully at longer range, and is only valid for targets
//  on or near the ground plane.
//
//  IMPORTANT: Assumes flat ground. On sloped terrain, steep pitch angles
//  produce wildly wrong estimates (e.g. 9.8m instead of 91m at -8.7°
//  on a downhill slope). A slope risk penalty reduces confidence when
//  pitch > 3° since this more likely indicates terrain slope than a
//  nearby flat-ground target.
//
//  Requires:
//  - Known camera height above ground (user-configurable, default 1.5m)
//  - Device pitch from InclinationManager (0=level, negative=down)
//

import Foundation
import os

struct GeometricRangeEstimator {

    // MARK: - Configuration

    /// Camera height above ground plane in meters.
    /// Default 1.5m assumes handheld at eye level (standing adult).
    /// User-configurable for tripod (1.2m), prone (0.3m), or vehicle (2.0m) use.
    var cameraHeight: Float = 1.5

    // MARK: - Constants

    /// Minimum downward pitch for valid geometric estimate (degrees).
    /// At 0.3 degrees below horizontal with 1.5m height: D = 1.5/tan(0.3°) ≈ 286m.
    /// Below this angle, IMU noise dominates the estimate.
    private let minPitchBelowHorizontalDeg: Float = 0.3

    /// Maximum valid geometric range (meters). Beyond this, IMU error dominates.
    private let maxGeometricRange: Float = 800.0

    /// Minimum valid geometric range (meters). Below ~5m, LiDAR is more reliable.
    private let minGeometricRange: Float = 5.0

    // MARK: - Estimate

    /// Compute ground-plane distance from pitch angle.
    ///
    /// - Parameters:
    ///   - pitchRadians: Device pitch in radians (0=level, negative=down, positive=up).
    ///                   This comes from InclinationManager.pitchRadians.
    /// - Returns: Estimated distance and confidence, or nil if not applicable.
    func estimate(pitchRadians: Double) -> GeometricEstimate? {
        // Only valid when looking downward (pitch < 0 from InclinationManager)
        // pitchBelowHorizontal is the angle below the horizon (positive when looking down)
        let pitchBelowHorizontal = -pitchRadians  // positive when camera tilts down

        // Reject if looking up or too close to level
        let pitchBelowHorizontalDeg = Float(pitchBelowHorizontal * 180.0 / .pi)
        guard pitchBelowHorizontalDeg > minPitchBelowHorizontalDeg else {
            return nil
        }

        // Core formula: D = h / tan(pitch)
        let tanPitch = tan(Float(pitchBelowHorizontal))
        guard tanPitch > 0.0001 else { return nil }  // Avoid division by near-zero

        let distance = cameraHeight / tanPitch

        // Clamp to valid range
        guard distance >= minGeometricRange,
              distance <= maxGeometricRange else {
            return nil
        }

        // Compute confidence based on pitch angle
        let confidence = computeConfidence(
            pitchBelowHorizontalDeg: pitchBelowHorizontalDeg
        )

        return GeometricEstimate(
            distanceMeters: distance,
            confidence: confidence,
            pitchBelowHorizontalDeg: pitchBelowHorizontalDeg
        )
    }

    // MARK: - Confidence

    /// Confidence decreases as pitch angle decreases (farther targets).
    ///
    /// IMU accuracy is ~0.1-0.5 degrees on iPhone. At various pitch angles with 1.5m height:
    /// - 10m  -> pitch=8.53°  -> 0.1° error -> ~1.2% distance error  (HIGH confidence)
    /// - 50m  -> pitch=1.72°  -> 0.1° error -> ~5.8% distance error  (HIGH confidence)
    /// - 100m -> pitch=0.86°  -> 0.1° error -> ~11.6% distance error (MODERATE confidence)
    /// - 200m -> pitch=0.43°  -> 0.1° error -> ~23.3% distance error (LOW confidence)
    /// - 500m -> pitch=0.17°  -> 0.1° error -> ~58.8% distance error (VERY LOW)
    ///
    /// The relative error in distance = delta_pitch / tan(pitch),
    /// which grows rapidly as pitch approaches zero.
    private func computeConfidence(
        pitchBelowHorizontalDeg: Float
    ) -> Float {
        // Base confidence from pitch angle (IMU noise vs signal ratio)
        let baseConfidence: Float
        if pitchBelowHorizontalDeg > 5.0 {
            baseConfidence = 0.85  // Excellent: target within ~17m at 1.5m height
        } else if pitchBelowHorizontalDeg > 2.0 {
            // 0.85 -> 0.70 linear interpolation
            let t = (pitchBelowHorizontalDeg - 2.0) / 3.0
            baseConfidence = 0.70 + t * 0.15
        } else if pitchBelowHorizontalDeg > 1.0 {
            // 0.70 -> 0.45
            let t = (pitchBelowHorizontalDeg - 1.0) / 1.0
            baseConfidence = 0.45 + t * 0.25
        } else if pitchBelowHorizontalDeg > 0.5 {
            // 0.45 -> 0.20
            let t = (pitchBelowHorizontalDeg - 0.5) / 0.5
            baseConfidence = 0.20 + t * 0.25
        } else {
            // Below 0.5 degrees: very low confidence, fading to near-zero
            baseConfidence = max(0.05, 0.20 * (pitchBelowHorizontalDeg / 0.5))
        }

        // Slope risk penalty: steep pitch (>3°) more likely indicates terrain
        // slope than a flat ground target nearby. At 1.5m height:
        //   3° → D=28.6m (plausible flat ground)
        //   5° → D=17.2m (plausible but suspicious)
        //   8.7° → D=9.8m (almost certainly looking downhill, not at ground 10m away)
        //   15° → D=5.6m (extreme: definitely slope)
        let slopePenalty: Float
        if pitchBelowHorizontalDeg <= 3.0 {
            slopePenalty = 1.0  // No penalty: shallow pitch is consistent with flat ground
        } else {
            slopePenalty = max(0.4, 1.0 - (pitchBelowHorizontalDeg - 3.0) * 0.08)
        }

        return baseConfidence * slopePenalty
    }
}

/// Result from geometric ground-plane ranging.
struct GeometricEstimate {
    let distanceMeters: Float
    let confidence: Float
    let pitchBelowHorizontalDeg: Float
}
