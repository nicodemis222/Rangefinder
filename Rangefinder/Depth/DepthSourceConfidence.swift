//
//  DepthSourceConfidence.swift
//  Rangefinder
//
//  Distance-dependent confidence curves for each depth source.
//
//  These overlapping curves are the key to seamless handover:
//  no hard boundaries, just gradually shifting weights.
//
//  ┌───────────────────────────────────────────────────────┐
//  │  c_lidar:    ████████████▓▓▓░░                        │
//  │              0m        3m    5m    8m                  │
//  │                                                        │
//  │  c_neural:      ░░▓▓████▓▓▓░░░░                      │
//  │              0m     8m  15m  30m 50m 80m  150m        │
//  │                                                        │
//  │  c_geometric:       ░░▓▓████████▓▓▓░░░               │
//  │                    5m  10m  50m 100m 200m  500m        │
//  │                                                        │
//  │  c_dem:                   ░░▓▓████████████▓▓░░       │
//  │                          20m 50m  500m   2000m        │
//  │                                                        │
//  │  c_object:           ░░▓▓████████████▓▓░░            │
//  │                     20m 50m  200m  500m 1000m         │
//  │                                                        │
//  │  Fusion: d_final = Σ(c_i × d_i) / Σ(c_i)            │
//  │  + outlier suppression (>2.0× from median → drop)    │
//  │  + DEM-dominance (DEM vs neural disagreement → DEM)  │
//  │  + neural extrapolation penalty (>15m calibrated)    │
//  └───────────────────────────────────────────────────────┘
//

import Foundation

struct DepthSourceConfidence {

    // MARK: - LiDAR Confidence

    /// LiDAR: peaks at 0.3-3m, fades 3-12m with a gentle tail.
    /// iPhone 16/17 Pro Max LiDAR can get readings out to ~12m in good light.
    static func lidar(distanceM: Float) -> Float {
        if distanceM < 0.3 { return 0.0 }      // Too close for reliable reading
        if distanceM < 3.0 { return 0.98 }      // Sweet spot
        if distanceM < 5.0 {                     // 0.98 → 0.7
            return 0.98 - (distanceM - 3.0) * 0.14
        }
        if distanceM < 8.0 {                     // 0.7 → 0.2
            return 0.70 - (distanceM - 5.0) * 0.167
        }
        if distanceM < 10.0 {                    // 0.2 → 0.05
            return 0.20 - (distanceM - 8.0) * 0.075
        }
        if distanceM < 12.0 {                    // 0.05 → 0.0 (extended tail)
            return 0.05 - (distanceM - 10.0) * 0.025
        }
        return 0.0
    }

    // MARK: - Neural Depth Confidence (calibrated)

    /// Neural (DepthAnythingV2, calibrated via LiDAR at 0.2-8m):
    /// Reliable where calibration data exists. Extrapolation degrades
    /// continuously beyond 8m because the inverse-depth transform amplifies
    /// noise and calibration was only trained on 0.2-8m LiDAR data.
    ///
    /// The curve extends to 150m with progressively declining confidence.
    /// Beyond ~50m, neural is a secondary source — DEM and object detection
    /// are preferred — but neural can still contribute useful signal out
    /// to 150m when other sources are unavailable.
    static func neural(distanceM: Float) -> Float {
        // Hard cap: neural readings beyond 150m are discarded by semantic selection.
        // Return 0.0 so even if somehow called, the weight is zero.
        if distanceM > AppConfiguration.neuralHardCapMeters { return 0.0 }

        if distanceM < 2.0 { return 0.3 }
        if distanceM < 5.0 {                     // 0.3 → 0.8 (fast ramp for LiDAR overlap)
            return 0.3 + (distanceM - 2.0) * 0.167
        }
        if distanceM < 8.0 {                     // 0.8 → 0.9
            return 0.8 + (distanceM - 5.0) * 0.033
        }
        if distanceM < 15.0 { return 0.9 }      // Close to training data — reliable
        if distanceM < 25.0 {                    // 0.9 → 0.70 (moderate extrapolation)
            return 0.9 - (distanceM - 15.0) * 0.02
        }
        if distanceM < 40.0 {                    // 0.70 → 0.45 (~5× beyond training range)
            return 0.70 - (distanceM - 25.0) * 0.0167
        }
        if distanceM < 50.0 {                    // 0.45 → 0.35
            return 0.45 - (distanceM - 40.0) * 0.01
        }
        if distanceM < 80.0 {                    // 0.35 → 0.25 (moderate extrapolation)
            return 0.35 - (distanceM - 50.0) * 0.00333
        }
        if distanceM < 120.0 {                   // 0.25 → 0.15 (significant extrapolation)
            return 0.25 - (distanceM - 80.0) * 0.0025
        }
        if distanceM < 150.0 {                   // 0.15 → 0.08 (extreme extrapolation, low weight)
            return 0.15 - (distanceM - 120.0) * 0.00233
        }
        return 0.0                               // Beyond 150m hard cap
    }

    // MARK: - Geometric Confidence

    /// Geometric ground-plane ranging: uses IMU pitch angle + camera height.
    /// Peaks at 10-50m, moderate at 50-100m, low at 100-200m, negligible beyond 500m.
    ///
    /// Note: the GeometricRangeEstimator already bakes pitch-based confidence
    /// into its estimate. This curve provides the distance-dependent fusion weight.
    static func geometric(distanceM: Float) -> Float {
        if distanceM < 5.0 { return 0.0 }       // Too close, LiDAR is better
        if distanceM < 10.0 {                     // 0 → 0.70 ramp
            return (distanceM - 5.0) * 0.14
        }
        if distanceM < 50.0 { return 0.70 }     // Sweet spot
        if distanceM < 100.0 {                    // 0.70 → 0.45
            return 0.70 - (distanceM - 50.0) * 0.005
        }
        if distanceM < 200.0 {                    // 0.45 → 0.20
            return 0.45 - (distanceM - 100.0) * 0.0025
        }
        if distanceM < 500.0 {                    // 0.20 → 0.05
            return 0.20 - (distanceM - 200.0) * 0.0005
        }
        return 0.0                                // Beyond 500m: too unreliable
    }

    // MARK: - DEM Raycast Confidence

    /// DEM ray-casting: terrain-aware ranging via GPS + IMU + elevation data.
    /// Most valuable at 50-500m where other sources fail on sloped terrain.
    /// GPS noise limits close-range use; heading error limits extreme range.
    ///
    /// Refined based on real-world accuracy data:
    /// - SRTM vertical RMSE: 5-10m on moderate slopes, 19m on steep (>10°)
    /// - iPhone GPS horizontal: 3-10m outdoor open sky
    /// - iPhone compass heading: ±5-10° outdoor, ±15-20° with interference
    /// - Barometric altitude: 1-5m (CMAltimeter), vs GPS 10-30m
    static func demRaycast(distanceM: Float, gpsAccuracy: Float, headingAccuracy: Float) -> Float {
        // GPS noise dominates at close range
        if distanceM < 20.0 { return 0.0 }

        // Distance factor: DEM reliability as a function of range.
        // Ramps quickly from 20-50m (GPS error becomes small relative to distance),
        // peaks at 100-300m (SRTM resolution ideal, heading error manageable),
        // slowly fades as heading error grows.
        let distanceFactor: Float
        if distanceM < 40.0 {                    // 0 → 0.75 (fast ramp — DEM viable here)
            distanceFactor = (distanceM - 20.0) / 20.0 * 0.75
        } else if distanceM < 100.0 {            // 0.75 → 0.92 (approaching sweet spot)
            distanceFactor = 0.75 + (distanceM - 40.0) / 60.0 * 0.17
        } else if distanceM < 300.0 {            // 0.92 → 0.95 (DEM sweet spot)
            distanceFactor = 0.92 + (distanceM - 100.0) / 200.0 * 0.03
        } else if distanceM < 600.0 {            // 0.95 → 0.88 (plateau, slight fade)
            distanceFactor = 0.95 - (distanceM - 300.0) / 300.0 * 0.07
        } else if distanceM < 2000.0 {           // 0.88 → 0.65 (heading error grows, but ray-march averaging keeps accuracy high)
            distanceFactor = 0.88 - (distanceM - 600.0) / 1400.0 * 0.23
        } else {
            return 0.0
        }

        // GPS accuracy factor: higher weight for better GPS.
        // 1M Monte Carlo analysis showed poor GPS causes 12-31× more error at
        // long range. Moderate penalty on fusion weight; steeper penalty on
        // displayed confidence is applied post-fusion.
        let gpsFactor: Float
        if gpsAccuracy < 5.0 {
            gpsFactor = 1.0
        } else if gpsAccuracy < 10.0 {
            gpsFactor = 0.85
        } else if gpsAccuracy < 15.0 {
            gpsFactor = 0.65
        } else {
            gpsFactor = 0.35
        }

        // Heading accuracy factor
        let headingFactor: Float
        if headingAccuracy < 5.0 {
            headingFactor = 1.0
        } else if headingAccuracy < 10.0 {
            headingFactor = 0.80
        } else if headingAccuracy < 15.0 {
            headingFactor = 0.55
        } else {
            headingFactor = 0.35
        }

        return distanceFactor * gpsFactor * headingFactor
    }

    // MARK: - Object Detection Confidence

    /// Object detection: depends on detection confidence × distance-dependent factor.
    /// Object-size ranging via pinhole model is the most reliable long-range method.
    static func object(distanceM: Float, detectionConfidence: Float) -> Float {
        let distanceFactor: Float
        if distanceM < 15.0 {
            distanceFactor = 0.3       // Objects too close, bounding box imprecise relative to size
        } else if distanceM < 30.0 {
            distanceFactor = 0.3 + (distanceM - 15.0) / 15.0 * 0.55  // 0.3 → 0.85
        } else if distanceM < 300.0 {
            distanceFactor = 0.90      // Sweet spot: best ranging method at this distance
        } else if distanceM < 600.0 {
            distanceFactor = 0.85      // Still very good
        } else if distanceM < 1000.0 {
            distanceFactor = 0.75      // Object may be small but still measurable
        } else {
            distanceFactor = max(0.35, 0.75 - (distanceM - 1000.0) * 0.0004)
        }
        return detectionConfidence * distanceFactor
    }

    // MARK: - Calibration Quality Modifier

    /// Adjusts neural confidence based on how fresh the calibration is.
    /// Recently calibrated = full confidence; stale calibration = reduced.
    ///
    /// Key insight: when the user walks past LiDAR range (8m), calibration
    /// samples stop. The fitted transform is still valid (it's a stable
    /// affine/inverse mapping), so we should trust it for a reasonable
    /// duration — especially since the user is likely still in a similar
    /// scene with similar depth characteristics.
    ///
    /// The calibration is a least-squares fit of scale and shift — it's
    /// mathematically stable and doesn't drift. Decay should be very slow.
    static func calibrationQuality(
        calibrationAge: TimeInterval,
        calibrationConfidence: Float
    ) -> Float {
        // Fresh calibration (< 60s): full quality
        // Gives the user a full minute of walking beyond LiDAR range
        // before any quality decay begins.
        if calibrationAge < 60.0 { return calibrationConfidence }
        // Aging calibration (60-180s): very slow decay
        if calibrationAge < 180.0 {
            let decay = Float((calibrationAge - 60.0) / 120.0) * 0.15
            return max(0.5, calibrationConfidence - decay)
        }
        // Stale calibration (180-600s): moderate decay
        if calibrationAge < 600.0 {
            let decay = 0.15 + Float((calibrationAge - 180.0) / 420.0) * 0.25
            return max(0.5, calibrationConfidence - decay)
        }
        // Very stale (>10min): minimum quality — still usable since
        // the affine fit is mathematically stable
        return 0.5
    }
}
