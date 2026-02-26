//
//  DepthSourceConfidenceTests.swift
//  RangefinderTests
//
//  Tests for overlapping confidence curves.
//

import XCTest
@testable import Rangefinder

final class DepthSourceConfidenceTests: XCTestCase {

    // MARK: - LiDAR Confidence

    func testLiDARPeaksNearRange() {
        let confidence = DepthSourceConfidence.lidar(distanceM: 1.5)
        XCTAssertEqual(confidence, 0.98, accuracy: 0.01)
    }

    func testLiDARFadesBeyond3m() {
        let at3 = DepthSourceConfidence.lidar(distanceM: 3.0)
        let at5 = DepthSourceConfidence.lidar(distanceM: 5.0)
        let at8 = DepthSourceConfidence.lidar(distanceM: 8.0)

        XCTAssertEqual(at3, 0.98, accuracy: 0.01)
        XCTAssertGreaterThan(at3, at5)
        XCTAssertGreaterThan(at5, at8)
    }

    func testLiDARZeroBeyond12m() {
        XCTAssertEqual(DepthSourceConfidence.lidar(distanceM: 12.0), 0.0, accuracy: 0.01)
        XCTAssertEqual(DepthSourceConfidence.lidar(distanceM: 100.0), 0.0, accuracy: 0.01)
    }

    func testLiDARGentleTail8to12m() {
        let at8 = DepthSourceConfidence.lidar(distanceM: 8.0)
        let at9 = DepthSourceConfidence.lidar(distanceM: 9.0)
        let at11 = DepthSourceConfidence.lidar(distanceM: 11.0)
        XCTAssertGreaterThan(at8, 0.1, "LiDAR should still be nonzero at 8m")
        XCTAssertGreaterThan(at9, 0.0, "LiDAR should still be nonzero at 9m")
        XCTAssertGreaterThan(at11, 0.0, "LiDAR should still be nonzero at 11m")
        XCTAssertGreaterThan(at8, at9, "LiDAR should be fading 8->9m")
        XCTAssertGreaterThan(at9, at11, "LiDAR should be fading 9->11m")
    }

    func testLiDARZeroTooClose() {
        XCTAssertEqual(DepthSourceConfidence.lidar(distanceM: 0.1), 0.0, accuracy: 0.01)
    }

    // MARK: - Neural Confidence

    func testNeuralPeaks8to15m() {
        let at8 = DepthSourceConfidence.neural(distanceM: 8.0)
        let at12 = DepthSourceConfidence.neural(distanceM: 12.0)
        XCTAssertEqual(at8, 0.9, accuracy: 0.01)
        XCTAssertEqual(at12, 0.9, accuracy: 0.01)
    }

    func testNeuralDecaysBeyond15m() {
        // Neural now decays faster beyond 15m (steepened for DEM handover)
        let at15 = DepthSourceConfidence.neural(distanceM: 15.0)
        let at25 = DepthSourceConfidence.neural(distanceM: 25.0)
        let at40 = DepthSourceConfidence.neural(distanceM: 40.0)
        XCTAssertEqual(at15, 0.9, accuracy: 0.01)
        XCTAssertLessThan(at25, at15, "Neural should decay beyond 15m")
        XCTAssertLessThan(at40, at25, "Neural should continue decaying")
        // at25: 0.90 - (25-15)*0.02 = 0.70
        XCTAssertEqual(at25, 0.70, accuracy: 0.02)
        // at40: enters 25-40 bracket: 0.70 - (40-25)*0.0167 = 0.70 - 0.25 = 0.45
        XCTAssertEqual(at40, 0.45, accuracy: 0.02)
    }

    func testNeuralStrongAtHandover() {
        // At 5m (where LiDAR starts fading), neural should already be strong
        let at5 = DepthSourceConfidence.neural(distanceM: 5.0)
        XCTAssertGreaterThanOrEqual(at5, 0.8, "Neural should be >= 0.8 at 5m for LiDAR handover")
    }

    func testNeuralLowAtVeryClose() {
        let at1 = DepthSourceConfidence.neural(distanceM: 1.0)
        XCTAssertEqual(at1, 0.3, accuracy: 0.01)
    }

    func testNeuralHardCapAt150m() {
        // Neural hard cap: returns 0.0 at and beyond 150m (neuralHardCapMeters)
        let at140 = DepthSourceConfidence.neural(distanceM: 140.0)
        let at149 = DepthSourceConfidence.neural(distanceM: 149.0)
        let at150 = DepthSourceConfidence.neural(distanceM: 150.0)
        let at151 = DepthSourceConfidence.neural(distanceM: 151.0)

        // At 140m: in 120-150 bracket: 0.15 - (140-120)*0.00233 ≈ 0.103
        XCTAssertGreaterThan(at140, 0.08, "Neural at 140m should still be positive")
        // At 149m: near the cap, very low but positive
        XCTAssertGreaterThan(at149, 0.05, "Just under 150m should still be positive")
        // At exactly 150m: falls through < 150 bracket, hits return 0.0
        XCTAssertEqual(at150, 0.0, accuracy: 0.001,
            "Neural should be zero at exactly 150m (boundary)")
        // Beyond cap: hard zero
        XCTAssertEqual(at151, 0.0, accuracy: 0.001,
            "Neural should be exactly 0 beyond the 150m hard cap")
    }

    func testNeuralZeroBeyondHardCap() {
        // All readings beyond 150m should be zero
        XCTAssertEqual(DepthSourceConfidence.neural(distanceM: 200.0), 0.0, accuracy: 0.001)
        XCTAssertEqual(DepthSourceConfidence.neural(distanceM: 500.0), 0.0, accuracy: 0.001)
        XCTAssertEqual(DepthSourceConfidence.neural(distanceM: 1000.0), 0.0, accuracy: 0.001)
    }

    // MARK: - DEM Raycast Confidence

    func testDEMZeroUnder20m() {
        // GPS noise dominates at close range
        XCTAssertEqual(DepthSourceConfidence.demRaycast(distanceM: 10.0, gpsAccuracy: 5.0, headingAccuracy: 5.0), 0.0, accuracy: 0.01)
        XCTAssertEqual(DepthSourceConfidence.demRaycast(distanceM: 19.0, gpsAccuracy: 5.0, headingAccuracy: 5.0), 0.0, accuracy: 0.01)
    }

    func testDEMRampsUp20to50m() {
        let at20 = DepthSourceConfidence.demRaycast(distanceM: 20.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)
        let at35 = DepthSourceConfidence.demRaycast(distanceM: 35.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)
        let at50 = DepthSourceConfidence.demRaycast(distanceM: 50.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)

        XCTAssertEqual(at20, 0.0, accuracy: 0.01, "Should be 0 at start of ramp")
        XCTAssertGreaterThan(at35, at20)
        XCTAssertGreaterThan(at50, at35)
    }

    func testDEMPeaks100to500m() {
        // With gpsAccuracy=3.0 (gpsFactor=1.0) and headingAccuracy=3.0 (headingFactor=1.0):
        // Boosted DEM curve: at100 = 0.92, at300 = 0.95
        let at100 = DepthSourceConfidence.demRaycast(distanceM: 100.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)
        let at300 = DepthSourceConfidence.demRaycast(distanceM: 300.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)

        XCTAssertGreaterThan(at100, 0.85, "DEM should be strong at 100m with good GPS")
        XCTAssertGreaterThan(at300, 0.90, "DEM should be very strong at mid-range")
        XCTAssertGreaterThan(at300, at100, "DEM should increase toward sweet spot")
    }

    func testDEMFadesBeyond500m() {
        let at500 = DepthSourceConfidence.demRaycast(distanceM: 500.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)
        let at1000 = DepthSourceConfidence.demRaycast(distanceM: 1000.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)
        let at2000 = DepthSourceConfidence.demRaycast(distanceM: 2000.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)

        XCTAssertGreaterThan(at500, at1000)
        XCTAssertGreaterThan(at1000, at2000)
    }

    func testDEMZeroBeyond2000m() {
        let at2100 = DepthSourceConfidence.demRaycast(distanceM: 2100.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)
        XCTAssertEqual(at2100, 0.0, accuracy: 0.01)
    }

    func testDEMGPSAccuracyAffectsConfidence() {
        // Better GPS = higher DEM confidence
        let goodGPS = DepthSourceConfidence.demRaycast(distanceM: 100.0, gpsAccuracy: 3.0, headingAccuracy: 5.0)
        let medGPS = DepthSourceConfidence.demRaycast(distanceM: 100.0, gpsAccuracy: 10.0, headingAccuracy: 5.0)
        let poorGPS = DepthSourceConfidence.demRaycast(distanceM: 100.0, gpsAccuracy: 20.0, headingAccuracy: 5.0)

        XCTAssertGreaterThan(goodGPS, medGPS, "Good GPS should give higher DEM confidence")
        XCTAssertGreaterThan(medGPS, poorGPS, "Medium GPS should beat poor GPS")
    }

    func testDEMHeadingAccuracyAffectsConfidence() {
        let goodHeading = DepthSourceConfidence.demRaycast(distanceM: 200.0, gpsAccuracy: 5.0, headingAccuracy: 5.0)
        let poorHeading = DepthSourceConfidence.demRaycast(distanceM: 200.0, gpsAccuracy: 5.0, headingAccuracy: 15.0)

        XCTAssertGreaterThan(goodHeading, poorHeading, "Good heading should give higher DEM confidence")
    }

    // MARK: - Neural + DEM Overlap

    func testNeuralDEMOverlap() {
        // At 30-100m, both neural and DEM should have non-zero confidence
        // Neural now extends to 150m with declining weight
        let neural40 = DepthSourceConfidence.neural(distanceM: 40.0)
        let dem40 = DepthSourceConfidence.demRaycast(distanceM: 40.0, gpsAccuracy: 5.0, headingAccuracy: 5.0)

        // Neural at 40m: in 25-40 bracket: 0.70 - (40-25)*0.0167 = 0.45
        XCTAssertEqual(neural40, 0.45, accuracy: 0.02, "Neural should still be active at 40m")
        XCTAssertGreaterThan(dem40, 0.2, "DEM should be active at 40m")

        // At 60m, neural is fading but still present; DEM is rising
        let neural60 = DepthSourceConfidence.neural(distanceM: 60.0)
        let dem60 = DepthSourceConfidence.demRaycast(distanceM: 60.0, gpsAccuracy: 5.0, headingAccuracy: 5.0)
        // Neural at 60m: in 50-80 bracket: 0.35 - (60-50)*0.00333 ≈ 0.317
        XCTAssertGreaterThan(neural60, 0.25, "Neural should still be active at 60m (extended curve)")
        XCTAssertGreaterThan(dem60, 0.4, "DEM should be strong at 60m")

        // Beyond 150m, neural is zero and DEM takes over completely
        let neural160 = DepthSourceConfidence.neural(distanceM: 160.0)
        XCTAssertEqual(neural160, 0.0, accuracy: 0.001, "Neural should be zero beyond 150m hard cap")
    }

    // MARK: - Geometric Confidence

    func testGeometricZeroUnder5m() {
        XCTAssertEqual(DepthSourceConfidence.geometric(distanceM: 3.0), 0.0, accuracy: 0.01)
        XCTAssertEqual(DepthSourceConfidence.geometric(distanceM: 5.0), 0.0, accuracy: 0.01)
    }

    func testGeometricPeaks10to50m() {
        let at20 = DepthSourceConfidence.geometric(distanceM: 20.0)
        let at40 = DepthSourceConfidence.geometric(distanceM: 40.0)
        XCTAssertEqual(at20, 0.70, accuracy: 0.01)
        XCTAssertEqual(at40, 0.70, accuracy: 0.01)
    }

    func testGeometricFadesBeyond50m() {
        let at50 = DepthSourceConfidence.geometric(distanceM: 50.0)
        let at100 = DepthSourceConfidence.geometric(distanceM: 100.0)
        let at200 = DepthSourceConfidence.geometric(distanceM: 200.0)
        XCTAssertGreaterThan(at50, at100)
        XCTAssertGreaterThan(at100, at200)
    }

    func testGeometricZeroBeyond500m() {
        XCTAssertEqual(DepthSourceConfidence.geometric(distanceM: 500.0), 0.0, accuracy: 0.02)
    }

    // MARK: - Overlap Zones

    func testLiDARNeuralOverlap() {
        // At 4m, both LiDAR and neural should have non-zero confidence
        let lidar = DepthSourceConfidence.lidar(distanceM: 4.0)
        let neural = DepthSourceConfidence.neural(distanceM: 4.0)

        XCTAssertGreaterThan(lidar, 0.1)
        XCTAssertGreaterThan(neural, 0.1)
    }

    func testNeuralGeometricOverlap() {
        // At 30m, both neural and geometric should have non-zero confidence
        let neural = DepthSourceConfidence.neural(distanceM: 30.0)
        let geometric = DepthSourceConfidence.geometric(distanceM: 30.0)

        XCTAssertGreaterThan(neural, 0.5, "Neural should be strong at 30m")
        XCTAssertGreaterThan(geometric, 0.5, "Geometric should be strong at 30m")
    }

    // MARK: - Calibration Quality

    func testFreshCalibration() {
        let quality = DepthSourceConfidence.calibrationQuality(
            calibrationAge: 1.0,
            calibrationConfidence: 0.9
        )
        XCTAssertEqual(quality, 0.9, accuracy: 0.01)
    }

    func testStaleCalibration() {
        let fresh = DepthSourceConfidence.calibrationQuality(
            calibrationAge: 1.0,
            calibrationConfidence: 0.9
        )
        let stale = DepthSourceConfidence.calibrationQuality(
            calibrationAge: 120.0,
            calibrationConfidence: 0.9
        )

        XCTAssertGreaterThan(fresh, stale)
    }

    func testCalibrationStillStrongAt60s() {
        // Calibration should still be full quality at 60s
        // (gives user a full minute to walk past LiDAR range)
        let at60s = DepthSourceConfidence.calibrationQuality(
            calibrationAge: 60.0,
            calibrationConfidence: 0.9
        )
        XCTAssertEqual(at60s, 0.9, accuracy: 0.01)
    }

    // MARK: - DEM Refined Curve Specifics

    func testDEMAt50mWithGoodGPS() {
        // At 50m: in 40-100m bracket: distanceFactor = 0.75 + (50-40)/60 * 0.17 ≈ 0.778
        // gpsFactor(3.0) = 1.0, headingFactor(3.0) = 1.0
        // Result ≈ 0.778
        let at50 = DepthSourceConfidence.demRaycast(distanceM: 50.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)
        XCTAssertEqual(at50, 0.778, accuracy: 0.02, "DEM at 50m should be ~0.78")
    }

    func testDEMAt200mWithGoodGPS() {
        // At 200m: in 100-300m bracket: distanceFactor = 0.92 + (200-100)/200 * 0.03 = 0.935
        // gpsFactor(3.0) = 1.0, headingFactor(3.0) = 1.0
        // Result = 0.935
        let at200 = DepthSourceConfidence.demRaycast(distanceM: 200.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)
        XCTAssertEqual(at200, 0.935, accuracy: 0.02, "DEM at 200m sweet spot should be ~0.935")
    }

    func testDEMSweetSpotPlateau() {
        // 100-600m should maintain high values with good GPS/heading
        let at200 = DepthSourceConfidence.demRaycast(distanceM: 200.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)
        let at350 = DepthSourceConfidence.demRaycast(distanceM: 350.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)
        let at500 = DepthSourceConfidence.demRaycast(distanceM: 500.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)

        XCTAssertGreaterThan(at200, 0.90, "DEM should be >0.90 at 200m")
        XCTAssertGreaterThan(at350, 0.90, "DEM should be >0.90 at 350m")
        // at500: in 300-600 bracket: 0.95 - (500-300)/300 * 0.07 ≈ 0.903
        XCTAssertEqual(at500, 0.903, accuracy: 0.03, "DEM at 500m should be ~0.90")
    }

    func testDEMHeadingFactorBrackets() {
        // Good heading (<5°): factor=1.0
        // Medium heading (5-10°): factor=0.80
        // Poor heading (>=15°): factor=0.35 (refined per 1M MC analysis)
        let good = DepthSourceConfidence.demRaycast(distanceM: 200.0, gpsAccuracy: 3.0, headingAccuracy: 3.0)
        let medium = DepthSourceConfidence.demRaycast(distanceM: 200.0, gpsAccuracy: 3.0, headingAccuracy: 7.0)
        let poor = DepthSourceConfidence.demRaycast(distanceM: 200.0, gpsAccuracy: 3.0, headingAccuracy: 15.0)

        // At 200m: distanceFactor=0.935, gpsFactor=1.0
        XCTAssertEqual(good, 0.935, accuracy: 0.02, "Good heading should give full confidence")
        // Medium: 0.935 * 0.80 = 0.748
        XCTAssertEqual(medium, 0.748, accuracy: 0.03, "Medium heading should be ~0.748")
        // Poor (15°): headingFactor = 0.35 (>=15 bracket)
        // 0.935 * 0.35 = 0.327
        XCTAssertEqual(poor, 0.327, accuracy: 0.03, "Poor heading should be ~0.327")
    }

    // MARK: - Neural Effective Weight (curve × extrapolation penalty)

    func testNeuralEffectiveWeightAt25m() {
        // Neural distance curve at 25m (steepened): 0.9 - (25-15)*0.02 = 0.70
        // Extrapolation penalty at 25m: 1.0 - (25-15)/100 = 0.90
        // Effective = 0.70 * 0.90 = 0.63 (before calQuality)
        let curve = DepthSourceConfidence.neural(distanceM: 25.0)
        XCTAssertEqual(curve, 0.70, accuracy: 0.02)
        let penalty: Float = 1.0 - (25.0 - 15.0) / 100.0
        let effective = curve * penalty
        XCTAssertEqual(effective, 0.63, accuracy: 0.03,
            "Neural effective weight at 25m should be reduced by extrapolation penalty")
    }

    func testNeuralEffectiveWeightAt45m() {
        // Neural distance curve at 45m: in 40-50 bracket: 0.45 - (45-40)*0.01 = 0.40
        // Extrapolation penalty at 45m: max(0.15, 0.85 - (45-30)/100) = 0.85 - 0.15 = 0.70
        // Effective = 0.40 * 0.70 = 0.28
        let curve = DepthSourceConfidence.neural(distanceM: 45.0)
        XCTAssertEqual(curve, 0.40, accuracy: 0.02)
        let penalty: Float = max(0.15, 0.85 - (45.0 - 30.0) / 100.0)
        let effective = curve * penalty
        XCTAssertEqual(effective, 0.28, accuracy: 0.03,
            "Neural effective weight at 45m (near hard cap) should be moderate")
    }

    func testNeuralBeyondHardCapEffectiveZero() {
        // Beyond 150m hard cap, neural curve is 0 so effective weight is always 0
        let curve = DepthSourceConfidence.neural(distanceM: 200.0)
        XCTAssertEqual(curve, 0.0, accuracy: 0.001,
            "Neural at 200m should be zero due to hard cap")
    }
}
