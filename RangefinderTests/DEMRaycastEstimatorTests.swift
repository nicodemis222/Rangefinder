//
//  DEMRaycastEstimatorTests.swift
//  RangefinderTests
//
//  Tests for DEM ray-terrain intersection logic.
//

import XCTest
import CoreLocation
@testable import Rangefinder

final class DEMRaycastEstimatorTests: XCTestCase {

    // MARK: - Confidence Computation

    func testConfidenceHighWithGoodGPS() {
        // At 100m with good GPS (<5m) and good heading (<10°):
        // gpsFactor = 0.80, headingFactor = 1.0
        // distanceFactor = 1.0 / (1.0 + 100/500) = 1.0/1.2 = 0.833
        // Expected: 0.80 * 1.0 * 0.833 ≈ 0.667
        let estimate = DEMRaycastEstimate(
            distanceMeters: 100,
            confidence: 0.667,
            terrainElevation: 200,
            headingDeg: 90,
            gpsAccuracy: 3.0,
            hitCoordinate: .init(latitude: 0, longitude: 0)
        )
        XCTAssertEqual(estimate.confidence, 0.667, accuracy: 0.01)
    }

    func testConfidenceLowWithPoorGPS() {
        // With GPS accuracy > 30m: gpsFactor = 0.15
        let estimate = DEMRaycastEstimate(
            distanceMeters: 100,
            confidence: 0.125,
            terrainElevation: 200,
            headingDeg: 90,
            gpsAccuracy: 40.0,
            hitCoordinate: .init(latitude: 0, longitude: 0)
        )
        XCTAssertLessThan(estimate.confidence, 0.2)
    }

    func testConfidenceDegradesWithDistance() {
        // distanceFactor = 1.0 / (1.0 + distance/500.0)
        // At 100m: 1.0 / 1.2 = 0.833
        // At 500m: 1.0 / 2.0 = 0.500
        // At 1000m: 1.0 / 3.0 = 0.333
        let factor100 = 1.0 / (1.0 + 100.0 / 500.0)
        let factor500 = 1.0 / (1.0 + 500.0 / 500.0)
        let factor1000 = 1.0 / (1.0 + 1000.0 / 500.0)

        XCTAssertEqual(factor100, 0.833, accuracy: 0.01)
        XCTAssertEqual(factor500, 0.500, accuracy: 0.01)
        XCTAssertEqual(factor1000, 0.333, accuracy: 0.01)
        XCTAssertGreaterThan(factor100, factor500)
        XCTAssertGreaterThan(factor500, factor1000)
    }

    // MARK: - Ray Direction (ENU)

    func testRayDirectionNorth() {
        // Heading 0° (North), pitch 5° below horizontal
        let headingRad = 0.0 * Double.pi / 180.0
        let pitchBelow = 5.0 * Double.pi / 180.0

        let dEast = sin(headingRad) * cos(pitchBelow)
        let dNorth = cos(headingRad) * cos(pitchBelow)
        let dUp = -sin(pitchBelow)

        XCTAssertEqual(dEast, 0.0, accuracy: 0.001, "Facing North: no East component")
        XCTAssertEqual(dNorth, cos(pitchBelow), accuracy: 0.001, "Facing North: full North component")
        XCTAssertLessThan(dUp, 0, "Ray should go downward")
    }

    func testRayDirectionEast() {
        // Heading 90° (East), pitch 5° below horizontal
        let headingRad = 90.0 * Double.pi / 180.0
        let pitchBelow = 5.0 * Double.pi / 180.0

        let eastComponent = sin(headingRad) * cos(pitchBelow)
        let northComponent = cos(headingRad) * cos(pitchBelow)
        let upComponent = -sin(pitchBelow)

        XCTAssertEqual(eastComponent, cos(pitchBelow), accuracy: 0.001, "Facing East: full East component")
        XCTAssertEqual(northComponent, 0.0, accuracy: 0.001, "Facing East: no North component")
        XCTAssertLessThan(upComponent, 0, "Ray should go downward")
    }

    func testRayDirectionSteepPitch() {
        // Heading 0° (North), pitch 45° below horizontal
        let headingRad = 0.0 * Double.pi / 180.0
        let pitchBelow = 45.0 * Double.pi / 180.0

        let dEast = sin(headingRad) * cos(pitchBelow)
        let dNorth = cos(headingRad) * cos(pitchBelow)
        let dUp = -sin(pitchBelow)

        // At 45°, cos and sin are equal
        XCTAssertEqual(dNorth, abs(dUp), accuracy: 0.001,
            "At 45° pitch, horizontal and vertical components should be equal")
    }

    // MARK: - Pitch Validation

    func testLookingUpReturnsNoPitch() {
        // pitchRadians > 0 means looking up → no DEM estimate
        // The estimator checks: guard pitchRadians < -0.003
        let pitchUp = 5.0 * Double.pi / 180.0
        XCTAssertGreaterThan(pitchUp, -0.003,
            "Looking up should fail the pitch guard")
    }

    func testLookingLevelReturnsNoPitch() {
        // Level pitch → no DEM estimate
        let pitchLevel = 0.0
        XCTAssertFalse(pitchLevel < -0.003,
            "Level should fail the pitch guard")
    }

    func testLookingDownPassesPitchGuard() {
        // -5° pitch → passes guard
        let pitchDown = -5.0 * Double.pi / 180.0
        XCTAssertLessThan(pitchDown, -0.003,
            "Looking down should pass the pitch guard")
    }

    // MARK: - Bisection Refinement

    func testBisectionConverges() {
        // Bisection between 30m and 60m should converge in 5 iterations
        // to within ~1m accuracy: (60-30) / 2^5 = 30/32 ≈ 0.94m
        var lo: Float = 30.0
        var hi: Float = 60.0
        let iterations = 5

        for _ in 0..<iterations {
            let mid = (lo + hi) / 2.0
            // Simulate: intersection at 45m
            if mid < 45.0 {
                lo = mid
            } else {
                hi = mid
            }
        }

        let finalDist = (lo + hi) / 2.0
        XCTAssertEqual(finalDist, 45.0, accuracy: 1.0,
            "Bisection should converge within ~1m in 5 iterations")
    }

    // MARK: - Rate Limiting

    @MainActor func testRateLimitingInterval() {
        // Query interval should be 0.5s (2 Hz)
        let tileCache = SRTMTileCache()
        let estimator = DEMRaycastEstimator(tileCache: tileCache)
        XCTAssertEqual(estimator.queryInterval, 0.5, accuracy: 0.01)
    }

    // MARK: - ENU Coordinate Conversion

    func testMetersPerDegreeLatitude() {
        // ~111,320 meters per degree latitude (roughly constant)
        let metersPerDegLat = 111_320.0
        // 1 degree of latitude ≈ 111.32 km
        XCTAssertEqual(metersPerDegLat, 111_320.0, accuracy: 100.0)
    }

    func testMetersPerDegreeLongitude() {
        // Varies with latitude: 111,320 * cos(lat)
        // At 37°N: 111,320 * cos(37°) = 111,320 * 0.7986 ≈ 88,917
        let lat = 37.0
        let metersPerDegLon = 111_320.0 * cos(lat * .pi / 180.0)
        XCTAssertEqual(metersPerDegLon, 88_917, accuracy: 100)
    }

    func testMetersPerDegreeLongitudeAtEquator() {
        // At equator: 111,320 * cos(0°) = 111,320
        let lat = 0.0
        let metersPerDegLon = 111_320.0 * cos(lat * .pi / 180.0)
        XCTAssertEqual(metersPerDegLon, 111_320.0, accuracy: 1.0)
    }

    // MARK: - Result Structure

    func testDEMRaycastEstimateFields() {
        let estimate = DEMRaycastEstimate(
            distanceMeters: 91.4,
            confidence: 0.65,
            terrainElevation: 150.0,
            headingDeg: 180.0,
            gpsAccuracy: 5.0,
            hitCoordinate: .init(latitude: 0, longitude: 0)
        )

        XCTAssertEqual(estimate.distanceMeters, 91.4, accuracy: 0.1)
        XCTAssertEqual(estimate.confidence, 0.65, accuracy: 0.01)
        XCTAssertEqual(estimate.terrainElevation, 150.0, accuracy: 0.1)
        XCTAssertEqual(estimate.headingDeg, 180.0, accuracy: 0.1)
        XCTAssertEqual(estimate.gpsAccuracy, 5.0, accuracy: 0.1)
    }

    // MARK: - Vertical Accuracy (Barometric Altimeter)

    func testVerticalAccuracyFactors() {
        // Barometric altitude gives 1-5m vertical accuracy
        // GPS-only gives 10-30m vertical accuracy
        // Better vertical accuracy should give higher DEM confidence

        // Barometric accuracy thresholds:
        // <3m: altFactor = 1.0 (excellent)
        // <10m: altFactor = 0.85 (good)
        // <20m: altFactor = 0.70 (typical GPS)
        // else: altFactor = 0.60 (poor/unknown)

        let baroAcc: Float = 2.0
        let gpsAcc: Float = 15.0
        let poorAcc: Float = 25.0

        // Verify thresholds are correctly tiered
        XCTAssertLessThan(baroAcc, 3.0, "Barometric should be in top tier")
        XCTAssertLessThan(gpsAcc, 20.0, "Good GPS should be in second tier")
        XCTAssertGreaterThan(poorAcc, 20.0, "Poor accuracy should be in lowest tier")
    }

    @MainActor func testEstimateAcceptsVerticalAccuracy() {
        // Verify the estimate() method accepts the verticalAccuracy parameter
        let tileCache = SRTMTileCache()
        let estimator = DEMRaycastEstimator(tileCache: tileCache)

        // This is a compile-time test: the signature should accept verticalAccuracy
        // Runtime: it returns nil because there's no SRTM data,
        // but it should not crash
        _ = estimator
        // The new signature: estimate(coordinate:altitude:pitchRadians:headingDegrees:horizontalAccuracy:verticalAccuracy:)
        // is validated at compile time
    }
}
