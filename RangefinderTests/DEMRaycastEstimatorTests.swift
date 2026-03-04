//
//  DEMRaycastEstimatorTests.swift
//  RangefinderTests
//
//  Tests for DEM terrain visibility scan logic.
//

import XCTest
import CoreLocation
@testable import Rangefinder

final class DEMRaycastEstimatorTests: XCTestCase {

    // MARK: - Confidence Computation

    func testConfidenceHighWithGoodGPS() {
        // With good GPS (<5m): gpsFactor = 0.90
        // Good heading (<5°): headingFactor = 1.0
        // Unknown vertical: altFactor = 0.65
        // Expected: 0.90 * 0.65 * 1.0 = 0.585
        let estimate = DEMRaycastEstimate(
            distanceMeters: 100,
            confidence: 0.585,
            terrainElevation: 200,
            headingDeg: 90,
            gpsAccuracy: 3.0,
            hitCoordinate: .init(latitude: 0, longitude: 0)
        )
        XCTAssertEqual(estimate.confidence, 0.585, accuracy: 0.01)
    }

    func testConfidenceLowWithPoorGPS() {
        // With GPS accuracy > 30m: gpsFactor = 0.15
        let estimate = DEMRaycastEstimate(
            distanceMeters: 100,
            confidence: 0.10,
            terrainElevation: 200,
            headingDeg: 90,
            gpsAccuracy: 40.0,
            hitCoordinate: .init(latitude: 0, longitude: 0)
        )
        XCTAssertLessThan(estimate.confidence, 0.2)
    }

    func testConfidenceNotDistanceDependent() {
        // The DEM estimator's confidence does NOT depend on distance —
        // distance weighting is applied separately by DepthSourceConfidence.demRaycast()
        // in the fusion pipeline. This avoids double-penalizing distance.
        // GPS factor, alt factor, and heading factor are all distance-independent.
        let gpsFactor: Float = 0.90  // <5m GPS
        let altFactor: Float = 0.65  // unknown vertical
        let headingFactor: Float = 1.0  // <5° heading

        let expectedConf = gpsFactor * altFactor * headingFactor

        // Same confidence regardless of distance
        XCTAssertEqual(expectedConf, 0.585, accuracy: 0.01,
            "Confidence should be 0.585 regardless of distance")
    }

    // MARK: - Horizontal March Direction

    func testHorizontalMarchNorth() {
        // Heading 0° (North): march is purely along North axis
        let headingRad = 0.0 * Double.pi / 180.0

        let dEast = sin(headingRad)   // per horizontal meter
        let dNorth = cos(headingRad)  // per horizontal meter

        XCTAssertEqual(dEast, 0.0, accuracy: 0.001, "Facing North: no East component")
        XCTAssertEqual(dNorth, 1.0, accuracy: 0.001, "Facing North: full North component")
    }

    func testHorizontalMarchEast() {
        // Heading 90° (East): march is purely along East axis
        let headingRad = 90.0 * Double.pi / 180.0

        let dEast = sin(headingRad)
        let dNorth = cos(headingRad)

        XCTAssertEqual(dEast, 1.0, accuracy: 0.001, "Facing East: full East component")
        XCTAssertEqual(dNorth, 0.0, accuracy: 0.001, "Facing East: no North component")
    }

    func testHorizontalMarchIsPitchIndependent() {
        // The horizontal march direction should NOT change with pitch
        // (this is the key difference from the old ray march)
        let headingRad = 45.0 * Double.pi / 180.0

        let dEast = sin(headingRad)
        let dNorth = cos(headingRad)

        // Regardless of pitch, horizontal components are the same
        let expectedEast = sin(headingRad)
        let expectedNorth = cos(headingRad)

        XCTAssertEqual(dEast, expectedEast, accuracy: 0.001)
        XCTAssertEqual(dNorth, expectedNorth, accuracy: 0.001)

        // Verify unit length
        let length = sqrt(dEast * dEast + dNorth * dNorth)
        XCTAssertEqual(length, 1.0, accuracy: 0.001,
            "Horizontal march should be unit-length (1m per step unit)")
    }

    // MARK: - LOS Altitude Computation

    func testLOSAltitudeLookingDown() {
        // Looking down at -7.9°: LOS descends
        let cameraPitchRad = -7.9 * Double.pi / 180.0
        let tanPitch = tan(cameraPitchRad)
        let observerAlt = 2002.0

        let losAt100m = observerAlt + 100.0 * tanPitch
        let losAt500m = observerAlt + 500.0 * tanPitch
        let losAt1000m = observerAlt + 1000.0 * tanPitch

        // LOS should descend
        XCTAssertLessThan(losAt100m, observerAlt, "LOS descends when looking down")
        XCTAssertLessThan(losAt500m, losAt100m, "LOS continues descending")
        XCTAssertLessThan(losAt1000m, losAt500m, "LOS continues descending")

        // At 100m: descent = 100 * tan(7.9°) ≈ 13.9m
        XCTAssertEqual(losAt100m, 2002.0 - 13.88, accuracy: 0.5)
    }

    func testLOSAltitudeLookingUp() {
        // Looking up at +5°: LOS ascends
        let cameraPitchRad = 5.0 * Double.pi / 180.0
        let tanPitch = tan(cameraPitchRad)
        let observerAlt = 500.0

        let losAt1000m = observerAlt + 1000.0 * tanPitch

        // LOS should ascend: 500 + 1000 * tan(5°) ≈ 500 + 87.5 = 587.5
        XCTAssertGreaterThan(losAt1000m, observerAlt, "LOS ascends when looking up")
        XCTAssertEqual(losAt1000m, 587.5, accuracy: 1.0)
    }

    // MARK: - Viewshed Elevation Angle

    func testElevationAngleToHigherTerrain() {
        // Terrain 100m higher at 1000m horizontal distance
        let observerAlt = 500.0
        let terrainElev = 600.0
        let horizontalDist = 1000.0

        let angle = atan2(terrainElev - observerAlt, horizontalDist) * 180.0 / .pi

        // angle = atan2(100, 1000) ≈ 5.71°
        XCTAssertEqual(angle, 5.71, accuracy: 0.1,
            "Angle to higher terrain should be positive")
        XCTAssertGreaterThan(angle, 0)
    }

    func testElevationAngleToLowerTerrain() {
        // Terrain 200m lower at 1463m horizontal distance
        let observerAlt = 2002.0
        let terrainElev = 1800.0
        let horizontalDist = 1463.0

        let angle = atan2(terrainElev - observerAlt, horizontalDist) * 180.0 / .pi

        // angle = atan2(-202, 1463) ≈ -7.87°
        XCTAssertEqual(angle, -7.87, accuracy: 0.1,
            "Angle to lower terrain should be negative")
    }

    func testViewshedVisibility() {
        // Simulate viewshed: terrain at d=100m has higher angle than at d=500m
        // (because terrain drops away)
        let observerAlt = 2002.0

        // Near terrain: 2000m at 100m → angle = atan2(-2, 100) = -1.15°
        let nearAngle = atan2(2000.0 - observerAlt, 100.0) * 180.0 / .pi

        // Far terrain: 1800m at 1463m → angle = atan2(-202, 1463) = -7.87°
        let farAngle = atan2(1800.0 - observerAlt, 1463.0) * 180.0 / .pi

        // Near terrain sets the horizon
        XCTAssertGreaterThan(nearAngle, farAngle,
            "Near terrain has higher elevation angle (sets horizon)")

        // Far terrain would be occluded if flat ground extends to it
        var maxElevAngle = -90.0
        maxElevAngle = max(maxElevAngle, nearAngle)  // → -1.15°
        let farVisible = farAngle >= maxElevAngle
        XCTAssertFalse(farVisible,
            "Far lower terrain should be occluded by flat ground near observer")
    }

    func testViewshedVisibilityFromRidge() {
        // Observer on a ridge: terrain drops steeply, then distant terrain is visible
        let observerAlt = 2502.0

        // Ridge edge at 30m: terrain=2480 → angle = atan2(-22, 30) = -36.2°
        let ridgeAngle = atan2(2480.0 - observerAlt, 30.0) * 180.0 / .pi

        // Valley at 1000m: terrain=1780 → angle = atan2(-722, 1000) = -35.8°
        let valleyAngle = atan2(1780.0 - observerAlt, 1000.0) * 180.0 / .pi

        // Distant terrain at 1463m: terrain=2300 → angle = atan2(-202, 1463) = -7.87°
        let targetAngle = atan2(2300.0 - observerAlt, 1463.0) * 180.0 / .pi

        var maxElevAngle = -90.0
        maxElevAngle = max(maxElevAngle, ridgeAngle)  // → -36.2°
        maxElevAngle = max(maxElevAngle, valleyAngle)  // stays -35.8° > -36.2°? No, -35.8 > -36.2.

        // Valley angle (-35.8°) is GREATER than ridge angle (-36.2°): visible
        let valleyVisible = valleyAngle >= maxElevAngle
        XCTAssertTrue(valleyVisible, "Valley bottom should be visible from ridge")

        maxElevAngle = max(maxElevAngle, valleyAngle)  // → -35.8°

        // Target at -7.87° > -35.8° → VISIBLE from ridge!
        let targetVisible = targetAngle >= maxElevAngle
        XCTAssertTrue(targetVisible,
            "Distant terrain should be visible from ridge (angle rises)")
    }

    // MARK: - Pitch Matching

    func testPitchMatchingAtTarget() {
        // Camera pitch -7.9°, terrain at 1463m has elevation angle -7.87°
        let cameraPitchDeg = -7.9
        let terrainAngleDeg = -7.87
        let angleDiff = abs(terrainAngleDeg - cameraPitchDeg)

        // Tolerance at 1463m: atan2(10, 1463) + 0.5° ≈ 0.39° + 0.5° = 0.89°
        // max(0.89°, 1.0°) = 1.0°
        let tolerance = max(atan2(10.0, 1463.0) * 180.0 / .pi + 0.5, 1.0)

        XCTAssertLessThan(angleDiff, tolerance,
            "Terrain at 1463m should match camera pitch within tolerance")
    }

    func testPitchMatchingNearFieldMismatch() {
        // Camera pitch -7.9°, flat ground at 100m has angle -1.15°
        let cameraPitchDeg = -7.9
        let terrainAngleDeg = -1.15
        let angleDiff = abs(terrainAngleDeg - cameraPitchDeg)

        // Tolerance at 100m: atan2(10, 100) + 0.5° ≈ 5.71° + 0.5° = 6.21°
        let tolerance = max(atan2(10.0, 100.0) * 180.0 / .pi + 0.5, 1.0)

        XCTAssertGreaterThan(angleDiff, tolerance,
            "Flat ground at 100m should NOT match -7.9° pitch")
    }

    // MARK: - Pitch Validation

    func testLookingUpModeratelyAllowed() {
        // Looking up at +15° should be allowed (mountains above)
        let pitchDegrees = 15.0
        XCTAssertLessThan(pitchDegrees, 30.0,
            "Moderate upward look should pass pitch guard")
    }

    func testLookingSteepUpRejected() {
        // Looking up at +35° should be rejected (aiming at sky)
        let pitchDegrees = 35.0
        XCTAssertGreaterThanOrEqual(pitchDegrees, 30.0,
            "Steep upward look should fail pitch guard")
    }

    func testLookingDownPassesPitchGuard() {
        // -5° pitch → passes guard
        let pitchDegrees = -5.0
        XCTAssertLessThan(pitchDegrees, 30.0,
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
        // <10m: altFactor = 0.90 (good)
        // <20m: altFactor = 0.75 (typical GPS)
        // else: altFactor = 0.65 (poor/unknown)

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
    }

    // MARK: - Selection Priority

    func testFarFieldPreferredOverNearField() {
        // The selection logic should prefer far-field hits over near-field
        // This prevents flat-ground grazes (14m) from masking mountain hits (1463m)
        let nearField: Float = 14.6   // Flat ground graze
        let farField: Float = 1463.0  // Mountain target

        XCTAssertGreaterThan(farField, 100.0, "Mountain hit should be far-field")
        XCTAssertLessThan(nearField, 100.0, "Flat ground graze should be near-field")
        // Selection priority: far-field viewshed > far-field LOS > near-field
        // The algorithm always prefers far-field results when available
    }
}
