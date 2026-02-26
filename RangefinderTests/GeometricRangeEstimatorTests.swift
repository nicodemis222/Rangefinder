//
//  GeometricRangeEstimatorTests.swift
//  RangefinderTests
//
//  Tests for IMU pitch-based ground-plane ranging.
//

import XCTest
@testable import Rangefinder

final class GeometricRangeEstimatorTests: XCTestCase {

    func testLookingUpReturnsNil() {
        let estimator = GeometricRangeEstimator()
        // Pitch +5 degrees (looking up)
        let result = estimator.estimate(pitchRadians: 5.0 * .pi / 180.0)
        XCTAssertNil(result, "Looking up should return nil")
    }

    func testLookingLevelReturnsNil() {
        let estimator = GeometricRangeEstimator()
        let result = estimator.estimate(pitchRadians: 0.0)
        XCTAssertNil(result, "Looking level should return nil")
    }

    func testNearlyLevelReturnsNil() {
        let estimator = GeometricRangeEstimator()
        // -0.1 degrees: too close to level
        let result = estimator.estimate(pitchRadians: -0.1 * .pi / 180.0)
        XCTAssertNil(result, "Nearly level should return nil")
    }

    func testKnownDistance10m() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        // At 10m with 1.5m height: pitch = atan(1.5/10) = 8.53 degrees
        let pitchRad = -atan(1.5 / 10.0)  // negative = looking down
        let result = estimator.estimate(pitchRadians: Double(pitchRad))
        XCTAssertNotNil(result)
        XCTAssertEqual(result!.distanceMeters, 10.0, accuracy: 0.5)
    }

    func testKnownDistance50m() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        let pitchRad = -atan(1.5 / 50.0)  // 1.72 degrees
        let result = estimator.estimate(pitchRadians: Double(pitchRad))
        XCTAssertNotNil(result)
        XCTAssertEqual(result!.distanceMeters, 50.0, accuracy: 2.0)
    }

    func testKnownDistance100m() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        let pitchRad = -atan(1.5 / 100.0)  // 0.86 degrees
        let result = estimator.estimate(pitchRadians: Double(pitchRad))
        XCTAssertNotNil(result)
        XCTAssertEqual(result!.distanceMeters, 100.0, accuracy: 5.0)
    }

    func testConfidenceHigherAtCloseRange() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5

        let close = estimator.estimate(pitchRadians: Double(-atan(1.5 / 10.0)))
        let far = estimator.estimate(pitchRadians: Double(-atan(1.5 / 200.0)))

        XCTAssertNotNil(close)
        XCTAssertNotNil(far)
        XCTAssertGreaterThan(close!.confidence, far!.confidence,
            "Confidence should be higher at close range")
    }

    func testCustomCameraHeight() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 3.0  // Vehicle mount
        let pitchRad = -atan(3.0 / 100.0)  // ~1.72 degrees
        let result = estimator.estimate(pitchRadians: Double(pitchRad))
        XCTAssertNotNil(result)
        XCTAssertEqual(result!.distanceMeters, 100.0, accuracy: 5.0)
    }

    func testVeryFarReturnsNil() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        // At 1000m, pitch = atan(1.5/1000) = 0.086 degrees — below minimum 0.3 degrees
        let pitchRad = -atan(1.5 / 1000.0)
        let result = estimator.estimate(pitchRadians: Double(pitchRad))
        XCTAssertNil(result, "1000m should be beyond geometric range")
    }

    func testPitchBelowHorizontalDegReported() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        let pitchRad = -atan(1.5 / 30.0)  // ~2.86 degrees
        let result = estimator.estimate(pitchRadians: Double(pitchRad))
        XCTAssertNotNil(result)
        XCTAssertEqual(result!.pitchBelowHorizontalDeg, 2.86, accuracy: 0.1)
    }

    // MARK: - Slope Risk Penalty

    func testSteepPitchReducedConfidence() {
        // At steep pitch (>5°), slope risk penalty should reduce confidence
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5

        // 2° pitch: D=42.9m, no slope penalty
        let shallowPitch = -2.0 * .pi / 180.0
        let shallow = estimator.estimate(pitchRadians: shallowPitch)

        // 8.7° pitch: D=9.8m, moderate slope penalty
        let steepPitch = -8.7 * .pi / 180.0
        let steep = estimator.estimate(pitchRadians: steepPitch)

        XCTAssertNotNil(shallow)
        XCTAssertNotNil(steep)

        // Steep should have lower confidence despite being "closer"
        // because slope penalty applies at >5°
        XCTAssertLessThan(steep!.confidence, shallow!.confidence,
            "Steep pitch should have reduced confidence due to slope penalty")
    }

    func testSlopePenaltyAt8Point7Degrees() {
        // At -8.7°, penalty is in the 5-12° bracket:
        //   1.0 - (8.7 - 5.0) * 0.0714 = 1.0 - 0.264 = 0.736
        // Base confidence at 8.7° = 0.85 (excellent: >5°)
        // Final: 0.85 * 0.736 ≈ 0.626
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        let pitchRad = -8.7 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)
        XCTAssertNotNil(result)
        XCTAssertEqual(result!.confidence, 0.626, accuracy: 0.05,
            "8.7° pitch should have ~0.63 confidence (relaxed slope penalty)")
    }

    func testNoSlopePenaltyBelow5Degrees() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        // 2° pitch: should NOT have slope penalty (threshold is 5°)
        let pitchRad = -2.0 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)
        XCTAssertNotNil(result)
        // At 2°: baseConfidence is in the 2.0-5.0° bracket:
        // t = (2.0 - 2.0) / 3.0 = 0; base = 0.70 + 0 * 0.15 = 0.70
        // slopePenalty = 1.0 (no penalty: below 5°)
        XCTAssertEqual(result!.confidence, 0.70, accuracy: 0.05)
    }

    func testSlopePenaltyModerateAt15Degrees() {
        // At 15°, penalty is in the 12-20° bracket:
        //   0.50 - (15.0 - 12.0) * 0.04375 = 0.50 - 0.131 = 0.369
        // Base confidence at 15° = 0.85
        // Final: 0.85 * 0.369 ≈ 0.313
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        let pitchRad = -15.0 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)
        XCTAssertNotNil(result)
        XCTAssertEqual(result!.confidence, 0.313, accuracy: 0.05)
    }
}
