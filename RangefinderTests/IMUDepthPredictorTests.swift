//
//  IMUDepthPredictorTests.swift
//  RangefinderTests
//
//  Tests for IMU-based depth prediction between inference frames.
//

import XCTest
import simd
@testable import Rangefinder

final class IMUDepthPredictorTests: XCTestCase {

    // MARK: - Helpers

    /// Create a camera transform matrix at a given position looking along -Z.
    private func makeTransform(x: Float = 0, y: Float = 0, z: Float = 0) -> simd_float4x4 {
        // Identity rotation (camera looking along -Z)
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(x, y, z, 1)
        return transform
    }

    // MARK: - Basic Motion

    func testNoMotionNoPrediction() {
        var predictor = IMUDepthPredictor()

        // Two identical poses
        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0.033)

        XCTAssertEqual(predictor.depthAdjustment, 0.0, accuracy: 0.01)
    }

    func testForwardMotionDecreasesDepth() {
        var predictor = IMUDepthPredictor()

        // Camera moves forward (negative Z in default orientation = forward)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: -1.0), timestamp: 0.1)

        // Moving forward should result in negative depth adjustment (getting closer)
        let predicted = predictor.predictDepth(from: 10.0)
        XCTAssertLessThan(predicted, 10.0, "Moving forward should decrease predicted depth")
    }

    func testBackwardMotionIncreasesDepth() {
        var predictor = IMUDepthPredictor()

        // Camera moves backward (positive Z)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: 1.0), timestamp: 0.1)

        let predicted = predictor.predictDepth(from: 10.0)
        XCTAssertGreaterThan(predicted, 10.0, "Moving backward should increase predicted depth")
    }

    // MARK: - Measurement Reset

    func testOnNewMeasurementResetsAccumulation() {
        var predictor = IMUDepthPredictor()

        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: -1.0), timestamp: 0.1)

        XCTAssertNotEqual(predictor.depthAdjustment, 0, "Should have accumulated displacement")

        predictor.onNewMeasurement()
        XCTAssertEqual(predictor.depthAdjustment, 0, accuracy: 0.001, "Should reset after new measurement")
    }

    // MARK: - Displacement Clamping

    func testDisplacementIsClamped() {
        var predictor = IMUDepthPredictor()

        // Huge jump (teleportation / tracking loss)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: -100), timestamp: 0.1)

        // Should be clamped to maxDisplacement (3m)
        let predicted = predictor.predictDepth(from: 10.0)
        XCTAssertGreaterThan(predicted, 6.5, "Displacement should be clamped")
    }

    // MARK: - Minimum Depth

    func testPredictionNeverGoesNegative() {
        var predictor = IMUDepthPredictor()

        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: -2.0), timestamp: 0.1)

        // Even with large forward motion on a close target, depth stays positive
        let predicted = predictor.predictDepth(from: 0.5)
        XCTAssertGreaterThanOrEqual(predicted, 0.1, "Depth should never go below 0.1m")
    }

    // MARK: - Reset

    func testResetClearsState() {
        var predictor = IMUDepthPredictor()

        predictor.updateCameraPose(cameraTransform: makeTransform(z: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(z: -1.0), timestamp: 0.1)

        predictor.reset()
        XCTAssertEqual(predictor.depthAdjustment, 0, accuracy: 0.001)
        XCTAssertEqual(predictor.currentForwardVelocity, 0, accuracy: 0.001)
    }

    // MARK: - Lateral Motion

    func testLateralMotionMinimalDepthEffect() {
        var predictor = IMUDepthPredictor()

        // Move sideways (X axis) — should have minimal depth effect
        // since depth is along the camera's forward (Z) axis
        predictor.updateCameraPose(cameraTransform: makeTransform(x: 0), timestamp: 0)
        predictor.updateCameraPose(cameraTransform: makeTransform(x: 2.0), timestamp: 0.1)

        let adjustment = abs(predictor.depthAdjustment)
        XCTAssertLessThan(adjustment, 0.5, "Lateral motion should minimally affect depth prediction")
    }
}
