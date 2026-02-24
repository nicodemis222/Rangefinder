//
//  ContinuousCalibratorTests.swift
//  RangefinderTests
//
//  Tests for always-on LiDAR→neural calibration.
//

import XCTest
@testable import Rangefinder

final class ContinuousCalibratorTests: XCTestCase {

    func testIdentityCalibrationOnInit() {
        let cal = ContinuousCalibrator()
        let result = cal.calibrate(5.0)
        // Identity: scale=1, shift=0 → output = input
        XCTAssertEqual(result, 5.0, accuracy: 0.01)
    }

    func testCalibrationFromSamples() {
        let cal = ContinuousCalibrator(maxSamples: 50)

        // Feed samples where lidar = 2 * neural + 1
        for i in 0..<20 {
            let neural = Float(i) + 1.0
            let lidar = 2.0 * neural + 1.0
            cal.ingestSample(
                neuralDepth: neural,
                lidarDepth: lidar,
                confidence: 0.9,
                timestamp: TimeInterval(i)
            )
        }

        // After sufficient samples, calibration should approximate scale=2, shift=1
        let calibrated = cal.calibrate(10.0) // Expected: 2*10+1 = 21
        XCTAssertEqual(calibrated, 21.0, accuracy: 1.0)
    }

    func testInverseDepthCalibration() {
        let cal = ContinuousCalibrator(maxSamples: 50)

        // Feed samples where lidar = 10.0 / neural (inverse depth)
        // neural=10 → lidar=1.0, neural=5 → lidar=2.0, neural=2 → lidar=5.0
        let pairs: [(Float, Float)] = [
            (20.0, 0.5), (10.0, 1.0), (5.0, 2.0),
            (4.0, 2.5), (2.0, 5.0), (1.0, 7.0),  // not exact but close
            (15.0, 0.67), (8.0, 1.25), (3.0, 3.3),
            (2.5, 4.0), (1.5, 6.5), (6.0, 1.7),
        ]

        for (i, pair) in pairs.enumerated() {
            cal.ingestSample(
                neuralDepth: pair.0,
                lidarDepth: pair.1,
                confidence: 0.9,
                timestamp: TimeInterval(i)
            )
        }

        // Should detect inverse model
        XCTAssertEqual(cal.modelType, .inverse, "Should detect inverse depth model")

        // Calibrate a mid-range value
        let calibrated = cal.calibrate(5.0)  // With scale≈10, shift≈0: 10/5 = 2.0
        XCTAssertGreaterThan(calibrated, 0.5)
        XCTAssertLessThan(calibrated, 10.0)
    }

    func testConfidenceIncreasesWithSamples() {
        let cal = ContinuousCalibrator(maxSamples: 50)

        XCTAssertEqual(cal.confidence, 0.0, accuracy: 0.01)

        for i in 0..<10 {
            cal.ingestSample(
                neuralDepth: Float(i + 1),
                lidarDepth: Float(i + 1) * 1.5,
                confidence: 0.9,
                timestamp: TimeInterval(i)
            )
        }

        XCTAssertGreaterThan(cal.confidence, 0.0)
    }

    func testRejectsInvalidSamples() {
        let cal = ContinuousCalibrator()

        // NaN, zero, and low-confidence samples should be rejected
        cal.ingestSample(neuralDepth: .nan, lidarDepth: 3.0, confidence: 0.9, timestamp: 0)
        cal.ingestSample(neuralDepth: 3.0, lidarDepth: .nan, confidence: 0.9, timestamp: 0)
        cal.ingestSample(neuralDepth: 0.0, lidarDepth: 3.0, confidence: 0.9, timestamp: 0)
        cal.ingestSample(neuralDepth: 3.0, lidarDepth: 3.0, confidence: 0.1, timestamp: 0)

        // Confidence should remain at 0 (no valid samples)
        XCTAssertEqual(cal.confidence, 0.0, accuracy: 0.01)
    }

    func testReset() {
        let cal = ContinuousCalibrator()

        for i in 0..<10 {
            cal.ingestSample(
                neuralDepth: Float(i + 1),
                lidarDepth: Float(i + 1) * 2.0,
                confidence: 0.9,
                timestamp: TimeInterval(i)
            )
        }

        XCTAssertGreaterThan(cal.confidence, 0.0)

        cal.reset()
        XCTAssertEqual(cal.confidence, 0.0, accuracy: 0.01)

        // After reset, should be identity again
        XCTAssertEqual(cal.calibrate(5.0), 5.0, accuracy: 0.01)
    }
}
