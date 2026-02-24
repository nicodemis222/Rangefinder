//
//  DepthKalmanFilterTests.swift
//  RangefinderTests
//
//  Tests for Kalman filter depth prediction and transition smoothing.
//

import XCTest
@testable import Rangefinder

final class DepthKalmanFilterTests: XCTestCase {

    // MARK: - Initialization

    func testFirstMeasurementPassesThrough() {
        var kf = DepthKalmanFilter()
        let result = kf.update(
            measurement: 10.0,
            confidence: 0.8,
            motionState: .stationary,
            timestamp: 1.0
        )
        XCTAssertEqual(result, 10.0, accuracy: 0.01)
    }

    func testPredictBeforeInitReturnsNil() {
        var kf = DepthKalmanFilter()
        XCTAssertNil(kf.predict(at: 1.0))
    }

    func testIsTrackingAfterFirstUpdate() {
        var kf = DepthKalmanFilter()
        XCTAssertFalse(kf.isTracking)
        kf.update(measurement: 5.0, confidence: 0.8, motionState: .stationary, timestamp: 0)
        XCTAssertTrue(kf.isTracking)
    }

    // MARK: - Prediction

    func testPredictStationary() {
        var kf = DepthKalmanFilter()
        // Feed a constant depth — velocity should settle near 0
        for i in 0..<10 {
            kf.update(
                measurement: 20.0,
                confidence: 0.9,
                motionState: .stationary,
                timestamp: Double(i) * 0.033
            )
        }

        // Predict 100ms into the future — should stay near 20
        let predicted = kf.predict(at: 10 * 0.033 + 0.1)
        XCTAssertNotNil(predicted)
        XCTAssertEqual(predicted!, 20.0, accuracy: 1.0, "Stationary target should predict near 20m")
    }

    func testPredictMovingTarget() {
        var kf = DepthKalmanFilter()
        // Feed linearly increasing depth (target moving away at ~1 m/s)
        for i in 0..<20 {
            let t = Double(i) * 0.05  // 50ms intervals
            let depth = 10.0 + t * 1.0  // 1 m/s away
            kf.update(
                measurement: depth,
                confidence: 0.8,
                motionState: .tracking,
                timestamp: t
            )
        }

        // The last measurement was at t=0.95, depth ~10.95
        // Predict at t=1.05 — should extrapolate forward beyond last update
        let lastDepth = kf.currentDepth
        let predicted = kf.predict(at: 1.05)
        XCTAssertNotNil(predicted)
        XCTAssertGreaterThan(predicted!, lastDepth, "Should extrapolate beyond last filtered depth")
        XCTAssertLessThan(predicted!, 12.0, "Should not extrapolate too far")
    }

    // MARK: - Transition Smoothing

    func testSmoothsAbruptTransition() {
        var kf = DepthKalmanFilter()

        // Feed 10m readings for a while
        for i in 0..<10 {
            kf.update(
                measurement: 10.0,
                confidence: 0.9,
                motionState: .stationary,
                timestamp: Double(i) * 0.033
            )
        }

        // Abrupt jump to 15m (source transition)
        let afterJump = kf.update(
            measurement: 15.0,
            confidence: 0.7,
            motionState: .stationary,
            timestamp: 0.5
        )

        // Should NOT snap to 15 immediately — Kalman should smooth it
        XCTAssertLessThan(afterJump, 14.5, "Should not snap to 15m immediately")
        XCTAssertGreaterThan(afterJump, 10.0, "Should move toward 15m")

        // After more measurements at 15m, should converge
        var converged = afterJump
        for i in 1..<10 {
            converged = kf.update(
                measurement: 15.0,
                confidence: 0.8,
                motionState: .stationary,
                timestamp: 0.5 + Double(i) * 0.033
            )
        }
        XCTAssertEqual(converged, 15.0, accuracy: 0.5, "Should converge to 15m")
    }

    // MARK: - Noise Adaptation

    func testLowConfidenceIncreasesNoiseFiltering() {
        var kf = DepthKalmanFilter()

        // Initialize at 10m
        for i in 0..<5 {
            kf.update(measurement: 10.0, confidence: 0.9, motionState: .stationary, timestamp: Double(i) * 0.033)
        }

        // Feed a noisy reading with low confidence
        let filtered = kf.update(
            measurement: 15.0,
            confidence: 0.1,
            motionState: .stationary,
            timestamp: 0.2
        )

        // With low confidence, the filter should barely move
        XCTAssertLessThan(filtered, 12.0, "Low confidence should cause less movement toward measurement")
    }

    func testPanningFollowsFaster() {
        var kf1 = DepthKalmanFilter()
        var kf2 = DepthKalmanFilter()

        // Both start at 10m
        for i in 0..<5 {
            let t = Double(i) * 0.033
            kf1.update(measurement: 10.0, confidence: 0.8, motionState: .stationary, timestamp: t)
            kf2.update(measurement: 10.0, confidence: 0.8, motionState: .stationary, timestamp: t)
        }

        // Jump to 20m — one in stationary, one in panning
        let statResult = kf1.update(measurement: 20.0, confidence: 0.8, motionState: .stationary, timestamp: 0.2)
        let panResult = kf2.update(measurement: 20.0, confidence: 0.8, motionState: .panning, timestamp: 0.2)

        // Panning should follow the change faster (higher process noise = trusts measurements more)
        XCTAssertGreaterThan(panResult, statResult, "Panning should follow changes faster than stationary")
    }

    // MARK: - Reset

    func testResetClearsState() {
        var kf = DepthKalmanFilter()
        kf.update(measurement: 50.0, confidence: 0.9, motionState: .stationary, timestamp: 1.0)
        XCTAssertTrue(kf.isTracking)

        kf.reset()
        XCTAssertFalse(kf.isTracking)
        XCTAssertNil(kf.predict(at: 2.0))
    }

    // MARK: - Edge Cases

    func testLargeDtResetsFilter() {
        var kf = DepthKalmanFilter()
        kf.update(measurement: 10.0, confidence: 0.9, motionState: .stationary, timestamp: 0)

        // 5 seconds later (app was backgrounded)
        let result = kf.update(measurement: 30.0, confidence: 0.8, motionState: .stationary, timestamp: 5.0)
        // Should reset and accept the new measurement directly
        XCTAssertEqual(result, 30.0, accuracy: 0.01, "Should reset on large dt")
    }

    func testZeroMeasurementIgnored() {
        var kf = DepthKalmanFilter()
        kf.update(measurement: 10.0, confidence: 0.9, motionState: .stationary, timestamp: 0)
        let result = kf.update(measurement: 0.0, confidence: 0.5, motionState: .stationary, timestamp: 0.1)
        XCTAssertEqual(result, 10.0, accuracy: 0.5, "Zero measurement should be ignored")
    }
}
