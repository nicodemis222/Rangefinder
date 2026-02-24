//
//  MotionAwareSmootherTests.swift
//  RangefinderTests
//
//  Tests for motion-aware temporal smoothing.
//

import XCTest
@testable import Rangefinder

final class MotionAwareSmootherTests: XCTestCase {

    func testFirstValuePassesThrough() {
        var smoother = MotionAwareSmoother()
        let result = smoother.smooth(newValue: 100.0, motionState: .stationary)
        XCTAssertEqual(result, 100.0, accuracy: 0.01)
    }

    func testStationarySmoothsHeavily() {
        var smoother = MotionAwareSmoother()
        _ = smoother.smooth(newValue: 100.0, motionState: .stationary)

        // Feed a new value — should not jump immediately
        let result = smoother.smooth(newValue: 200.0, motionState: .stationary)
        XCTAssertLessThan(result, 150.0, "Stationary should smooth heavily, not jump to 200")
        XCTAssertGreaterThan(result, 100.0)
    }

    func testPanningSmootsLightly() {
        var smoother = MotionAwareSmoother()
        _ = smoother.smooth(newValue: 100.0, motionState: .panning)

        let result = smoother.smooth(newValue: 200.0, motionState: .panning)
        // Panning alpha ~0.6, so result should be closer to 200 than 100
        XCTAssertGreaterThan(result, 140.0, "Panning should follow changes quickly")
    }

    func testDiscontinuityDetection() {
        var smoother = MotionAwareSmoother()
        // Start at a short-range value where the discontinuity threshold is
        // the base 30% and only 4 confirmation frames are needed.
        _ = smoother.smooth(newValue: 10.0, motionState: .stationary)

        // Feed several frames at 20 (100% change = well above 30% threshold)
        // Need 4 confirmation frames (confirmationFrames = 4)
        let r1 = smoother.smooth(newValue: 20.0, motionState: .stationary)
        let r2 = smoother.smooth(newValue: 20.0, motionState: .stationary)
        let r3 = smoother.smooth(newValue: 20.0, motionState: .stationary)
        let r4 = smoother.smooth(newValue: 20.0, motionState: .stationary)

        // After 4 frames confirming, should snap to new value
        XCTAssertEqual(r4, 20.0, accuracy: 1.0, "Should snap after 4 consistent frames")
        _ = r1
        _ = r2
        _ = r3
    }

    func testConfidenceHysteresis() {
        var smoother = MotionAwareSmoother()

        let c1 = smoother.smoothConfidence(0.9)
        XCTAssertEqual(c1, 0.9, accuracy: 0.01)

        // Drop to low confidence — should decay slowly
        let c2 = smoother.smoothConfidence(0.2)
        XCTAssertGreaterThan(c2, 0.5, "Confidence should not drop immediately")
    }

    func testResetClearsState() {
        var smoother = MotionAwareSmoother()
        _ = smoother.smooth(newValue: 500.0, motionState: .stationary)
        smoother.reset()

        let result = smoother.smooth(newValue: 100.0, motionState: .stationary)
        XCTAssertEqual(result, 100.0, accuracy: 0.01, "After reset, first value should pass through")
    }
}
