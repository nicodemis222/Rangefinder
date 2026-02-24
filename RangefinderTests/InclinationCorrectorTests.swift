//
//  InclinationCorrectorTests.swift
//  RangefinderTests
//
//  Tests for cosine inclination correction.
//

import XCTest
@testable import Rangefinder

final class InclinationCorrectorTests: XCTestCase {

    func testLevelNoCorrection() {
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 500.0,
            pitchRadians: 0.0
        )
        XCTAssertEqual(adjusted, 500.0, accuracy: 0.01)
        XCTAssertEqual(factor, 1.0, accuracy: 0.01)
    }

    func testSmallAngleSkipped() {
        // 1 degree ≈ 0.017 rad — below threshold
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 500.0,
            pitchRadians: 0.017
        )
        XCTAssertEqual(adjusted, 500.0, accuracy: 0.01)
        XCTAssertEqual(factor, 1.0, accuracy: 0.01)
    }

    func test30DegreeCorrection() {
        // cos(30°) ≈ 0.866
        let pitchRad = 30.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 500.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, cos(pitchRad), accuracy: 0.001)
        XCTAssertEqual(adjusted, 500.0 * cos(pitchRad), accuracy: 0.01)
        XCTAssertLessThan(adjusted, 500.0)
    }

    func test45DegreeCorrection() {
        // cos(45°) ≈ 0.707
        let pitchRad = 45.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(adjusted, 1000.0 * cos(pitchRad), accuracy: 0.1)
        XCTAssertEqual(factor, cos(pitchRad), accuracy: 0.001)
    }

    func testNegativeAngleSameCorrection() {
        // Downhill should correct same as uphill
        let pitchRad = 20.0 * .pi / 180.0
        let (adjustedUp, factorUp) = InclinationCorrector.correct(
            lineOfSightRange: 300.0,
            pitchRadians: pitchRad
        )
        let (adjustedDown, factorDown) = InclinationCorrector.correct(
            lineOfSightRange: 300.0,
            pitchRadians: -pitchRad
        )
        XCTAssertEqual(adjustedUp, adjustedDown, accuracy: 0.01)
        XCTAssertEqual(factorUp, factorDown, accuracy: 0.001)
    }
}
