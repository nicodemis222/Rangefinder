//
//  ReticleGeometryTests.swift
//  RangefinderTests
//
//  Tests for FFP reticle geometry calculations.
//

import XCTest
@testable import Rangefinder

final class ReticleGeometryTests: XCTestCase {

    func testPixelsPerMilIncreasesWithZoom() {
        let ppm1x = ReticleGeometry.pixelsPerMil(screenWidth: 430, zoomFactor: 1.0)
        let ppm5x = ReticleGeometry.pixelsPerMil(screenWidth: 430, zoomFactor: 5.0)
        let ppm10x = ReticleGeometry.pixelsPerMil(screenWidth: 430, zoomFactor: 10.0)

        XCTAssertGreaterThan(ppm5x, ppm1x)
        XCTAssertGreaterThan(ppm10x, ppm5x)

        // FFP: ppm should scale linearly with zoom
        XCTAssertEqual(ppm5x / ppm1x, 5.0, accuracy: 0.01)
        XCTAssertEqual(ppm10x / ppm1x, 10.0, accuracy: 0.01)
    }

    func testBaseFOVMils() {
        // 78 deg * 17.453 = ~1361 mils
        XCTAssertEqual(ReticleGeometry.baseFOVMils, 78.0 * 17.453292519943, accuracy: 0.01)
    }

    func testMilToPixels() {
        let screenWidth: CGFloat = 430
        let zoomFactor: CGFloat = 1.0

        let oneMilPx = ReticleGeometry.milToPixels(1.0, screenWidth: screenWidth, zoomFactor: zoomFactor)
        let twoMilPx = ReticleGeometry.milToPixels(2.0, screenWidth: screenWidth, zoomFactor: zoomFactor)

        // 2 mils should be exactly 2x 1 mil
        XCTAssertEqual(twoMilPx, oneMilPx * 2, accuracy: 0.001)

        // At 1x zoom, 1 mil should be a small fraction of screen width
        XCTAssertGreaterThan(oneMilPx, 0)
        XCTAssertLessThan(oneMilPx, screenWidth / 10)
    }
}
