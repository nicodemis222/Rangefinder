//
//  StadiametricRangingTests.swift
//  RangefinderTests
//
//  Tests for the stadiametric (manual bracket) ranging mode.
//  Validates the pinhole formula: R = (knownSize * focalLength) / pixelSize.
//

import XCTest
@testable import Rangefinder

final class StadiametricRangingTests: XCTestCase {

    // MARK: - Pinhole Formula

    func testPinholeFormulaBasic() {
        // Person (1.8m) at 100m with f=2160px:
        // pixelSize = (1.8 * 2160) / 100 = 38.88px
        // Reverse: R = (1.8 * 2160) / 38.88 = 100m
        let input = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 38.88,
            focalLengthPixels: 2160
        )
        XCTAssertEqual(input.computedRange, 100.0, accuracy: 0.1)
    }

    func testPinholeFormulaCloseRange() {
        // Person (1.8m) at 10m with f=2160px:
        // pixelSize = (1.8 * 2160) / 10 = 388.8px
        let input = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 388.8,
            focalLengthPixels: 2160
        )
        XCTAssertEqual(input.computedRange, 10.0, accuracy: 0.1)
    }

    func testPinholeFormulaLongRange() {
        // Vehicle (1.5m) at 500m with f=2160px:
        // pixelSize = (1.5 * 2160) / 500 = 6.48px
        let input = StadiametricInput(
            knownSizeMeters: 1.5,
            pixelSize: 6.48,
            focalLengthPixels: 2160
        )
        XCTAssertEqual(input.computedRange, 500.0, accuracy: 1.0)
    }

    func testPinholeFormulaDeer() {
        // Deer (1.0m) at 200m with f=2160px:
        // pixelSize = (1.0 * 2160) / 200 = 10.8px
        let input = StadiametricInput(
            knownSizeMeters: 1.0,
            pixelSize: 10.8,
            focalLengthPixels: 2160
        )
        XCTAssertEqual(input.computedRange, 200.0, accuracy: 0.5)
    }

    // MARK: - Edge Cases

    func testZeroPixelSize() {
        let input = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 0,
            focalLengthPixels: 2160
        )
        XCTAssertEqual(input.computedRange, 0, "Zero pixel size should return zero")
    }

    func testVerySmallPixelSize() {
        // Very far target: 1px bracket
        let input = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 1.0,
            focalLengthPixels: 2160
        )
        // R = 1.8 * 2160 / 1 = 3888m
        XCTAssertEqual(input.computedRange, 3888.0, accuracy: 0.1)
    }

    func testVeryLargePixelSize() {
        // Very close target: 1000px bracket
        let input = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 1000,
            focalLengthPixels: 2160
        )
        // R = 1.8 * 2160 / 1000 = 3.888m
        XCTAssertEqual(input.computedRange, 3.888, accuracy: 0.01)
    }

    // MARK: - Target Presets

    func testTargetPresetsExist() {
        let presets = AppConfiguration.stadiametricTargetPresets
        XCTAssertGreaterThan(presets.count, 0, "Should have at least one preset")

        // Verify person preset
        let person = presets.first { $0.label == "PERSON" }
        XCTAssertNotNil(person, "PERSON preset should exist")
        XCTAssertEqual(person!.heightMeters, 1.8, accuracy: 0.01)

        // Verify vehicle preset
        let vehicle = presets.first { $0.label == "VEHICLE" }
        XCTAssertNotNil(vehicle, "VEHICLE preset should exist")
        XCTAssertEqual(vehicle!.heightMeters, 1.5, accuracy: 0.01)

        // Verify deer preset
        let deer = presets.first { $0.label == "DEER" }
        XCTAssertNotNil(deer, "DEER preset should exist")
        XCTAssertEqual(deer!.heightMeters, 1.0, accuracy: 0.01)
    }

    // MARK: - Different Focal Lengths

    func testDifferentFocalLengths() {
        // Same target at same distance with different focal lengths
        // should give different pixel sizes but same computed range
        let targetSize = 1.8
        let trueRange = 100.0

        // Main camera: f=2160px
        let px1 = (targetSize * 2160) / trueRange
        let input1 = StadiametricInput(
            knownSizeMeters: targetSize,
            pixelSize: px1,
            focalLengthPixels: 2160
        )

        // Telephoto camera: f=5000px
        let px2 = (targetSize * 5000) / trueRange
        let input2 = StadiametricInput(
            knownSizeMeters: targetSize,
            pixelSize: px2,
            focalLengthPixels: 5000
        )

        XCTAssertEqual(input1.computedRange, 100.0, accuracy: 0.1)
        XCTAssertEqual(input2.computedRange, 100.0, accuracy: 0.1)
        XCTAssertGreaterThan(px2, px1, "Telephoto should have larger pixel size for same range")
    }

    // MARK: - Sensitivity Analysis

    func testPixelErrorSensitivity() {
        // At 200m, a 1-pixel error in bracket size changes range by:
        // True pixelSize for person at 200m: (1.8 * 2160) / 200 = 19.44px
        let truePixels = 19.44
        let rangeTrue = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: truePixels,
            focalLengthPixels: 2160
        ).computedRange

        let rangePlus1 = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: truePixels + 1.0,
            focalLengthPixels: 2160
        ).computedRange

        let rangeMinus1 = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: truePixels - 1.0,
            focalLengthPixels: 2160
        ).computedRange

        // 1px error at ~19.44px bracket ≈ 5% range change
        let errorPctPlus = abs(rangePlus1 - rangeTrue) / rangeTrue * 100
        let errorPctMinus = abs(rangeMinus1 - rangeTrue) / rangeTrue * 100

        XCTAssertLessThan(errorPctPlus, 10,
            "1px error at 200m should cause <10% range error")
        XCTAssertLessThan(errorPctMinus, 10,
            "1px error at 200m should cause <10% range error")
    }
}
