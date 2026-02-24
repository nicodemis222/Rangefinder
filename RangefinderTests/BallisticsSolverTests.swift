//
//  BallisticsSolverTests.swift
//  RangefinderTests
//
//  Tests for the ballistic holdover solver.
//  Reference values from published .308 Win 168gr SMK ballistic tables
//  at 2650 fps muzzle velocity, 100-yard zero, 1.5" sight height.
//
//  Published reference data (G7 model, approximate):
//    200yd: ~3.5" drop / 0.5 mil     500yd: ~52" / 2.9 mil
//    300yd: ~12.5" / 1.2 mil        1000yd: ~370" / 10.3 mil
//
//  Our G1 drag model systematically over-predicts drop for
//  boat-tail bullets by ~15-20% vs G7 (which better matches
//  their form factor). G1 is used because it's the universal
//  standard that works across all bullet shapes with published BC.
//
//  Test tolerances are set to ±25% to accommodate the G1-vs-G7
//  discrepancy. For a field rangefinder holdover indicator, this
//  accuracy is adequate — the reticle subtension itself is only
//  accurate to ~0.1 mil, and real-world conditions (wind, altitude,
//  temperature) introduce larger variations.
//

import XCTest
@testable import Rangefinder

final class BallisticsSolverTests: XCTestCase {

    private var solver: BallisticsSolver!

    override func setUp() {
        super.setUp()
        solver = BallisticsSolver()
        solver.selectedCaliber = .cal308
        solver.zeroDistance = 100  // 100 yards
        solver.isEnabled = true
    }

    // MARK: - .308 Win 168gr SMK Reference Values

    func testZeroDistanceNoHoldover() {
        // At zero distance (100 yards), holdover should be ~0
        let result = solver.calculateHoldover(targetDistance: 91.44, useMetric: false)
        // 91.44m = 100 yards
        XCTAssertLessThan(result.holdoverMils, 0.15,
            "Holdover at zero distance should be negligible, got \(result.holdoverMils) mil")
    }

    func testHoldoverAt200Yards() {
        // 200 yards = 182.88m
        // G7 reference: ~3.5" / 0.5 mil.  G1 model: ~4" / 0.6 mil (+15%)
        let result = solver.calculateHoldover(targetDistance: 182.88, useMetric: false)
        XCTAssertTrue(result.holdHigh,
            "Beyond zero, should hold high (bullet drops below sight line)")
        XCTAssertEqual(result.holdoverMils, 0.6, accuracy: 0.3,
            "200yd holdover should be ~0.5-0.7 mil, got \(result.holdoverMils)")
        XCTAssertEqual(abs(result.holdoverInches), 4.0, accuracy: 2.5,
            "200yd holdover should be ~3.5-5\", got \(abs(result.holdoverInches))\"")
    }

    func testHoldoverAt300Yards() {
        // 300 yards = 274.32m
        // G7 reference: ~12.5" / 1.2 mil.  G1 model: ~15" / 1.4 mil (+17%)
        let result = solver.calculateHoldover(targetDistance: 274.32, useMetric: false)
        XCTAssertTrue(result.holdHigh, "Should hold high at 300yd")
        XCTAssertEqual(result.holdoverMils, 1.4, accuracy: 0.5,
            "300yd holdover should be ~1.2-1.5 mil, got \(result.holdoverMils)")
        XCTAssertEqual(abs(result.holdoverInches), 15.0, accuracy: 6.0,
            "300yd holdover should be ~12-18\", got \(abs(result.holdoverInches))\"")
    }

    func testHoldoverAt500Yards() {
        // 500 yards = 457.2m
        // G7 reference: ~52" / 2.9 mil.  G1 model: ~62-68" / 3.4-3.8 mil (+17-30%)
        let result = solver.calculateHoldover(targetDistance: 457.2, useMetric: false)
        XCTAssertTrue(result.holdHigh, "Should hold high at 500yd")
        XCTAssertEqual(result.holdoverMils, 3.5, accuracy: 1.2,
            "500yd holdover should be ~2.9-4.0 mil, got \(result.holdoverMils)")
        XCTAssertEqual(abs(result.holdoverInches), 62.0, accuracy: 20.0,
            "500yd holdover should be ~50-75\", got \(abs(result.holdoverInches))\"")
    }

    func testHoldoverAt1000Yards() {
        // 1000 yards = 914.4m
        // G7 reference: ~370" / 10.3 mil.  G1 model: ~400" / 11 mil (+8%)
        let result = solver.calculateHoldover(targetDistance: 914.4, useMetric: false)
        XCTAssertTrue(result.holdHigh, "Should hold high at 1000yd")
        XCTAssertEqual(result.holdoverMils, 11.0, accuracy: 3.0,
            "1000yd holdover should be ~9-13 mil, got \(result.holdoverMils)")
        XCTAssertEqual(abs(result.holdoverInches), 400.0, accuracy: 120.0,
            "1000yd holdover should be ~300-500\", got \(abs(result.holdoverInches))\"")
    }

    // MARK: - Hold Direction Logic

    func testHoldLowInsideZero() {
        // At 50 yards (45.72m), bullet is still rising — should hold LOW
        let result = solver.calculateHoldover(targetDistance: 45.72, useMetric: false)
        // At close range inside zero, bullet may be above or near sight line
        // Depending on exact trajectory, this could be near zero or slightly low
        if result.isSignificant {
            XCTAssertFalse(result.holdHigh,
                "Inside zero distance, should hold low (bullet above sight line)")
        }
    }

    func testHoldHighBeyondZero() {
        // At 400 yards (365.76m), bullet is well below sight line
        let result = solver.calculateHoldover(targetDistance: 365.76, useMetric: false)
        XCTAssertTrue(result.holdHigh,
            "Well beyond zero, must hold high")
        XCTAssertGreaterThan(result.holdoverMils, 1.0,
            "400yd should have significant holdover")
    }

    // MARK: - Metric Mode

    func testMetricZeroDistance() {
        // Set zero to 100m (instead of 100yd)
        solver.zeroDistance = 100
        // Target at 500m
        let result = solver.calculateHoldover(targetDistance: 500, useMetric: true)
        XCTAssertTrue(result.holdHigh, "Should hold high at 500m")
        XCTAssertGreaterThan(result.holdoverMils, 2.0,
            "500m with 100m zero should have significant holdover")
        // Check CM output is populated
        XCTAssertGreaterThan(result.holdoverCM, 0,
            "CM value should be populated")
    }

    // MARK: - Multi-Unit Display

    func testDisplayFormats() {
        let result = solver.calculateHoldover(targetDistance: 457.2, useMetric: false)
        XCTAssertFalse(result.displayMils.isEmpty, "Mils display should not be empty")
        XCTAssertFalse(result.displayInches.isEmpty, "Inches display should not be empty")
        XCTAssertFalse(result.displayCM.isEmpty, "CM display should not be empty")
        XCTAssertEqual(result.unitLabel, "IN", "Imperial mode should show IN")
        XCTAssertEqual(result.directionText, "UP", "Hold high should show UP")
    }

    func testMetricDisplayFormats() {
        let result = solver.calculateHoldover(targetDistance: 457.2, useMetric: true)
        XCTAssertEqual(result.unitLabel, "CM", "Metric mode should show CM")
    }

    // MARK: - 5.56 NATO Comparison

    func test556At300Yards() {
        // 5.56 NATO 77gr @ 2750fps has more drop than .308 at same distance
        // (lower BC = more drag)
        solver.selectedCaliber = .cal556
        let result556 = solver.calculateHoldover(targetDistance: 274.32, useMetric: false)

        solver.selectedCaliber = .cal308
        let result308 = solver.calculateHoldover(targetDistance: 274.32, useMetric: false)

        XCTAssertGreaterThan(result556.holdoverMils, result308.holdoverMils,
            "5.56 (lower BC) should have more holdover than .308 at 300yd")
    }

    // MARK: - Edge Cases

    func testDisabledReturnsZero() {
        solver.isEnabled = false
        let result = solver.calculateHoldover(targetDistance: 500, useMetric: false)
        XCTAssertEqual(result.holdoverMils, 0)
        XCTAssertFalse(result.isSignificant)
    }

    func testZeroDistanceInput() {
        let result = solver.calculateHoldover(targetDistance: 0, useMetric: false)
        XCTAssertEqual(result.holdoverMils, 0)
    }

    func testNegativeDistanceInput() {
        let result = solver.calculateHoldover(targetDistance: -100, useMetric: false)
        XCTAssertEqual(result.holdoverMils, 0)
    }

    // MARK: - Monotonicity

    func testHoldoverIncreasesWithDistance() {
        // Holdover should increase monotonically beyond zero
        var lastMils = 0.0
        let distances = [200.0, 300.0, 400.0, 500.0, 600.0, 800.0, 1000.0]
        for distYards in distances {
            let distMeters = distYards / 1.09361
            let result = solver.calculateHoldover(targetDistance: distMeters, useMetric: false)
            XCTAssertGreaterThanOrEqual(result.holdoverMils, lastMils,
                "Holdover should increase: \(distYards)yd gave \(result.holdoverMils) mil, previous was \(lastMils)")
            lastMils = result.holdoverMils
        }
    }

    // MARK: - 6.5 Creedmoor (high BC, should have less drop)

    func test65CMHasLessDropThan308() {
        // 6.5 CM has higher BC (0.564 vs 0.462), should have less holdover
        solver.selectedCaliber = .cal65CM
        let result65 = solver.calculateHoldover(targetDistance: 457.2, useMetric: false)

        solver.selectedCaliber = .cal308
        let result308 = solver.calculateHoldover(targetDistance: 457.2, useMetric: false)

        XCTAssertLessThan(result65.holdoverMils, result308.holdoverMils,
            "6.5 CM (higher BC) should have less holdover than .308 at 500yd")
    }
}
