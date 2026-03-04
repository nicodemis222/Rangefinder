//
//  SteepInclinationTests.swift
//  RangefinderTests
//
//  Tests for phone inclination angles up to ±75° and beyond.
//
//  At steep angles the rangefinder pipeline must:
//  1. Apply correct cosine correction (cos(75°) ≈ 0.259)
//  2. Reject geometric ground-plane estimates (distance < 5m at steep pitch)
//  3. Maintain correction symmetry (uphill = downhill)
//  4. Degrade gracefully toward vertical (89°)
//  5. Produce valid fusion results with extreme inclination correction
//
//  Reference values:
//    cos(60°) = 0.500     cos(70°) = 0.342     cos(75°) = 0.259
//    cos(80°) = 0.174     cos(85°) = 0.087     cos(89°) = 0.017
//
//  At 75° with 500m LOS → adjusted = 129.4m (74% reduction)
//  At 75° with 1000m LOS → adjusted = 258.8m
//

import XCTest
@testable import Rangefinder

// MARK: - Inclination Corrector at Steep Angles

final class SteepInclinationCorrectorTests: XCTestCase {

    // MARK: - Core 75° Tests

    func test75DegreeCorrection() {
        // cos(75°) ≈ 0.2588
        let pitchRad = 75.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 500.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, cos(pitchRad), accuracy: 0.001,
            "Correction factor at 75° should be cos(75°) ≈ 0.259")
        XCTAssertEqual(adjusted, 500.0 * cos(pitchRad), accuracy: 0.1,
            "500m LOS at 75° should adjust to ~129.4m")
        XCTAssertLessThan(adjusted, 130.0,
            "Adjusted range should be < 130m at 75°")
        XCTAssertGreaterThan(adjusted, 128.0,
            "Adjusted range should be > 128m at 75°")
    }

    func test75DegreeNegativeCorrection() {
        // -75° (looking steeply downward) should give same correction
        let pitchRad = -75.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 500.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, cos(75.0 * .pi / 180.0), accuracy: 0.001,
            "Negative 75° should produce same factor as positive 75°")
        XCTAssertEqual(adjusted, 500.0 * cos(75.0 * .pi / 180.0), accuracy: 0.1)
    }

    func test75DegreeSymmetry() {
        // Up and down at 75° must produce identical results
        let pitch75 = 75.0 * .pi / 180.0
        let (adjUp, facUp) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: pitch75
        )
        let (adjDown, facDown) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: -pitch75
        )
        XCTAssertEqual(adjUp, adjDown, accuracy: 0.01,
            "75° uphill and downhill must produce identical adjusted range")
        XCTAssertEqual(facUp, facDown, accuracy: 0.0001,
            "Correction factors must be symmetric")
    }

    // MARK: - Sweep from 60° to 85°

    func test60DegreeCorrection() {
        // cos(60°) = 0.500 exactly
        let pitchRad = 60.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, 0.5, accuracy: 0.001)
        XCTAssertEqual(adjusted, 500.0, accuracy: 0.5)
    }

    func test70DegreeCorrection() {
        // cos(70°) ≈ 0.342
        let pitchRad = 70.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, cos(pitchRad), accuracy: 0.001)
        XCTAssertEqual(adjusted, 1000.0 * cos(pitchRad), accuracy: 0.5)
    }

    func test80DegreeCorrection() {
        // cos(80°) ≈ 0.174
        let pitchRad = 80.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, cos(pitchRad), accuracy: 0.001)
        XCTAssertEqual(adjusted, 1000.0 * cos(pitchRad), accuracy: 0.5)
        XCTAssertLessThan(adjusted, 175.0, "80° should reduce 1000m to ~174m")
    }

    func test85DegreeCorrection() {
        // cos(85°) ≈ 0.087
        let pitchRad = 85.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, cos(pitchRad), accuracy: 0.001)
        XCTAssertEqual(adjusted, 1000.0 * cos(pitchRad), accuracy: 0.5)
        XCTAssertLessThan(adjusted, 90.0, "85° should reduce 1000m to ~87m")
    }

    func test89DegreeNearVertical() {
        // cos(89°) ≈ 0.0175 — nearly vertical
        let pitchRad = 89.0 * .pi / 180.0
        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0,
            pitchRadians: pitchRad
        )
        XCTAssertEqual(factor, cos(pitchRad), accuracy: 0.001)
        XCTAssertLessThan(adjusted, 20.0, "89° should reduce 1000m to ~17.5m")
        XCTAssertGreaterThan(adjusted, 15.0)
    }

    // MARK: - Monotonicity

    func testCorrectionFactorDecreasesMonotonically() {
        // Correction factor must decrease monotonically from 0° to 90°
        var lastFactor = 1.0
        let angles: [Double] = [3, 10, 20, 30, 45, 60, 70, 75, 80, 85, 89]

        for deg in angles {
            let pitchRad = deg * .pi / 180.0
            let (_, factor) = InclinationCorrector.correct(
                lineOfSightRange: 500.0,
                pitchRadians: pitchRad
            )
            XCTAssertLessThan(factor, lastFactor,
                "Factor at \(deg)° (\(factor)) must be < factor at previous angle (\(lastFactor))")
            lastFactor = factor
        }
    }

    func testAdjustedRangeDecreasesMonotonically() {
        // With constant LOS range, adjusted range must shrink as angle increases
        let losRange = 500.0
        var lastAdjusted = losRange
        let angles: [Double] = [3, 10, 20, 30, 45, 60, 70, 75, 80, 85, 89]

        for deg in angles {
            let pitchRad = deg * .pi / 180.0
            let (adjusted, _) = InclinationCorrector.correct(
                lineOfSightRange: losRange,
                pitchRadians: pitchRad
            )
            XCTAssertLessThan(adjusted, lastAdjusted,
                "Adjusted range at \(deg)° (\(adjusted)) must be < at previous angle (\(lastAdjusted))")
            lastAdjusted = adjusted
        }
    }

    // MARK: - Multiple LOS Ranges at 75°

    func test75DegreeAtVariousRanges() {
        // At 75°, correction factor is constant regardless of LOS range
        let pitchRad = 75.0 * .pi / 180.0
        let expectedFactor = cos(pitchRad)
        let losRanges: [Double] = [10, 50, 100, 500, 1000, 2000]

        for los in losRanges {
            let (adjusted, factor) = InclinationCorrector.correct(
                lineOfSightRange: los,
                pitchRadians: pitchRad
            )
            XCTAssertEqual(factor, expectedFactor, accuracy: 0.001,
                "Factor should be constant at 75° regardless of LOS range (\(los)m)")
            XCTAssertEqual(adjusted, los * expectedFactor, accuracy: 0.1,
                "Adjusted = LOS × cos(75°) for LOS=\(los)m")
        }
    }

    // MARK: - Format Display at Extreme Angles

    func testFormatAngleAt75Degrees() {
        let formatted = InclinationCorrector.formatAngle(75.0)
        XCTAssertEqual(formatted, "+75.0°")
    }

    func testFormatAngleAtNeg75Degrees() {
        let formatted = InclinationCorrector.formatAngle(-75.0)
        XCTAssertEqual(formatted, "-75.0°")
    }

    func testFormatCorrectionFactorAt75Degrees() {
        let factor = cos(75.0 * .pi / 180.0)
        let formatted = InclinationCorrector.formatCorrectionFactor(factor)
        XCTAssertTrue(formatted.contains("0.259"), "Should format as ×0.259, got \(formatted)")
    }
}

// MARK: - Geometric Range Estimator at Steep Angles

final class SteepGeometricRangeTests: XCTestCase {

    func testLookingDown75DegreesReturnsResult() {
        // At -75° (looking steeply down): D = 1.5 / tan(75°) = 0.40m
        // This is below minGeometricRange (5m), so it may return nil
        // OR it may return a result with very low confidence due to slope penalty
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        let pitchRad = -75.0 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)

        // At 75° below horizontal: D = 1.5/tan(75°) ≈ 0.40m < minGeometricRange (5m)
        // Should return nil because distance < 5m
        XCTAssertNil(result,
            "At -75° with 1.5m height, geometric distance ≈ 0.40m is below 5m minimum")
    }

    func testLookingUp75DegreesReturnsNil() {
        // Looking up should always return nil (only valid when looking down)
        let estimator = GeometricRangeEstimator()
        let result = estimator.estimate(pitchRadians: 75.0 * .pi / 180.0)
        XCTAssertNil(result, "Looking up at 75° should return nil")
    }

    func testSlopePenaltyAt75Degrees() {
        // After the 45° hard pitch gate, the geometric estimator rejects all
        // estimates at >= 45° pitch — even with a tall camera height that would
        // produce a valid distance. This is correct: at 75° the flat-ground
        // assumption (D = h/tan(θ)) is universally invalid.
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 30.0  // Building / observation tower
        let pitchRad = -75.0 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)

        XCTAssertNil(result,
            "75° exceeds the 45° pitch gate — geometric should return nil regardless of camera height")
    }

    func testSlopePenaltyFloorBeyond20Degrees() {
        // Slope penalty is constant at 0.10 for all angles > 20°.
        // After the 45° hard gate, only angles < 45° produce estimates.
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 100.0  // Tall structure so distances stay valid

        // Angles below the 45° gate that are still > 20° (slope penalty floor)
        let validSteepAngles: [Double] = [25, 35, 44]
        for deg in validSteepAngles {
            let pitchRad = -deg * .pi / 180.0
            let result = estimator.estimate(pitchRadians: pitchRad)
            XCTAssertNotNil(result, "At \(deg)° with 100m height, should produce valid estimate")
            if let r = result {
                // Base confidence = 0.85 (all > 5°)
                // Slope penalty = 0.10 (all > 20°)
                XCTAssertEqual(r.confidence, 0.085, accuracy: 0.02,
                    "At \(deg)° (>20°), confidence should be 0.85 × 0.10 = 0.085")
            }
        }

        // Angles at or above the 45° gate return nil
        let rejectedAngles: [Double] = [45, 60, 75]
        for deg in rejectedAngles {
            let pitchRad = -deg * .pi / 180.0
            let result = estimator.estimate(pitchRadians: pitchRad)
            XCTAssertNil(result,
                "At \(deg)° (>= 45° gate), geometric should return nil")
        }
    }

    func testGeometricReturnsNilForStandardHeight() {
        // At 1.5m camera height, all steep angles (>~17°) produce D < 5m
        // atan(1.5/5) ≈ 16.7°, so anything steeper should return nil
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5

        let steepAngles: [Double] = [60, 65, 70, 75, 80, 85]
        for deg in steepAngles {
            let pitchRad = -deg * .pi / 180.0
            let result = estimator.estimate(pitchRadians: pitchRad)
            XCTAssertNil(result,
                "At 1.5m height and -\(deg)°, geometric should return nil (D < 5m)")
        }
    }
}

// MARK: - Monte Carlo Fusion at Steep Inclination

final class SteepInclinationFusionTests: XCTestCase {

    // MARK: - Inclination Correction Applied to Fused Depth

    func testFusionWithInclinationAt75Degrees() {
        // Simulate: target at 500m LOS, phone tilted 75° up (e.g., looking up a cliff)
        // The fusion pipeline produces LOS range, then InclinationCorrector
        // converts to horizontal distance.
        //
        // At 75°: adjusted = 500 × cos(75°) ≈ 129.4m
        let losRange = 500.0
        let pitchRad = 75.0 * .pi / 180.0

        let (adjusted, factor) = InclinationCorrector.correct(
            lineOfSightRange: losRange,
            pitchRadians: pitchRad
        )

        // Verify the correction produces a reasonable horizontal distance
        XCTAssertEqual(adjusted, 129.4, accuracy: 1.0,
            "500m LOS at 75° → ~129.4m horizontal")
        XCTAssertEqual(factor, 0.2588, accuracy: 0.001)

        // Verify the ratio is dramatically different from 1:1
        XCTAssertLessThan(adjusted / losRange, 0.30,
            "At 75°, horizontal range should be < 30% of LOS range")
    }

    func testFusionWithInclinationSweep() {
        // Sweep from 0° to 85° and verify the adjusted range curve
        let losRange = 1000.0
        var results: [(deg: Double, adjusted: Double, factor: Double)] = []

        let angles: [Double] = [0, 5, 10, 15, 20, 30, 45, 60, 70, 75, 80, 85]
        for deg in angles {
            let pitchRad = deg * .pi / 180.0
            let (adj, fac) = InclinationCorrector.correct(
                lineOfSightRange: losRange,
                pitchRadians: pitchRad
            )
            results.append((deg, adj, fac))
        }

        // At 0°: no correction (but note threshold at 2°)
        XCTAssertEqual(results[0].adjusted, 1000.0, accuracy: 0.1, "0° → no correction")

        // At 75°: massive reduction
        let r75 = results.first { $0.deg == 75.0 }!
        XCTAssertEqual(r75.adjusted, 258.8, accuracy: 1.0, "75° → ~259m from 1000m LOS")

        // At 85°: nearly vertical
        let r85 = results.first { $0.deg == 85.0 }!
        XCTAssertLessThan(r85.adjusted, 90.0, "85° → <90m from 1000m LOS")
    }

    // MARK: - Synthetic Source Readings with Steep Angle

    func testMonteCarloAt75DegreesUphill() {
        // Simulate 500 frames of ranging at 75° inclination
        // with a true LOS distance of 200m (cliff face / tall structure)
        //
        // Geometric source should be nil (distance too short at 1.5m height)
        // Neural + DEM should provide LOS estimate
        // Inclination correction then converts to ~51.8m horizontal
        var rng = SeededRNG(seed: 75_001)
        let trueLOS: Float = 200.0
        let nFrames = 500
        var adjustedResults: [Double] = []

        let pitchRad = 75.0 * .pi / 180.0

        for _ in 0..<nFrames {
            // Neural provides LOS estimate with noise
            let neuralReading = SensorNoiseModel.neuralReading(
                trueD: trueLOS, calibrationConf: 0.65, rng: &rng)

            // Geometric should return nil at this steep angle with standard height
            let geoReading = SensorNoiseModel.geometricReading(
                trueD: trueLOS, cameraHeight: 1.5, terrainSlope: 75.0, rng: &rng)

            // Use whatever estimate we got (neural or geo)
            let losEstimate: Double
            if let neural = neuralReading {
                losEstimate = Double(neural)
            } else if let geo = geoReading {
                losEstimate = Double(geo.0)
            } else {
                continue
            }

            // Apply inclination correction
            let (adjusted, _) = InclinationCorrector.correct(
                lineOfSightRange: losEstimate,
                pitchRadians: pitchRad
            )
            adjustedResults.append(adjusted)
        }

        XCTAssertGreaterThan(adjustedResults.count, 100,
            "Should have enough valid readings")

        // Expected horizontal distance: 200 × cos(75°) ≈ 51.8m
        let mean = adjustedResults.reduce(0, +) / Double(adjustedResults.count)
        let expectedHorizontal = Double(trueLOS) * cos(pitchRad)

        // Allow 30% tolerance for neural noise + cosine amplification
        XCTAssertEqual(mean, expectedHorizontal, accuracy: expectedHorizontal * 0.30,
            "Mean adjusted range should be near \(expectedHorizontal)m, got \(mean)m")
    }

    func testMonteCarloAt75DegreesDownhill() {
        // Same as uphill but at -75° — results should be identical
        // since InclinationCorrector uses abs(pitch)
        var rng = SeededRNG(seed: 75_002)
        let trueLOS: Float = 200.0
        let nFrames = 500
        var adjustedResults: [Double] = []

        let pitchRad = -75.0 * .pi / 180.0

        for _ in 0..<nFrames {
            let neuralReading = SensorNoiseModel.neuralReading(
                trueD: trueLOS, calibrationConf: 0.65, rng: &rng)
            guard let neural = neuralReading else { continue }

            let (adjusted, _) = InclinationCorrector.correct(
                lineOfSightRange: Double(neural),
                pitchRadians: pitchRad
            )
            adjustedResults.append(adjusted)
        }

        XCTAssertGreaterThan(adjustedResults.count, 100)

        let mean = adjustedResults.reduce(0, +) / Double(adjustedResults.count)
        let expectedHorizontal = Double(trueLOS) * cos(75.0 * .pi / 180.0)

        XCTAssertEqual(mean, expectedHorizontal, accuracy: expectedHorizontal * 0.30,
            "Downhill -75° should match uphill +75° adjusted range")
    }

    // MARK: - Error Amplification at Steep Angles

    func testNoiseAmplificationAt75Degrees() {
        // At steep angles, sensor noise in LOS distance gets REDUCED by cosine
        // but the RELATIVE error stays the same.
        //
        // If LOS has ±10% noise:
        //   At 0°:  500 ± 50m horizontal
        //   At 75°: 129.4 ± 12.9m horizontal
        //
        // The absolute error shrinks by cos(angle), but relative error is preserved.
        let pitchRad = 75.0 * .pi / 180.0
        let losCenter = 500.0
        let losError = 50.0  // 10% noise

        let (adjCenter, _) = InclinationCorrector.correct(
            lineOfSightRange: losCenter,
            pitchRadians: pitchRad
        )
        let (adjHigh, _) = InclinationCorrector.correct(
            lineOfSightRange: losCenter + losError,
            pitchRadians: pitchRad
        )
        let (adjLow, _) = InclinationCorrector.correct(
            lineOfSightRange: losCenter - losError,
            pitchRadians: pitchRad
        )

        let relErrorHigh = (adjHigh - adjCenter) / adjCenter
        let relErrorLow = (adjCenter - adjLow) / adjCenter

        // Relative error should be preserved (~10%)
        XCTAssertEqual(relErrorHigh, 0.10, accuracy: 0.01,
            "Relative error should be preserved through cosine correction")
        XCTAssertEqual(relErrorLow, 0.10, accuracy: 0.01,
            "Relative error should be preserved through cosine correction")
    }

    // MARK: - Angle Uncertainty Impact

    func testAngleUncertaintyImpactAt75Degrees() {
        // IMU has ~0.3° uncertainty. At 75°, what's the impact on adjusted range?
        //
        // d/dθ [D × cos(θ)] = -D × sin(θ)
        // At 75° with D=500: sensitivity = -500 × sin(75°) ≈ -483 m/rad
        // For 0.3° = 0.00524 rad: ΔR ≈ 483 × 0.00524 ≈ 2.53m
        //
        // Compare to 10°: sensitivity = -500 × sin(10°) ≈ -86.8 m/rad
        // For 0.3° = 0.00524 rad: ΔR ≈ 86.8 × 0.00524 ≈ 0.46m
        //
        // Angle uncertainty has ~5.5× more impact at 75° vs 10°
        let losRange = 500.0
        let imuUncertaintyRad = 0.3 * .pi / 180.0

        let pitch75 = 75.0 * .pi / 180.0
        let (adj75, _) = InclinationCorrector.correct(
            lineOfSightRange: losRange, pitchRadians: pitch75)
        let (adj75plus, _) = InclinationCorrector.correct(
            lineOfSightRange: losRange, pitchRadians: pitch75 + imuUncertaintyRad)
        let rangeError75 = abs(adj75 - adj75plus)

        let pitch10 = 10.0 * .pi / 180.0
        let (adj10, _) = InclinationCorrector.correct(
            lineOfSightRange: losRange, pitchRadians: pitch10)
        let (adj10plus, _) = InclinationCorrector.correct(
            lineOfSightRange: losRange, pitchRadians: pitch10 + imuUncertaintyRad)
        let rangeError10 = abs(adj10 - adj10plus)

        // 75° should have much larger absolute error from angle uncertainty
        XCTAssertGreaterThan(rangeError75, rangeError10 * 3.0,
            "Angle uncertainty impact at 75° (\(rangeError75)m) should be >3× the impact at 10° (\(rangeError10)m)")

        // But the relative error on adjusted range should be bounded
        let relError75 = rangeError75 / adj75
        XCTAssertLessThan(relError75, 0.03,
            "0.3° IMU error at 75° should cause <3% relative error on adjusted range")
    }
}

// MARK: - Ballistic Holdover with Steep Inclination

final class SteepInclinationBallisticsTests: XCTestCase {

    /// Rifleman's Rule: use the HORIZONTAL distance for holdover calculation,
    /// not the line-of-sight distance. This matters enormously at 75°.
    func testRiflemansRuleAt75Degrees() {
        let solver = BallisticsSolver()
        solver.selectedCaliber = .cal308
        solver.zeroDistance = 100
        solver.isEnabled = true

        // Scenario: target at 500m LOS, 75° uphill
        let losRange = 500.0
        let pitchRad = 75.0 * .pi / 180.0
        let (horizontalRange, _) = InclinationCorrector.correct(
            lineOfSightRange: losRange,
            pitchRadians: pitchRad
        )
        // horizontalRange ≈ 129.4m

        // Holdover should be computed on horizontal distance (~129m)
        // not on LOS distance (500m)
        let holdoverHorizontal = solver.calculateHoldover(
            targetDistance: horizontalRange, useMetric: false)
        let holdoverLOS = solver.calculateHoldover(
            targetDistance: losRange, useMetric: false)

        // The horizontal-distance holdover should be MUCH less than LOS holdover
        XCTAssertLessThan(holdoverHorizontal.holdoverMils,
            holdoverLOS.holdoverMils * 0.5,
            "Holdover at 129m should be < 50% of holdover at 500m")

        // At ~129m (141 yards) with 100yd zero, holdover should be modest
        XCTAssertLessThan(holdoverHorizontal.holdoverMils, 1.0,
            "Holdover at ~141yd should be < 1.0 mil")
    }

    func testHoldoverDecreasesWithSteepAngle() {
        // As inclination increases, horizontal distance decreases,
        // so holdover should decrease (for same LOS range)
        let solver = BallisticsSolver()
        solver.selectedCaliber = .cal308
        solver.zeroDistance = 100
        solver.isEnabled = true

        let losRange = 500.0
        var lastHoldover = Double.infinity

        let angles: [Double] = [0, 15, 30, 45, 60, 75]
        for deg in angles {
            let pitchRad = deg * .pi / 180.0
            let (horizontal, _) = InclinationCorrector.correct(
                lineOfSightRange: losRange,
                pitchRadians: pitchRad
            )
            let result = solver.calculateHoldover(
                targetDistance: horizontal, useMetric: false)

            XCTAssertLessThanOrEqual(result.holdoverMils, lastHoldover,
                "Holdover at \(deg)° (\(result.holdoverMils) mil) should be ≤ previous (\(lastHoldover) mil)")
            lastHoldover = result.holdoverMils
        }
    }
}

// MARK: - Steep Inclination Ranging Sweep

final class SteepInclinationSweepTests: XCTestCase {

    /// Sweep across angles from 0° to 85° at multiple LOS distances.
    /// Verify adjusted range, correction factor, and source behavior.
    func testInclinationSweepAcrossDistances() {
        let losDistances: [Double] = [50, 100, 200, 500, 1000]
        let angles: [Double] = [0, 10, 20, 30, 45, 60, 70, 75, 80, 85]

        for los in losDistances {
            for deg in angles {
                let pitchRad = deg * .pi / 180.0
                let expectedFactor = deg < 2.0 ? 1.0 : cos(pitchRad)
                let expectedAdj = los * expectedFactor

                let (adjusted, factor) = InclinationCorrector.correct(
                    lineOfSightRange: los,
                    pitchRadians: pitchRad
                )

                XCTAssertEqual(factor, expectedFactor, accuracy: 0.001,
                    "Factor mismatch at \(deg)° / \(los)m LOS")
                XCTAssertEqual(adjusted, expectedAdj, accuracy: 0.5,
                    "Adjusted mismatch at \(deg)° / \(los)m LOS: expected \(expectedAdj), got \(adjusted)")
            }
        }
    }

    /// At 75° with various LOS distances, verify the horizontal
    /// distance stays sensible for downstream consumers.
    func testHorizontalDistanceRangeAt75Degrees() {
        let pitchRad = 75.0 * .pi / 180.0

        // Close LOS: 10m at 75° → ~2.6m horizontal
        let (adj10, _) = InclinationCorrector.correct(
            lineOfSightRange: 10.0, pitchRadians: pitchRad)
        XCTAssertEqual(adj10, 2.59, accuracy: 0.1)

        // Medium LOS: 100m at 75° → ~25.9m horizontal
        let (adj100, _) = InclinationCorrector.correct(
            lineOfSightRange: 100.0, pitchRadians: pitchRad)
        XCTAssertEqual(adj100, 25.9, accuracy: 0.5)

        // Far LOS: 1000m at 75° → ~258.8m horizontal
        let (adj1000, _) = InclinationCorrector.correct(
            lineOfSightRange: 1000.0, pitchRadians: pitchRad)
        XCTAssertEqual(adj1000, 258.8, accuracy: 1.0)

        // Extreme LOS: 2000m at 75° → ~517.6m horizontal
        let (adj2000, _) = InclinationCorrector.correct(
            lineOfSightRange: 2000.0, pitchRadians: pitchRad)
        XCTAssertEqual(adj2000, 517.6, accuracy: 2.0)
    }

    // MARK: - Monte Carlo Fusion with Inclination Correction Sweep

    func testMonteCarloFusionWithInclinationSweep() {
        // Run fusion at multiple angles with a fixed true horizontal distance of 300m.
        // As angle increases, LOS range increases (LOS = horizontal / cos(angle)).
        // The fusion + correction should recover the ~300m horizontal distance.
        var rng = SeededRNG(seed: 75_003)
        let trueHorizontal: Float = 300.0
        let nFrames = 200

        let angles: [Double] = [0, 30, 45, 60, 75]

        for angleDeg in angles {
            let pitchRad = angleDeg * .pi / 180.0
            let trueLOS = angleDeg < 2.0 ? Double(trueHorizontal) : Double(trueHorizontal) / cos(pitchRad)
            var adjustedResults: [Double] = []

            for _ in 0..<nFrames {
                // Neural provides LOS estimate
                let neural = SensorNoiseModel.neuralReading(
                    trueD: Float(trueLOS), calibrationConf: 0.65, rng: &rng)

                let demReading = SensorNoiseModel.demReading(
                    trueD: Float(trueLOS), gpsAccuracy: 5.0,
                    headingAccuracy: 8.0, terrainSlope: 2.0,
                    rng: &rng, srtmBias: 0)

                // Use best available estimate
                let losEstimate: Double
                if let n = neural, n <= Float(AppConfiguration.neuralHardCapMeters) {
                    losEstimate = Double(n)
                } else if let d = demReading {
                    losEstimate = Double(d.0)
                } else {
                    continue
                }

                let (adjusted, _) = InclinationCorrector.correct(
                    lineOfSightRange: losEstimate,
                    pitchRadians: pitchRad
                )
                adjustedResults.append(adjusted)
            }

            guard !adjustedResults.isEmpty else { continue }

            let mean = adjustedResults.reduce(0, +) / Double(adjustedResults.count)

            // At angles where neural can reach the LOS distance,
            // the mean adjusted range should approximate the true horizontal distance
            if trueLOS <= Double(AppConfiguration.neuralHardCapMeters) {
                XCTAssertEqual(mean, Double(trueHorizontal),
                    accuracy: Double(trueHorizontal) * 0.35,
                    "At \(angleDeg)° (LOS=\(Int(trueLOS))m), mean adjusted (\(Int(mean))m) should approximate \(trueHorizontal)m horizontal")
            }
        }
    }
}

// MARK: - RangeOutput Validation at Steep Inclination

final class SteepInclinationRangeOutputTests: XCTestCase {

    func testRangeOutputFieldsAt75Degrees() {
        // Verify RangeOutput struct correctly represents steep inclination
        let pitchDeg = 75.0
        let pitchRad = pitchDeg * .pi / 180.0
        let losMeters = 500.0
        let (adjMeters, factor) = InclinationCorrector.correct(
            lineOfSightRange: losMeters, pitchRadians: pitchRad)

        let output = RangeOutput(
            lineOfSightRange: .init(value: losMeters, unit: .meters),
            adjustedRange: .init(value: adjMeters, unit: .meters),
            confidence: 0.65,
            uncertainty: .init(value: 15.0, unit: .meters),
            inclinationDegrees: pitchDeg,
            inclinationCorrectionFactor: factor,
            primarySource: .neural,
            sourceWeights: [.neural: 0.8],
            timestamp: Date()
        )

        XCTAssertTrue(output.isValid)
        XCTAssertEqual(
            output.lineOfSightRange.converted(to: .meters).value, 500.0, accuracy: 0.1)
        XCTAssertEqual(
            output.adjustedRange.converted(to: .meters).value, adjMeters, accuracy: 0.1)
        XCTAssertEqual(output.inclinationDegrees, 75.0)
        XCTAssertEqual(output.inclinationCorrectionFactor, factor, accuracy: 0.001)
        XCTAssertLessThan(output.inclinationCorrectionFactor, 0.30,
            "At 75°, correction factor should be < 0.30")

        // LOS and adjusted should differ dramatically
        let los = output.lineOfSightRange.converted(to: .meters).value
        let adj = output.adjustedRange.converted(to: .meters).value
        XCTAssertGreaterThan(los / adj, 3.5,
            "At 75°, LOS should be >3.5× the adjusted range")
    }

    func testRangeOutputLOSvsAdjustedRatio() {
        // At various steep angles, verify the ratio between LOS and adjusted
        let angles: [(deg: Double, minRatio: Double)] = [
            (60, 1.9),   // cos(60°)=0.5 → ratio ≥ 2.0
            (70, 2.8),   // cos(70°)≈0.342 → ratio ≥ 2.9
            (75, 3.5),   // cos(75°)≈0.259 → ratio ≥ 3.8
            (80, 5.0),   // cos(80°)≈0.174 → ratio ≥ 5.7
        ]

        for (deg, minRatio) in angles {
            let pitchRad = deg * .pi / 180.0
            let losRange = 1000.0
            let (adjRange, _) = InclinationCorrector.correct(
                lineOfSightRange: losRange, pitchRadians: pitchRad)
            let ratio = losRange / adjRange
            XCTAssertGreaterThan(ratio, minRatio,
                "At \(deg)°, LOS/adjusted ratio (\(ratio)) should exceed \(minRatio)")
        }
    }
}

// MARK: - Geometric Estimator 45° Pitch Gate Tests

final class GeometricPitchGateTests: XCTestCase {

    func testGateAt44DegreesAllowsEstimate() {
        // 44° is below the 45° gate — should produce a valid estimate
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 50.0  // Tall enough for D > 5m
        let pitchRad = -44.0 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)
        XCTAssertNotNil(result,
            "44° is below the 45° gate — estimate should be valid")
        if let r = result {
            // D = 50 / tan(44°) ≈ 51.8m
            XCTAssertEqual(r.distanceMeters, 51.8, accuracy: 1.0)
        }
    }

    func testGateAt45DegreesRejectsEstimate() {
        // 45° is exactly at the gate boundary (guard uses <, not <=)
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 50.0
        let pitchRad = -45.0 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)
        XCTAssertNil(result,
            "45° is at the gate boundary — should be rejected (guard uses strict <)")
    }

    func testGateAt46DegreesRejectsEstimate() {
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 50.0
        let pitchRad = -46.0 * .pi / 180.0
        let result = estimator.estimate(pitchRadians: pitchRad)
        XCTAssertNil(result,
            "46° is above the 45° gate — should be rejected")
    }

    func testGateRejectsSteepAnglesRegardlessOfHeight() {
        // Even with extreme camera heights (tall building, cliff),
        // the 45° gate rejects the estimate
        var estimator = GeometricRangeEstimator()
        let steepAngles: [Double] = [45, 50, 60, 70, 75, 80, 85]
        let heights: [Float] = [1.5, 10.0, 30.0, 100.0, 500.0]

        for height in heights {
            estimator.cameraHeight = height
            for deg in steepAngles {
                let pitchRad = -deg * .pi / 180.0
                let result = estimator.estimate(pitchRadians: pitchRad)
                XCTAssertNil(result,
                    "At \(deg)° with \(height)m height, gate should reject (≥ 45°)")
            }
        }
    }

    func testGateDoesNotAffectShallowAngles() {
        // Angles well below 45° should be unaffected
        var estimator = GeometricRangeEstimator()
        estimator.cameraHeight = 1.5
        let shallowAngles: [Double] = [1.0, 2.0, 5.0, 8.53, 10.0, 15.0]
        for deg in shallowAngles {
            let pitchRad = -deg * .pi / 180.0
            let result = estimator.estimate(pitchRadians: pitchRad)
            // May be nil due to distance limits, but not due to pitch gate
            let distance = 1.5 / tan(Float(deg * .pi / 180.0))
            if distance >= 5.0 && distance <= 800.0 {
                XCTAssertNotNil(result,
                    "At \(deg)° with valid distance \(distance)m, should produce estimate")
            }
        }
    }
}

// MARK: - Operator Guidance Engine: Steep Angle Hints

@MainActor
final class SteepAngleGuidanceTests: XCTestCase {

    func testNoSteepAngleHintBelow45() {
        let engine = OperatorGuidanceEngine()
        engine.cameraPitchDegrees = -30.0  // 30° below horizontal
        engine.isBallisticsEnabled = true

        // Feed enough samples to compute hints
        for i in 0..<15 {
            engine.update(angularVelocity: 0.01, timestamp: Double(i) * 0.1)
        }

        XCTAssertFalse(engine.activeHints.contains(.steepAngle),
            "No steep angle hint at 30°")
        XCTAssertFalse(engine.activeHints.contains(.ballisticsAngleWarning),
            "No ballistics warning at 30°")
    }

    func testSteepAngleHintAt50Degrees() {
        let engine = OperatorGuidanceEngine()
        engine.cameraPitchDegrees = -50.0  // 50° below horizontal
        engine.calibrationSampleCount = 20 // Suppress calibration hints
        engine.hasGPSFix = true
        engine.isGPSAuthorized = true
        engine.gpsAccuracy = 5

        for i in 0..<15 {
            engine.update(angularVelocity: 0.01, timestamp: Double(i) * 0.1)
        }

        XCTAssertTrue(engine.activeHints.contains(.steepAngle),
            "Should show steep angle hint at 50° (> 45° threshold)")
    }

    func testSteepAngleHintAt75Degrees() {
        let engine = OperatorGuidanceEngine()
        engine.cameraPitchDegrees = -75.0
        engine.calibrationSampleCount = 20
        engine.hasGPSFix = true
        engine.isGPSAuthorized = true
        engine.gpsAccuracy = 5

        for i in 0..<15 {
            engine.update(angularVelocity: 0.01, timestamp: Double(i) * 0.1)
        }

        XCTAssertTrue(engine.activeHints.contains(.steepAngle),
            "Should show steep angle hint at 75°")
    }

    func testBallisticsWarningAbove60Degrees() {
        let engine = OperatorGuidanceEngine()
        engine.cameraPitchDegrees = -65.0  // 65° below horizontal
        engine.isBallisticsEnabled = true
        engine.calibrationSampleCount = 20
        engine.hasGPSFix = true
        engine.isGPSAuthorized = true
        engine.gpsAccuracy = 5

        for i in 0..<15 {
            engine.update(angularVelocity: 0.01, timestamp: Double(i) * 0.1)
        }

        XCTAssertTrue(engine.activeHints.contains(.steepAngle),
            "Should show steep angle hint at 65°")
        XCTAssertTrue(engine.activeHints.contains(.ballisticsAngleWarning),
            "Should show ballistics warning at 65° with ballistics enabled")
    }

    func testNoBallisticsWarningWhenDisabled() {
        let engine = OperatorGuidanceEngine()
        engine.cameraPitchDegrees = -70.0
        engine.isBallisticsEnabled = false  // Ballistics OFF
        engine.calibrationSampleCount = 20
        engine.hasGPSFix = true
        engine.isGPSAuthorized = true
        engine.gpsAccuracy = 5

        for i in 0..<15 {
            engine.update(angularVelocity: 0.01, timestamp: Double(i) * 0.1)
        }

        XCTAssertTrue(engine.activeHints.contains(.steepAngle),
            "Steep angle hint should still show")
        XCTAssertFalse(engine.activeHints.contains(.ballisticsAngleWarning),
            "Ballistics warning should NOT show when solver is disabled")
    }

    func testNoBallisticsWarningAt55Degrees() {
        let engine = OperatorGuidanceEngine()
        engine.cameraPitchDegrees = -55.0  // Between 45-60°
        engine.isBallisticsEnabled = true
        engine.calibrationSampleCount = 20
        engine.hasGPSFix = true
        engine.isGPSAuthorized = true
        engine.gpsAccuracy = 5

        for i in 0..<15 {
            engine.update(angularVelocity: 0.01, timestamp: Double(i) * 0.1)
        }

        XCTAssertTrue(engine.activeHints.contains(.steepAngle),
            "Steep angle hint at 55° (> 45°)")
        XCTAssertFalse(engine.activeHints.contains(.ballisticsAngleWarning),
            "Ballistics warning should NOT show at 55° (< 60° threshold)")
    }

    func testSteepAngleHintMessage() {
        XCTAssertEqual(GuidanceHint.steepAngle.message, "STEEP ANGLE")
        XCTAssertEqual(GuidanceHint.ballisticsAngleWarning.message, "ANGLE — VERIFY SOLUTION")
    }

    func testSteepAngleHintSeverity() {
        XCTAssertEqual(GuidanceHint.steepAngle.severity, .caution,
            "Steep angle should be caution (amber)")
        XCTAssertEqual(GuidanceHint.ballisticsAngleWarning.severity, .warning,
            "Ballistics angle warning should be warning (red)")
    }

    func testSteepAngleHintPriority() {
        // ballisticsAngleWarning (88) should outrank steepAngle (85)
        XCTAssertGreaterThan(
            GuidanceHint.ballisticsAngleWarning.priority,
            GuidanceHint.steepAngle.priority,
            "Ballistics warning should have higher priority than steep angle")
    }
}

// MARK: - Adaptive Stability Thresholds

@MainActor
final class AdaptiveStabilityTests: XCTestCase {

    func testStabilityRelaxationAtLevel() {
        // At level (0°), stability thresholds should be normal.
        // Test by verifying that moderate motion produces marginal stability at level,
        // but the same motion at steep pitch produces better stability.
        let levelEngine = OperatorGuidanceEngine()
        levelEngine.cameraPitchDegrees = 0.0

        let steepEngine = OperatorGuidanceEngine()
        steepEngine.cameraPitchDegrees = -70.0  // Steep angle

        // Feed identical angular velocity to both
        let angularVelocity = 0.15  // Should be marginal at level
        for i in 0..<20 {
            levelEngine.update(angularVelocity: angularVelocity, timestamp: Double(i) * 0.1)
            steepEngine.update(angularVelocity: angularVelocity, timestamp: Double(i) * 0.1)
        }

        // At level, 0.15 rad/s > 0.1 threshold → marginal
        XCTAssertLessThanOrEqual(levelEngine.stabilityLevel, .marginal,
            "At level pitch, 0.15 rad/s should be marginal or unstable")

        // At 70°, thresholds are relaxed by ~1.67× → 0.15 / (0.1 × 1.67) < 1 → adequate or better
        XCTAssertGreaterThan(steepEngine.stabilityLevel, levelEngine.stabilityLevel,
            "At 70° pitch, same motion should produce better stability due to relaxed thresholds")
    }

    func testStabilityPercentHigherAtSteepAngle() {
        let levelEngine = OperatorGuidanceEngine()
        levelEngine.cameraPitchDegrees = 0.0

        let steepEngine = OperatorGuidanceEngine()
        steepEngine.cameraPitchDegrees = -75.0

        let angularVelocity = 0.08
        for i in 0..<20 {
            levelEngine.update(angularVelocity: angularVelocity, timestamp: Double(i) * 0.1)
            steepEngine.update(angularVelocity: angularVelocity, timestamp: Double(i) * 0.1)
        }

        // Stability percent should be higher at steep angles (relaxed thresholds)
        XCTAssertGreaterThanOrEqual(steepEngine.stabilityPercent, levelEngine.stabilityPercent,
            "Stability percent should be equal or higher at steep angles")
    }

    func testRespiratoryPauseDetectsAtSteepAngle() {
        // At steep angles, the motion threshold for respiratory pause is relaxed.
        // At 75°: relaxation ≈ 1.8×
        //   stillness threshold: 0.02 × 1.8 = 0.036
        //   motion threshold:    0.05 × 1.8 = 0.09
        //
        // A motion value of 0.03 is BELOW 0.036 (triggers pause at 75°)
        // but ABOVE 0.02 (would NOT trigger at 0° level pitch).
        let engine = OperatorGuidanceEngine()
        engine.cameraPitchDegrees = -75.0
        engine.calibrationSampleCount = 20
        engine.hasGPSFix = true
        engine.isGPSAuthorized = true
        engine.gpsAccuracy = 5

        // Phase 1: simulate motion (breathing in) — must exceed the relaxed
        // "earlier motion" threshold of 0.09 rad/s
        for i in 0..<10 {
            engine.update(angularVelocity: 0.12, timestamp: Double(i) * 0.1)
        }

        // Phase 2: simulate stillness (exhale pause) — 0.03 is below 0.036
        // Only 3 samples so the "earlier" window (suffix(8).prefix(5)) still
        // captures the high-motion phase.
        for i in 10..<13 {
            engine.update(angularVelocity: 0.03, timestamp: Double(i) * 0.1)
        }

        XCTAssertTrue(engine.isCapturePrimed,
            "Respiratory pause should detect at steep angles with relaxed thresholds")
    }

    func testRenamedHintMessages() {
        // Verify the renamed hint messages
        XCTAssertEqual(GuidanceHint.respiratoryPause.message, "STEADY — TAKE READING",
            "Capture window hint should read 'STEADY — TAKE READING'")
        XCTAssertEqual(GuidanceHint.terrainRangeLimited.message, "DOWNLOAD TERRAIN",
            "Terrain range limited hint should read 'DOWNLOAD TERRAIN'")
    }
}
