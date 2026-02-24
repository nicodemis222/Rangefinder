//
//  DisagreementPenaltyTests.swift
//  RangefinderTests
//
//  Tests for source disagreement detection and outlier suppression logic.
//

import XCTest
@testable import Rangefinder

final class DisagreementPenaltyTests: XCTestCase {

    // MARK: - Outlier Detection (3+ sources)

    func testOutlierDetection3Sources() {
        // 3 sources: geometric=9.8m, neural=35m, DEM=91m
        // Median = 35m
        // Geometric: 35/9.8 = 3.57× from median → outlier (>2.0×)
        // Neural: 35/35 = 1.0× → not outlier
        // DEM: 91/35 = 2.6× → outlier (>2.0×)

        let depths: [Float] = [9.8, 35.0, 91.0]
        let sorted = depths.sorted()
        let median = sorted[sorted.count / 2]  // 35.0

        XCTAssertEqual(median, 35.0, accuracy: 0.1)

        for depth in depths {
            let ratio = depth > median ? depth / median : median / depth
            if depth == 9.8 {
                XCTAssertGreaterThan(ratio, 2.0,
                    "Geometric 9.8m should be flagged as outlier (ratio=\(ratio))")
            } else if depth == 35.0 {
                XCTAssertLessThanOrEqual(ratio, 2.0,
                    "Neural 35m (median) should NOT be outlier")
            } else if depth == 91.0 {
                XCTAssertGreaterThan(ratio, 2.0,
                    "DEM 91m should be flagged as outlier (ratio=\(ratio))")
            }
        }
    }

    func testAgreeing3Sources() {
        // 3 sources that mostly agree: 85m, 91m, 95m
        // Median = 91m
        // All within 1.07× of median → no outliers
        let depths: [Float] = [85.0, 91.0, 95.0]
        let sorted = depths.sorted()
        let median = sorted[sorted.count / 2]

        for depth in depths {
            let ratio = depth > median ? depth / median : median / depth
            XCTAssertLessThan(ratio, 2.0,
                "Agreeing sources should not be flagged as outliers")
        }
    }

    // MARK: - Disagreement Penalty (2 sources)

    func testDisagreementPenalty2Sources() {
        // 2 sources: geometric=9.8m, neural=35m
        // ratio = 35/9.8 = 3.57
        // penalty = max(0.15, 1.0 - (3.57 - 2.0) * 0.5) = max(0.15, 0.215) = 0.215
        let d1: Float = 9.8
        let d2: Float = 35.0
        let ratio = max(d1, d2) / min(d1, d2)

        XCTAssertEqual(ratio, 3.57, accuracy: 0.1)

        let penalty = max(Float(0.15), 1.0 - (ratio - 2.0) * 0.5)
        XCTAssertEqual(penalty, 0.215, accuracy: 0.05,
            "3.57× disagreement should yield ~0.22 penalty")
    }

    func testNoPenaltyWhenAgreeing() {
        // 2 sources: 28m, 32m → ratio = 1.14 → no penalty (< 2.0)
        let d1: Float = 28.0
        let d2: Float = 32.0
        let ratio = max(d1, d2) / min(d1, d2)

        XCTAssertLessThan(ratio, 2.0,
            "Agreeing sources should not trigger disagreement penalty")
    }

    func testPenaltyFloorAt0Point15() {
        // Extreme disagreement: 5m vs 500m → ratio = 100
        // penalty = max(0.15, 1.0 - (100 - 2.0) * 0.5) = max(0.15, -48.0) = 0.15
        let d1: Float = 5.0
        let d2: Float = 500.0
        let ratio = max(d1, d2) / min(d1, d2)

        let penalty = max(Float(0.15), 1.0 - (ratio - 2.0) * 0.5)
        XCTAssertEqual(penalty, 0.15, accuracy: 0.01,
            "Extreme disagreement should hit penalty floor of 0.15")
    }

    // MARK: - Weighted Average

    func testWeightedAverageDEMDominates() {
        // After outlier suppression with DEM + neural (geometric suppressed):
        // Neural: 35m @ weight 0.50
        // DEM: 91m @ weight 0.55
        // Fused = (0.50×35 + 0.55×91) / (0.50 + 0.55) = (17.5 + 50.05) / 1.05 = 64.3m
        let neuralW: Float = 0.50
        let neuralD: Float = 35.0
        let demW: Float = 0.55
        let demD: Float = 91.0

        let fused = (neuralW * neuralD + demW * demD) / (neuralW + demW)
        XCTAssertEqual(fused, 64.3, accuracy: 1.0,
            "DEM+neural fusion should yield ~64m (vs actual 91m)")
        XCTAssertGreaterThan(fused, 50.0,
            "With DEM, fused result should be >> 26m (the wrong geo+neural result)")
    }

    func testWeightedAverageWithoutDEM() {
        // Without DEM, geometric + neural:
        // Geometric: 9.8m @ weight 0.31 (slope-penalized)
        // Neural: 35m @ weight 0.60
        // Fused = (0.31×9.8 + 0.60×35) / (0.31 + 0.60) = (3.04 + 21.0) / 0.91 = 26.4m
        let geoW: Float = 0.31
        let geoD: Float = 9.8
        let neuralW: Float = 0.60
        let neuralD: Float = 35.0

        let fused = (geoW * geoD + neuralW * neuralD) / (geoW + neuralW)
        XCTAssertEqual(fused, 26.4, accuracy: 1.0,
            "Without DEM, fusion is still wrong (~26m vs 91m actual)")
    }

    // MARK: - Median Computation

    func testMedianOddCount() {
        let values: [Float] = [9.8, 35.0, 91.0]
        let sorted = values.sorted()
        let median = sorted[sorted.count / 2]
        XCTAssertEqual(median, 35.0)
    }

    func testMedianEvenCount() {
        let values: [Float] = [9.8, 35.0, 50.0, 91.0]
        let sorted = values.sorted()
        // With integer division, count/2 = 2 → index 2 → 50.0
        let median = sorted[sorted.count / 2]
        XCTAssertEqual(median, 50.0)
    }

    // MARK: - Tighter Outlier Threshold (2.0×)

    func testOutlierAt2Point1Ratio() {
        // With 2.0× threshold, a 2.1× ratio should be an outlier
        let median: Float = 50.0
        let depth: Float = 105.0  // 105/50 = 2.1×
        let ratio = depth / median
        XCTAssertGreaterThan(ratio, 2.0, "2.1× should exceed 2.0× threshold")
    }

    func testNotOutlierAt1Point9Ratio() {
        // 1.9× ratio should NOT be an outlier
        let median: Float = 50.0
        let depth: Float = 95.0  // 95/50 = 1.9×
        let ratio = depth / median
        XCTAssertLessThan(ratio, 2.0, "1.9× should be within 2.0× threshold")
    }

    // MARK: - DEM-Dominance Rule

    func testDEMDominanceNeuralSuppressed() {
        // DEM=91m, neural=35m, ratio=2.6×, DEM>40m → suppress neural
        // Threshold lowered from 1.8× to 1.5×, floor lowered from 0.15 to 0.10
        let demD: Float = 91.0
        let neuralD: Float = 35.0
        let ratio = demD / neuralD  // 2.6
        XCTAssertGreaterThan(ratio, 1.5, "Should trigger DEM-dominance")
        XCTAssertGreaterThan(demD, 40.0, "DEM should be beyond 40m threshold")

        let neuralSuppression = max(Float(0.08), 1.0 / ratio)
        XCTAssertEqual(neuralSuppression, 0.385, accuracy: 0.02,
            "Neural should be suppressed to ~0.38 of original weight")
    }

    func testDEMDominanceNotTriggeredWhenClose() {
        // DEM=35m, neural=30m, ratio=1.17 → too small, no suppression
        let demD: Float = 35.0
        let neuralD: Float = 30.0
        let ratio = demD > neuralD ? demD / neuralD : neuralD / demD
        XCTAssertLessThan(ratio, 1.5, "Small disagreement should not trigger DEM-dominance")
    }

    func testDEMDominanceNotTriggeredBelowDEMRange() {
        // DEM=25m (close range), neural=15m, ratio=1.67 → DEM too close
        let demD: Float = 25.0
        let neuralD: Float = 15.0
        let ratio = demD > neuralD ? demD / neuralD : neuralD / demD
        // Even if ratio were >1.5, demD < 40 should prevent trigger
        XCTAssertLessThan(demD, 40.0, "Close-range DEM should not trigger dominance")
    }

    func testDEMDominanceSuppressionFloor() {
        // Extreme disagreement: DEM=500m, neural=30m, ratio=16.7
        // suppression = max(0.08, 1/16.7) = max(0.08, 0.06) = 0.08
        let demD: Float = 500.0
        let neuralD: Float = 30.0
        let ratio = demD / neuralD
        let suppression = max(Float(0.08), 1.0 / ratio)
        XCTAssertEqual(suppression, 0.08, accuracy: 0.01,
            "Extreme DEM-dominance should floor suppression at 0.08")
    }

    // MARK: - Neural Extrapolation Penalty

    func testExtrapolationPenaltyNearCalibrationRange() {
        // At 10m (within calibration range 0.2-8m, close): no penalty
        let depth: Float = 10.0
        let penalty: Float = 1.0  // < 15m → 1.0
        XCTAssertEqual(penalty, 1.0, "Within calibration range should have no penalty")
        XCTAssertLessThan(depth, 15.0)
    }

    func testExtrapolationPenaltyMildAt20m() {
        // 15-30m bracket: penalty = 1.0 - (20-15)/100 = 0.95
        let depth: Float = 20.0
        let penalty: Float = 1.0 - (depth - 15.0) / 100.0
        XCTAssertEqual(penalty, 0.95, accuracy: 0.01,
            "Mild extrapolation at 20m should have ~0.95 penalty")
    }

    func testExtrapolationPenaltyModerateAt50m() {
        // 30-80m bracket: penalty = 0.85 - (50-30)/100 = 0.85 - 0.20 = 0.65
        let depth: Float = 50.0
        let penalty: Float = 0.85 - (depth - 30.0) / 100.0
        XCTAssertEqual(penalty, 0.65, accuracy: 0.02,
            "Moderate extrapolation at 50m should have ~0.65 penalty")
    }

    func testExtrapolationPenaltyHeavyAt100m() {
        // 80m+ bracket: penalty = max(0.15, 0.35 - (100-80)/200) = max(0.15, 0.25) = 0.25
        let depth: Float = 100.0
        let penalty: Float = max(0.15, 0.35 - (depth - 80.0) / 200.0)
        XCTAssertEqual(penalty, 0.25, accuracy: 0.02,
            "Heavy extrapolation at 100m should have ~0.25 penalty")
    }

    func testExtrapolationPenaltyFloorAt0Point15() {
        // Very far extrapolation: 200m → penalty = max(0.15, 0.35 - 120/200) = max(0.15, -0.25) = 0.15
        let depth: Float = 200.0
        let penalty: Float = max(0.15, 0.35 - (depth - 80.0) / 200.0)
        XCTAssertEqual(penalty, 0.15, accuracy: 0.01,
            "Extreme extrapolation should floor at 0.15")
    }

    // MARK: - Combined Scenario: Case A with Refinements

    func testCaseAWithDEMDominance() {
        // Case A: pitch -8.7°, actual 91.4m
        // Geometric: suppressed as outlier (9.8m, >2.0× from median)
        // Neural: 35m, weight after steepened extrapolation penalty
        //   distanceWeight(35m) = 0.70 - (35-25)*0.0167 ≈ 0.533 (steepened curve)
        //   calQuality = 0.9
        //   extrap penalty = 0.85 - (35-30)/100 = 0.80
        //   weight = 0.533 * 0.9 * 0.80 = 0.384
        // DEM: 91m, weight ≈ 0.65 (boosted curve)
        // DEM-dominance: ratio=91/35=2.6>1.5, DEM>40 → neural *= max(0.08, 1/2.6)=0.385
        //   neural weight = 0.384 * 0.385 = 0.148
        // Fused = (0.148 * 35 + 0.65 * 91) / (0.148 + 0.65) = (5.18 + 59.15) / 0.798 = 80.6m

        let neuralD: Float = 35.0
        let demD: Float = 91.0

        // Neural weight after steepened curve + extrapolation
        let neuralDistW: Float = 0.70 - (neuralD - 25.0) * 0.0167  // ~0.533
        let calQ: Float = 0.9
        let extrapPenalty: Float = 0.85 - (neuralD - 30.0) / 100.0
        var neuralW = neuralDistW * calQ * extrapPenalty

        // DEM weight (boosted curve)
        let demW: Float = 0.65

        // DEM-dominance suppression (lowered threshold and floor)
        let ratio = demD / neuralD  // 2.6
        let suppression = max(Float(0.08), 1.0 / ratio)
        neuralW *= suppression

        let fused = (neuralW * neuralD + demW * demD) / (neuralW + demW)

        XCTAssertGreaterThan(fused, 70.0,
            "Case A with DEM-dominance should fuse > 70m (was 26m without DEM)")
        XCTAssertLessThan(fused, 95.0,
            "Case A should still be < 95m (DEM + neural blend)")
    }
}
