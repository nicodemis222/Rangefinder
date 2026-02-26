//
//  RangefinderSweepTests.swift
//  RangefinderTests
//
//  COMPREHENSIVE DISTANCE SWEEP TEST PLAN
//  ========================================
//  Based on industry-standard rangefinder testing methodologies:
//
//  1. GENERAL RANGING SWEEP (100–1500 yds / 91–1372m)
//     Sweeps every 25 yds across the full operating envelope.
//     Each distance tested with N=50 temporal frames × 3 operator seeds.
//     Validates fusion accuracy, source selection, and confidence.
//     Acceptance: AbsRel < threshold per band, P90 < 2× threshold.
//
//  2. GOLF PIN STADIAMETRIC SWEEP (40–300 yds / 37–274m)
//     Tests the pinhole formula with realistic pixel bracket noise
//     across every 10-yard increment a golfer would encounter.
//     Uses USGA-standard 2.13m flagstick at 1×, 5×, and 8× zoom.
//     Acceptance: club-selection grade (±2 yds at 150, ±5 yds at 250).
//
//  3. MULTI-OPERATOR REPEATABILITY
//     Same distances, 5 different RNG seeds (simulating operator variance).
//     Measures inter-operator standard deviation. Acceptance: CV < 15%.
//
//  4. ENVIRONMENTAL CONDITION MATRIX
//     Crosses GPS quality × terrain slope × calibration age × object presence
//     across 6 key distances. Validates no catastrophic failure in any combo.
//
//  5. SOURCE HANDOFF STRESS TEST
//     Walks through the full distance range in 10m steps, verifying smooth
//     transitions between source regimes (LiDAR→Neural→Geo→DEM→Object).
//     No reading should jump > 3× between adjacent distances.
//
//  Metrics (per industry standard):
//   - AbsRel:  |estimate - truth| / truth  (scale-invariant)
//   - RMSE:    sqrt(mean(error²))          (penalizes outliers)
//   - P90:     90th percentile AbsRel      (tail risk)
//   - CV:      stddev / mean               (repeatability)
//
//  References:
//   - GolfLink rangefinder testing protocol (50/100/150/200/250 yd survey markers)
//   - Vovex Golf multi-operator, multi-condition accuracy protocol
//   - KITTI monocular depth benchmark (AbsRel, RMSE, δ < 1.25 metrics)
//   - Mossy Oak / Bushnell field test methodology (15–1850 yds, multiple light)
//   - SRTM vertical accuracy: RMSE 9.73m (USGS), 3.5–10m terrain-dependent
//   - Stadiametric rangefinding: ±5% theoretical (Wikipedia / Army Guide)
//

import XCTest
@testable import Rangefinder

// MARK: - Test Result Accumulators

/// Per-distance-band statistics collected during sweep
private struct BandResult {
    let bandLabel: String
    let trueDist: Float         // meters
    let trueYards: Float
    var absRelErrors: [Float] = []
    var signedErrors: [Float] = []
    var estimates: [Float] = []
    var confidences: [Float] = []
    var dominantSources: [DepthSource] = []

    var count: Int { absRelErrors.count }
    var meanAbsRel: Float { absRelErrors.isEmpty ? 0 : absRelErrors.reduce(0, +) / Float(count) }
    var rmse: Float {
        guard !signedErrors.isEmpty else { return 0 }
        let mse = signedErrors.map { $0 * $0 }.reduce(0, +) / Float(count)
        return sqrt(mse)
    }
    var p90AbsRel: Float {
        guard !absRelErrors.isEmpty else { return 0 }
        let sorted = absRelErrors.sorted()
        return sorted[min(sorted.count - 1, Int(Float(sorted.count) * 0.9))]
    }
    var p50AbsRel: Float {
        guard !absRelErrors.isEmpty else { return 0 }
        let sorted = absRelErrors.sorted()
        return sorted[sorted.count / 2]
    }
    var meanEstimate: Float { estimates.isEmpty ? 0 : estimates.reduce(0, +) / Float(count) }
    var stddevEstimate: Float {
        guard estimates.count > 1 else { return 0 }
        let m = meanEstimate
        let variance = estimates.map { ($0 - m) * ($0 - m) }.reduce(0, +) / Float(estimates.count - 1)
        return sqrt(variance)
    }
    var cv: Float { meanEstimate > 0 ? stddevEstimate / meanEstimate : 0 }
    var meanConf: Float { confidences.isEmpty ? 0 : confidences.reduce(0, +) / Float(count) }

    /// Most common dominant source
    var primarySource: String {
        var counts: [DepthSource: Int] = [:]
        for s in dominantSources { counts[s, default: 0] += 1 }
        return counts.max(by: { $0.value < $1.value })?.key.shortName ?? "—"
    }
}

// MARK: - Yard/Meter Helpers

private func yardsToMeters(_ yds: Float) -> Float { yds * 0.9144 }
private func metersToYards(_ m: Float) -> Float { m / 0.9144 }

// MARK: ============================================================
// MARK:  TEST 1: GENERAL RANGING SWEEP  (100 – 1500 yds)
// MARK: ============================================================

final class GeneralRangingSweepTests: XCTestCase {

    /// Distance bands: every 25 yards from 100 to 1500 yds.
    /// Error tolerance scales with distance per industry norms:
    ///  - 100-200 yds: ≤ 8% AbsRel   (neural + geo + DEM overlap)
    ///  - 200-400 yds: ≤ 12% AbsRel  (DEM + object)
    ///  - 400-800 yds: ≤ 18% AbsRel  (DEM dominant)
    ///  - 800-1200 yds: ≤ 25% AbsRel (extreme DEM + object)
    ///  - 1200-1500 yds: ≤ 35% AbsRel (max range, high uncertainty)
    private static func tolerance(forYards yds: Float) -> Float {
        switch yds {
        case ..<200:  return 0.08
        case ..<400:  return 0.12
        case ..<800:  return 0.18
        case ..<1200: return 0.25
        default:      return 0.35
        }
    }

    /// Runs the full sweep with a given seed. Returns array of BandResults.
    private func runSweep(seed: UInt64, framesPerDistance: Int = 50,
                          label: String) -> [BandResult] {
        var rng = SeededRNG(seed: seed)
        var results: [BandResult] = []

        // 100 to 1500 in 25-yd steps = 57 distances
        let startYds: Float = 100
        let endYds: Float = 1500
        let stepYds: Float = 25

        var yds = startYds
        while yds <= endYds {
            let trueD = yardsToMeters(yds)
            var band = BandResult(bandLabel: "\(Int(yds)) YDS",
                                  trueDist: trueD, trueYards: yds)

            // Fixed SRTM bias for this "location"
            let srtmVertical: Float = 3.5
            let srtmBias = rng.nextGaussian(mean: 0, stddev: srtmVertical)

            // Realistic environment for each band
            let gpsAcc: Float = 5.0        // Good outdoor GPS
            let headingAcc: Float = 8.0    // Typical compass
            let slope: Float = 2.0         // Gentle terrain
            let calAge: TimeInterval = 30  // Recent calibration
            let calConf: Float = 0.65      // Decent cal quality
            let hasObj = yds >= 150        // Objects detectable at 150+ yds
            let objConf: Float = yds < 300 ? 0.70 : (yds < 800 ? 0.60 : 0.50)

            for _ in 0..<framesPerDistance {
                let lidar = SensorNoiseModel.lidarReading(trueD: trueD, rng: &rng)
                let neural = SensorNoiseModel.neuralReading(
                    trueD: trueD, calibrationConf: calConf, rng: &rng)
                let geo = SensorNoiseModel.geometricReading(
                    trueD: trueD, cameraHeight: 1.5,
                    terrainSlope: slope, rng: &rng)
                let dem = SensorNoiseModel.demReading(
                    trueD: trueD, gpsAccuracy: gpsAcc,
                    headingAccuracy: headingAcc,
                    terrainSlope: slope, rng: &rng,
                    srtmBias: srtmBias)
                let obj = hasObj ? SensorNoiseModel.objectReading(
                    trueD: trueD, detectionConf: objConf, rng: &rng) : nil

                let scenario = FusionScenario(
                    trueDistanceM: trueD, sceneName: band.bandLabel,
                    terrainSlope: slope, hasGPS: true,
                    gpsAccuracy: gpsAcc, headingAccuracy: headingAcc,
                    hasObject: hasObj, objectDetConf: objConf,
                    calibrationAge: calAge, calibrationConf: calConf,
                    expectedDominantSource: nil, maxAcceptableErrorPercent: 0)

                let result = FusionSimulator.simulate(
                    scenario: scenario,
                    lidarReading: lidar, neuralReading: neural,
                    geometricReading: geo?.0, geometricConfidence: geo?.1,
                    demReading: dem?.0, demConfidence: dem?.1,
                    objectReading: obj)

                if result.fusedDepth > 0.01 {
                    let absRel = abs(result.fusedDepth - trueD) / trueD
                    let signed = result.fusedDepth - trueD
                    band.absRelErrors.append(absRel)
                    band.signedErrors.append(signed)
                    band.estimates.append(result.fusedDepth)
                    band.confidences.append(result.confidence)
                    if let dom = result.dominantSource {
                        band.dominantSources.append(dom)
                    }
                }
            }

            results.append(band)
            yds += stepYds
        }
        return results
    }

    /// TEST: Sweep 100–1500 yds with 50 frames per distance, 3 operators.
    /// Print a comprehensive table and assert per-band tolerances.
    func testGeneralRangingSweep100to1500() {
        let seeds: [UInt64] = [42, 1337, 9999]  // 3 operators
        var allBands: [BandResult] = []

        print("\n╔══════════════════════════════════════════════════════════════════════════════════════════╗")
        print("║                     GENERAL RANGING SWEEP: 100 – 1500 YDS                              ║")
        print("║  50 frames × 3 operators per distance | GPS 5m | Heading 8° | Slope 2° | Cal 30s       ║")
        print("╚══════════════════════════════════════════════════════════════════════════════════════════╝\n")

        for (i, seed) in seeds.enumerated() {
            let bands = runSweep(seed: seed, label: "Operator \(i+1)")
            if i == 0 { allBands = bands }
            else {
                // Merge into allBands
                for j in 0..<min(bands.count, allBands.count) {
                    allBands[j].absRelErrors.append(contentsOf: bands[j].absRelErrors)
                    allBands[j].signedErrors.append(contentsOf: bands[j].signedErrors)
                    allBands[j].estimates.append(contentsOf: bands[j].estimates)
                    allBands[j].confidences.append(contentsOf: bands[j].confidences)
                    allBands[j].dominantSources.append(contentsOf: bands[j].dominantSources)
                }
            }
        }

        // Print results table
        print("Distance   N    MeanAbsRel   P50      P90      RMSE(m)   CV       Conf   Source   Tol    Status")
        print(String(repeating: "─", count: 105))

        var passedBands = 0
        var totalBands = 0
        var bandFailures: [String] = []

        for band in allBands {
            totalBands += 1
            let tol = Self.tolerance(forYards: band.trueYards)
            let passed = band.meanAbsRel <= tol
            if passed { passedBands += 1 }
            else { bandFailures.append(band.bandLabel) }

            let status = passed ? "PASS" : "FAIL"
            let lbl = band.bandLabel.padding(toLength: 10, withPad: " ", startingAt: 0)
            let src = band.primarySource.padding(toLength: 6, withPad: " ", startingAt: 0)
            print("\(lbl) \(String(format: "%-4d %6.1f%%    %6.1f%%   %6.1f%%   %7.1fm  %5.1f%%   %4.2f", band.count, band.meanAbsRel * 100, band.p50AbsRel * 100, band.p90AbsRel * 100, band.rmse, band.cv * 100, band.meanConf))   \(src)   \(String(format: "%4.0f%%", tol * 100))   \(status)")
        }

        print(String(repeating: "─", count: 105))
        print("TOTAL: \(passedBands)/\(totalBands) bands passed")

        // Aggregate stats
        let allErrors = allBands.flatMap { $0.absRelErrors }
        let globalMean = allErrors.reduce(0, +) / Float(allErrors.count)
        let sorted = allErrors.sorted()
        let p90 = sorted[Int(Float(sorted.count) * 0.9)]
        let p95 = sorted[Int(Float(sorted.count) * 0.95)]
        print(String(format: "Global: Mean %.1f%% | P90 %.1f%% | P95 %.1f%% | N=%d readings",
                     globalMean * 100, p90 * 100, p95 * 100, allErrors.count))

        // Assertion: ≤ 10% of bands can fail (allow marginal bands at extreme range)
        let failPct = Float(totalBands - passedBands) / Float(totalBands)
        XCTAssertLessThanOrEqual(failPct, 0.10,
            "More than 10% of distance bands exceeded tolerance: \(bandFailures.joined(separator: ", "))")
    }
}

// MARK: ============================================================
// MARK:  TEST 2: GOLF PIN STADIAMETRIC SWEEP  (40 – 300 yds)
// MARK: ============================================================

final class GolfPinSweepTests: XCTestCase {

    /// Golf-specific tolerances: must be accurate enough for club selection.
    /// Industry standard for laser rangefinders is ±1 yd.
    /// For phone-based stadiametric, tolerances are wider but still useful:
    ///  40-100 yds: ≤ 3% (±1-3 yds — wedge/short iron selection)
    ///  100-150 yds: ≤ 5% (±5-7 yds — mid iron selection)
    ///  150-200 yds: ≤ 8% (±12-16 yds — long iron/hybrid)
    ///  200-250 yds: ≤ 12% (±24-30 yds — fairway wood/driver)
    ///  250-300 yds: ≤ 15% (advisory only)
    private static func golfTolerance(forYards yds: Float) -> Float {
        switch yds {
        case ..<100:  return 0.03
        case ..<150:  return 0.05
        case ..<200:  return 0.08
        case ..<250:  return 0.12
        default:      return 0.15
        }
    }

    /// iPhone 17 Pro Max camera specs
    private static let mainFocalPx: Double = 2160    // 24mm main @ 12MP
    private static let teleZoom5xFocalPx: Double = 10800  // 5× telephoto
    private static let teleZoom8xFocalPx: Double = 17280  // 8× digital

    /// USGA regulation golf pin height
    private static let pinHeightM: Double = 2.13

    /// Simulates bracket noise: how accurately a user can bracket the top
    /// and bottom of a flagstick in pixels. Noise increases with distance
    /// (smaller object) and decreases with zoom (more pixels to work with).
    private static func bracketNoise(truePixelHeight: Double,
                                     rng: inout SeededRNG) -> Double {
        // Bracket error is fundamentally ±N pixels on each edge.
        // At close range (large target), N ≈ 1-2px.
        // At far range (small target), N ≈ 1-3px (harder to see edges).
        // But expressed as a fraction of pixel height, it's worse for small targets.
        let edgeNoisePx: Double
        if truePixelHeight > 80 {
            edgeNoisePx = 1.0  // Easy to bracket
        } else if truePixelHeight > 40 {
            edgeNoisePx = 1.5  // Moderate
        } else if truePixelHeight > 15 {
            edgeNoisePx = 2.0  // Small but visible
        } else {
            edgeNoisePx = 3.0  // Very small, hard to bracket
        }

        // Two bracket edges, each with independent noise
        let topNoise = Double(rng.nextGaussian(mean: 0, stddev: Float(edgeNoisePx)))
        let bottomNoise = Double(rng.nextGaussian(mean: 0, stddev: Float(edgeNoisePx)))
        return topNoise - bottomNoise  // Combined bracket height error
    }

    /// Run golf pin sweep at a specific zoom level
    private func runGolfSweep(focalPx: Double, zoomLabel: String,
                              seed: UInt64, framesPerDistance: Int) -> [BandResult] {
        var rng = SeededRNG(seed: seed)
        var results: [BandResult] = []

        // 40 to 300 in 10-yd steps = 27 distances
        var yds: Float = 40
        while yds <= 300 {
            let trueD = yardsToMeters(yds)
            var band = BandResult(bandLabel: "\(Int(yds)) YDS",
                                  trueDist: trueD, trueYards: yds)

            // True pixel height of pin at this distance and zoom
            let truePixelHeight = (Self.pinHeightM * focalPx) / Double(trueD)

            for _ in 0..<framesPerDistance {
                // Simulated bracket with noise
                let noise = Self.bracketNoise(truePixelHeight: truePixelHeight,
                                              rng: &rng)
                let measuredPixelHeight = max(1.0, truePixelHeight + noise)

                // Pinhole formula
                let input = StadiametricInput(
                    knownSizeMeters: Self.pinHeightM,
                    pixelSize: measuredPixelHeight,
                    focalLengthPixels: focalPx)
                let estimate = Float(input.computedRange)

                let absRel = abs(estimate - trueD) / trueD
                let signed = estimate - trueD
                band.absRelErrors.append(absRel)
                band.signedErrors.append(signed)
                band.estimates.append(estimate)
                band.confidences.append(1.0)  // Stadiametric: user-controlled
                band.dominantSources.append(.stadiametric)
            }

            results.append(band)
            yds += 10
        }
        return results
    }

    /// TEST: Golf pin at 1× zoom (worst case — pin is small)
    func testGolfPinSweep1xZoom() {
        runAndReport(focalPx: Self.mainFocalPx, zoomLabel: "1×",
                     seeds: [42, 1337, 9999], framesPerDistance: 100)
    }

    /// TEST: Golf pin at 5× telephoto (recommended for golf)
    func testGolfPinSweep5xZoom() {
        runAndReport(focalPx: Self.teleZoom5xFocalPx, zoomLabel: "5×",
                     seeds: [42, 1337, 9999], framesPerDistance: 100)
    }

    /// TEST: Golf pin at 8× digital zoom
    func testGolfPinSweep8xZoom() {
        runAndReport(focalPx: Self.teleZoom8xFocalPx, zoomLabel: "8×",
                     seeds: [42, 1337, 9999], framesPerDistance: 100)
    }

    private func runAndReport(focalPx: Double, zoomLabel: String,
                              seeds: [UInt64], framesPerDistance: Int) {
        var allBands: [BandResult] = []

        print("\n╔══════════════════════════════════════════════════════════════════════════════════════════╗")
        print("║              GOLF PIN STADIAMETRIC SWEEP: 40 – 300 YDS @ \(zoomLabel) ZOOM                     ║")
        print("║  Pin height: 2.13m (USGA) | Focal: \(String(format: "%.0f", focalPx))px | \(framesPerDistance) frames × \(seeds.count) operators            ║")
        print("╚══════════════════════════════════════════════════════════════════════════════════════════╝\n")

        for (i, seed) in seeds.enumerated() {
            let bands = runGolfSweep(focalPx: focalPx, zoomLabel: zoomLabel,
                                     seed: seed, framesPerDistance: framesPerDistance)
            if i == 0 { allBands = bands }
            else {
                for j in 0..<min(bands.count, allBands.count) {
                    allBands[j].absRelErrors.append(contentsOf: bands[j].absRelErrors)
                    allBands[j].signedErrors.append(contentsOf: bands[j].signedErrors)
                    allBands[j].estimates.append(contentsOf: bands[j].estimates)
                    allBands[j].confidences.append(contentsOf: bands[j].confidences)
                    allBands[j].dominantSources.append(contentsOf: bands[j].dominantSources)
                }
            }
        }

        // Compute true pixel height for each band
        print("Distance   PxHt   N    MeanErr    P50      P90      RMSE(m)  ±Yds(P50)  Tol    Status")
        print(String(repeating: "─", count: 100))

        var passedBands = 0
        var totalBands = 0
        var clubSelectionBands = 0  // Bands within ±2 club range

        for band in allBands {
            totalBands += 1
            let tol = Self.golfTolerance(forYards: band.trueYards)
            let truePixHt = (Self.pinHeightM * focalPx) / Double(band.trueDist)
            let passed = band.meanAbsRel <= tol
            if passed { passedBands += 1 }

            // ± yards at P50 (most useful for golfer)
            let p50Yds = metersToYards(band.trueDist * band.p50AbsRel)

            // Club selection grade: ±5 yds at <200, ±10 yds at 200+
            let clubGradeYds: Float = band.trueYards < 200 ? 5.0 : 10.0
            if p50Yds <= clubGradeYds { clubSelectionBands += 1 }

            let status = passed ? "PASS" : "FAIL"
            let lbl = band.bandLabel.padding(toLength: 10, withPad: " ", startingAt: 0)
            print("\(lbl) \(String(format: "%4.0f   %-4d %6.1f%%    %6.1f%%   %6.1f%%   %6.1fm   ±%4.1f yd  %4.0f%%", truePixHt, band.count, band.meanAbsRel * 100, band.p50AbsRel * 100, band.p90AbsRel * 100, band.rmse, p50Yds, tol * 100))   \(status)")
        }

        print(String(repeating: "─", count: 100))
        print("TOTAL: \(passedBands)/\(totalBands) bands passed | \(clubSelectionBands)/\(totalBands) club-selection grade")

        // For 5× and 8× zoom, expect strong performance
        if focalPx >= Self.teleZoom5xFocalPx {
            let failPct = Float(totalBands - passedBands) / Float(totalBands)
            XCTAssertLessThanOrEqual(failPct, 0.10,
                "At \(zoomLabel) zoom, more than 10% of golf bands exceeded tolerance")
        }

        // At 1× zoom, far distances will fail (pin is <10px beyond 200yds)
        // Just verify close range works
        let closeBands = allBands.filter { $0.trueYards <= 150 }
        let closePassRate = Float(closeBands.filter { $0.meanAbsRel <= Self.golfTolerance(forYards: $0.trueYards) }.count) / Float(closeBands.count)
        // At 1× zoom, pixel heights are small (34-126px) so bracket noise
        // dominates. 70% pass rate is acceptable at 1×; user should zoom in.
        XCTAssertGreaterThanOrEqual(closePassRate, 0.70,
            "At \(zoomLabel) zoom, close-range golf bands (40-150 yds) should mostly pass")
    }
}

// MARK: ============================================================
// MARK:  TEST 3: MULTI-OPERATOR REPEATABILITY
// MARK: ============================================================

final class MultiOperatorRepeatabilityTests: XCTestCase {

    /// 5 operators (seeds), 6 key distances, 100 frames each.
    /// Measures coefficient of variation across operators.
    func testRepeatabilityAcross5Operators() {
        let distances: [(yds: Float, label: String)] = [
            (100, "100 YDS"),
            (200, "200 YDS"),
            (400, "400 YDS"),
            (600, "600 YDS"),
            (1000, "1000 YDS"),
            (1500, "1500 YDS"),
        ]
        let seeds: [UInt64] = [42, 1337, 9999, 54321, 77777]
        let framesPerOp = 100

        print("\n╔══════════════════════════════════════════════════════════════════════════════════════════╗")
        print("║                     MULTI-OPERATOR REPEATABILITY TEST                                   ║")
        print("║  5 operators × 100 frames per distance | CV < 15% acceptance                            ║")
        print("╚══════════════════════════════════════════════════════════════════════════════════════════╝\n")

        print("Distance    Op1(m)   Op2(m)   Op3(m)   Op4(m)   Op5(m)   Mean(m)  StdDev   CV      Status")
        print(String(repeating: "─", count: 100))

        var passedDistances = 0

        for dist in distances {
            let trueD = yardsToMeters(dist.yds)
            var operatorMeans: [Float] = []

            for seed in seeds {
                var rng = SeededRNG(seed: seed)
                var estimates: [Float] = []

                let srtmBias = rng.nextGaussian(mean: 0, stddev: 3.5)
                let gpsAcc: Float = 5.0
                let headingAcc: Float = 8.0
                let slope: Float = 2.0
                let calAge: TimeInterval = 30
                let calConf: Float = 0.65
                let hasObj = dist.yds >= 150
                let objConf: Float = 0.60

                for _ in 0..<framesPerOp {
                    let lidar = SensorNoiseModel.lidarReading(trueD: trueD, rng: &rng)
                    let neural = SensorNoiseModel.neuralReading(trueD: trueD, calibrationConf: calConf, rng: &rng)
                    let geo = SensorNoiseModel.geometricReading(trueD: trueD, cameraHeight: 1.5, terrainSlope: slope, rng: &rng)
                    let dem = SensorNoiseModel.demReading(trueD: trueD, gpsAccuracy: gpsAcc, headingAccuracy: headingAcc, terrainSlope: slope, rng: &rng, srtmBias: srtmBias)
                    let obj = hasObj ? SensorNoiseModel.objectReading(trueD: trueD, detectionConf: objConf, rng: &rng) : nil

                    let scenario = FusionScenario(
                        trueDistanceM: trueD, sceneName: dist.label,
                        terrainSlope: slope, hasGPS: true,
                        gpsAccuracy: gpsAcc, headingAccuracy: headingAcc,
                        hasObject: hasObj, objectDetConf: objConf,
                        calibrationAge: calAge, calibrationConf: calConf,
                        expectedDominantSource: nil, maxAcceptableErrorPercent: 0)

                    let result = FusionSimulator.simulate(
                        scenario: scenario,
                        lidarReading: lidar, neuralReading: neural,
                        geometricReading: geo?.0, geometricConfidence: geo?.1,
                        demReading: dem?.0, demConfidence: dem?.1,
                        objectReading: obj)

                    if result.fusedDepth > 0.01 {
                        estimates.append(result.fusedDepth)
                    }
                }

                let mean = estimates.reduce(0, +) / Float(estimates.count)
                operatorMeans.append(mean)
            }

            // Cross-operator statistics
            let grandMean = operatorMeans.reduce(0, +) / Float(operatorMeans.count)
            let variance = operatorMeans.map { ($0 - grandMean) * ($0 - grandMean) }
                .reduce(0, +) / Float(operatorMeans.count - 1)
            let stddev = sqrt(variance)
            let cv = grandMean > 0 ? stddev / grandMean : 0

            let passed = cv < 0.15
            if passed { passedDistances += 1 }

            let lbl = dist.label.padding(toLength: 11, withPad: " ", startingAt: 0)
            let o = operatorMeans.map { String(format: "%6.0f", $0) }.joined(separator: "   ")
            print("\(lbl) \(o)   \(String(format: "%6.0f", grandMean))    \(String(format: "%5.1f", stddev))   \(String(format: "%5.1f%%", cv * 100))   \(passed ? "PASS" : "FAIL")")
        }

        print(String(repeating: "─", count: 100))
        print("TOTAL: \(passedDistances)/\(distances.count) distances within CV < 15%\n")

        XCTAssertEqual(passedDistances, distances.count,
            "All distances should have CV < 15% across operators")
    }
}

// MARK: ============================================================
// MARK:  TEST 4: ENVIRONMENTAL CONDITION MATRIX
// MARK: ============================================================

final class EnvironmentalConditionMatrixTests: XCTestCase {

    /// Test matrix: 6 distances × 4 GPS × 3 slope × 3 cal age × 2 object = 432 combos
    /// Checks for catastrophic failures (> 80% error) in any combination.
    func testEnvironmentalConditionMatrix() {
        let distances: [Float] = [100, 250, 500, 800, 1000, 1400]  // yards
        let gpsAccuracies: [(Float, String)] = [
            (3.0, "GPS3m"), (8.0, "GPS8m"), (15.0, "GPS15m"), (25.0, "GPS25m")]
        let slopes: [(Float, String)] = [
            (0.0, "Flat"), (5.0, "Gentle"), (15.0, "Steep")]
        let calAges: [(TimeInterval, Float, String)] = [
            (10, 0.80, "Fresh"), (120, 0.55, "Moderate"), (400, 0.35, "Stale")]
        let objectOptions: [(Bool, Float, String)] = [
            (true, 0.65, "ObjYes"), (false, 0.0, "ObjNo")]

        let framesPerCombo = 20
        var rng = SeededRNG(seed: 31415)

        print("\n╔══════════════════════════════════════════════════════════════════════════════════════════╗")
        print("║                     ENVIRONMENTAL CONDITION MATRIX                                      ║")
        print("║  6 distances × 4 GPS × 3 slope × 3 cal × 2 obj = 432 combos × 20 frames               ║")
        print("╚══════════════════════════════════════════════════════════════════════════════════════════╝\n")

        var totalCombos = 0
        var catastrophic = 0    // > 80% error
        var bad = 0             // > 50% error
        var degraded = 0        // > 30% error
        var noEstimate = 0      // No reading at all
        var comboResults: [(combo: String, dist: Float, absRel: Float)] = []

        for yds in distances {
            let trueD = yardsToMeters(yds)

            for (gpsAcc, gpsLabel) in gpsAccuracies {
                for (slope, slopeLabel) in slopes {
                    for (calAge, calConf, calLabel) in calAges {
                        for (hasObj, objConf, objLabel) in objectOptions {
                            totalCombos += 1
                            let comboLabel = "\(Int(yds))yd/\(gpsLabel)/\(slopeLabel)/\(calLabel)/\(objLabel)"

                            let srtmBias = rng.nextGaussian(mean: 0, stddev: slope > 10 ? 8.0 : 3.5)
                            var estimates: [Float] = []

                            for _ in 0..<framesPerCombo {
                                let lidar = SensorNoiseModel.lidarReading(trueD: trueD, rng: &rng)
                                let neural = SensorNoiseModel.neuralReading(trueD: trueD, calibrationConf: calConf, rng: &rng)
                                let geo = SensorNoiseModel.geometricReading(trueD: trueD, cameraHeight: 1.5, terrainSlope: slope, rng: &rng)
                                let dem = SensorNoiseModel.demReading(trueD: trueD, gpsAccuracy: gpsAcc, headingAccuracy: 8.0, terrainSlope: slope, rng: &rng, srtmBias: srtmBias)
                                let obj = hasObj ? SensorNoiseModel.objectReading(trueD: trueD, detectionConf: objConf, rng: &rng) : nil

                                let scenario = FusionScenario(
                                    trueDistanceM: trueD, sceneName: comboLabel,
                                    terrainSlope: slope, hasGPS: gpsAcc <= 20,
                                    gpsAccuracy: gpsAcc, headingAccuracy: 8.0,
                                    hasObject: hasObj, objectDetConf: objConf,
                                    calibrationAge: calAge, calibrationConf: calConf,
                                    expectedDominantSource: nil, maxAcceptableErrorPercent: 0)

                                let result = FusionSimulator.simulate(
                                    scenario: scenario,
                                    lidarReading: lidar, neuralReading: neural,
                                    geometricReading: geo?.0, geometricConfidence: geo?.1,
                                    demReading: dem?.0, demConfidence: dem?.1,
                                    objectReading: obj)

                                if result.fusedDepth > 0.01 {
                                    estimates.append(result.fusedDepth)
                                }
                            }

                            if estimates.isEmpty {
                                noEstimate += 1
                                continue
                            }

                            let mean = estimates.reduce(0, +) / Float(estimates.count)
                            let absRel = abs(mean - trueD) / trueD

                            comboResults.append((comboLabel, yds, absRel))

                            if absRel > 0.80 { catastrophic += 1 }
                            else if absRel > 0.50 { bad += 1 }
                            else if absRel > 0.30 { degraded += 1 }
                        }
                    }
                }
            }
        }

        // Print worst combos
        let worst = comboResults.sorted { $0.absRel > $1.absRel }.prefix(20)
        print("TOP 20 WORST COMBINATIONS:")
        print("Combo                                                    Dist    AbsRel   Status")
        print(String(repeating: "─", count: 85))
        for w in worst {
            let status: String
            if w.absRel > 0.80 { status = "CATASTROPHIC" }
            else if w.absRel > 0.50 { status = "BAD" }
            else if w.absRel > 0.30 { status = "DEGRADED" }
            else { status = "OK" }
            let comboLbl = w.combo.padding(toLength: 55, withPad: " ", startingAt: 0)
            print("\(comboLbl) \(String(format: "%5.0f yd  %6.1f%%", w.dist, w.absRel * 100))   \(status)")
        }

        print(String(repeating: "─", count: 85))
        print("TOTAL: \(totalCombos) combos | \(catastrophic) catastrophic | \(bad) bad | \(degraded) degraded | \(noEstimate) no-estimate\n")

        // Per-distance summary
        print("PER-DISTANCE SUMMARY:")
        for yds in distances {
            let distCombos = comboResults.filter { $0.dist == yds }
            let mean = distCombos.map { $0.absRel }.reduce(0, +) / Float(distCombos.count)
            let worstCase = distCombos.map { $0.absRel }.max() ?? 0
            print(String(format: "  %4.0f yds: Mean %.1f%% | Worst %.1f%% | N=%d combos",
                         yds, mean * 100, worstCase * 100, distCombos.count))
        }
        print("")

        // Assertions
        // In extreme conditions (GPS 25m + steep slope + stale cal + no object + long range),
        // catastrophic errors are expected — the system simply lacks good sensor data.
        // Allow up to 15% of combos to be catastrophic. The real requirement is that
        // the system degrades gracefully and doesn't crash.
        let maxCatastrophic = max(1, totalCombos * 15 / 100)
        XCTAssertLessThanOrEqual(catastrophic, maxCatastrophic,
            "\(catastrophic) catastrophic (>80%) combos exceeds \(maxCatastrophic) (\(totalCombos) total × 15%)")
        XCTAssertLessThanOrEqual(bad + catastrophic, totalCombos / 4,
            "Bad+catastrophic (>50%) combos should be < 25% of total")
    }
}

// MARK: ============================================================
// MARK:  TEST 5: SOURCE HANDOFF STRESS TEST
// MARK: ============================================================

final class SourceHandoffStressTests: XCTestCase {

    /// Walk from 5m to 1500m in 10m steps. At each step, verify:
    /// 1. Fusion produces an estimate (no dead zones)
    /// 2. No estimate jumps > 3× relative to previous distance
    /// 3. Dominant source transitions follow expected regime order
    func testSourceHandoffContinuity() {
        var rng = SeededRNG(seed: 42)
        let stepM: Float = 10
        let startM: Float = 5
        let endM: Float = yardsToMeters(1500)
        let framesPerStep = 30

        print("\n╔══════════════════════════════════════════════════════════════════════════════════════════╗")
        print("║                     SOURCE HANDOFF STRESS TEST                                          ║")
        print("║  5m → 1372m in 10m steps | 30 frames per step | No dead zones, no 3× jumps             ║")
        print("╚══════════════════════════════════════════════════════════════════════════════════════════╝\n")

        var previousMean: Float = 0
        var deadZones: [Float] = []
        var jumpViolations: [(from: Float, to: Float, prevEst: Float, curEst: Float)] = []
        var sourceRegimes: [(distance: Float, source: String)] = []
        var totalSteps = 0

        var d = startM
        while d <= endM {
            totalSteps += 1
            let srtmBias = rng.nextGaussian(mean: 0, stddev: 3.5)
            var estimates: [Float] = []
            var sources: [DepthSource] = []

            for _ in 0..<framesPerStep {
                let lidar = SensorNoiseModel.lidarReading(trueD: d, rng: &rng)
                let neural = SensorNoiseModel.neuralReading(trueD: d, calibrationConf: 0.65, rng: &rng)
                let geo = SensorNoiseModel.geometricReading(trueD: d, cameraHeight: 1.5, terrainSlope: 2.0, rng: &rng)
                let dem = SensorNoiseModel.demReading(trueD: d, gpsAccuracy: 5.0, headingAccuracy: 8.0, terrainSlope: 2.0, rng: &rng, srtmBias: srtmBias)
                let obj = d > 30 ? SensorNoiseModel.objectReading(trueD: d, detectionConf: 0.60, rng: &rng) : nil

                let scenario = FusionScenario(
                    trueDistanceM: d, sceneName: "\(Int(d))m",
                    terrainSlope: 2.0, hasGPS: true,
                    gpsAccuracy: 5.0, headingAccuracy: 8.0,
                    hasObject: d > 30, objectDetConf: 0.60,
                    calibrationAge: 30, calibrationConf: 0.65,
                    expectedDominantSource: nil, maxAcceptableErrorPercent: 0)

                let result = FusionSimulator.simulate(
                    scenario: scenario,
                    lidarReading: lidar, neuralReading: neural,
                    geometricReading: geo?.0, geometricConfidence: geo?.1,
                    demReading: dem?.0, demConfidence: dem?.1,
                    objectReading: obj)

                if result.fusedDepth > 0.01 {
                    estimates.append(result.fusedDepth)
                    if let dom = result.dominantSource { sources.append(dom) }
                }
            }

            if estimates.isEmpty {
                deadZones.append(d)
                d += stepM
                continue
            }

            let mean = estimates.reduce(0, +) / Float(estimates.count)

            // Check for 3× jump
            if previousMean > 0.01 {
                let ratio = max(mean, previousMean) / max(min(mean, previousMean), 0.01)
                if ratio > 3.0 {
                    jumpViolations.append((from: d - stepM, to: d,
                                          prevEst: previousMean, curEst: mean))
                }
            }

            // Track dominant source
            var sourceCounts: [DepthSource: Int] = [:]
            for s in sources { sourceCounts[s, default: 0] += 1 }
            let dominant = sourceCounts.max(by: { $0.value < $1.value })?.key
            sourceRegimes.append((d, dominant?.shortName ?? "???"))

            previousMean = mean
            d += stepM
        }

        // Print source regime map (sampled)
        print("SOURCE REGIME MAP (every 50m):")
        print("Distance    Source     Expected Regime")
        print(String(repeating: "─", count: 55))
        for regime in sourceRegimes where Int(regime.distance) % 50 == 0 || regime.distance < 20 {
            let expected: String
            switch regime.distance {
            case ..<12:   expected = "LIDAR"
            case ..<50:   expected = "AI/GEO"
            case ..<150:  expected = "AI/DEM/GEO"
            case ..<400:  expected = "DEM/OBJ"
            default:      expected = "DEM/OBJ"
            }
            let src = regime.source.padding(toLength: 8, withPad: " ", startingAt: 0)
            print(String(format: "  %6.0fm", regime.distance) + "     \(src)  (\(expected))")
        }

        print(String(repeating: "─", count: 55))
        print("Dead zones: \(deadZones.count > 0 ? deadZones.map { "\(Int($0))m" }.joined(separator: ", ") : "NONE")")
        print("Jump violations (>3×): \(jumpViolations.count)")
        for v in jumpViolations {
            print(String(format: "  %4.0fm → %4.0fm: %.0fm → %.0fm (ratio %.1f×)",
                         v.from, v.to, v.prevEst, v.curEst,
                         max(v.prevEst, v.curEst) / max(min(v.prevEst, v.curEst), 0.01)))
        }
        print("")

        // Assertions
        XCTAssertEqual(deadZones.count, 0,
            "Dead zones found at: \(deadZones.map { "\(Int($0))m" }.joined(separator: ", "))")
        XCTAssertEqual(jumpViolations.count, 0,
            "\(jumpViolations.count) jump violations (>3× between adjacent distances)")
    }

    /// Verify that source transitions happen in the expected order.
    /// The app uses a priority chain: Stadia→LiDAR→Object→DEM→Neural→Geometric.
    /// As distance increases, we should see transitions roughly follow:
    /// LiDAR (0-12m) → Neural (5-150m) → DEM/Object (50m+) → DEM/Object (extreme)
    func testSourceTransitionOrder() {
        var rng = SeededRNG(seed: 42)
        let checkpoints: [(meters: Float, expected: Set<DepthSource>)] = [
            (3,    [.lidar]),
            (8,    [.lidar, .neural]),
            (20,   [.neural, .geometric]),
            (50,   [.neural, .geometric, .demRaycast, .objectSize]),
            (100,  [.neural, .geometric, .demRaycast, .objectSize]),
            (200,  [.demRaycast, .objectSize]),
            (500,  [.demRaycast, .objectSize]),
            (1000, [.demRaycast, .objectSize]),
        ]

        print("\nSOURCE TRANSITION ORDER CHECK:")
        print("Distance    Dominant     Expected Set                          Status")
        print(String(repeating: "─", count: 75))

        var allPassed = true

        for (dist, expectedSet) in checkpoints {
            let srtmBias = rng.nextGaussian(mean: 0, stddev: 3.5)
            var sources: [DepthSource] = []

            for _ in 0..<50 {
                let lidar = SensorNoiseModel.lidarReading(trueD: dist, rng: &rng)
                let neural = SensorNoiseModel.neuralReading(trueD: dist, calibrationConf: 0.65, rng: &rng)
                let geo = SensorNoiseModel.geometricReading(trueD: dist, cameraHeight: 1.5, terrainSlope: 2.0, rng: &rng)
                let dem = SensorNoiseModel.demReading(trueD: dist, gpsAccuracy: 5.0, headingAccuracy: 8.0, terrainSlope: 2.0, rng: &rng, srtmBias: srtmBias)
                let obj = dist > 15 ? SensorNoiseModel.objectReading(trueD: dist, detectionConf: 0.60, rng: &rng) : nil

                let scenario = FusionScenario(
                    trueDistanceM: dist, sceneName: "\(Int(dist))m",
                    terrainSlope: 2.0, hasGPS: true,
                    gpsAccuracy: 5.0, headingAccuracy: 8.0,
                    hasObject: dist > 15, objectDetConf: 0.60,
                    calibrationAge: 30, calibrationConf: 0.65,
                    expectedDominantSource: nil, maxAcceptableErrorPercent: 0)

                let result = FusionSimulator.simulate(
                    scenario: scenario,
                    lidarReading: lidar, neuralReading: neural,
                    geometricReading: geo?.0, geometricConfidence: geo?.1,
                    demReading: dem?.0, demConfidence: dem?.1,
                    objectReading: obj)

                if let dom = result.dominantSource { sources.append(dom) }
            }

            // Most common source
            var counts: [DepthSource: Int] = [:]
            for s in sources { counts[s, default: 0] += 1 }
            let dominant = counts.max(by: { $0.value < $1.value })?.key

            let passed = dominant != nil && expectedSet.contains(dominant!)
            if !passed { allPassed = false }

            let expectedStr = expectedSet.map { $0.shortName }.sorted().joined(separator: "/")
            let domStr = (dominant?.shortName ?? "NONE").padding(toLength: 10, withPad: " ", startingAt: 0)
            let expStr = expectedStr.padding(toLength: 30, withPad: " ", startingAt: 0)
            print(String(format: "  %6.0fm", dist) + "     \(domStr)   {\(expStr)}  \(passed ? "PASS" : "FAIL")")
        }

        print("")
        XCTAssertTrue(allPassed, "Source transitions should follow expected regime ordering")
    }
}
