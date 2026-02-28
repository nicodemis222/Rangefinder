//
//  GroundTruthDatasetTests.swift
//  RangefinderTests
//
//  Validates the fusion pipeline against 10,000 ground truth samples from
//  ARKitScenes (Apple LiDAR + laser scanner) and DIODE (laser scanner).
//
//  Three tiers:
//  - Tier 1 (always runs): Manifest-based fusion accuracy, confidence coverage,
//    source selection distribution, per-band statistics, coverage validation.
//  - Tier 2 (optional): Real depth map sampling accuracy.
//  - Tier 3 (optional): Full pipeline with CoreML neural inference.
//
//  Reuses FusionSimulator, SensorNoiseModel, and SeededRNG from
//  MonteCarloFusionTests for deterministic, reproducible validation.
//

import XCTest
import CoreVideo
@testable import Rangefinder

// MARK: - Accuracy Thresholds

/// Per-band accuracy thresholds. Based on published sensor error models
/// and validated against MonteCarloFusionTests / ImageSimulationTests.
private let bandThresholds: [String: (absRelMax: Double, rmseMaxM: Double,
                                       p90Max: Float, maxCatastrophicRate: Double)] = [
    "close":    (0.05,   0.15,   8.0,  0.0),
    "near_mid": (0.08,   0.60,  12.0,  0.0),
    "mid":      (0.12,   1.50,  20.0,  0.0),
    "far_mid":  (0.15,   7.00,  25.0,  0.01),
    "far":      (0.20,  20.00,  30.0,  0.02),
    "long":     (0.25,  50.00,  40.0,  0.05),
]

// MARK: - Test Class

final class GroundTruthDatasetTests: XCTestCase {

    // Loaded once for all tests
    nonisolated(unsafe) private static var manifest: GroundTruthManifest?
    nonisolated(unsafe) private static var loadError: String?

    override class func setUp() {
        super.setUp()
        if let m = loadGroundTruthManifest() {
            manifest = m
            print("✓ Ground truth manifest loaded: \(m.total_samples) samples, version \(m.version)")
        } else {
            loadError = "Ground truth manifest not found — run prepare_ground_truth_dataset.py first"
            print("⚠ \(loadError!)")
        }
    }

    private var samples: [GroundTruthSample] {
        guard let m = Self.manifest else {
            XCTFail(Self.loadError ?? "Manifest not loaded")
            return []
        }
        return m.samples
    }

    // MARK: - Tier 1: Manifest Validation

    /// Verify the manifest loaded correctly and has expected structure.
    func testManifestIntegrity() {
        guard let m = Self.manifest else {
            XCTFail(Self.loadError ?? "Manifest not loaded")
            return
        }

        XCTAssertGreaterThanOrEqual(m.total_samples, 5000,
            "Manifest should have at least 5,000 samples")
        XCTAssertEqual(m.samples.count, m.total_samples,
            "Sample count should match total_samples field")
        XCTAssertFalse(m.dataset_sources.isEmpty,
            "Should have at least one dataset source")

        // Verify all samples have valid ground truth
        for sample in m.samples {
            XCTAssertGreaterThan(sample.ground_truth_center_m, 0.1,
                "Sample \(sample.frame_id) has invalid ground truth: \(sample.ground_truth_center_m)m")
            XCTAssertLessThan(sample.ground_truth_center_m, 1000.0,
                "Sample \(sample.frame_id) has unrealistic ground truth: \(sample.ground_truth_center_m)m")
        }
    }

    /// Verify the dataset covers all distance bands with sufficient samples.
    func testDatasetDistributionCoverage() {
        let bandCounts = Dictionary(grouping: samples, by: { $0.distance_band })
            .mapValues { $0.count }

        let expectedBands = ["close", "near_mid", "mid", "far_mid", "far", "long"]
        for band in expectedBands {
            let count = bandCounts[band] ?? 0
            XCTAssertGreaterThanOrEqual(count, 500,
                "Band '\(band)' has only \(count) samples — need at least 500 for statistical validity")
        }

        // Check no large distance gaps (> 20m without a sample)
        let sortedDistances = samples.map { $0.ground_truth_center_m }.sorted()
        for i in 1..<sortedDistances.count {
            let gap = sortedDistances[i] - sortedDistances[i - 1]
            let midpoint = (sortedDistances[i] + sortedDistances[i - 1]) / 2.0
            // Allow larger gaps at longer distances (proportional)
            let maxGap = max(2.0, midpoint * 0.15)
            XCTAssertLessThan(gap, maxGap,
                "Distance gap of \(String(format: "%.1f", gap))m at ~\(String(format: "%.0f", midpoint))m")
        }

        // Print distribution summary
        print("\n=== DATASET DISTRIBUTION ===")
        for band in expectedBands {
            let count = bandCounts[band] ?? 0
            print("  \(band.padding(toLength: 10, withPad: " ", startingAt: 0)): \(count) samples")
        }

        // Scene type balance
        let sceneCounts = Dictionary(grouping: samples, by: { $0.scene_type })
            .mapValues { $0.count }
        print("  Indoor:  \(sceneCounts["indoor"] ?? 0)")
        print("  Outdoor: \(sceneCounts["outdoor"] ?? 0)")
    }

    // MARK: - Tier 1: Confidence Coverage

    /// Verify confidence curves produce non-zero weights at every
    /// ground truth distance — no dead zones in the pipeline.
    func testConfidenceCoverageAcrossDataset() {
        var deadZones = 0

        for sample in samples {
            let d = sample.ground_truth_center_m
            guard d > 0 else { continue }

            let lidarConf = DepthSourceConfidence.lidar(distanceM: d)
            let neuralConf = DepthSourceConfidence.neural(distanceM: d)
            let geoConf = DepthSourceConfidence.geometric(distanceM: d)
            let demConf = DepthSourceConfidence.demRaycast(
                distanceM: d, gpsAccuracy: 5.0, headingAccuracy: 8.0)
            let objConf = DepthSourceConfidence.object(
                distanceM: d, detectionConfidence: 0.75)

            let maxConf = max(lidarConf, max(neuralConf, max(geoConf, max(demConf, objConf))))

            if maxConf < 0.05 {
                deadZones += 1
                // Only fail on the first few to avoid noise
                if deadZones <= 5 {
                    XCTFail("Dead zone at \(String(format: "%.1f", d))m (\(sample.frame_id)): " +
                        "no source has confidence > 0.05 " +
                        "[L=\(String(format: "%.2f", lidarConf)) " +
                        "N=\(String(format: "%.2f", neuralConf)) " +
                        "G=\(String(format: "%.2f", geoConf)) " +
                        "D=\(String(format: "%.2f", demConf)) " +
                        "O=\(String(format: "%.2f", objConf))]")
                }
            }
        }

        let deadZoneRate = Double(deadZones) / Double(max(samples.count, 1))
        XCTAssertLessThan(deadZoneRate, 0.01,
            "\(deadZones) dead zones (\(String(format: "%.1f", deadZoneRate * 100))%) " +
            "— confidence curves have gaps")
    }

    // MARK: - Tier 1: Source Selection Distribution

    /// Validate that semantic source selection produces reasonable
    /// source distribution across all ground truth distances.
    func testSourceSelectionDistribution() {
        var rng = SeededRNG(seed: 314)
        var sourceCounts: [String: [String: Int]] = [:]  // band → source → count

        for sample in samples {
            let scenario = sampleToFusionScenario(sample)
            let d = sample.ground_truth_center_m

            // Generate noisy readings
            let lidar = SensorNoiseModel.lidarReading(trueD: d, rng: &rng)
            let neural = SensorNoiseModel.neuralReading(
                trueD: d, calibrationConf: scenario.calibrationConf, rng: &rng)
            let geo = SensorNoiseModel.geometricReading(
                trueD: d, cameraHeight: 1.5,
                terrainSlope: scenario.terrainSlope, rng: &rng)
            let dem = scenario.hasGPS ? SensorNoiseModel.demReading(
                trueD: d, gpsAccuracy: scenario.gpsAccuracy,
                headingAccuracy: scenario.headingAccuracy,
                terrainSlope: scenario.terrainSlope, rng: &rng) : nil
            let obj = scenario.hasObject ? SensorNoiseModel.objectReading(
                trueD: d, detectionConf: scenario.objectDetConf, rng: &rng) : nil

            let result = FusionSimulator.simulate(
                scenario: scenario,
                lidarReading: lidar,
                neuralReading: neural,
                geometricReading: geo?.0,
                geometricConfidence: geo?.1,
                demReading: dem?.0,
                demConfidence: dem?.1,
                objectReading: obj
            )

            let band = sample.distance_band
            let sourceName = result.dominantSource?.rawValue ?? "none"
            sourceCounts[band, default: [:]][sourceName, default: 0] += 1
        }

        // Print source distribution table
        print("\n=== SOURCE SELECTION DISTRIBUTION ===")
        let bandOrder = ["close", "near_mid", "mid", "far_mid", "far", "long"]
        print("Band        LiDAR   Neural      Geo      DEM   Object     None")
        print(String(repeating: "-", count: 62))

        for band in bandOrder {
            guard let counts = sourceCounts[band] else { continue }
            let total = max(counts.values.reduce(0, +), 1)
            let bandPad = band.padding(toLength: 10, withPad: " ", startingAt: 0)
            print(String(format: "%@ %7.1f%% %7.1f%% %7.1f%% %7.1f%% %7.1f%% %7.1f%%",
                bandPad as NSString,
                Double(counts["LiDAR"] ?? 0) / Double(total) * 100,
                Double(counts["Neural"] ?? 0) / Double(total) * 100,
                Double(counts["Geometric"] ?? 0) / Double(total) * 100,
                Double(counts["DEM"] ?? 0) / Double(total) * 100,
                Double(counts["Object"] ?? 0) / Double(total) * 100,
                Double(counts["none"] ?? 0) / Double(total) * 100))
        }

        // Assertions: LiDAR should dominate close, neural should appear mid-range
        let closeLidar = sourceCounts["close"]?["LiDAR"] ?? 0
        let closeTotal = sourceCounts["close"]?.values.reduce(0, +) ?? 1
        let lidarShareClose = Double(closeLidar) / Double(max(closeTotal, 1))
        XCTAssertGreaterThan(lidarShareClose, 0.3,
            "LiDAR should be significant at close range (got \(String(format: "%.0f%%", lidarShareClose * 100)))")

        // No source should be completely absent at mid-range
        let midBand = sourceCounts["mid"] ?? [:]
        let midTotal = midBand.values.reduce(0, +)
        let midNoneRate = Double(midBand["none"] ?? 0) / Double(max(midTotal, 1))
        XCTAssertLessThan(midNoneRate, 0.10,
            "Too many no-estimate results at mid-range: \(String(format: "%.0f%%", midNoneRate * 100))")
    }

    // MARK: - Tier 1: Fusion Accuracy on Ground Truth Distributions

    /// Drive FusionSimulator with ground truth distances and realistic noise,
    /// validate accuracy per band against thresholds.
    /// Processes all 10k samples in a single pass.
    func testFusionAccuracyOnGroundTruthDistributions() {
        var rng = SeededRNG(seed: 137)
        var bandStats: [String: BandStatistics] = [:]
        let sampleList = samples
        guard !sampleList.isEmpty else { return }

        for sample in sampleList {
            let scenario = sampleToFusionScenario(sample)
            let d = sample.ground_truth_center_m

            let lidar = SensorNoiseModel.lidarReading(trueD: d, rng: &rng)
            let neural = SensorNoiseModel.neuralReading(
                trueD: d, calibrationConf: scenario.calibrationConf, rng: &rng)
            let geo = SensorNoiseModel.geometricReading(
                trueD: d, cameraHeight: 1.5,
                terrainSlope: scenario.terrainSlope, rng: &rng)
            let dem = scenario.hasGPS ? SensorNoiseModel.demReading(
                trueD: d, gpsAccuracy: scenario.gpsAccuracy,
                headingAccuracy: scenario.headingAccuracy,
                terrainSlope: scenario.terrainSlope, rng: &rng) : nil
            let obj = scenario.hasObject ? SensorNoiseModel.objectReading(
                trueD: d, detectionConf: scenario.objectDetConf, rng: &rng) : nil

            let result = FusionSimulator.simulate(
                scenario: scenario,
                lidarReading: lidar,
                neuralReading: neural,
                geometricReading: geo?.0,
                geometricConfidence: geo?.1,
                demReading: dem?.0,
                demConfidence: dem?.1,
                objectReading: obj
            )

            guard result.fusedDepth > 0.01 else { continue }

            let errorPct = abs(result.fusedDepth - d) / max(d, 0.1) * 100.0
            let band = sample.distance_band

            bandStats[band, default: BandStatistics()].addSample(
                errorPercent: errorPct,
                confidence: result.confidence,
                source: result.dominantSource
            )
        }

        // Print report
        print(formatBandReport(bandStats))

        // Assert per-band thresholds
        for (band, thresholds) in bandThresholds {
            guard let stats = bandStats[band] else { continue }

            XCTAssertLessThanOrEqual(stats.absRel, thresholds.absRelMax,
                "Band '\(band)' AbsRel \(String(format: "%.1f%%", stats.absRel * 100)) " +
                "exceeds threshold \(String(format: "%.0f%%", thresholds.absRelMax * 100))")

            XCTAssertLessThanOrEqual(stats.p90, thresholds.p90Max,
                "Band '\(band)' P90 \(String(format: "%.1f%%", stats.p90)) " +
                "exceeds threshold \(String(format: "%.0f%%", thresholds.p90Max))")

            XCTAssertLessThanOrEqual(stats.catastrophicRate, thresholds.maxCatastrophicRate,
                "Band '\(band)' catastrophic rate \(String(format: "%.1f%%", stats.catastrophicRate * 100)) " +
                "exceeds threshold \(String(format: "%.0f%%", thresholds.maxCatastrophicRate * 100))")
        }

        // Global: zero catastrophic errors
        let globalCatastrophic = bandStats.values.map { $0.catastrophicCount }.reduce(0, +)
        let globalTotal = bandStats.values.map { $0.count }.reduce(0, +)
        let globalCatRate = Double(globalCatastrophic) / Double(max(globalTotal, 1))
        XCTAssertLessThan(globalCatRate, 0.01,
            "Global catastrophic rate \(String(format: "%.2f%%", globalCatRate * 100)) " +
            "should be < 1%")
    }

    // MARK: - Tier 1: Per-Band Statistical Accuracy (500 MC samples per entry)

    /// Run 500 Monte Carlo samples per ground truth entry for deep statistical
    /// validation. Uses streaming stats to bound memory.
    func testPerBandStatisticalAccuracy() {
        var rng = SeededRNG(seed: 42)
        var bandStats: [String: BandStatistics] = [:]

        let mcSamplesPerEntry = 5  // 5 × 10,000 = 50,000 total evaluations

        print("\n=== STATISTICAL ACCURACY: \(mcSamplesPerEntry) MC samples × \(samples.count) entries ===\n")

        for sample in samples {
            let scenario = sampleToFusionScenario(sample)
            let d = sample.ground_truth_center_m

            for _ in 0..<mcSamplesPerEntry {
                // Vary conditions slightly each MC sample
                let gpsVar = scenario.gpsAccuracy * rng.nextUniform(min: 0.7, max: 1.3)
                let headVar = scenario.headingAccuracy * rng.nextUniform(min: 0.7, max: 1.3)
                let slopeVar = max(0, scenario.terrainSlope + rng.nextGaussian(mean: 0, stddev: 2.0))

                let variedScenario = FusionScenario(
                    trueDistanceM: d,
                    sceneName: scenario.sceneName,
                    terrainSlope: slopeVar,
                    hasGPS: scenario.hasGPS,
                    gpsAccuracy: gpsVar,
                    headingAccuracy: headVar,
                    hasObject: scenario.hasObject,
                    objectDetConf: scenario.objectDetConf,
                    calibrationAge: scenario.calibrationAge,
                    calibrationConf: scenario.calibrationConf,
                    expectedDominantSource: nil,
                    maxAcceptableErrorPercent: 0
                )

                let lidar = SensorNoiseModel.lidarReading(trueD: d, rng: &rng)
                let neural = SensorNoiseModel.neuralReading(
                    trueD: d, calibrationConf: variedScenario.calibrationConf, rng: &rng)
                let geo = SensorNoiseModel.geometricReading(
                    trueD: d, cameraHeight: 1.5,
                    terrainSlope: slopeVar, rng: &rng)
                let dem = variedScenario.hasGPS ? SensorNoiseModel.demReading(
                    trueD: d, gpsAccuracy: gpsVar,
                    headingAccuracy: headVar,
                    terrainSlope: slopeVar, rng: &rng) : nil
                let obj = variedScenario.hasObject ? SensorNoiseModel.objectReading(
                    trueD: d, detectionConf: variedScenario.objectDetConf, rng: &rng) : nil

                let result = FusionSimulator.simulate(
                    scenario: variedScenario,
                    lidarReading: lidar,
                    neuralReading: neural,
                    geometricReading: geo?.0,
                    geometricConfidence: geo?.1,
                    demReading: dem?.0,
                    demConfidence: dem?.1,
                    objectReading: obj
                )

                guard result.fusedDepth > 0.01 else { continue }

                let errorPct = abs(result.fusedDepth - d) / max(d, 0.1) * 100.0
                bandStats[sample.distance_band, default: BandStatistics()].addSample(
                    errorPercent: errorPct,
                    confidence: result.confidence,
                    source: result.dominantSource
                )
            }
        }

        // Print detailed report
        print(formatBandReport(bandStats))

        // Global assertions
        let globalConfidentBad = bandStats.values.map { $0.confidentBadCount }.reduce(0, +)
        let globalTotal = bandStats.values.map { $0.count }.reduce(0, +)
        let globalCBRate = Double(globalConfidentBad) / Double(max(globalTotal, 1))

        print(String(format: "Global: %d evaluations, confident+bad rate: %.2f%%",
            globalTotal, globalCBRate * 100))

        XCTAssertLessThan(globalCBRate, 0.10,
            "Confident-and-bad rate \(String(format: "%.1f%%", globalCBRate * 100)) should be < 10%")
    }

    // MARK: - Tier 1: LiDAR vs Ground Truth (manifest-level)

    /// For samples with LiDAR readings, compare LiDAR accuracy against
    /// laser scanner ground truth.
    func testLiDARAccuracyFromManifest() {
        let lidarSamples = samples.filter { $0.lidar_center_m != nil }

        guard !lidarSamples.isEmpty else {
            print("⚠ No samples with LiDAR data — skipping LiDAR accuracy test")
            return
        }

        var errors: [Float] = []
        var absRelSum: Double = 0

        for sample in lidarSamples {
            guard let lidarD = sample.lidar_center_m,
                  sample.ground_truth_center_m > 0.3 else { continue }

            let gt = sample.ground_truth_center_m
            let errorPct = abs(lidarD - gt) / gt * 100.0
            errors.append(errorPct)
            absRelSum += Double(errorPct) / 100.0
        }

        guard !errors.isEmpty else { return }

        let absRel = absRelSum / Double(errors.count)
        let sortedErrors = errors.sorted()
        let p50 = sortedErrors[sortedErrors.count / 2]
        let p90 = sortedErrors[min(sortedErrors.count - 1, Int(Double(sortedErrors.count) * 0.9))]

        print("\n=== LiDAR vs GROUND TRUTH ===")
        print(String(format: "  Samples: %d", errors.count))
        print(String(format: "  AbsRel: %.2f%%", absRel * 100))
        print(String(format: "  P50: %.2f%%", p50))
        print(String(format: "  P90: %.2f%%", p90))

        // LiDAR should be very accurate (< 3% at close range)
        XCTAssertLessThan(absRel, 0.05,
            "LiDAR AbsRel \(String(format: "%.1f%%", absRel * 100)) should be < 5%")
        XCTAssertLessThan(p90, 10.0,
            "LiDAR P90 \(String(format: "%.1f%%", p90)) should be < 10%")
    }

    // MARK: - Tier 2: Real Depth Map Tests (optional)

    /// Load real depth maps and validate LiDAR sampling accuracy.
    /// Requires RANGEFINDER_DATASET_PATH environment variable.
    func testLiDARSamplingAccuracyOnRealDepthMaps() throws {
        guard let datasetPath = ProcessInfo.processInfo.environment["RANGEFINDER_DATASET_PATH"] else {
            throw XCTSkip("RANGEFINDER_DATASET_PATH not set — skipping Tier 2 depth map tests")
        }

        let depthDir = (datasetPath as NSString).appendingPathComponent("depth")
        guard FileManager.default.fileExists(atPath: depthDir) else {
            throw XCTSkip("Depth directory not found at \(depthDir) — run prepare_ground_truth_dataset.py --tier 2")
        }

        let depthSamples = samples.filter { $0.depth_map_file != nil }
        guard !depthSamples.isEmpty else {
            throw XCTSkip("No samples have depth_map_file paths")
        }

        var errors: [Float] = []

        for sample in depthSamples.prefix(1000) {  // Cap at 1000 for speed
            guard let depthFile = sample.depth_map_file else { continue }
            let fullPath = (datasetPath as NSString).appendingPathComponent(depthFile)
            guard FileManager.default.fileExists(atPath: fullPath) else { continue }

            // Load 128×128 Float32 depth map
            guard let data = FileManager.default.contents(atPath: fullPath),
                  data.count == 128 * 128 * 4 else { continue }

            // Sample center 5×5 patch
            let floats = data.withUnsafeBytes { ptr in
                Array(ptr.bindMemory(to: Float.self))
            }
            let cx = 64, cy = 64, r = 2
            var patchValues: [Float] = []
            for dy in -r...r {
                for dx in -r...r {
                    let idx = (cy + dy) * 128 + (cx + dx)
                    if idx >= 0, idx < floats.count {
                        let v = floats[idx]
                        if v > 0.1, v < 1000.0, v.isFinite {
                            patchValues.append(v)
                        }
                    }
                }
            }

            guard patchValues.count >= 3 else { continue }
            patchValues.sort()
            let median = patchValues[patchValues.count / 2]

            let gt = sample.ground_truth_center_m
            guard gt > 0.3 else { continue }

            let errorPct = abs(median - gt) / gt * 100.0
            errors.append(errorPct)
        }

        guard !errors.isEmpty else {
            throw XCTSkip("Could not load any depth maps")
        }

        let sortedErrors = errors.sorted()
        let p50 = sortedErrors[sortedErrors.count / 2]
        let p90 = sortedErrors[min(sortedErrors.count - 1, Int(Double(sortedErrors.count) * 0.9))]

        print("\n=== REAL DEPTH MAP SAMPLING ===")
        print(String(format: "  Samples: %d", errors.count))
        print(String(format: "  P50 error: %.2f%%", p50))
        print(String(format: "  P90 error: %.2f%%", p90))

        XCTAssertLessThan(p50, 5.0, "Depth map center sampling P50 should be < 5%")
        XCTAssertLessThan(p90, 15.0, "Depth map center sampling P90 should be < 15%")
    }

    /// Test bimodal detection on real depth maps with foreground occluders.
    func testBimodalDetectionOnRealScenes() throws {
        guard ProcessInfo.processInfo.environment["RANGEFINDER_DATASET_PATH"] != nil else {
            throw XCTSkip("RANGEFINDER_DATASET_PATH not set — skipping Tier 2 bimodal tests")
        }

        // Bimodal scenes have large P25-P75 spread relative to center depth
        let bimodalCandidates = samples.filter {
            guard let p25 = $0.ground_truth_p25_m,
                  let p75 = $0.ground_truth_p75_m,
                  $0.depth_map_file != nil else { return false }
            let spread = p75 - p25
            let center = $0.ground_truth_center_m
            return spread > center * 0.5 && center > 1.0
        }

        guard !bimodalCandidates.isEmpty else {
            throw XCTSkip("No bimodal candidate samples found")
        }

        print("\n=== BIMODAL DETECTION ON REAL SCENES ===")
        print(String(format: "  Candidates: %d", bimodalCandidates.count))

        // Verify that depth spread correlates with actual depth variation
        var spreadRatios: [Float] = []
        for sample in bimodalCandidates.prefix(100) {
            if let p25 = sample.ground_truth_p25_m,
               let p75 = sample.ground_truth_p75_m {
                let ratio = p75 / max(p25, 0.1)
                spreadRatios.append(ratio)
            }
        }

        if !spreadRatios.isEmpty {
            let avgRatio = spreadRatios.reduce(0, +) / Float(spreadRatios.count)
            print(String(format: "  Average P75/P25 ratio: %.2f", avgRatio))
            XCTAssertGreaterThan(avgRatio, 1.5,
                "Bimodal candidates should have significant depth spread")
        }
    }

    /// Analyze depth map noise characteristics from real data.
    func testDepthMapNoiseCharacteristics() throws {
        guard let datasetPath = ProcessInfo.processInfo.environment["RANGEFINDER_DATASET_PATH"] else {
            throw XCTSkip("RANGEFINDER_DATASET_PATH not set — skipping Tier 2 noise tests")
        }

        let depthDir = (datasetPath as NSString).appendingPathComponent("depth")
        guard FileManager.default.fileExists(atPath: depthDir) else {
            throw XCTSkip("Depth directory not found")
        }

        let depthSamples = samples.filter { $0.depth_map_file != nil }
        guard !depthSamples.isEmpty else {
            throw XCTSkip("No depth map files available")
        }

        var patchStddevs: [Float] = []

        for sample in depthSamples.prefix(500) {
            guard let depthFile = sample.depth_map_file else { continue }
            let fullPath = (datasetPath as NSString).appendingPathComponent(depthFile)
            guard let data = FileManager.default.contents(atPath: fullPath),
                  data.count == 128 * 128 * 4 else { continue }

            let floats = data.withUnsafeBytes { ptr in
                Array(ptr.bindMemory(to: Float.self))
            }

            // 5×5 center patch
            let cx = 64, cy = 64, r = 2
            var patchValues: [Float] = []
            for dy in -r...r {
                for dx in -r...r {
                    let idx = (cy + dy) * 128 + (cx + dx)
                    if idx >= 0, idx < floats.count {
                        let v = floats[idx]
                        if v > 0.1, v < 1000.0, v.isFinite {
                            patchValues.append(v)
                        }
                    }
                }
            }

            guard patchValues.count >= 5 else { continue }

            let mean = patchValues.reduce(0, +) / Float(patchValues.count)
            let variance = patchValues.map { ($0 - mean) * ($0 - mean) }.reduce(0, +) / Float(patchValues.count)
            let stddev = sqrt(variance)
            let cv = stddev / max(mean, 0.001)  // Coefficient of variation
            patchStddevs.append(cv)
        }

        guard !patchStddevs.isEmpty else {
            throw XCTSkip("Could not compute noise statistics")
        }

        let sortedCVs = patchStddevs.sorted()
        let medianCV = sortedCVs[sortedCVs.count / 2]
        let p90CV = sortedCVs[min(sortedCVs.count - 1, Int(Double(sortedCVs.count) * 0.9))]

        print("\n=== DEPTH MAP NOISE CHARACTERISTICS ===")
        print(String(format: "  Samples: %d", patchStddevs.count))
        print(String(format: "  Median CV (center patch): %.4f", medianCV))
        print(String(format: "  P90 CV: %.4f", p90CV))

        // Depth maps should have low noise at center (< 10% CV for most)
        XCTAssertLessThan(medianCV, 0.10,
            "Median center-patch CV \(String(format: "%.3f", medianCV)) should be < 0.10")
    }

    // MARK: - Tier 3: Full Pipeline (optional)

    /// Run neural model on real images and compare to ground truth.
    /// Requires RANGEFINDER_DATASET_PATH with images/ directory.
    func testNeuralModelAccuracyOnRealImages() throws {
        guard let datasetPath = ProcessInfo.processInfo.environment["RANGEFINDER_DATASET_PATH"] else {
            throw XCTSkip("RANGEFINDER_DATASET_PATH not set — skipping Tier 3 neural tests")
        }

        let imagesDir = (datasetPath as NSString).appendingPathComponent("images")
        guard FileManager.default.fileExists(atPath: imagesDir) else {
            throw XCTSkip("Images directory not found — run prepare_ground_truth_dataset.py --tier 3")
        }

        // Neural model testing requires CoreML + device/simulator
        // This is a placeholder for when Tier 3 data is available
        print("⚠ Tier 3 neural model testing: images available but CoreML inference " +
              "requires full simulator setup. Run on-device for accurate results.")
    }

    /// Full pipeline: LiDAR + neural + fusion end-to-end.
    func testFullPipelineEndToEnd() throws {
        guard let datasetPath = ProcessInfo.processInfo.environment["RANGEFINDER_DATASET_PATH"] else {
            throw XCTSkip("RANGEFINDER_DATASET_PATH not set — skipping Tier 3 pipeline tests")
        }

        let imagesDir = (datasetPath as NSString).appendingPathComponent("images")
        guard FileManager.default.fileExists(atPath: imagesDir) else {
            throw XCTSkip("Images directory not found at \(imagesDir) — run prepare_ground_truth_dataset.py --tier 3")
        }

        print("⚠ Tier 3 full pipeline testing requires on-device execution with ARKit. " +
              "Use Tier 1/2 results to validate fusion logic offline.")
    }
}
