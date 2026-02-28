//
//  GroundTruthTestHelpers.swift
//  RangefinderTests
//
//  Shared helpers for ground truth dataset validation tests.
//  Loads the manifest.json, maps samples to FusionScenario,
//  and computes per-band accuracy statistics.
//

import Foundation
@testable import Rangefinder

// MARK: - Manifest Data Model

struct IntrinsicsData: Codable {
    let fx: Double
    let fy: Double
    let cx: Double
    let cy: Double
}

struct GroundTruthSample: Codable {
    let dataset: String
    let frame_id: String
    let ground_truth_center_m: Float
    let lidar_center_m: Float?
    let ground_truth_p25_m: Float?
    let ground_truth_p75_m: Float?
    let intrinsics: IntrinsicsData?
    let image_width: Int
    let image_height: Int
    let scene_type: String       // "indoor" | "outdoor"
    let distance_band: String    // "close" | "near_mid" | "mid" | "far_mid" | "far" | "long"
    let depth_map_file: String?
    let image_file: String?
}

struct DistanceBandInfo: Codable {
    let min_m: Float
    let max_m: Float
    let count: Int
}

struct GroundTruthManifest: Codable {
    let version: String
    let generated_date: String
    let dataset_sources: [String]
    let total_samples: Int
    let distance_bands: [String: DistanceBandInfo]
    let samples: [GroundTruthSample]
}

// MARK: - Band Statistics

/// Streaming statistics accumulator for accuracy metrics per distance band.
/// Matches the metrics pattern used in MonteCarloFusionTests and
/// ImageSimulationTests (AbsRel, RMSE, P90, CV).
struct BandStatistics {
    var absRelSum: Double = 0
    var squaredErrorSum: Double = 0
    var count: Int = 0
    var catastrophicCount: Int = 0     // > 100% error
    var confidentBadCount: Int = 0     // > 50% error + confidence > 0.3
    var errors: [Float] = []           // For percentile computation
    var sourceDistribution: [String: Int] = [:]

    mutating func addSample(errorPercent: Float, confidence: Float, source: DepthSource?) {
        let absRel = Double(errorPercent) / 100.0
        absRelSum += absRel
        squaredErrorSum += absRel * absRel
        count += 1
        errors.append(errorPercent)

        if errorPercent > 100.0 {
            catastrophicCount += 1
        }
        if errorPercent > 50.0 && confidence > 0.3 {
            confidentBadCount += 1
        }

        if let src = source {
            let key = src.rawValue
            sourceDistribution[key, default: 0] += 1
        }
    }

    var absRel: Double {
        count > 0 ? absRelSum / Double(count) : 0
    }

    var rmse: Double {
        count > 0 ? sqrt(squaredErrorSum / Double(count)) : 0
    }

    var p50: Float {
        percentile(0.50)
    }

    var p90: Float {
        percentile(0.90)
    }

    var p95: Float {
        percentile(0.95)
    }

    var catastrophicRate: Double {
        count > 0 ? Double(catastrophicCount) / Double(count) : 0
    }

    var confidentBadRate: Double {
        count > 0 ? Double(confidentBadCount) / Double(count) : 0
    }

    func percentile(_ p: Double) -> Float {
        guard !errors.isEmpty else { return 0 }
        let sorted = errors.sorted()
        let idx = min(sorted.count - 1, Int(Double(sorted.count) * p))
        return sorted[idx]
    }

    /// Dominant source name (most frequent)
    var dominantSource: String {
        sourceDistribution.max { $0.value < $1.value }?.key ?? "none"
    }
}

// MARK: - Manifest Loading

/// Load the ground truth manifest from the test data directory.
func loadGroundTruthManifest() -> GroundTruthManifest? {
    let candidates = [
        // Relative to test source
        "/Users/matthewjohnson/range/Rangefinder/RangefinderTests/GroundTruthData/manifest.json",
        // Environment variable override
        ProcessInfo.processInfo.environment["RANGEFINDER_DATASET_PATH"]
            .map { ($0 as NSString).appendingPathComponent("manifest.json") },
    ].compactMap { $0 }

    for path in candidates {
        if FileManager.default.fileExists(atPath: path),
           let data = FileManager.default.contents(atPath: path) {
            do {
                let decoder = JSONDecoder()
                return try decoder.decode(GroundTruthManifest.self, from: data)
            } catch {
                print("⚠ Failed to decode manifest at \(path): \(error)")
            }
        }
    }

    return nil
}

// MARK: - Sample → FusionScenario Mapping

/// Map a ground truth sample to sensor conditions for the FusionSimulator.
/// Uses distance-appropriate defaults for GPS, heading, calibration, etc.
func sampleToFusionScenario(_ sample: GroundTruthSample) -> FusionScenario {
    let d = sample.ground_truth_center_m

    // Outdoor scenes have GPS; indoor generally don't
    let hasGPS = sample.scene_type == "outdoor"

    // GPS accuracy varies: good outdoors, poor indoors (if available)
    let gpsAccuracy: Float = hasGPS ? 5.0 : 30.0
    let headingAccuracy: Float = hasGPS ? 8.0 : 20.0

    // Object detection: available at mid-range+ when there are discrete objects
    let hasObject = d > 10.0 && d < 1000.0
    let objectDetConf: Float = hasObject ? 0.75 : 0.0

    // Calibration: fresh indoors (LiDAR calibrates continuously), aging outdoors
    let calibrationAge: TimeInterval = sample.scene_type == "indoor" ? 10.0 : 120.0
    let calibrationConf: Float = sample.scene_type == "indoor" ? 0.90 : 0.70

    // Terrain slope: flat indoors, moderate outdoors
    let terrainSlope: Float = sample.scene_type == "outdoor" ? 3.0 : 0.0

    return FusionScenario(
        trueDistanceM: d,
        sceneName: sample.frame_id,
        terrainSlope: terrainSlope,
        hasGPS: hasGPS,
        gpsAccuracy: gpsAccuracy,
        headingAccuracy: headingAccuracy,
        hasObject: hasObject,
        objectDetConf: objectDetConf,
        calibrationAge: calibrationAge,
        calibrationConf: calibrationConf,
        expectedDominantSource: nil,
        maxAcceptableErrorPercent: 0
    )
}

// MARK: - Reporting

/// Format a per-band accuracy report table.
func formatBandReport(_ stats: [String: BandStatistics]) -> String {
    let bandOrder = ["close", "near_mid", "mid", "far_mid", "far", "long"]
    let bandRanges: [String: String] = [
        "close": "0.5–3m", "near_mid": "3–8m", "mid": "8–15m",
        "far_mid": "15–50m", "far": "50–150m", "long": "150–350m"
    ]

    var lines: [String] = []
    lines.append("")
    lines.append("=== GROUND TRUTH DATASET ACCURACY REPORT ===")
    lines.append("")
    lines.append("Band       Range        N AbsRel    P50    P90   Cat   C+B   DomSrc")
    lines.append(String(repeating: "-", count: 80))

    var globalStats = BandStatistics()

    for band in bandOrder {
        guard let s = stats[band] else { continue }
        let range = (bandRanges[band] ?? band).padding(toLength: 8, withPad: " ", startingAt: 0)
        let bandPad = band.padding(toLength: 10, withPad: " ", startingAt: 0)
        let srcPad = s.dominantSource.padding(toLength: 8, withPad: " ", startingAt: 0)

        lines.append(String(format: "%@ %@ %6d %6.1f%% %6.1f%% %6.1f%% %4d %4d %@",
            bandPad as NSString, range as NSString, s.count,
            s.absRel * 100, s.p50, s.p90,
            s.catastrophicCount, s.confidentBadCount,
            srcPad as NSString))

        // Accumulate global stats
        for e in s.errors {
            globalStats.addSample(errorPercent: e, confidence: 0.5, source: nil)
        }
    }

    lines.append(String(repeating: "-", count: 80))
    lines.append(String(format: "GLOBAL              %6d %6.1f%% %6.1f%% %6.1f%% %4d %4d",
        globalStats.count,
        globalStats.absRel * 100, globalStats.p50, globalStats.p90,
        globalStats.catastrophicCount, globalStats.confidentBadCount))
    lines.append("")

    return lines.joined(separator: "\n")
}
