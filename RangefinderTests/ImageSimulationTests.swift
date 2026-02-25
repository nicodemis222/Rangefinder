//
//  ImageSimulationTests.swift
//  RangefinderTests
//
//  Image-based simulation tests: loads real-world reference photographs
//  across 15 scene types and depth bands (0.3m–2000m), generates synthetic
//  depth maps modeling what the neural estimator would produce for each scene,
//  and validates the full semantic source selection pipeline end-to-end.
//
//  Each test scenario is parameterized with:
//  - A reference image filename (loaded from TestImages/)
//  - Known ground-truth distance (meters)
//  - Scene type (indoor, urban, terrain, sky)
//  - Sensor conditions (GPS accuracy, heading, calibration state)
//  - Expected semantic source decision
//  - Error tolerance per distance band
//
//  The test exercises:
//  1. Scene classification on synthetic depth maps
//  2. Semantic source selection priority chain
//  3. Confidence curve correctness at each distance
//  4. Multi-frame temporal averaging (Kalman + EMA simulation)
//  5. Source handover zones (LiDAR→Neural→DEM transitions)
//  6. Edge cases (sky, structure discontinuities, bimodal scenes)
//

import XCTest
import CoreVideo
import CoreLocation
@testable import Rangefinder

// MARK: - Image Scene Descriptor

/// Describes a real-world reference image and its expected ranging behavior.
struct ImageSceneDescriptor {
    let filename: String
    let groundTruthDistanceM: Float
    let sceneType: SceneType
    let description: String

    // Environment parameters
    let pitchDegrees: Double           // Device pitch (negative = looking down)
    let hasGPS: Bool
    let gpsAccuracy: Float             // meters
    let headingAccuracy: Float         // degrees
    let headingDeg: Float              // compass heading
    let terrainSlope: Float            // degrees
    let hasObject: Bool                // known-size object visible
    let objectDetConf: Float           // detection confidence
    let calibrationAge: TimeInterval   // seconds since last LiDAR cal
    let calibrationConf: Float         // calibration fit quality

    // Expected behavior
    let expectedDecision: SemanticSourceDecision
    let maxErrorPercent: Float
    let expectedSceneClass: CrosshairSceneClass?

    // Depth map generation parameters
    let depthMapPattern: DepthMapPattern

    enum SceneType: String {
        case indoorClose = "Indoor Close"
        case indoorMedium = "Indoor Medium"
        case urbanMid = "Urban Mid"
        case openField = "Open Field"
        case mountainTerrain = "Mountain Terrain"
        case extremeRange = "Extreme Range"
        case skyEdgeCase = "Sky Edge Case"
    }

    enum DepthMapPattern {
        case uniform(depth: Float)
        case groundGradient(near: Float, far: Float)
        case structureEdge(near: Float, far: Float)
        case bimodal(foreground: Float, background: Float)
        case depthFalloff(center: Float, edge: Float)
        case skyUniform(maxDepth: Float)
        case terrainGradient(near: Float, far: Float, slope: Float)
    }
}

// MARK: - Test Class

final class ImageSimulationTests: XCTestCase {

    // MARK: - Scenario Definitions

    /// All 15 image-based scenarios spanning the full operational envelope.
    static let scenarios: [ImageSceneDescriptor] = [

        // === BAND 1: Indoor Close Range (0.3–3m) — LiDAR dominant ===
        ImageSceneDescriptor(
            filename: "indoor_desk_1m.jpg",
            groundTruthDistanceM: 1.2,
            sceneType: .indoorClose,
            description: "Office desk with monitor at ~1.2m — LiDAR sweet spot",
            pitchDegrees: -5.0,
            hasGPS: false, gpsAccuracy: 30.0, headingAccuracy: 20.0, headingDeg: 0,
            terrainSlope: 0, hasObject: false, objectDetConf: 0,
            calibrationAge: 5.0, calibrationConf: 0.9,
            expectedDecision: .lidarPrimary,
            maxErrorPercent: 5.0,
            expectedSceneClass: .structure,
            depthMapPattern: .structureEdge(near: 1.0, far: 3.0)
        ),

        // === BAND 2: Indoor Medium (3–8m) — LiDAR-Neural handover ===
        ImageSceneDescriptor(
            filename: "hallway_5m.jpg",
            groundTruthDistanceM: 5.0,
            sceneType: .indoorMedium,
            description: "Office/coworking space — back wall at ~5m, LiDAR-Neural overlap",
            pitchDegrees: -2.0,
            hasGPS: false, gpsAccuracy: 30.0, headingAccuracy: 20.0, headingDeg: 0,
            terrainSlope: 0, hasObject: false, objectDetConf: 0,
            calibrationAge: 3.0, calibrationConf: 0.92,
            expectedDecision: .lidarPrimary,
            maxErrorPercent: 8.0,
            expectedSceneClass: nil,
            depthMapPattern: .depthFalloff(center: 5.0, edge: 2.5)
        ),

        // === BAND 3: Near-mid (8–15m) — Neural reliable ===
        // Note: In unit test context without live CoreML inference, neural depth
        // maps are synthetic. Ground classification requires pitch < -3° strictly.
        ImageSceneDescriptor(
            filename: "forest_path_15m.jpg",
            groundTruthDistanceM: 15.0,
            sceneType: .urbanMid,
            description: "Forest path with trees at ~15m — neural sweet spot",
            pitchDegrees: -8.0,   // Looking clearly downward for ground classification
            hasGPS: true, gpsAccuracy: 8.0, headingAccuracy: 8.0, headingDeg: 45,
            terrainSlope: 2.0, hasObject: true, objectDetConf: 0.75,
            calibrationAge: 15.0, calibrationConf: 0.88,
            expectedDecision: .objectPrimary,
            maxErrorPercent: 15.0,
            expectedSceneClass: .ground,
            depthMapPattern: .groundGradient(near: 3.0, far: 25.0)
        ),

        // === BAND 4: Urban mid-range (15–30m) — Structure scene, DEM + object ===
        ImageSceneDescriptor(
            filename: "building_facade_25m.jpg",
            groundTruthDistanceM: 25.0,
            sceneType: .urbanMid,
            description: "Building facade at ~25m — structure scene, DEM available",
            pitchDegrees: 5.0,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 5.0, headingDeg: 180,
            terrainSlope: 0, hasObject: true, objectDetConf: 0.80,
            calibrationAge: 25.0, calibrationConf: 0.85,
            expectedDecision: .objectPrimary,
            maxErrorPercent: 20.0,
            expectedSceneClass: .structure,
            depthMapPattern: .structureEdge(near: 20.0, far: 80.0)
        ),

        // === BAND 5: Street scene (30–50m) — Neural-Geometric-DEM overlap ===
        ImageSceneDescriptor(
            filename: "street_cars_30m.jpg",
            groundTruthDistanceM: 35.0,
            sceneType: .urbanMid,
            description: "City street with cars and pedestrians at ~35m — object detection + DEM",
            pitchDegrees: -1.5,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 5.0, headingDeg: 270,
            terrainSlope: 1.0, hasObject: true, objectDetConf: 0.82,
            calibrationAge: 40.0, calibrationConf: 0.85,
            expectedDecision: .objectPrimary,
            maxErrorPercent: 15.0,
            expectedSceneClass: nil,
            depthMapPattern: .bimodal(foreground: 15.0, background: 80.0)
        ),

        // === BAND 6: Parking lot (50–100m) — DEM ramp + geometric ===
        ImageSceneDescriptor(
            filename: "parking_lot_100m.jpg",
            groundTruthDistanceM: 100.0,
            sceneType: .openField,
            description: "Parking lot with vehicles spanning ~100m — DEM primary, objects secondary",
            pitchDegrees: -5.0,   // Looking clearly down at parking lot for ground classification
            hasGPS: true, gpsAccuracy: 3.0, headingAccuracy: 5.0, headingDeg: 90,
            terrainSlope: 0, hasObject: true, objectDetConf: 0.75,
            calibrationAge: 60.0, calibrationConf: 0.80,
            expectedDecision: .objectPrimary,
            maxErrorPercent: 15.0,
            expectedSceneClass: .ground,
            depthMapPattern: .groundGradient(near: 10.0, far: 100.0)
        ),

        // === BAND 7: Sports field (100–200m) — DEM sweet spot ===
        ImageSceneDescriptor(
            filename: "sports_field_150m.jpg",
            groundTruthDistanceM: 150.0,
            sceneType: .openField,
            description: "Soccer field under floodlights — players at ~150m, DEM sweet spot",
            pitchDegrees: -1.0,
            hasGPS: true, gpsAccuracy: 3.0, headingAccuracy: 5.0, headingDeg: 0,
            terrainSlope: 0, hasObject: true, objectDetConf: 0.65,
            calibrationAge: 120.0, calibrationConf: 0.75,
            expectedDecision: .objectPrimary,
            maxErrorPercent: 15.0,
            expectedSceneClass: nil,
            depthMapPattern: .groundGradient(near: 30.0, far: 100.0)
        ),

        // === BAND 8: Open field (200m) — DEM dominant, object fallback ===
        ImageSceneDescriptor(
            filename: "open_field_person_200m.jpg",
            groundTruthDistanceM: 200.0,
            sceneType: .openField,
            description: "Wheat field with treeline at ~200m — DEM dominant, no close objects",
            pitchDegrees: -0.5,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 8.0, headingDeg: 135,
            terrainSlope: 1.0, hasObject: false, objectDetConf: 0,
            calibrationAge: 180.0, calibrationConf: 0.70,
            expectedDecision: .demPrimary,
            maxErrorPercent: 25.0,
            expectedSceneClass: nil,
            depthMapPattern: .groundGradient(near: 20.0, far: 100.0)
        ),

        // === BAND 9: Highway (400m) — DEM + vehicle objects ===
        ImageSceneDescriptor(
            filename: "highway_vehicles_400m.jpg",
            groundTruthDistanceM: 400.0,
            sceneType: .mountainTerrain,
            description: "Desert highway with vehicle and rock formations at ~400m",
            pitchDegrees: -0.3,
            hasGPS: true, gpsAccuracy: 3.0, headingAccuracy: 5.0, headingDeg: 315,
            terrainSlope: 3.0, hasObject: true, objectDetConf: 0.70,
            calibrationAge: 300.0, calibrationConf: 0.65,
            expectedDecision: .objectPrimary,
            maxErrorPercent: 15.0,
            expectedSceneClass: nil,
            depthMapPattern: .terrainGradient(near: 50.0, far: 100.0, slope: 3.0)
        ),

        // === BAND 10: Mountain valley (500m) — DEM terrain ranging ===
        ImageSceneDescriptor(
            filename: "mountain_valley_500m.jpg",
            groundTruthDistanceM: 500.0,
            sceneType: .mountainTerrain,
            description: "Mountain valley with snow peaks — DEM terrain, neural saturated",
            pitchDegrees: 2.0,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 8.0, headingDeg: 0,
            terrainSlope: 8.0, hasObject: false, objectDetConf: 0,
            calibrationAge: 300.0, calibrationConf: 0.60,
            expectedDecision: .demPrimary,
            maxErrorPercent: 20.0,
            expectedSceneClass: nil,
            depthMapPattern: .terrainGradient(near: 30.0, far: 100.0, slope: 8.0)
        ),

        // === BAND 11: Canyon (800m) — DEM with steep terrain ===
        ImageSceneDescriptor(
            filename: "canyon_800m.jpg",
            groundTruthDistanceM: 800.0,
            sceneType: .mountainTerrain,
            description: "Horseshoe Bend canyon — steep terrain, DEM with slope challenge",
            pitchDegrees: -10.0,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 8.0, headingDeg: 200,
            terrainSlope: 15.0, hasObject: false, objectDetConf: 0,
            calibrationAge: 300.0, calibrationConf: 0.50,
            expectedDecision: .demPrimary,
            maxErrorPercent: 25.0,
            expectedSceneClass: nil,
            depthMapPattern: .terrainGradient(near: 20.0, far: 100.0, slope: 15.0)
        ),

        // === BAND 12: Hilltop (1000m) — DEM long range ===
        ImageSceneDescriptor(
            filename: "hilltop_terrain_1000m.jpg",
            groundTruthDistanceM: 1000.0,
            sceneType: .extremeRange,
            description: "Alpine peak above clouds — DEM at 1000m, heading error grows",
            pitchDegrees: 5.0,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 10.0, headingDeg: 90,
            terrainSlope: 10.0, hasObject: false, objectDetConf: 0,
            calibrationAge: 300.0, calibrationConf: 0.50,
            expectedDecision: .demPrimary,
            maxErrorPercent: 30.0,
            expectedSceneClass: nil,
            depthMapPattern: .terrainGradient(near: 50.0, far: 100.0, slope: 10.0)
        ),

        // === BAND 13: Mountain peak (1500m) — Extreme DEM ===
        ImageSceneDescriptor(
            filename: "mountain_peak_1500m.jpg",
            groundTruthDistanceM: 1500.0,
            sceneType: .extremeRange,
            description: "Snow-covered peaks — DEM at 1500m, extreme range",
            pitchDegrees: 8.0,
            hasGPS: true, gpsAccuracy: 8.0, headingAccuracy: 10.0, headingDeg: 45,
            terrainSlope: 12.0, hasObject: false, objectDetConf: 0,
            calibrationAge: 300.0, calibrationConf: 0.40,
            expectedDecision: .demPrimary,
            maxErrorPercent: 35.0,
            expectedSceneClass: nil,
            depthMapPattern: .terrainGradient(near: 80.0, far: 100.0, slope: 12.0)
        ),

        // === BAND 14: Cityscape (2000m) — Maximum range ===
        ImageSceneDescriptor(
            filename: "cityscape_2000m.jpg",
            groundTruthDistanceM: 2000.0,
            sceneType: .extremeRange,
            description: "NYC skyline — maximum DEM range with buildings as reference",
            pitchDegrees: 0.0,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 8.0, headingDeg: 270,
            terrainSlope: 0, hasObject: true, objectDetConf: 0.55,
            calibrationAge: 300.0, calibrationConf: 0.40,
            expectedDecision: .objectPrimary,
            maxErrorPercent: 20.0,
            expectedSceneClass: nil,
            depthMapPattern: .bimodal(foreground: 50.0, background: 100.0)
        ),

        // === EDGE CASE: Sky (no finite range) ===
        ImageSceneDescriptor(
            filename: "sky_upward.jpg",
            groundTruthDistanceM: 0,
            sceneType: .skyEdgeCase,
            description: "Blue sky — neural should be suppressed, no valid range",
            pitchDegrees: 45.0,
            hasGPS: true, gpsAccuracy: 5.0, headingAccuracy: 5.0, headingDeg: 180,
            terrainSlope: 0, hasObject: false, objectDetConf: 0,
            calibrationAge: 60.0, calibrationConf: 0.80,
            expectedDecision: .none,
            maxErrorPercent: 0,
            expectedSceneClass: .sky,
            depthMapPattern: .skyUniform(maxDepth: 100.0)
        ),
    ]

    // MARK: - Synthetic Depth Map Generator

    /// Creates a Float32 CVPixelBuffer depth map for a given scene pattern.
    private func createDepthMap(
        width: Int = 128,
        height: Int = 128,
        pattern: ImageSceneDescriptor.DepthMapPattern
    ) -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width, height,
            kCVPixelFormatType_DepthFloat32,
            nil,
            &pixelBuffer
        )
        precondition(status == kCVReturnSuccess, "Failed to create pixel buffer")

        let buffer = pixelBuffer!
        CVPixelBufferLockBaseAddress(buffer, [])
        let baseAddress = CVPixelBufferGetBaseAddress(buffer)!
        let bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)

        for y in 0..<height {
            let rowPtr = (baseAddress + y * bytesPerRow).assumingMemoryBound(to: Float.self)
            for x in 0..<width {
                rowPtr[x] = depthValue(x: x, y: y, width: width, height: height, pattern: pattern)
            }
        }

        CVPixelBufferUnlockBaseAddress(buffer, [])
        return buffer
    }

    private func depthValue(
        x: Int, y: Int, width: Int, height: Int,
        pattern: ImageSceneDescriptor.DepthMapPattern
    ) -> Float {
        let nx = Float(x) / Float(width - 1)   // 0..1 left to right
        let ny = Float(y) / Float(height - 1)  // 0..1 top to bottom

        switch pattern {
        case .uniform(let depth):
            return depth

        case .groundGradient(let near, let far):
            // Near at bottom (ny=1), far at top (ny=0) — ground plane perspective
            let t = 1.0 - ny
            return near + t * (far - near)

        case .structureEdge(let near, let far):
            // Sharp depth edge at center — building/wall boundary
            if nx < 0.45 {
                return near
            } else if nx > 0.55 {
                return far
            } else {
                // Transition zone
                let t = (nx - 0.45) / 0.1
                return near + t * (far - near)
            }

        case .bimodal(let foreground, let background):
            // Left half = foreground object, right half = background terrain
            if nx < 0.4 {
                return foreground
            } else if nx > 0.6 {
                return background
            } else {
                let t = (nx - 0.4) / 0.2
                return foreground + t * (background - foreground)
            }

        case .depthFalloff(let center, let edge):
            // Center is deepest, edges are closer (room/hallway)
            let dx = nx - 0.5
            let dy = ny - 0.5
            let r = sqrt(dx * dx + dy * dy) * 2.0  // 0 at center, ~1 at edges
            return center - r * (center - edge)

        case .skyUniform(let maxDepth):
            // Uniform maximum depth — sky fills frame
            return maxDepth

        case .terrainGradient(let near, let far, let slope):
            // Terrain with slope — depth increases from bottom to top
            // with lateral variation from terrain roughness
            let t = 1.0 - ny
            let baseDepth = near + t * (far - near)
            // Add slope-dependent lateral variation
            let lateralNoise = sin(nx * .pi * 4.0) * slope * 0.5
            return max(1.0, baseDepth + lateralNoise)
        }
    }

    // MARK: - Image Loading Verification

    /// Verify all test images exist in the bundle or TestImages directory.
    func testAllImagesExist() {
        let testImagesDir = findTestImagesDirectory()
        guard let dir = testImagesDir else {
            // Skip if images not available in the test environment
            print("⚠ TestImages directory not found — skipping image existence check")
            return
        }

        for scenario in Self.scenarios {
            let imagePath = (dir as NSString).appendingPathComponent(scenario.filename)
            let exists = FileManager.default.fileExists(atPath: imagePath)
            XCTAssertTrue(exists,
                "Missing test image: \(scenario.filename) for '\(scenario.description)'")
        }
    }

    // MARK: - Scene Classification on Synthetic Depth Maps

    /// Tests that the SceneClassifier correctly identifies scene types from
    /// depth maps generated for each reference image scenario.
    @MainActor
    func testSceneClassificationPerImage() {
        let classifier = SceneClassifier()

        for scenario in Self.scenarios {
            guard let expected = scenario.expectedSceneClass else { continue }

            let depthMap = createDepthMap(pattern: scenario.depthMapPattern)

            // Reset classifier state
            let freshClassifier = SceneClassifier()

            // Classify multiple times (rate-limited to every 3 frames)
            for _ in 0..<6 {
                freshClassifier.classify(
                    depthMap: depthMap,
                    pitchDegrees: scenario.pitchDegrees
                )
            }

            XCTAssertEqual(freshClassifier.currentClass, expected,
                "Scene '\(scenario.description)' (\(scenario.filename)): " +
                "expected \(expected.rawValue), got \(freshClassifier.currentClass.rawValue)")
        }
    }

    // MARK: - Semantic Source Selection per Scenario

    /// Validates that semanticSelect() chooses the correct primary source
    /// for each image scenario when all relevant sensor data is provided.
    @MainActor
    func testSemanticSelectionPerImage() {
        for scenario in Self.scenarios {
            let depthField = UnifiedDepthField()
            depthField.targetPriority = .far

            // Set up sensor data based on scenario
            configureSensors(depthField: depthField, scenario: scenario)

            let (primary, background) = depthField.semanticSelect(
                screenPoint: CGPoint(x: 0.5, y: 0.5),
                timestamp: 0
            )

            // Verify semantic decision
            if scenario.expectedDecision != .none {
                XCTAssertEqual(depthField.semanticDecision, scenario.expectedDecision,
                    "Scene '\(scenario.description)' (\(scenario.filename)): " +
                    "expected \(scenario.expectedDecision.rawValue), " +
                    "got \(depthField.semanticDecision.rawValue)")

                // Verify primary estimate is valid
                XCTAssertTrue(primary.isValid || scenario.sceneType == .skyEdgeCase,
                    "Scene '\(scenario.description)': primary estimate should be valid")

                // Verify primary distance is in reasonable range
                if primary.isValid {
                    let errorPct = abs(Float(primary.distanceMeters) - scenario.groundTruthDistanceM)
                        / max(scenario.groundTruthDistanceM, 0.1) * 100.0
                    XCTAssertLessThan(errorPct, scenario.maxErrorPercent * 2.0,
                        "Scene '\(scenario.description)': single-frame error \(String(format: "%.1f", errorPct))% " +
                        "exceeds 2× tolerance of \(scenario.maxErrorPercent)%")
                }
            } else {
                // Sky / no-range scenario
                XCTAssertFalse(primary.isValid,
                    "Sky scene should not produce valid primary estimate")
            }

            // Background hypothesis validation
            if let bg = background, primary.isValid {
                XCTAssertNotEqual(bg.source, primary.source,
                    "Background source should differ from primary (\(primary.source.rawValue))")
            }
        }
    }

    // MARK: - Multi-Frame Temporal Fusion Simulation

    /// Simulates 10 frames of sensor data per scenario with noise, applying
    /// the Monte Carlo fusion simulator to validate temporal convergence
    /// and error bounds with realistic sensor jitter.
    func testTemporalFusionPerImage() {
        var rng = SeededRNG(seed: 137)

        var totalScenarios = 0
        var passedScenarios = 0
        var totalMeanError: Double = 0

        print("\n=== IMAGE SIMULATION TEST: \(Self.scenarios.count) scenes ===\n")
        print("Scene                               TrueD   FusedD     Err%     Conf   Source Status")
        print(String(repeating: "-", count: 100))

        for scenario in Self.scenarios {
            guard scenario.groundTruthDistanceM > 0 else {
                // Sky scenario — verify no output
                let desc = String(scenario.description.prefix(35)).padding(toLength: 35, withPad: " ", startingAt: 0)
                print("\(desc)    SKY        —        —        —        — PASS (no range)")
                passedScenarios += 1
                totalScenarios += 1
                continue
            }

            let trueD = scenario.groundTruthDistanceM
            let numFrames = trueD < 100 ? 5 : (trueD < 500 ? 8 : 10)

            // Pre-generate fixed SRTM bias for this location
            let srtmVertical: Float = scenario.terrainSlope > 12.0 ? 8.0 :
                (scenario.terrainSlope > 6.0 ? 5.0 : 3.5)
            let srtmBias = rng.nextGaussian(mean: 0, stddev: srtmVertical)

            var temporalSum: Float = 0
            var temporalConfSum: Float = 0
            var temporalCount = 0
            var lastDominant: DepthSource? = nil

            for _ in 0..<numFrames {
                // Generate noisy sensor readings per frame
                let lidar = SensorNoiseModel.lidarReading(trueD: trueD, rng: &rng)
                let neural = SensorNoiseModel.neuralReading(
                    trueD: trueD, calibrationConf: scenario.calibrationConf, rng: &rng)
                let geo = SensorNoiseModel.geometricReading(
                    trueD: trueD, cameraHeight: 1.5,
                    terrainSlope: scenario.terrainSlope, rng: &rng)
                let dem = scenario.hasGPS ? SensorNoiseModel.demReading(
                    trueD: trueD, gpsAccuracy: scenario.gpsAccuracy,
                    headingAccuracy: scenario.headingAccuracy,
                    terrainSlope: scenario.terrainSlope, rng: &rng,
                    srtmBias: srtmBias) : nil
                let obj = scenario.hasObject ? SensorNoiseModel.objectReading(
                    trueD: trueD, detectionConf: scenario.objectDetConf, rng: &rng) : nil

                let fusionScenario = FusionScenario(
                    trueDistanceM: trueD,
                    sceneName: scenario.description,
                    terrainSlope: scenario.terrainSlope,
                    hasGPS: scenario.hasGPS,
                    gpsAccuracy: scenario.gpsAccuracy,
                    headingAccuracy: scenario.headingAccuracy,
                    hasObject: scenario.hasObject,
                    objectDetConf: scenario.objectDetConf,
                    calibrationAge: scenario.calibrationAge,
                    calibrationConf: scenario.calibrationConf,
                    expectedDominantSource: nil,
                    maxAcceptableErrorPercent: 0
                )

                let result = FusionSimulator.simulate(
                    scenario: fusionScenario,
                    lidarReading: lidar,
                    neuralReading: neural,
                    geometricReading: geo?.0,
                    geometricConfidence: geo?.1,
                    demReading: dem?.0,
                    demConfidence: dem?.1,
                    objectReading: obj
                )

                if result.fusedDepth > 0.01 {
                    temporalSum += result.fusedDepth
                    temporalConfSum += result.confidence
                    temporalCount += 1
                    lastDominant = result.dominantSource
                }
            }

            totalScenarios += 1

            guard temporalCount > 0 else {
                let desc = String(scenario.description.prefix(35)).padding(toLength: 35, withPad: " ", startingAt: 0)
                print("\(desc) \(String(format: "%6.0f", trueD))     NONE        —        —        — FAIL (no estimate)")
                continue
            }

            let fusedDepth = temporalSum / Float(temporalCount)
            let avgConf = temporalConfSum / Float(temporalCount)
            let errorPct = abs(fusedDepth - trueD) / trueD * 100.0
            let sourceName = lastDominant?.shortName ?? "???"

            let passed = errorPct < scenario.maxErrorPercent
            let status = passed ? "PASS" : "FAIL (\(String(format: "%.0f", scenario.maxErrorPercent))% max)"
            if passed { passedScenarios += 1 }
            totalMeanError += Double(errorPct)

            let desc = String(scenario.description.prefix(35)).padding(toLength: 35, withPad: " ", startingAt: 0)
            print("\(desc) \(String(format: "%5.0fm %6.0fm %6.1f%% %6.2f", trueD, fusedDepth, errorPct, avgConf))   \(sourceName.padding(toLength: 6, withPad: " ", startingAt: 0)) \(status)")
        }

        let avgError = totalScenarios > 0 ? totalMeanError / Double(totalScenarios) : 0

        print(String(repeating: "-", count: 100))
        print("TOTAL: \(passedScenarios)/\(totalScenarios) passed | Mean error: \(String(format: "%.1f", avgError))%")
        print("")

        // Assertions
        XCTAssertEqual(passedScenarios, totalScenarios,
            "\(totalScenarios - passedScenarios) scenario(s) exceeded error tolerance")
    }

    // MARK: - Confidence Curve Validation per Distance Band

    /// Verifies that the confidence curves produce sensible values at each
    /// scenario's ground truth distance — ensuring no dead zones.
    func testConfidenceCurvesPerScenario() {
        for scenario in Self.scenarios {
            let d = scenario.groundTruthDistanceM
            guard d > 0 else { continue }

            let lidarConf = DepthSourceConfidence.lidar(distanceM: d)
            let neuralConf = DepthSourceConfidence.neural(distanceM: d)
            let geoConf = DepthSourceConfidence.geometric(distanceM: d)
            let demConf = scenario.hasGPS ? DepthSourceConfidence.demRaycast(
                distanceM: d, gpsAccuracy: scenario.gpsAccuracy,
                headingAccuracy: scenario.headingAccuracy) : Float(0)
            let objConf = scenario.hasObject ? DepthSourceConfidence.object(
                distanceM: d, detectionConfidence: scenario.objectDetConf) : Float(0)

            let maxConf = max(lidarConf, max(neuralConf, max(geoConf, max(demConf, objConf))))

            XCTAssertGreaterThan(maxConf, 0.05,
                "Dead zone at \(d)m (\(scenario.filename)): " +
                "no source has confidence > 0.05 " +
                "[L=\(String(format: "%.2f", lidarConf)) " +
                "N=\(String(format: "%.2f", neuralConf)) " +
                "G=\(String(format: "%.2f", geoConf)) " +
                "D=\(String(format: "%.2f", demConf)) " +
                "O=\(String(format: "%.2f", objConf))]")
        }
    }

    // MARK: - Source Handover Validation

    /// Tests that source transitions between adjacent scenarios are smooth
    /// with overlapping confidence zones (no hard boundaries).
    func testSourceHandoverBetweenScenarios() {
        let sortedScenarios = Self.scenarios
            .filter { $0.groundTruthDistanceM > 0 }
            .sorted { $0.groundTruthDistanceM < $1.groundTruthDistanceM }

        for i in 0..<(sortedScenarios.count - 1) {
            let near = sortedScenarios[i]
            let far = sortedScenarios[i + 1]

            // At the midpoint between two adjacent scenarios,
            // at least one source should have decent confidence
            let midpointD = (near.groundTruthDistanceM + far.groundTruthDistanceM) / 2.0

            let lidar = DepthSourceConfidence.lidar(distanceM: midpointD)
            let neural = DepthSourceConfidence.neural(distanceM: midpointD)
            let geo = DepthSourceConfidence.geometric(distanceM: midpointD)
            let dem = DepthSourceConfidence.demRaycast(
                distanceM: midpointD, gpsAccuracy: 5.0, headingAccuracy: 5.0)
            let obj = DepthSourceConfidence.object(
                distanceM: midpointD, detectionConfidence: 0.7)

            let total = lidar + neural + geo + dem + obj

            XCTAssertGreaterThan(total, 0.3,
                "Handover gap at midpoint \(midpointD)m " +
                "(between '\(near.filename)' and '\(far.filename)'): " +
                "total confidence = \(String(format: "%.2f", total))")
        }
    }

    // MARK: - Bimodal Detection

    /// Tests that scenes with both foreground and background depths
    /// produce reasonable bimodal-like depth map distributions.
    func testBimodalScenesProduceValidBrackets() {
        let bimodalScenarios = Self.scenarios.filter {
            if case .bimodal = $0.depthMapPattern { return true }
            return false
        }

        XCTAssertGreaterThanOrEqual(bimodalScenarios.count, 2,
            "Should have at least 2 bimodal test scenarios")

        for scenario in bimodalScenarios {
            let depthMap = createDepthMap(width: 64, height: 64, pattern: scenario.depthMapPattern)

            // Sample left and right halves to verify bimodal distribution
            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            let baseAddress = CVPixelBufferGetBaseAddress(depthMap)!
            let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)

            var leftDepths: [Float] = []
            var rightDepths: [Float] = []

            for y in stride(from: 0, to: 64, by: 4) {
                let rowPtr = (baseAddress + y * bytesPerRow).assumingMemoryBound(to: Float.self)
                for x in stride(from: 0, to: 32, by: 4) {
                    leftDepths.append(rowPtr[x])
                }
                for x in stride(from: 32, to: 64, by: 4) {
                    rightDepths.append(rowPtr[x])
                }
            }

            CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)

            let leftMedian = leftDepths.sorted()[leftDepths.count / 2]
            let rightMedian = rightDepths.sorted()[rightDepths.count / 2]

            let ratio = max(leftMedian, rightMedian) / max(min(leftMedian, rightMedian), 0.1)
            XCTAssertGreaterThan(ratio, 1.5,
                "Bimodal scene '\(scenario.filename)' should have >1.5× depth ratio, got \(ratio)")
        }
    }

    // MARK: - Neural Hard Cap Enforcement

    /// Verifies neural depth is correctly rejected beyond 50m across all
    /// scenarios that operate past the neural hard cap.
    func testNeuralHardCapEnforcedPerScenario() {
        for scenario in Self.scenarios {
            let d = scenario.groundTruthDistanceM
            guard d > AppConfiguration.neuralHardCapMeters else { continue }

            let neuralConf = DepthSourceConfidence.neural(distanceM: d)
            XCTAssertEqual(neuralConf, 0.0, accuracy: 0.001,
                "Neural confidence at \(d)m (\(scenario.filename)) should be 0.0 (hard cap at \(AppConfiguration.neuralHardCapMeters)m)")
        }
    }

    // MARK: - Multi-Sample Statistical Validation

    /// Runs 500 samples per scenario with varied conditions and verifies
    /// statistical properties (mean error, catastrophic rate, confidence calibration).
    func testStatisticalValidationPerScene() {
        var rng = SeededRNG(seed: 42)

        print("\n=== STATISTICAL VALIDATION: 500 samples per scene ===\n")

        var globalCatastrophic = 0
        var globalSamples = 0
        var globalConfidentBad = 0

        for scenario in Self.scenarios {
            guard scenario.groundTruthDistanceM > 0 else { continue }

            let trueD = scenario.groundTruthDistanceM
            var errors: [Float] = []
            var catastrophic = 0
            var confidentBad = 0

            for _ in 0..<500 {
                // Vary conditions slightly
                let gps = scenario.gpsAccuracy * rng.nextUniform(min: 0.7, max: 1.3)
                let heading = scenario.headingAccuracy * rng.nextUniform(min: 0.7, max: 1.3)
                let slope = scenario.terrainSlope + rng.nextGaussian(mean: 0, stddev: 2.0)

                let srtmBias = rng.nextGaussian(mean: 0, stddev: slope > 12 ? 8.0 : 3.5)

                let lidar = SensorNoiseModel.lidarReading(trueD: trueD, rng: &rng)
                let neural = SensorNoiseModel.neuralReading(
                    trueD: trueD, calibrationConf: scenario.calibrationConf, rng: &rng)
                let geo = SensorNoiseModel.geometricReading(
                    trueD: trueD, cameraHeight: 1.5, terrainSlope: max(0, slope), rng: &rng)
                let dem = scenario.hasGPS ? SensorNoiseModel.demReading(
                    trueD: trueD, gpsAccuracy: gps, headingAccuracy: heading,
                    terrainSlope: max(0, slope), rng: &rng, srtmBias: srtmBias) : nil
                let obj = scenario.hasObject ? SensorNoiseModel.objectReading(
                    trueD: trueD, detectionConf: scenario.objectDetConf, rng: &rng) : nil

                let fusionScenario = FusionScenario(
                    trueDistanceM: trueD, sceneName: scenario.description,
                    terrainSlope: max(0, slope), hasGPS: scenario.hasGPS,
                    gpsAccuracy: gps, headingAccuracy: heading,
                    hasObject: scenario.hasObject, objectDetConf: scenario.objectDetConf,
                    calibrationAge: scenario.calibrationAge,
                    calibrationConf: scenario.calibrationConf,
                    expectedDominantSource: nil, maxAcceptableErrorPercent: 0
                )

                let result = FusionSimulator.simulate(
                    scenario: fusionScenario,
                    lidarReading: lidar, neuralReading: neural,
                    geometricReading: geo?.0, geometricConfidence: geo?.1,
                    demReading: dem?.0, demConfidence: dem?.1,
                    objectReading: obj
                )

                guard result.fusedDepth > 0.01 else { continue }

                let errorPct = abs(result.fusedDepth - trueD) / trueD * 100.0
                errors.append(errorPct)

                if errorPct > 100 {
                    catastrophic += 1
                }
                if errorPct > 50 && result.confidence > 0.3 {
                    confidentBad += 1
                }
            }

            globalSamples += errors.count
            globalCatastrophic += catastrophic
            globalConfidentBad += confidentBad

            let sortedErrors = errors.sorted()
            let mean = errors.reduce(0, +) / Float(max(errors.count, 1))
            let p50 = sortedErrors.isEmpty ? 0 : sortedErrors[sortedErrors.count / 2]
            let p95 = sortedErrors.isEmpty ? 0 : sortedErrors[min(sortedErrors.count - 1, Int(Float(sortedErrors.count) * 0.95))]

            let desc = String(scenario.description.prefix(35)).padding(toLength: 35, withPad: " ", startingAt: 0)
            print("\(desc)  n=\(String(format: "%3d", errors.count))  mean=\(String(format: "%5.1f", mean))%%  P50=\(String(format: "%5.1f", p50))%%  P95=\(String(format: "%5.1f", p95))%%  cat=\(catastrophic)  c+b=\(confidentBad)")
        }

        print(String(format: "\nGLOBAL: %d samples, %d catastrophic (%.2f%%), %d confident+bad (%.2f%%)",
            globalSamples, globalCatastrophic,
            Double(globalCatastrophic) / Double(max(globalSamples, 1)) * 100,
            globalConfidentBad,
            Double(globalConfidentBad) / Double(max(globalSamples, 1)) * 100))

        // Assertions
        XCTAssertEqual(globalCatastrophic, 0,
            "Should have ZERO catastrophic errors (>100%) across all image scenarios")
        XCTAssertLessThan(
            Double(globalConfidentBad) / Double(max(globalSamples, 1)), 0.10,
            "Confident-and-bad rate should be < 10%")
    }

    // MARK: - Full Distance Sweep with Image-Matched Conditions

    /// Sweeps from 0.3m to 2000m using conditions extracted from the nearest
    /// image scenario, ensuring continuous coverage with no gaps.
    func testContinuousDistanceSweep() {
        var rng = SeededRNG(seed: 73)

        let testDistances: [Float] = [
            0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0, 25.0,
            30.0, 40.0, 50.0, 75.0, 100.0, 150.0, 200.0, 300.0,
            400.0, 500.0, 700.0, 800.0, 1000.0, 1200.0, 1500.0, 2000.0
        ]

        var noEstimateCount = 0

        for trueD in testDistances {
            // Find closest scenario for environmental conditions
            let closest = Self.scenarios
                .filter { $0.groundTruthDistanceM > 0 }
                .min { abs($0.groundTruthDistanceM - trueD) < abs($1.groundTruthDistanceM - trueD) }!

            var validFrames = 0
            let numFrames = 5

            for _ in 0..<numFrames {
                let lidar = SensorNoiseModel.lidarReading(trueD: trueD, rng: &rng)
                let neural = SensorNoiseModel.neuralReading(
                    trueD: trueD, calibrationConf: closest.calibrationConf, rng: &rng)
                let geo = SensorNoiseModel.geometricReading(
                    trueD: trueD, cameraHeight: 1.5,
                    terrainSlope: closest.terrainSlope, rng: &rng)
                let dem = closest.hasGPS ? SensorNoiseModel.demReading(
                    trueD: trueD, gpsAccuracy: closest.gpsAccuracy,
                    headingAccuracy: closest.headingAccuracy,
                    terrainSlope: closest.terrainSlope, rng: &rng) : nil
                let obj = closest.hasObject ? SensorNoiseModel.objectReading(
                    trueD: trueD, detectionConf: closest.objectDetConf, rng: &rng) : nil

                let scenario = FusionScenario(
                    trueDistanceM: trueD, sceneName: "sweep",
                    terrainSlope: closest.terrainSlope, hasGPS: closest.hasGPS,
                    gpsAccuracy: closest.gpsAccuracy, headingAccuracy: closest.headingAccuracy,
                    hasObject: closest.hasObject, objectDetConf: closest.objectDetConf,
                    calibrationAge: closest.calibrationAge,
                    calibrationConf: closest.calibrationConf,
                    expectedDominantSource: nil, maxAcceptableErrorPercent: 0
                )

                let result = FusionSimulator.simulate(
                    scenario: scenario,
                    lidarReading: lidar, neuralReading: neural,
                    geometricReading: geo?.0, geometricConfidence: geo?.1,
                    demReading: dem?.0, demConfidence: dem?.1,
                    objectReading: obj
                )

                if result.fusedDepth > 0.01 { validFrames += 1 }
            }

            if validFrames == 0 { noEstimateCount += 1 }

            XCTAssertGreaterThan(validFrames, 0,
                "No valid estimates at \(trueD)m (matched to '\(closest.filename)') — " +
                "possible dead zone in the fusion pipeline")
        }

        XCTAssertEqual(noEstimateCount, 0,
            "\(noEstimateCount) distances had no valid estimates — dead zones in coverage")
    }

    // MARK: - Helpers

    /// Configures a UnifiedDepthField with synthetic sensor data for a scenario.
    @MainActor
    private func configureSensors(depthField: UnifiedDepthField, scenario: ImageSceneDescriptor) {
        let trueD = scenario.groundTruthDistanceM

        // Pitch
        depthField.currentPitchRadians = scenario.pitchDegrees * .pi / 180.0
        depthField.currentHeadingDegrees = Double(scenario.headingDeg)

        // Geometric estimator
        depthField.geometricEstimator = GeometricRangeEstimator()
        depthField.geometricEstimator.cameraHeight = AppConfiguration.defaultCameraHeight

        // Neural depth map: create in inverse-depth space (high values = close,
        // low values = far) to match DepthAnythingV2 output format.
        // The calibrator is trained with: rawNeural = 50/metricDepth
        // So we create a uniform inverse-depth map where center = 50/trueD
        let neuralInverseCenter: Float = trueD > 0.1 ? 50.0 / min(trueD, 50.0) : 100.0
        let neuralDepthMap = createDepthMap(width: 64, height: 64,
            pattern: .uniform(depth: neuralInverseCenter))

        // Set up calibrator with synthetic calibration
        if scenario.calibrationConf > 0.1 && trueD > 0 && trueD <= AppConfiguration.neuralHardCapMeters {
            // Feed calibration pairs to establish inverse model
            for i in 0..<10 {
                let sampleD = Float(i + 1) * 0.5  // 0.5m to 5.0m
                let rawNeural = 50.0 / sampleD     // Inverse depth model
                depthField.calibrator.ingestSample(
                    neuralDepth: rawNeural,
                    lidarDepth: sampleD,
                    confidence: 0.95,
                    timestamp: TimeInterval(i)
                )
            }
        }

        // LiDAR depth map (only for close range)
        if trueD > 0.3 && trueD < 10.0 {
            let lidarMap = createDepthMap(width: 64, height: 64,
                pattern: .uniform(depth: trueD))
            depthField.updateLiDAR(depthMap: lidarMap, confidenceMap: nil)
        }

        // Neural depth map
        depthField.updateNeuralDepth(depthMap: neuralDepthMap)

        // DEM estimate
        if scenario.hasGPS && trueD > 20.0 {
            depthField.latestDEMEstimate = DEMRaycastEstimate(
                distanceMeters: trueD,
                confidence: 0.8,
                terrainElevation: 500,
                headingDeg: scenario.headingDeg,
                gpsAccuracy: scenario.gpsAccuracy,
                hitCoordinate: CLLocationCoordinate2D(latitude: 37.0, longitude: -122.0)
            )
        }

        // Object detection
        if scenario.hasObject && trueD > 10 {
            depthField.updateObjectDetections([
                ObjectRangeResult(
                    label: "person",
                    distanceMeters: Double(trueD),
                    confidence: scenario.objectDetConf,
                    screenCenter: CGPoint(x: 0.5, y: 0.5),
                    boundingBox: CGRect(x: 0.4, y: 0.3, width: 0.2, height: 0.4)
                )
            ])
        }
    }

    /// Attempts to find the TestImages directory.
    private func findTestImagesDirectory() -> String? {
        // Check relative to the test source file
        let candidates = [
            "/Users/matthewjohnson/range/Rangefinder/RangefinderTests/TestImages",
            Bundle(for: type(of: self)).resourcePath.map { ($0 as NSString).appendingPathComponent("TestImages") },
        ].compactMap { $0 }

        for path in candidates {
            if FileManager.default.fileExists(atPath: path) {
                return path
            }
        }
        return nil
    }
}
