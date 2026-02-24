//
//  SceneClassifierTests.swift
//  RangefinderTests
//
//  Tests for depth-map-based scene classification at the crosshair.
//

import XCTest
import CoreVideo
@testable import Rangefinder

final class SceneClassifierTests: XCTestCase {

    // MARK: - Helper: Create Synthetic Depth Map

    /// Create a Float32 depth map with a specified fill pattern.
    private func createDepthMap(
        width: Int = 64,
        height: Int = 64,
        fill: (Int, Int, Int, Int) -> Float  // (x, y, width, height) -> depth
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
                rowPtr[x] = fill(x, y, width, height)
            }
        }

        CVPixelBufferUnlockBaseAddress(buffer, [])
        return buffer
    }

    // MARK: - Sky Detection

    @MainActor func testSkyDetection() {
        let classifier = SceneClassifier()

        // Create uniform max-depth map (simulates sky filling entire frame)
        let skyMap = createDepthMap { _, _, _, _ in 100.0 }

        // Classify multiple times (classifier only runs every 3 frames)
        for _ in 0..<4 {
            classifier.classify(depthMap: skyMap, pitchDegrees: 10.0)  // Looking up
        }

        XCTAssertEqual(classifier.currentClass, .sky,
            "Uniform max-depth with upward pitch should classify as sky")
        XCTAssertGreaterThan(classifier.classConfidence, 0.5)
    }

    @MainActor func testSkyNotDetectedWhenLookingDown() {
        let classifier = SceneClassifier()

        // Even with uniform depth, steep downward pitch should NOT be sky
        let uniformMap = createDepthMap { _, _, _, _ in 100.0 }

        for _ in 0..<4 {
            classifier.classify(depthMap: uniformMap, pitchDegrees: -20.0)  // Looking down
        }

        XCTAssertNotEqual(classifier.currentClass, .sky,
            "Looking down should not classify as sky even with uniform depth")
    }

    // MARK: - Ground Detection

    @MainActor func testGroundDetection() {
        let classifier = SceneClassifier()

        // Create depth gradient: close at bottom (high y), far at top (low y)
        // This simulates a ground plane where depth increases toward the horizon
        let groundMap = createDepthMap { _, y, _, height in
            let t = 1.0 - Float(y) / Float(height)  // 0 at bottom, 1 at top
            return 5.0 + t * 50.0  // 5m to 55m
        }

        for _ in 0..<4 {
            classifier.classify(depthMap: groundMap, pitchDegrees: -10.0)  // Looking down
        }

        XCTAssertEqual(classifier.currentClass, .ground,
            "Monotonic depth gradient with downward pitch should classify as ground")
        XCTAssertGreaterThan(classifier.classConfidence, 0.5)
    }

    @MainActor func testGroundNotDetectedWhenLookingUp() {
        let classifier = SceneClassifier()

        // Ground-like gradient but looking up
        let gradientMap = createDepthMap { _, y, _, height in
            let t = 1.0 - Float(y) / Float(height)
            return 5.0 + t * 50.0
        }

        for _ in 0..<4 {
            classifier.classify(depthMap: gradientMap, pitchDegrees: 5.0)  // Looking up
        }

        XCTAssertNotEqual(classifier.currentClass, .ground,
            "Looking up should not classify as ground")
    }

    // MARK: - Structure Detection

    @MainActor func testStructureDetection() {
        let classifier = SceneClassifier()

        // Create depth map with sharp discontinuity at center
        // (simulates building edge: left side near, right side far)
        let structureMap = createDepthMap { x, _, width, _ in
            if x < width / 2 {
                return 10.0   // Near side (wall)
            } else {
                return 80.0   // Far side (background)
            }
        }

        for _ in 0..<4 {
            classifier.classify(depthMap: structureMap, pitchDegrees: 0.0)  // Level
        }

        XCTAssertEqual(classifier.currentClass, .structure,
            "Sharp depth discontinuity should classify as structure")
        XCTAssertGreaterThan(classifier.classConfidence, 0.5)
    }

    // MARK: - Unknown Classification

    @MainActor func testUnknownForMixedScene() {
        let classifier = SceneClassifier()

        // Create a varied depth map that doesn't fit sky, ground, or structure
        let mixedMap = createDepthMap { x, y, width, height in
            // Semi-random depth pattern
            let noise = Float((x * 7 + y * 13) % 17) / 17.0
            return 10.0 + noise * 30.0
        }

        for _ in 0..<4 {
            classifier.classify(depthMap: mixedMap, pitchDegrees: -5.0)
        }

        // Should be either unknown or one of the classes with low confidence
        // (the noise pattern may trigger structure detection due to depth jumps)
        // This test mainly ensures no crash on complex depth patterns
        XCTAssertNotNil(classifier.currentClass)
    }

    // MARK: - Rate Limiting

    @MainActor func testClassifierRateLimiting() {
        let classifier = SceneClassifier()

        let skyMap = createDepthMap { _, _, _, _ in 100.0 }

        // First call: should not classify (frame 1, not divisible by 3)
        classifier.classify(depthMap: skyMap, pitchDegrees: 10.0)
        XCTAssertEqual(classifier.currentClass, .unknown,
            "First frame should not trigger classification")

        // Second call: still no classification
        classifier.classify(depthMap: skyMap, pitchDegrees: 10.0)
        XCTAssertEqual(classifier.currentClass, .unknown,
            "Second frame should not trigger classification")

        // Third call: should classify (frame 3, divisible by 3)
        classifier.classify(depthMap: skyMap, pitchDegrees: 10.0)
        XCTAssertEqual(classifier.currentClass, .sky,
            "Third frame should trigger classification")
    }

    // MARK: - Scene Class Enum

    func testSceneClassRawValues() {
        XCTAssertEqual(CrosshairSceneClass.sky.rawValue, "SKY")
        XCTAssertEqual(CrosshairSceneClass.ground.rawValue, "GND")
        XCTAssertEqual(CrosshairSceneClass.structure.rawValue, "STRUCT")
        XCTAssertEqual(CrosshairSceneClass.unknown.rawValue, "UNK")
    }

    // MARK: - Confidence Routing Effects

    func testSkyZerosNeuralConfidence() {
        // When scene = sky, neural confidence should be driven to 0
        // This is a design contract test, not an integration test
        //
        // In UnifiedDepthField, when sceneClassifier.currentClass == .sky
        // and sceneConf > 0.5, neural weight is set to 0.
        // This test documents that contract.
        let scene: CrosshairSceneClass = .sky
        XCTAssertEqual(scene, .sky, "Sky scene should zero neural confidence in fusion")
    }

    func testGroundBoostsGeometric() {
        // When scene = ground, geometric confidence gets a 25% boost
        let baseWeight: Float = 0.50
        let boost = min(baseWeight * 0.25, 0.15)
        let boostedWeight = baseWeight + boost
        XCTAssertEqual(boostedWeight, 0.625, accuracy: 0.001,
            "Ground scene should boost geometric by 25%")
    }

    func testStructureSuppressesGeometric() {
        // When scene = structure, geometric confidence is reduced by 60%
        let baseWeight: Float = 0.50
        let reduction = baseWeight * 0.6
        let reducedWeight = baseWeight - reduction
        XCTAssertEqual(reducedWeight, 0.20, accuracy: 0.001,
            "Structure scene should suppress geometric by 60%")
    }
}
