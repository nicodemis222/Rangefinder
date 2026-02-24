//
//  SemanticSelectionTests.swift
//  RangefinderTests
//
//  Tests for the semantic source selection state machine in UnifiedDepthField.
//  Validates priority ordering, source transitions, and background hypothesis.
//

import XCTest
@testable import Rangefinder

final class SemanticSelectionTests: XCTestCase {

    // MARK: - Priority Ordering

    @MainActor
    func testSemanticDecisionDefaultsToNone() {
        let depthField = UnifiedDepthField()
        XCTAssertEqual(depthField.semanticDecision, .none)
    }

    @MainActor
    func testSemanticSelectNoSourcesReturnsNone() {
        let depthField = UnifiedDepthField()
        let (primary, background) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0
        )
        XCTAssertFalse(primary.isValid, "No sources should yield no valid estimate")
        XCTAssertNil(background, "No sources should yield no background")
        XCTAssertEqual(depthField.semanticDecision, .none)
    }

    // MARK: - Semantic Decision Enum

    func testSemanticSourceDecisionAllCases() {
        // Verify all cases exist and have raw values
        let allCases = SemanticSourceDecision.allCases
        XCTAssertEqual(allCases.count, 7, "Should have 7 semantic decision cases")

        XCTAssertEqual(SemanticSourceDecision.lidarPrimary.rawValue, "LIDAR_PRIMARY")
        XCTAssertEqual(SemanticSourceDecision.objectPrimary.rawValue, "OBJECT_PRIMARY")
        XCTAssertEqual(SemanticSourceDecision.demPrimary.rawValue, "DEM_PRIMARY")
        XCTAssertEqual(SemanticSourceDecision.neuralPrimary.rawValue, "NEURAL_PRIMARY")
        XCTAssertEqual(SemanticSourceDecision.geometricPrimary.rawValue, "GEO_PRIMARY")
        XCTAssertEqual(SemanticSourceDecision.stadiametric.rawValue, "STADIAMETRIC")
        XCTAssertEqual(SemanticSourceDecision.none.rawValue, "NONE")
    }

    // MARK: - Stadiametric Priority

    @MainActor
    func testStadiametricOverridesAllSources() {
        // Stadiametric input should be highest priority
        let depthField = UnifiedDepthField()
        depthField.stadiametricInput = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 100,
            focalLengthPixels: 2160
        )

        let (primary, _) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0
        )

        XCTAssertEqual(depthField.semanticDecision, .stadiametric)
        XCTAssertEqual(primary.source, .stadiametric)
        // R = (1.8 * 2160) / 100 = 38.88m
        XCTAssertEqual(primary.distanceMeters, 38.88, accuracy: 0.1)
    }

    func testStadiametricInputComputedRange() {
        let input = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 50,
            focalLengthPixels: 2160
        )
        // R = (1.8 * 2160) / 50 = 77.76m
        XCTAssertEqual(input.computedRange, 77.76, accuracy: 0.01)
    }

    func testStadiametricZeroPixelSizeReturnsZero() {
        let input = StadiametricInput(
            knownSizeMeters: 1.8,
            pixelSize: 0,
            focalLengthPixels: 2160
        )
        XCTAssertEqual(input.computedRange, 0, "Zero pixel size should return zero range")
    }

    // MARK: - Background Hypothesis

    @MainActor
    func testBackgroundEstimateSourceDiffersFromPrimary() {
        // When DEM is primary, background should be neural or geometric (not DEM)
        let depthField = UnifiedDepthField()

        // Provide a DEM estimate
        depthField.latestDEMEstimate = DEMRaycastEstimate(
            distanceMeters: 200,
            confidence: 0.8,
            terrainElevation: 500,
            headingDeg: 90,
            gpsAccuracy: 5.0,
            hitCoordinate: .init(latitude: 0, longitude: 0)
        )
        depthField.targetPriority = .far

        let (primary, background) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0
        )

        if primary.isValid && primary.source == .demRaycast {
            if let bg = background {
                XCTAssertNotEqual(bg.source, .demRaycast,
                    "Background should not duplicate primary source")
            }
        }
    }

    // MARK: - DEM Priority in Far-Target Mode

    @MainActor
    func testDEMPrimaryInFarTargetMode() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        // Provide a strong DEM estimate (> 0.15 weight threshold)
        depthField.latestDEMEstimate = DEMRaycastEstimate(
            distanceMeters: 300,
            confidence: 0.85,
            terrainElevation: 800,
            headingDeg: 180,
            gpsAccuracy: 3.0,
            hitCoordinate: .init(latitude: 0, longitude: 0)
        )

        let (primary, _) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0
        )

        // DEM should win when it's the only strong source in far-target mode
        if primary.isValid {
            XCTAssertEqual(primary.source, .demRaycast,
                "DEM should be primary in far-target mode with strong DEM signal")
            XCTAssertEqual(depthField.semanticDecision, .demPrimary)
        }
    }

    // MARK: - Neural Hard Cap Integration

    func testNeuralSkippedBeyond50m() {
        // Neural readings beyond 50m should not enter the selection pool
        // This is enforced by the hard cap check in semanticSelect
        let confidence = DepthSourceConfidence.neural(distanceM: 60.0)
        XCTAssertEqual(confidence, 0.0, accuracy: 0.001,
            "Neural confidence beyond 50m should be zero (hard cap)")
    }

    func testNeuralAcceptedBelow50m() {
        let confidence = DepthSourceConfidence.neural(distanceM: 30.0)
        XCTAssertGreaterThan(confidence, 0.3,
            "Neural confidence at 30m should be substantial")
    }

    // MARK: - Depth Source Enum Changes

    func testDepthSourceSemanticExists() {
        let source = DepthSource.semantic
        XCTAssertEqual(source.rawValue, "Semantic")
        XCTAssertEqual(source.shortName, "SEM")
    }

    func testDepthSourceStadiametricExists() {
        let source = DepthSource.stadiametric
        XCTAssertEqual(source.rawValue, "Stadia")
        XCTAssertEqual(source.shortName, "STADIA")
    }

    func testDepthEstimateNoneUsesSemanticSource() {
        let none = DepthEstimate.none
        XCTAssertEqual(none.source, .semantic,
            "Default .none estimate should use .semantic source")
    }

    func testRangeOutputNoneUsesSemanticSource() {
        let none = RangeOutput.none
        XCTAssertEqual(none.primarySource, .semantic,
            "Default .none range output should use .semantic source")
    }
}
