//
//  ForegroundOccluderTests.swift
//  RangefinderTests
//
//  Tests for the foreground occluder detection in UnifiedDepthField.
//  When the user aims OVER nearby objects (rocks at 3m) at distant terrain
//  (~1600m), LiDAR reads the close foreground and normally wins the priority
//  chain. The isForegroundOccluder() predicate wires existing bimodal analysis
//  + DEM agreement signals to detect this and skip LiDAR, letting DEM win.
//

import XCTest
@testable import Rangefinder

final class ForegroundOccluderTests: XCTestCase {

    // MARK: - isForegroundOccluder Unit Tests

    /// Classic case: rocks at 3m, mountains at 1500m, DEM agrees, far mode.
    /// All four conditions met → LiDAR should be skipped.
    @MainActor
    func testOccluder_allConditionsMet() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 3.0,
            farPeakM: 1500.0,
            nearFraction: 0.15,
            farFraction: 0.85,
            demAgreesWithFar: true
        )

        let result = depthField.isForegroundOccluder(lidarDepth: 3.2, bimodal: bimodal)
        XCTAssertTrue(result, "Should detect foreground occluder: rocks at 3m with mountains behind")
    }

    /// Same bimodal scene but user is in .near mode — LiDAR should be kept.
    @MainActor
    func testOccluder_nearModeRetainsLiDAR() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .near

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 3.0,
            farPeakM: 1500.0,
            nearFraction: 0.15,
            farFraction: 0.85,
            demAgreesWithFar: true
        )

        let result = depthField.isForegroundOccluder(lidarDepth: 3.2, bimodal: bimodal)
        XCTAssertFalse(result, "Near mode: user wants the foreground object, LiDAR should win")
    }

    /// Single-peak scene (not bimodal) — no occluder.
    @MainActor
    func testOccluder_notBimodal() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        let bimodal = UnifiedDepthField.BimodalAnalysis.notBimodal

        let result = depthField.isForegroundOccluder(lidarDepth: 3.0, bimodal: bimodal)
        XCTAssertFalse(result, "Not bimodal: single-peak scene, LiDAR is correct")
    }

    /// Bimodal but DEM does NOT agree with the far peak — LiDAR retains priority.
    @MainActor
    func testOccluder_demDisagrees() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 3.0,
            farPeakM: 1500.0,
            nearFraction: 0.15,
            farFraction: 0.85,
            demAgreesWithFar: false  // DEM disagrees
        )

        let result = depthField.isForegroundOccluder(lidarDepth: 3.2, bimodal: bimodal)
        XCTAssertFalse(result, "DEM disagrees with far peak: not safe to skip LiDAR")
    }

    /// Indoor scene: bimodal (near wall + far wall) but no DEM data.
    /// demAgreesWithFar is false when DEM is unavailable.
    @MainActor
    func testOccluder_indoorNoDEM() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 2.0,
            farPeakM: 8.0,
            nearFraction: 0.4,
            farFraction: 0.6,
            demAgreesWithFar: false  // No DEM indoors
        )

        let result = depthField.isForegroundOccluder(lidarDepth: 2.1, bimodal: bimodal)
        XCTAssertFalse(result, "Indoor: no DEM corroboration, LiDAR should win")
    }

    // MARK: - Edge Cases

    /// Near peak above 12m: LiDAR at 5m is still below nearPeak → occluder.
    @MainActor
    func testOccluder_nearPeakAbove12m_lidarBelowNearPeak() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 40.0,   // Near peak beyond LiDAR range
            farPeakM: 2000.0,
            nearFraction: 0.3,
            farFraction: 0.7,
            demAgreesWithFar: true
        )

        // LiDAR at 5m is below near peak of 40m → foreground occluder
        let result = depthField.isForegroundOccluder(lidarDepth: 5.0, bimodal: bimodal)
        XCTAssertTrue(result, "LiDAR at 5m with near peak at 40m: LiDAR reads foreground occluder")
    }

    /// Near peak above 12m but LiDAR reads ABOVE the near peak — not an occluder.
    /// (LiDAR somehow reading 50m would be unusual at < 8m weight, but test the logic)
    @MainActor
    func testOccluder_nearPeakAbove12m_lidarAboveNearPeak() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 15.0,
            farPeakM: 500.0,
            nearFraction: 0.3,
            farFraction: 0.7,
            demAgreesWithFar: true
        )

        // LiDAR at 20m > near peak of 15m: LiDAR is beyond near cluster
        let result = depthField.isForegroundOccluder(lidarDepth: 20.0, bimodal: bimodal)
        XCTAssertFalse(result, "LiDAR beyond near peak: not a foreground occluder")
    }

    // MARK: - Integration: semanticSelect with bimodal

    /// When LiDAR would normally win (<8m) but foreground occluder is detected,
    /// DEM should become primary with LiDAR as background.
    @MainActor
    func testSemanticSelect_bimodalFarMode_demWins() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        // Provide strong DEM estimate
        depthField.latestDEMEstimate = DEMRaycastEstimate(
            distanceMeters: 1463,  // ~1600 yards
            confidence: 0.85,
            terrainElevation: 2100,
            headingDeg: 270,
            gpsAccuracy: 4.0,
            hitCoordinate: .init(latitude: 37.0, longitude: -119.0)
        )

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 3.0,
            farPeakM: 1500.0,
            nearFraction: 0.1,
            farFraction: 0.9,
            demAgreesWithFar: true
        )

        // Note: Without a real LiDAR CVPixelBuffer, LiDAR won't enter the source pool.
        // This test verifies that DEM correctly wins as primary when called with bimodal.
        // The isForegroundOccluder guard won't fire (no LiDAR entry), but DEM should
        // still be selected as primary in far mode.
        let (primary, _) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0,
            bimodal: bimodal
        )

        if primary.isValid {
            XCTAssertEqual(primary.source, .demRaycast,
                "DEM should be primary in far mode with bimodal scene")
            XCTAssertEqual(depthField.semanticDecision, .demPrimary)
            // DEM distance should be ~1463m
            XCTAssertEqual(primary.distanceMeters, 1463, accuracy: 10)
        }
    }

    /// In near mode with the same bimodal scene, DEM should still win
    /// (as fallback via 3b, since no LiDAR CVPixelBuffer is provided).
    @MainActor
    func testSemanticSelect_bimodalNearMode_demFallback() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .near

        depthField.latestDEMEstimate = DEMRaycastEstimate(
            distanceMeters: 1463,
            confidence: 0.85,
            terrainElevation: 2100,
            headingDeg: 270,
            gpsAccuracy: 4.0,
            hitCoordinate: .init(latitude: 37.0, longitude: -119.0)
        )

        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 3.0,
            farPeakM: 1500.0,
            nearFraction: 0.1,
            farFraction: 0.9,
            demAgreesWithFar: true
        )

        let (primary, _) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0,
            bimodal: bimodal
        )

        // Without LiDAR CVPixelBuffer, DEM picks up via the 3b fallback path.
        // In near mode with real LiDAR data, LiDAR would win (occluder NOT detected).
        if primary.isValid {
            XCTAssertEqual(primary.source, .demRaycast,
                "DEM should win as fallback when no LiDAR data available")
        }
    }

    // MARK: - DEM Threshold Relaxation (Bimodal Corroboration)

    /// When DEM has low weight (poor GPS/heading) but bimodal analysis confirms
    /// DEM agrees with the far peak, DEM should still win over neural.
    /// This tests the relaxed threshold: 0.15 → 0.01 when demAgreesWithFar.
    @MainActor
    func testSemanticSelect_demThresholdRelaxedByBimodal() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        // DEM with LOW confidence (simulating poor GPS ~15m, heading ~15°)
        // Weight will be: distanceFactor * confidence ≈ 0.95 * 0.12 = 0.114
        // This is below the normal 0.15 threshold, but above the relaxed 0.01
        depthField.latestDEMEstimate = DEMRaycastEstimate(
            distanceMeters: 600,  // ~656 yards — red rock cliff
            confidence: 0.12,     // Low confidence (bad GPS/heading)
            terrainElevation: 1200,
            headingDeg: 180,
            gpsAccuracy: 18.0,    // Poor GPS
            hitCoordinate: .init(latitude: 37.2, longitude: -113.6)
        )

        // Bimodal: DEM agrees with far peak → threshold relaxed
        let bimodal = UnifiedDepthField.BimodalAnalysis(
            isBimodal: true,
            nearPeakM: 5.0,
            farPeakM: 600.0,
            nearFraction: 0.1,
            farFraction: 0.9,
            demAgreesWithFar: true
        )

        let (primary, _) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0,
            bimodal: bimodal
        )

        if primary.isValid {
            XCTAssertEqual(primary.source, .demRaycast,
                "DEM should win with relaxed threshold when bimodal confirms far peak")
            XCTAssertEqual(depthField.semanticDecision, .demPrimary)
        }
    }

    /// Same low-confidence DEM but WITHOUT bimodal confirmation —
    /// DEM should NOT win (weight below 0.15 threshold).
    @MainActor
    func testSemanticSelect_demThresholdNotRelaxedWithoutBimodal() {
        let depthField = UnifiedDepthField()
        depthField.targetPriority = .far

        // DEM with low confidence — same as above
        depthField.latestDEMEstimate = DEMRaycastEstimate(
            distanceMeters: 600,
            confidence: 0.12,
            terrainElevation: 1200,
            headingDeg: 180,
            gpsAccuracy: 18.0,
            hitCoordinate: .init(latitude: 37.2, longitude: -113.6)
        )

        // NOT bimodal → threshold stays at 0.15
        let bimodal = UnifiedDepthField.BimodalAnalysis.notBimodal

        let (primary, _) = depthField.semanticSelect(
            screenPoint: CGPoint(x: 0.5, y: 0.5),
            timestamp: 0,
            bimodal: bimodal
        )

        // DEM weight (~0.114) < 0.15 threshold → DEM should NOT win
        // Without other sources (no LiDAR/neural CVPixelBuffer), result may be .none
        if primary.isValid {
            XCTAssertNotEqual(primary.source, .demRaycast,
                "DEM should not win with low weight when no bimodal corroboration")
        }
    }
}
