//
//  DepthZoneBracketTests.swift
//  RangefinderTests
//
//  Tests for the depth zone bracket overlay — the always-on reticle
//  visualization that shows crosshair depth vs DEM terrain depth.
//  When the two disagree, dual brackets visually separate to alert
//  the operator of a depth-scene conflict (e.g., 42m rocks vs 1600m cliff).
//

import XCTest
@testable import Rangefinder

@MainActor
final class DepthZoneBracketTests: XCTestCase {

    // MARK: - DepthZoneOverlay Defaults

    func testDepthZoneOverlayNone() {
        let overlay = DepthZoneOverlay.none
        XCTAssertFalse(overlay.isBimodal)
        XCTAssertEqual(overlay.crosshairDepthM, 0)
        XCTAssertEqual(overlay.demDepthM, 0)
        XCTAssertFalse(overlay.hasDEM)
        XCTAssertEqual(overlay.activeZone, .far)
        XCTAssertEqual(overlay.nearPeakM, 0)
        XCTAssertEqual(overlay.farPeakM, 0)
    }

    func testDepthZoneOverlayNoneIsNotActive() {
        let overlay = DepthZoneOverlay.none
        XCTAssertFalse(overlay.isActive, ".none should not be active (no depth data)")
    }

    func testDepthZoneOverlayNoneHasNoDisagreement() {
        let overlay = DepthZoneOverlay.none
        XCTAssertFalse(overlay.hasDisagreement, ".none should have no disagreement")
    }

    // MARK: - DepthZoneOverlay Construction

    func testDepthZoneOverlayCrosshairOnly() {
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 42,
            demDepthM: 0,
            hasDEM: false,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertFalse(overlay.isBimodal)
        XCTAssertEqual(overlay.crosshairDepthM, 42)
        XCTAssertTrue(overlay.isActive, "Should be active with crosshair depth > 1")
        XCTAssertFalse(overlay.hasDisagreement, "No DEM means no disagreement")
    }

    func testDepthZoneOverlayDEMOnly() {
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 0,
            demDepthM: 1600,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertTrue(overlay.isActive, "Should be active with DEM data")
        XCTAssertFalse(overlay.hasDisagreement, "Crosshair < 1 means no disagreement check")
    }

    func testDepthZoneOverlayAgreement() {
        // Crosshair and DEM agree (ratio < 2.0)
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 800,
            demDepthM: 900,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertTrue(overlay.isActive)
        XCTAssertFalse(overlay.hasDisagreement, "900/800 = 1.125 — should agree")
    }

    func testDepthZoneOverlayDisagreement() {
        // The classic problem: crosshair reads 42m rocks, DEM reads 1600m cliff
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 42,
            demDepthM: 1600,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertTrue(overlay.isActive)
        XCTAssertTrue(overlay.hasDisagreement, "1600/42 = 38× — massive disagreement")
    }

    func testDepthZoneOverlayDisagreementThreshold() {
        // Exactly at 2.0× ratio — should NOT disagree (> 2.0 required)
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 100,
            demDepthM: 200,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertFalse(overlay.hasDisagreement, "Exactly 2.0 ratio should not disagree")

        // Just above 2.0× — should disagree
        let overlay2 = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 100,
            demDepthM: 201,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertTrue(overlay2.hasDisagreement, "201/100 > 2.0 — should disagree")
    }

    func testDepthZoneOverlayBimodalWithDEM() {
        let overlay = DepthZoneOverlay(
            isBimodal: true,
            crosshairDepthM: 42,
            demDepthM: 1600,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 40,
            farPeakM: 1600
        )
        XCTAssertTrue(overlay.isBimodal)
        XCTAssertTrue(overlay.isActive)
        XCTAssertTrue(overlay.hasDisagreement)
        XCTAssertEqual(overlay.nearPeakM, 40)
        XCTAssertEqual(overlay.farPeakM, 1600)
    }

    func testDepthZoneOverlayNearPriority() {
        let overlay = DepthZoneOverlay(
            isBimodal: true,
            crosshairDepthM: 5,
            demDepthM: 300,
            hasDEM: true,
            activeZone: .near,
            nearPeakM: 5, farPeakM: 300
        )
        XCTAssertEqual(overlay.activeZone, .near)
    }

    // MARK: - isActive Edge Cases

    func testIsActiveWithSmallCrosshairDepth() {
        // Crosshair depth of exactly 1.0 is NOT active (must be > 1)
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 1.0,
            demDepthM: 0,
            hasDEM: false,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertFalse(overlay.isActive, "1.0m exactly should not be active")
    }

    func testIsActiveWithJustAboveOneDepth() {
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 1.1,
            demDepthM: 0,
            hasDEM: false,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertTrue(overlay.isActive, "1.1m should be active")
    }

    // MARK: - Equatable

    func testDepthZoneOverlayEquatable() {
        let a = DepthZoneOverlay(
            isBimodal: true,
            crosshairDepthM: 42,
            demDepthM: 1600,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 40, farPeakM: 1600
        )
        let b = DepthZoneOverlay(
            isBimodal: true,
            crosshairDepthM: 42,
            demDepthM: 1600,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 40, farPeakM: 1600
        )
        XCTAssertEqual(a, b)
    }

    func testDepthZoneOverlayNotEqual() {
        let a = DepthZoneOverlay.none
        let b = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 42,
            demDepthM: 1600,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        XCTAssertNotEqual(a, b)
    }

    // MARK: - Depth Label Formatting

    func testFormatDepthMeters() {
        XCTAssertEqual(DepthZoneOverlay.formatDepth(5), "~5m")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(42), "~42m")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(300), "~300m")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(999), "~999m")
    }

    func testFormatDepthKilometers() {
        XCTAssertEqual(DepthZoneOverlay.formatDepth(1000), "~1.0km")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(1500), "~1.5km")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(1600), "~1.6km")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(2000), "~2.0km")
    }

    func testFormatDepthSubMeter() {
        // Sub-meter values should show "--" (noise floor)
        XCTAssertEqual(DepthZoneOverlay.formatDepth(0), "--")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(0.5), "--")
        XCTAssertEqual(DepthZoneOverlay.formatDepth(0.99), "--")
    }

    func testFormatDepthBoundary() {
        // Exactly 1.0 should show "~1m"
        XCTAssertEqual(DepthZoneOverlay.formatDepth(1.0), "~1m")
    }

    // MARK: - AppState Integration

    func testAppStateDepthZoneOverlayDefault() {
        let appState = AppState()
        let overlay = appState.depthZoneOverlay
        XCTAssertFalse(overlay.isBimodal, "Default state should not be bimodal")
        // activeZone reflects targetPriority which may be persisted — just check it's valid
        XCTAssertTrue(overlay.activeZone == .far || overlay.activeZone == .near)
        XCTAssertFalse(overlay.isActive, "No depth data in default state")
    }

    func testAppStateDepthZoneOverlayReflectsPriority() {
        let appState = AppState()

        // Explicitly set to far, then verify
        appState.targetPriority = .far
        XCTAssertEqual(appState.depthZoneOverlay.activeZone, .far)

        // Switch to near
        appState.targetPriority = .near
        XCTAssertEqual(appState.depthZoneOverlay.activeZone, .near,
                       "Overlay should reflect current target priority")

        // Switch back to far
        appState.targetPriority = .far
        XCTAssertEqual(appState.depthZoneOverlay.activeZone, .far,
                       "Overlay should update when priority changes back")
    }

    // MARK: - UnifiedDepthField BimodalAnalysis Exposure

    func testDepthFieldExposesLatestBimodalAnalysis() {
        let depthField = UnifiedDepthField()
        XCTAssertFalse(depthField.latestBimodalAnalysis.isBimodal)
        XCTAssertEqual(depthField.latestBimodalAnalysis.nearPeakM, 0)
        XCTAssertEqual(depthField.latestBimodalAnalysis.farPeakM, 0)
    }

    // MARK: - FFPReticleView Instantiation

    func testFFPReticleViewAcceptsDepthZones() {
        let overlay = DepthZoneOverlay(
            isBimodal: false,
            crosshairDepthM: 42,
            demDepthM: 1600,
            hasDEM: true,
            activeZone: .far,
            nearPeakM: 0, farPeakM: 0
        )
        let view = FFPReticleView(
            zoomFactor: 1.0,
            configuration: .default,
            depthZones: overlay
        )
        XCTAssertNotNil(view)
    }

    func testFFPReticleViewDefaultsToNoDepthZones() {
        let view = FFPReticleView(
            zoomFactor: 1.0,
            configuration: .default
        )
        XCTAssertFalse(view.depthZones.isBimodal)
        XCTAssertFalse(view.depthZones.isActive)
    }
}
