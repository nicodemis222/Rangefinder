//
//  RangefinderView.swift
//  Rangefinder
//
//  Main screen: camera preview + scene range ladder + tactical HUD.
//
//  Layout — compact single-row top bar + scene range overlay:
//
//    ┌──────────────────────────────────────┐
//    │ [2x] [FAR] [.308] [📏][🗺] [045°+3°] [≡] │  ← single row
//    │         ● 124 YDS  ±3              │  ← range readout
//    │      ◉ STABILIZED  ▓▓▓░            │  ← guidance
//    │                                      │
//    │     ◯ ─ ─ ─ ─ ─ ─ ─ [847 YDS ◯]   │  ← range ladder
//    │          ◉ [156 YDS M ●]            │  ← center pill
//    │   ◯ ─ ─ ─ ─ ─ ─ ─ ─ [ 52 YDS ◯]   │
//    │                                      │
//    │       ▓▓▓▓▓░░░ LIDAR  AI            │  ← source blend
//    └──────────────────────────────────────┘
//

import SwiftUI

struct RangefinderView: View {
    @EnvironmentObject var appState: AppState
    @Environment(\.scenePhase) private var scenePhase
    @State private var isPinching = false
    @State private var showTerrainSheet = false

    var body: some View {
        ZStack {
            // Layer 1: Camera preview (full bleed, hardware-zoomed)
            // Zoom is applied directly to the capture device by CameraManager —
            // real lens switching + ISP digital zoom at native quality.
            // LiDAR depth maps stay at full wide-angle FOV regardless.
            CameraPreviewView(
                cameraManager: appState.cameraManager
            )
            .ignoresSafeArea()

            // Layer 2: Scene-aware multi-point range overlay
            // Renders 5 range pills at ML-selected positions on the camera feed
            SceneRangeOverlay(
                result: appState.sceneRangeResult,
                displayUnit: appState.displayUnit
            )
            .ignoresSafeArea()
            .allowsHitTesting(false)

            // Layer 3: HUD elements — data in fixed zones
            VStack(spacing: 0) {
                // === TOP ZONE: chips + holdover + guidance ===
                VStack(spacing: 6) {
                    topBar
                        .padding(.horizontal, 16)

                    // Holdover — directly below top bar when active
                    if appState.ballisticsSolver.isEnabled,
                       let holdover = appState.currentHoldover,
                       holdover.isSignificant {
                        CompactHoldoverView(holdover: holdover)
                    }

                    // Operator guidance — stability + coaching hints
                    if appState.isModelsLoaded {
                        OperatorGuidanceView(engine: appState.guidanceEngine)
                    }
                }
                .padding(.top, 8)

                Spacer()

                // === BOTTOM ZONE: coherence + consensus + background chip + source blend ===
                VStack(spacing: 4) {
                    // Spatial coherence indicator (replaces semantic decision label)
                    SceneCoherenceIndicator(
                        coherence: appState.sceneRangeResult.spatialCoherence
                    )

                    // Anchor consensus — shown when center is suspect
                    if appState.sceneRangeResult.spatialCoherence == .centerSuspect,
                       let median = appState.sceneRangeResult.anchorMedianMeters {
                        AnchorConsensusView(
                            anchorMedianMeters: median,
                            displayUnit: appState.displayUnit
                        )
                    }

                    // Background hypothesis chip (alternate reading)
                    if appState.backgroundRange.isValid {
                        BackgroundRangeChip(
                            backgroundRange: appState.backgroundRange,
                            displayUnit: appState.displayUnit
                        )
                    }

                    // Source blend bar + legend
                    if appState.currentRange.isValid && !appState.currentRange.sourceWeights.isEmpty {
                        SourceBlendView(
                            sourceWeights: appState.currentRange.sourceWeights
                        )
                        .padding(.horizontal, 60)
                        .shadow(color: .black.opacity(0.5), radius: 3)
                    }
                }
                .padding(.bottom, 16)
            }

            // Layer 4: Tactical minimap (bottom-left, always-on with GPS)
            if appState.locationManager.hasValidFix {
                VStack {
                    Spacer()
                    HStack {
                        TacticalMinimapView(
                            userCoordinate: appState.locationManager.coordinate,
                            headingDegrees: appState.headingDegrees,
                            hitCoordinate: appState.demHitCoordinate
                        )
                        .onTapGesture {
                            showTerrainSheet = true
                        }
                        .padding(.leading, 12)
                        .padding(.bottom, 60)
                        Spacer()
                    }
                }
            }

            // Layer 7: Loading overlay (centered)
            if !appState.isModelsLoaded {
                LoadingOverlayView()
            }

            // Layer 8: Error banner (top, below safe area)
            if let error = appState.sessionError {
                VStack {
                    ErrorBannerView(message: error)
                        .padding(.top, 8)
                    Spacer()
                }
            }
        }
        .gesture(pinchGesture)
        .sheet(isPresented: $appState.showSettings) {
            SettingsView(
                displayUnit: $appState.displayUnit,
                reticleConfig: $appState.reticleConfig,
                cameraHeight: $appState.cameraHeight,
                stadiametricTargetSize: $appState.stadiametricTargetSize,
                ballisticsSolver: appState.ballisticsSolver,
                locationManager: appState.locationManager,
                regionManager: appState.regionManager
            )
        }
        .sheet(isPresented: $showTerrainSheet) {
            NavigationStack {
                SRTMDownloadView(
                    regionManager: appState.regionManager,
                    locationManager: appState.locationManager
                )
                .toolbar {
                    ToolbarItem(placement: .topBarTrailing) {
                        Button("DONE") { showTerrainSheet = false }
                            .font(.system(size: 14, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreen)
                    }
                }
            }
            .preferredColorScheme(.dark)
        }
        .onChange(of: scenePhase) { _, newPhase in
            switch newPhase {
            case .active:
                appState.resumeSession()
            case .inactive, .background:
                appState.pauseSession()
            @unknown default:
                break
            }
        }
        .statusBarHidden(true)
        .preferredColorScheme(.dark)
    }

    // MARK: - Top Bar

    /// Single-row consolidated top bar. Labels removed — values only.
    /// HDG + ELEV merged into one orientation chip.
    /// STADIA and MAP collapsed to icon-only toggles.
    private var topBar: some View {
        HStack(spacing: 6) {
            // Zoom (label-free)
            MilHUDChip {
                Text(zoomText)
                    .font(.system(size: 13, weight: .bold, design: .monospaced))
            }

            // Ballistics (only when enabled)
            if appState.ballisticsSolver.isEnabled {
                ballisticsChip
            }

            // Stadiametric mode (icon-only toggle)
            Button {
                withAnimation(.easeInOut(duration: 0.2)) {
                    appState.toggleStadiametricMode()
                }
            } label: {
                Image(systemName: "ruler")
                    .font(.system(size: 11, weight: .medium))
                    .foregroundColor(appState.isStadiametricMode ? Theme.milAmber : Theme.milGreenDim.opacity(0.5))
                    .frame(width: 28, height: 28)
                    .background(
                        RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                            .fill(Theme.panelBackground)
                            .overlay(
                                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                                    .stroke(
                                        appState.isStadiametricMode
                                            ? Theme.milAmber.opacity(0.5)
                                            : Theme.hudBorder.opacity(0.4),
                                        lineWidth: Theme.borderWidth
                                    )
                            )
                    )
            }

            Spacer()

            // Orientation: HDG + pitch combined (label-free)
            MilHUDChip {
                HStack(spacing: 4) {
                    if appState.locationManager.hasValidFix {
                        Text(headingText)
                            .font(.system(size: 11, weight: .medium, design: .monospaced))
                            .foregroundColor(Theme.milGreen.opacity(0.8))
                    }
                    Text(InclinationCorrector.formatAngle(appState.pitchDegrees))
                        .font(.system(size: 11, weight: .medium, design: .monospaced))
                        .foregroundColor(pitchColor)
                }
            }

            // Settings
            Button {
                appState.showSettings = true
            } label: {
                Image(systemName: "line.3.horizontal")
                    .font(.system(size: 13, weight: .medium))
                    .foregroundColor(Theme.milGreenDim)
                    .frame(width: 28, height: 28)
                    .background(
                        RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                            .fill(Theme.panelBackground)
                            .overlay(
                                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                                    .stroke(Theme.hudBorder.opacity(0.6), lineWidth: Theme.borderWidth)
                            )
                    )
            }
        }
    }

    // MARK: - Ballistics Chip

    private var ballisticsChip: some View {
        Menu {
            // Caliber selection
            Section("CALIBER") {
                ForEach(BallisticsSolver.Caliber.allCases) { caliber in
                    Button {
                        appState.ballisticsSolver.selectedCaliber = caliber
                    } label: {
                        HStack {
                            Text(caliber.rawValue)
                            if caliber == appState.ballisticsSolver.selectedCaliber {
                                Image(systemName: "checkmark")
                            }
                        }
                    }
                }
            }

            // Zero distance selection
            Section("ZERO DISTANCE") {
                ForEach(zeroDistanceOptions, id: \.self) { distance in
                    Button {
                        appState.ballisticsSolver.zeroDistance = Double(distance)
                    } label: {
                        HStack {
                            Text("\(distance) \(appState.displayUnit == .meters ? "M" : "YDS")")
                            if distance == Int(appState.ballisticsSolver.zeroDistance) {
                                Image(systemName: "checkmark")
                            }
                        }
                    }
                }
            }
        } label: {
            MilHUDChip {
                HStack(spacing: 4) {
                    Text(appState.ballisticsSolver.selectedCaliber.shortName)
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .foregroundColor(Theme.milAmber)
                    Text("Z\(Int(appState.ballisticsSolver.zeroDistance))")
                        .font(.system(size: 10, weight: .medium, design: .monospaced))
                        .foregroundColor(Theme.milAmberDim)
                }
            }
        }
    }

    private var zeroDistanceOptions: [Int] {
        stride(from: 25, through: 500, by: 25).map { $0 }
    }

    // MARK: - Computed

    private var zoomText: String {
        if appState.zoomFactor < 1.0 {
            return String(format: "%.1fx", appState.zoomFactor)
        } else if appState.zoomFactor == floor(appState.zoomFactor) {
            return String(format: "%.0fx", appState.zoomFactor)
        } else {
            return String(format: "%.1fx", appState.zoomFactor)
        }
    }

    private var headingText: String {
        let h = appState.headingDegrees
        let cardinal: String
        switch h {
        case 337.5..., 0..<22.5: cardinal = "N"
        case 22.5..<67.5:  cardinal = "NE"
        case 67.5..<112.5: cardinal = "E"
        case 112.5..<157.5: cardinal = "SE"
        case 157.5..<202.5: cardinal = "S"
        case 202.5..<247.5: cardinal = "SW"
        case 247.5..<292.5: cardinal = "W"
        case 292.5..<337.5: cardinal = "NW"
        default: cardinal = ""
        }
        return String(format: "%03.0f\u{00B0}%@", h, cardinal)
    }

    private var pitchColor: Color {
        let absPitch = abs(appState.pitchDegrees)
        if absPitch < 5 { return Theme.milGreen.opacity(0.8) }
        if absPitch < 15 { return Theme.milGreen }
        if absPitch < 30 { return Theme.milAmber }
        return Theme.milRed
    }

    // MARK: - Pinch Gesture

    private var pinchGesture: some Gesture {
        MagnifyGesture()
            .onChanged { value in
                if !isPinching {
                    isPinching = true
                    appState.zoomController.handlePinchBegan()
                }
                appState.handleZoom(magnification: value.magnification)
            }
            .onEnded { _ in
                isPinching = false
            }
    }
}

// MARK: - MilHUDChip (hard-edge tactical chip)

struct MilHUDChip<Content: View>: View {
    let content: () -> Content

    init(@ViewBuilder content: @escaping () -> Content) {
        self.content = content
    }

    var body: some View {
        content()
            .foregroundColor(Theme.milGreen.opacity(0.85))
            .padding(.horizontal, 10)
            .padding(.vertical, 6)
            .background(
                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                    .fill(Theme.panelBackground)
                    .overlay(
                        RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                            .stroke(Theme.hudBorder.opacity(0.6), lineWidth: Theme.borderWidth)
                    )
            )
    }
}

// MARK: - Range Readout (top HUD zone — fixed position, never overlaps reticle)

/// Primary range display: large number + unit + confidence dot + uncertainty.
/// Lives in the top data zone below the chip bar.
struct CrosshairRangeDisplay: View {
    let range: RangeOutput
    let displayUnit: UnitLength

    var body: some View {
        if range.isValid {
            HStack(alignment: .firstTextBaseline, spacing: 0) {
                // Confidence dot
                ConfidenceDot(confidence: range.confidence)
                    .padding(.trailing, 6)

                // Main range number
                Text(formattedRange)
                    .font(.system(size: 36, weight: .bold, design: .monospaced))
                    .foregroundColor(Theme.confidenceColor(for: range.confidence))
                    .contentTransition(.numericText())
                    .animation(.easeInOut(duration: 0.15), value: formattedRange)

                // Unit label
                Text(unitLabel)
                    .font(.system(size: 14, weight: .medium, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim)
                    .padding(.leading, 3)
                    .padding(.bottom, 1)

                // Uncertainty inline
                if range.uncertainty.converted(to: .meters).value > 0.5 {
                    Text("\u{00B1}\(formattedUncertainty)")
                        .font(.system(size: 11, weight: .medium, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim.opacity(0.5))
                        .padding(.leading, 4)
                        .padding(.bottom, 1)
                }
            }
            .shadow(color: .black.opacity(0.7), radius: 3, x: 0, y: 1)
        } else {
            Text("- - -")
                .font(.system(size: 36, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milGreenDim.opacity(0.3))
                .shadow(color: .black.opacity(0.5), radius: 3)
        }
    }

    // MARK: - Formatting

    private var formattedRange: String {
        let value = range.adjustedRange.converted(to: displayUnit).value
        if value < 10 {
            return String(format: "%.1f", value)
        } else {
            return String(format: "%.0f", value)
        }
    }

    private var formattedUncertainty: String {
        let value = range.uncertainty.converted(to: displayUnit).value
        if value < 1 {
            return String(format: "%.1f", value)
        } else {
            return String(format: "%.0f", value)
        }
    }

    private var unitLabel: String {
        displayUnit == .yards ? "YDS" : "M"
    }
}

// MARK: - Compact Holdover View (top HUD zone, inline with range)

/// Horizontal holdover readout: triangle + mils + inches/cm + direction.
/// Sits directly below the range number in the top data zone.
/// Shows mils (primary) and inches or cm (secondary) based on display unit setting.
struct CompactHoldoverView: View {
    let holdover: HoldoverResult

    var body: some View {
        HStack(spacing: 6) {
            // Direction triangle (up = hold high, down = hold low)
            TriangleShape(pointingUp: holdover.holdHigh)
                .fill(Theme.milAmber)
                .frame(width: 12, height: 8)

            // Mil value (primary)
            Text(holdover.displayMils)
                .font(.system(size: 16, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milAmber)

            // MIL label + direction
            Text("MIL \(holdover.directionText)")
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milAmber.opacity(0.7))

            // Separator
            Text("·")
                .font(.system(size: 10, weight: .bold))
                .foregroundColor(Theme.milAmber.opacity(0.4))

            // Inches/CM value (secondary)
            Text("\(holdover.displayValue) \(holdover.unitLabel)")
                .font(.system(size: 12, weight: .medium, design: .monospaced))
                .foregroundColor(Theme.milAmber.opacity(0.7))
        }
        .shadow(color: .black.opacity(0.6), radius: 3, x: 0, y: 1)
    }
}

// MARK: - Confidence Dot (inline, tactical colors)

struct ConfidenceDot: View {
    let confidence: Float
    @State private var displayedColor: Color = Theme.confidenceHigh

    var body: some View {
        Circle()
            .fill(displayedColor)
            .frame(width: 10, height: 10)
            .overlay(
                Circle()
                    .stroke(displayedColor.opacity(0.4), lineWidth: 2)
                    .frame(width: 16, height: 16)
            )
            .onChange(of: confidence) { _, newValue in
                withAnimation(.easeInOut(duration: 0.5)) {
                    displayedColor = Theme.confidenceColor(for: newValue)
                }
            }
            .onAppear {
                displayedColor = Theme.confidenceColor(for: confidence)
            }
    }
}

// (PinchZoomHint removed — zoom is discoverable via MAG chip in top bar)

// MARK: - Loading Overlay (tactical "INITIALIZING")

struct LoadingOverlayView: View {
    @State private var progress: CGFloat = 0

    var body: some View {
        VStack(spacing: 14) {
            Text("INITIALIZING")
                .font(.system(size: 14, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milGreen)
                .tracking(2)

            GeometryReader { geometry in
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: 2)
                        .fill(Theme.panelBackground)
                        .frame(height: 4)

                    RoundedRectangle(cornerRadius: 2)
                        .fill(Theme.milGreen)
                        .frame(width: geometry.size.width * progress, height: 4)
                }
            }
            .frame(width: 160, height: 4)

            Text("LOADING MODELS...")
                .font(.system(size: 10, weight: .medium, design: .monospaced))
                .foregroundColor(Theme.milGreenDim)
        }
        .padding(24)
        .background(
            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                .fill(Theme.panelBackground)
                .overlay(
                    RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                        .stroke(Theme.hudBorder.opacity(0.5), lineWidth: Theme.borderWidth)
                )
        )
        .onAppear {
            withAnimation(.easeInOut(duration: 2.0).repeatForever(autoreverses: true)) {
                progress = 0.85
            }
        }
    }
}

// MARK: - Error Banner (CAUTION/WARNING style)

struct ErrorBannerView: View {
    let message: String

    var body: some View {
        HStack(spacing: 8) {
            Text("\u{26A0} CAUTION")
                .font(.system(size: 11, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milAmber)

            Text(message)
                .font(.system(size: 13, weight: .medium, design: .monospaced))
                .foregroundColor(Theme.milAmber.opacity(0.85))
                .lineLimit(2)
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 10)
        .background(
            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                .fill(Theme.panelBackground)
                .overlay(
                    RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                        .stroke(Theme.milAmber.opacity(0.6), lineWidth: Theme.borderWidth)
                )
        )
        .padding(.horizontal, 24)
    }
}
