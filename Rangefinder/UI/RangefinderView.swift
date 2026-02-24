//
//  RangefinderView.swift
//  Rangefinder
//
//  Main screen: camera preview + FFP reticle + tactical HUD overlays.
//
//  Layout (modeled on Burris Eliminator 6 / SIG Kilo HUD zones):
//  Data lives in fixed HUD zones at top/bottom — reticle stays clear.
//
//    ┌──────────────────────────────────────┐
//    │ [MAG 1x] [.308 Z100] [HDG 045°]   │
//    │                   [ELEV +2°] [≡]   │  ← top bar chips
//    │         ● 124 YDS  ±3              │  ← range readout (top zone)
//    │         ▲ 1.2 MIL HIGH             │  ← holdover (amber)
//    │      ◉ STABILIZED  ▓▓▓░            │  ← operator guidance
//    │                                      │
//    │            ╋ reticle (CLEAR)         │  ← crosshair — unobstructed
//    │                                      │
//    │       ▓▓▓▓▓░░░ LIDAR  AI            │  ← source blend (bottom zone)
//    └──────────────────────────────────────┘
//

import SwiftUI

struct RangefinderView: View {
    @EnvironmentObject var appState: AppState
    @Environment(\.scenePhase) private var scenePhase
    @State private var isPinching = false
    @State private var showPinchHint = true

    var body: some View {
        ZStack {
            // Layer 1: Camera preview (full bleed)
            CameraPreviewView(cameraManager: appState.cameraManager)
                .ignoresSafeArea()

            // Layer 2: FFP Reticle (scales with zoom, full bleed)
            // Depth zone brackets appear when bimodal detection is active
            FFPReticleView(
                zoomFactor: appState.zoomFactor,
                configuration: appState.reticleConfig,
                depthZones: appState.depthZoneOverlay
            )
            .ignoresSafeArea()
            .allowsHitTesting(false)

            // Layer 2b: Stadiametric bracket overlay (when active)
            if appState.isStadiametricMode {
                StadiametricBracketOverlay { pixelSize in
                    appState.updateStadiametricInput(pixelSize: pixelSize)
                }
                .ignoresSafeArea()
            }

            // Layer 3: Pinch-to-zoom hint (near crosshair, fades after first pinch)
            if showPinchHint && !isPinching {
                PinchZoomHint(zoomFactor: appState.zoomFactor)
                    .transition(.opacity)
            }

            // Layer 4: HUD elements — data in fixed zones, reticle stays clear
            VStack(spacing: 0) {
                // === TOP ZONE: chips + range + holdover ===
                VStack(spacing: 6) {
                    topBar
                        .padding(.horizontal, 16)

                    // Range readout — primary data
                    CrosshairRangeDisplay(
                        range: appState.currentRange,
                        displayUnit: appState.displayUnit
                    )

                    // Holdover — directly below range when active
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

                // === BOTTOM ZONE: semantic label + background chip + source blend ===
                VStack(spacing: 4) {
                    // Semantic decision label (which source won)
                    if appState.semanticDecision != .none {
                        SemanticDecisionLabel(decision: appState.semanticDecision)
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

            // Layer 5: Map PiP (bottom-right corner, when enabled)
            if appState.showMapPiP,
               appState.locationManager.hasValidFix {
                VStack {
                    Spacer()
                    HStack {
                        Spacer()
                        MapPiPView(
                            userCoordinate: appState.locationManager.coordinate,
                            hitCoordinate: appState.demHitCoordinate,
                            headingDegrees: appState.headingDegrees
                        )
                        .padding(.trailing, 12)
                        .padding(.bottom, 120)
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
                targetPriority: $appState.targetPriority,
                stadiametricTargetSize: $appState.stadiametricTargetSize,
                ballisticsSolver: appState.ballisticsSolver,
                locationManager: appState.locationManager,
                regionManager: appState.regionManager
            )
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

    private var topBar: some View {
        VStack(spacing: 4) {
            // Row 1: Core data chips
            HStack(spacing: 8) {
                // Magnification indicator
                MilHUDChip {
                    HStack(spacing: 4) {
                        Text("MAG")
                            .font(.system(size: 10, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim)
                        Text(zoomText)
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                    }
                }

                // Target priority chip (Near/Far — tappable toggle)
                targetPriorityChip

                // Ballistics chip (only when enabled)
                if appState.ballisticsSolver.isEnabled {
                    ballisticsChip
                }

                // Heading chip (compass bearing)
                if appState.locationManager.hasValidFix {
                    MilHUDChip {
                        HStack(spacing: 4) {
                            Text("HDG")
                                .font(.system(size: 10, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milGreenDim)
                            Text(headingText)
                                .font(.system(size: 13, weight: .medium, design: .monospaced))
                        }
                    }
                }

                Spacer()

                // Elevation indicator
                MilHUDChip {
                    HStack(spacing: 4) {
                        Text("ELEV")
                            .font(.system(size: 10, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim)
                        Text(InclinationCorrector.formatAngle(appState.pitchDegrees))
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                    }
                    .foregroundColor(pitchColor)
                }

                // Settings — tactical menu icon
                Button {
                    appState.showSettings = true
                } label: {
                    Image(systemName: "line.3.horizontal")
                        .font(.system(size: 14, weight: .medium))
                        .foregroundColor(Theme.milGreenDim)
                        .frame(width: 34, height: 34)
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

            // Row 2: Mode toggles (only when modes are available)
            if appState.isStadiametricMode || appState.showMapPiP || appState.locationManager.hasValidFix {
                HStack(spacing: 8) {
                    // Stadiametric mode toggle
                    Button {
                        withAnimation(.easeInOut(duration: 0.2)) {
                            appState.toggleStadiametricMode()
                        }
                    } label: {
                        MilHUDChip {
                            HStack(spacing: 3) {
                                Image(systemName: "ruler")
                                    .font(.system(size: 10, weight: .medium))
                                Text("STADIA")
                                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                            }
                            .foregroundColor(appState.isStadiametricMode ? Theme.milAmber : Theme.milGreenDim)
                        }
                    }

                    // Map PiP toggle (only with GPS)
                    if appState.locationManager.hasValidFix {
                        Button {
                            withAnimation(.easeInOut(duration: 0.2)) {
                                appState.showMapPiP.toggle()
                            }
                        } label: {
                            MilHUDChip {
                                HStack(spacing: 3) {
                                    Image(systemName: "map")
                                        .font(.system(size: 10, weight: .medium))
                                    Text("MAP")
                                        .font(.system(size: 10, weight: .bold, design: .monospaced))
                                }
                                .foregroundColor(appState.showMapPiP ? Theme.milAmber : Theme.milGreenDim)
                            }
                        }
                    }

                    Spacer()
                }
            }
        }
    }

    // MARK: - Target Priority Chip

    /// Tappable chip showing Near/Far target priority mode.
    /// Shows a bimodal indicator when foreground occluders are detected.
    private var targetPriorityChip: some View {
        Button {
            withAnimation(.easeInOut(duration: 0.2)) {
                appState.targetPriority = appState.targetPriority == .near ? .far : .near
            }
        } label: {
            MilHUDChip {
                HStack(spacing: 4) {
                    // Bimodal indicator (flashes when occluder detected)
                    if appState.depthField.isBimodal {
                        Circle()
                            .fill(Theme.milAmber)
                            .frame(width: 6, height: 6)
                    }
                    Text(appState.targetPriority.shortLabel)
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .foregroundColor(appState.targetPriority == .far ? Theme.milAmber : Theme.milGreenDim)
                    Text("TGT")
                        .font(.system(size: 10, weight: .medium, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }
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
                    if showPinchHint {
                        withAnimation(.easeOut(duration: 0.3)) {
                            showPinchHint = false
                        }
                    }
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

// MARK: - Pinch-to-Zoom Hint (tactical styling)

struct PinchZoomHint: View {
    let zoomFactor: CGFloat
    @State private var pulsePhase = false

    var body: some View {
        VStack(spacing: 6) {
            Spacer()

            VStack(spacing: 4) {
                Image(systemName: "arrow.up.left.and.arrow.down.right")
                    .font(.system(size: 14, weight: .medium))
                    .foregroundColor(Theme.milGreen.opacity(0.35))
                    .scaleEffect(pulsePhase ? 1.15 : 1.0)

                Text("MAG \(zoomText)")
                    .font(.system(size: 11, weight: .medium, design: .monospaced))
                    .foregroundColor(Theme.milGreen.opacity(0.35))
            }
            .padding(.horizontal, 10)
            .padding(.vertical, 6)
            .background(
                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                    .fill(Theme.panelBackground.opacity(0.6))
            )
            .offset(y: 60)

            Spacer()
            Spacer()
        }
        .allowsHitTesting(false)
        .onAppear {
            withAnimation(.easeInOut(duration: 1.5).repeatForever(autoreverses: true)) {
                pulsePhase = true
            }
        }
    }

    private var zoomText: String {
        if zoomFactor < 1.0 {
            return String(format: "%.1fx", zoomFactor)
        } else if zoomFactor == floor(zoomFactor) {
            return String(format: "%.0fx", zoomFactor)
        } else {
            return String(format: "%.1fx", zoomFactor)
        }
    }
}

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
