//
//  SettingsView.swift
//  Rangefinder
//
//  CONFIGURATION — system settings panel.
//  Units, reticle, ballistics, sensitivity. Military styling.
//

import SwiftUI

struct SettingsView: View {
    @Binding var displayUnit: UnitLength
    @Binding var reticleConfig: ReticleConfiguration
    @Binding var cameraHeight: Float
    @Binding var targetPriority: TargetPriority
    @Binding var stadiametricTargetSize: Double
    @ObservedObject var ballisticsSolver: BallisticsSolver
    @ObservedObject var locationManager: LocationManager
    @ObservedObject var regionManager: SRTMRegionManager
    @Environment(\.dismiss) private var dismiss
    @AppStorage("hasCompletedTutorial") private var hasCompletedTutorial = true

    var body: some View {
        NavigationStack {
            Form {
                // Units
                Section {
                    HStack(spacing: 12) {
                        Button("YDS") { displayUnit = .yards }
                            .buttonStyle(MilToggleButton(isSelected: displayUnit == .yards))
                        Button("M") { displayUnit = .meters }
                            .buttonStyle(MilToggleButton(isSelected: displayUnit == .meters))
                        Spacer()
                    }
                } header: {
                    Text("DISPLAY UNITS")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                }

                // Target Priority (Near/Far target mode)
                Section {
                    HStack(spacing: 12) {
                        Button("NEAR (1ST)") { targetPriority = .near }
                            .buttonStyle(MilToggleButton(isSelected: targetPriority == .near))
                        Button("FAR (LST)") { targetPriority = .far }
                            .buttonStyle(MilToggleButton(isSelected: targetPriority == .far))
                        Spacer()
                    }
                } header: {
                    Text("TARGET MODE")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                } footer: {
                    Text(targetPriority == .far
                        ? "LAST TARGET — ranges through foreground occluders (rocks, brush) to the distant terrain. Uses DEM cross-validation."
                        : "FIRST TARGET — ranges the closest object at the crosshair. Use for close targets with no occluders.")
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }

                // Ballistics
                Section {
                    Toggle("ENABLE BALLISTICS", isOn: $ballisticsSolver.isEnabled)

                    if ballisticsSolver.isEnabled {
                        // Caliber picker
                        Menu {
                            ForEach(BallisticsSolver.Caliber.allCases) { caliber in
                                Button {
                                    ballisticsSolver.selectedCaliber = caliber
                                } label: {
                                    HStack {
                                        Text(caliber.rawValue)
                                        if caliber == ballisticsSolver.selectedCaliber {
                                            Image(systemName: "checkmark")
                                        }
                                    }
                                }
                            }
                        } label: {
                            HStack {
                                Text("CALIBER")
                                    .font(.system(size: 13, weight: .medium, design: .monospaced))
                                Spacer()
                                Text(ballisticsSolver.selectedCaliber.rawValue)
                                    .font(.system(size: 13, weight: .bold, design: .monospaced))
                                    .foregroundColor(Theme.milAmber)
                                Image(systemName: "chevron.right")
                                    .font(.system(size: 10))
                                    .foregroundColor(Theme.milGreenDim)
                            }
                        }

                        // Zero distance picker
                        Menu {
                            ForEach(zeroOptions, id: \.self) { distance in
                                Button {
                                    ballisticsSolver.zeroDistance = Double(distance)
                                } label: {
                                    HStack {
                                        Text("\(distance) \(displayUnit == .meters ? "M" : "YDS")")
                                        if distance == Int(ballisticsSolver.zeroDistance) {
                                            Image(systemName: "checkmark")
                                        }
                                    }
                                }
                            }
                        } label: {
                            HStack {
                                Text("ZERO DISTANCE")
                                    .font(.system(size: 13, weight: .medium, design: .monospaced))
                                Spacer()
                                Text("\(Int(ballisticsSolver.zeroDistance)) \(displayUnit == .meters ? "M" : "YDS")")
                                    .font(.system(size: 13, weight: .bold, design: .monospaced))
                                    .foregroundColor(Theme.milAmber)
                                Image(systemName: "chevron.right")
                                    .font(.system(size: 10))
                                    .foregroundColor(Theme.milGreenDim)
                            }
                        }

                        // Caliber reference data
                        VStack(alignment: .leading, spacing: 4) {
                            HStack {
                                Text("BC (G1)")
                                    .font(.system(size: 11, weight: .medium, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                                Spacer()
                                Text(String(format: "%.3f", ballisticsSolver.selectedCaliber.ballisticCoefficient))
                                    .font(.system(size: 11, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                            }
                            HStack {
                                Text("MUZZLE VEL")
                                    .font(.system(size: 11, weight: .medium, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                                Spacer()
                                Text("\(Int(ballisticsSolver.selectedCaliber.muzzleVelocity)) FPS")
                                    .font(.system(size: 11, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                            }
                            HStack {
                                Text("BULLET WT")
                                    .font(.system(size: 11, weight: .medium, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                                Spacer()
                                Text("\(Int(ballisticsSolver.selectedCaliber.bulletWeight)) GR")
                                    .font(.system(size: 11, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                            }
                        }
                        .padding(.vertical, 4)
                    }
                } header: {
                    Text("BALLISTICS")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                }

                // Geometric Ranging
                Section {
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text("CAMERA HEIGHT")
                                .font(.system(size: 13, weight: .medium, design: .monospaced))
                            Spacer()
                            Text(String(format: "%.1f M", cameraHeight))
                                .font(.system(size: 13, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milAmber)
                        }
                        Slider(
                            value: Binding(
                                get: { cameraHeight },
                                set: { cameraHeight = $0 }
                            ),
                            in: AppConfiguration.minCameraHeight...AppConfiguration.maxCameraHeight,
                            step: 0.1
                        )
                        .tint(Theme.milGreen)

                        HStack(spacing: 12) {
                            Text("PRESETS")
                                .font(.system(size: 11, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milGreenDim)
                            Spacer()
                            Button("PRONE") { cameraHeight = 0.3 }
                                .buttonStyle(MilPresetButton(isActive: false))
                            Button("TRIPOD") { cameraHeight = 1.2 }
                                .buttonStyle(MilPresetButton(isActive: false))
                            Button("STAND") { cameraHeight = 1.5 }
                                .buttonStyle(MilPresetButton(isActive: false))
                            Button("VEHICLE") { cameraHeight = 2.0 }
                                .buttonStyle(MilPresetButton(isActive: false))
                        }
                    }
                } header: {
                    Text("GEOMETRIC RANGING")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                } footer: {
                    Text("Height of camera above ground. Used for IMU-based ground-plane distance estimation at 10-300m.")
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }

                // Stadiametric Ranging
                Section {
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text("TARGET HEIGHT")
                                .font(.system(size: 13, weight: .medium, design: .monospaced))
                            Spacer()
                            Text(String(format: "%.1f M", stadiametricTargetSize))
                                .font(.system(size: 13, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milAmber)
                        }
                        Slider(
                            value: $stadiametricTargetSize,
                            in: 0.3...12.0,
                            step: 0.1
                        )
                        .tint(Theme.milGreen)

                        // Presets from AppConfiguration
                        VStack(alignment: .leading, spacing: 6) {
                            Text("PRESETS")
                                .font(.system(size: 11, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milGreenDim)

                            // Use wrapping layout to avoid overflow on small screens
                            let presets = AppConfiguration.stadiametricTargetPresets.prefix(4)
                            HStack(spacing: 8) {
                                ForEach(Array(presets), id: \.label) { preset in
                                    Button {
                                        stadiametricTargetSize = preset.heightMeters
                                    } label: {
                                        Text(preset.label)
                                            .frame(maxWidth: .infinity)
                                    }
                                    .buttonStyle(MilPresetButton(isActive: abs(stadiametricTargetSize - preset.heightMeters) < 0.05))
                                }
                            }
                        }
                    }
                } header: {
                    Text("STADIAMETRIC RANGING")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                } footer: {
                    Text("Set the known height of the target. Use the STADIA overlay to bracket the target top/bottom, then range is computed via pinhole formula.")
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }

                // Terrain Ranging (DEM)
                Section {
                    // GPS Status
                    HStack {
                        Text("GPS")
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                        Spacer()
                        if !locationManager.isAuthorized {
                            Text("NOT AUTHORIZED")
                                .font(.system(size: 13, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milRed)
                        } else if locationManager.hasValidFix {
                            HStack(spacing: 6) {
                                Circle()
                                    .fill(gpsAccuracyColor)
                                    .frame(width: 8, height: 8)
                                Text(String(format: "\u{00B1}%.0fM", locationManager.horizontalAccuracy))
                                    .font(.system(size: 13, weight: .bold, design: .monospaced))
                                    .foregroundColor(gpsAccuracyColor)
                            }
                        } else {
                            Text("ACQUIRING...")
                                .font(.system(size: 13, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milAmber)
                        }
                    }

                    // Altitude (shows barometric source when active)
                    if locationManager.hasValidFix {
                        HStack {
                            Text("ALTITUDE")
                                .font(.system(size: 13, weight: .medium, design: .monospaced))
                            Spacer()
                            HStack(spacing: 4) {
                                Text(String(format: "%.0f M", locationManager.bestAltitude))
                                    .font(.system(size: 13, weight: .bold, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                                Text("±\(String(format: "%.0f", max(0, locationManager.bestVerticalAccuracy)))M")
                                    .font(.system(size: 11, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                                Text(locationManager.altitudeSource.rawValue)
                                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                                    .foregroundColor(locationManager.isBarometerActive ? Theme.milGreen : Theme.milGreenDim)
                            }
                        }
                    }

                    // DEM Status
                    HStack {
                        Text("DEM STATUS")
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                        Spacer()
                        Text(demStatusText)
                            .font(.system(size: 13, weight: .bold, design: .monospaced))
                            .foregroundColor(demStatusColor)
                    }

                    // Terrain Data Download
                    NavigationLink {
                        SRTMDownloadView(
                            regionManager: regionManager,
                            locationManager: locationManager
                        )
                    } label: {
                        HStack {
                            Text("TERRAIN DATA")
                                .font(.system(size: 13, weight: .medium, design: .monospaced))
                            Spacer()
                            Text(regionManager.installedTileCount > 0
                                ? "\(regionManager.installedTileCount) TILES"
                                : "NONE")
                                .font(.system(size: 13, weight: .bold, design: .monospaced))
                                .foregroundColor(regionManager.installedTileCount > 0
                                    ? Theme.milGreen : Theme.milAmber)
                            Image(systemName: "chevron.right")
                                .font(.system(size: 10))
                                .foregroundColor(Theme.milGreenDim)
                        }
                    }
                } header: {
                    Text("TERRAIN RANGING")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                } footer: {
                    Text("DEM ray-casting uses GPS + IMU heading/pitch to intersect SRTM terrain data. Effective 50-2000m on terrain. Requires GPS fix and local tile data.")
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }

                // Reticle
                Section {
                    // Reticle style picker
                    VStack(alignment: .leading, spacing: 8) {
                        Text("RETICLE STYLE")
                            .font(.system(size: 11, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim)

                        HStack(spacing: 8) {
                            ForEach(ReticleStyle.allCases) { style in
                                Button(style.rawValue) {
                                    reticleConfig.style = style
                                }
                                .buttonStyle(MilToggleButton(isSelected: reticleConfig.style == style))
                            }
                            Spacer()
                        }

                        Text(reticleConfig.style.description)
                            .font(.system(size: 10, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim.opacity(0.7))
                    }

                    ColorPicker("RETICLE COLOR", selection: $reticleConfig.color)

                    // Reticle color presets
                    HStack(spacing: 12) {
                        Text("PRESETS")
                            .font(.system(size: 11, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim)

                        Spacer()

                        Button("NVG") { reticleConfig.color = Theme.milGreen }
                            .buttonStyle(MilPresetButton(isActive: false))
                        Button("DAY") { reticleConfig.color = Theme.reticleAmber }
                            .buttonStyle(MilPresetButton(isActive: false))
                        Button("RED") { reticleConfig.color = Theme.reticleRed }
                            .buttonStyle(MilPresetButton(isActive: false))
                    }

                    // Style-specific options
                    if reticleConfig.style == .milDot {
                        Toggle("FILLED DOTS", isOn: $reticleConfig.dotFilled)
                        Toggle("HASH MARKS", isOn: $reticleConfig.showHalfMilHashes)
                        Toggle("MIL LABELS", isOn: $reticleConfig.showMilLabels)
                    }

                    Toggle("OUTLINE", isOn: $reticleConfig.showOutline)

                    VStack(alignment: .leading) {
                        Text("LINE WIDTH")
                            .font(.system(size: 11, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim)
                        Slider(value: $reticleConfig.fineLineWidth, in: 0.5...3.0, step: 0.25)
                            .tint(Theme.milGreen)
                    }

                    VStack(alignment: .leading) {
                        Text("OPACITY")
                            .font(.system(size: 11, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim)
                        Slider(value: $reticleConfig.opacity, in: 0.3...1.0, step: 0.05)
                            .tint(Theme.milGreen)
                    }
                } header: {
                    Text("RETICLE")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                }

                // Help
                Section {
                    Button("SHOW BRIEFING") {
                        hasCompletedTutorial = false
                        dismiss()
                    }
                    .font(.system(size: 14, weight: .medium, design: .monospaced))
                    .foregroundColor(Theme.milGreen)
                } header: {
                    Text("SYSTEM")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                }

                // About
                Section {
                    LabeledContent {
                        Text("1.0")
                            .font(.system(size: 13, design: .monospaced))
                    } label: {
                        Text("VERSION")
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                    }
                    LabeledContent {
                        Text("DAV2 + GEO + DEM + OBJ")
                            .font(.system(size: 13, design: .monospaced))
                    } label: {
                        Text("DEPTH MODELS")
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                    }
                    LabeledContent {
                        Text("YOLO + VN + SCENE")
                            .font(.system(size: 13, design: .monospaced))
                    } label: {
                        Text("DETECTION")
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                    }
                    LabeledContent {
                        Text(locationManager.isBarometerActive ? "ACTIVE" : "N/A")
                            .font(.system(size: 13, design: .monospaced))
                            .foregroundColor(locationManager.isBarometerActive ? Theme.milGreen : Theme.milGreenDim)
                    } label: {
                        Text("BAROMETER")
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                    }
                } header: {
                    Text("SYSTEM INFO")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                }
            }
            .scrollContentBackground(.hidden)
            .background(Theme.darkBackground)
            .navigationTitle("CONFIGURATION")
            .navigationBarTitleDisplayMode(.inline)
            .toolbarColorScheme(.dark, for: .navigationBar)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button("DONE") { dismiss() }
                        .font(.system(size: 14, weight: .bold, design: .monospaced))
                        .foregroundColor(Theme.milGreen)
                }
            }
            .tint(Theme.milGreen)
        }
        .preferredColorScheme(.dark)
    }

    private var zeroOptions: [Int] {
        stride(from: 25, through: 500, by: 25).map { $0 }
    }

    // MARK: - GPS / DEM Computed

    private var gpsAccuracyColor: Color {
        let acc = locationManager.horizontalAccuracy
        if acc < 5.0 { return Theme.milGreen }
        if acc < 15.0 { return Theme.milAmber }
        return Theme.milRed
    }

    private var demStatusText: String {
        if !locationManager.isAuthorized { return "NO GPS" }
        if !locationManager.hasValidFix { return "WAITING" }
        if locationManager.horizontalAccuracy > 100 { return "LOW ACCURACY" }
        return "READY"
    }

    private var demStatusColor: Color {
        if !locationManager.isAuthorized { return Theme.milRed }
        if !locationManager.hasValidFix { return Theme.milAmber }
        if locationManager.horizontalAccuracy > 100 { return Theme.milAmber }
        return Theme.milGreen
    }
}

// MARK: - Mil Toggle Button Style

/// Toggle button with clear selected/unselected states.
/// Selected: bright green text + green border + subtle green fill.
/// Unselected: dim text + dim border + dark fill.
struct MilToggleButton: ButtonStyle {
    let isSelected: Bool

    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.system(size: 11, weight: .bold, design: .monospaced))
            .foregroundColor(isSelected ? .black : Theme.milGreenDim)
            .padding(.horizontal, 10)
            .padding(.vertical, 6)
            .background(
                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                    .fill(isSelected ? Theme.milGreen : Theme.panelBackground)
                    .overlay(
                        RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                            .stroke(isSelected ? Theme.milGreen : Theme.hudBorder.opacity(0.4),
                                    lineWidth: isSelected ? 1.5 : Theme.borderWidth)
                    )
            )
            .opacity(configuration.isPressed ? 0.6 : 1.0)
    }
}

// MARK: - Mil Preset Button Style

struct MilPresetButton: ButtonStyle {
    let isActive: Bool

    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.system(size: 11, weight: .bold, design: .monospaced))
            .foregroundColor(isActive ? .black : Theme.milGreen)
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(
                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                    .fill(isActive ? Theme.milGreen : Theme.panelBackground)
                    .overlay(
                        RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                            .stroke(isActive ? Theme.milGreen : Theme.hudBorder.opacity(0.6),
                                    lineWidth: Theme.borderWidth)
                    )
            )
            .opacity(configuration.isPressed ? 0.6 : 1.0)
    }
}
