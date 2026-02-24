//
//  SRTMDownloadView.swift
//  Rangefinder
//
//  SRTM tile download UI — download elevation data by US region.
//  Military-styled to match SettingsView conventions.
//

import SwiftUI

struct SRTMDownloadView: View {
    @ObservedObject var regionManager: SRTMRegionManager
    @ObservedObject var locationManager: LocationManager

    var body: some View {
        Form {
            // Storage summary
            Section {
                HStack {
                    Text("INSTALLED TILES")
                        .font(.system(size: 13, weight: .medium, design: .monospaced))
                    Spacer()
                    Text("\(regionManager.installedTileCount)")
                        .font(.system(size: 13, weight: .bold, design: .monospaced))
                        .foregroundColor(regionManager.installedTileCount > 0
                            ? Theme.milGreen : Theme.milAmber)
                }

                HStack {
                    Text("STORAGE USED")
                        .font(.system(size: 13, weight: .medium, design: .monospaced))
                    Spacer()
                    Text(formatStorage(regionManager.totalStorageMB()))
                        .font(.system(size: 13, weight: .bold, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }
            } header: {
                Text("SRTM ELEVATION DATA")
                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                    .tracking(1)
            } footer: {
                Text("30m resolution terrain data for DEM ray-casting. Each tile covers 1\u{00B0} \u{00D7} 1\u{00B0} (~25MB). Download regions containing your area of operations.")
                    .font(.system(size: 11, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim)
            }

            // Current location suggestion
            if locationManager.hasValidFix,
               let currentRegion = regionManager.regionForCoordinate(locationManager.coordinate) {
                Section {
                    HStack {
                        Image(systemName: "location.fill")
                            .foregroundColor(Theme.milGreen)
                            .font(.system(size: 12))
                        Text("YOUR LOCATION: \(currentRegion.name)")
                            .font(.system(size: 13, weight: .bold, design: .monospaced))
                            .foregroundColor(Theme.milGreen)
                        Spacer()
                        if let state = regionManager.regionStates[currentRegion.id], state.isComplete {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(Theme.milGreen)
                                .font(.system(size: 14))
                        } else {
                            Text("RECOMMENDED")
                                .font(.system(size: 10, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milAmber)
                        }
                    }
                } header: {
                    Text("MY LOCATION")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                }
            }

            // Download progress (shown when active)
            if let activeId = regionManager.activeDownloadRegion,
               let state = regionManager.regionStates[activeId] {
                Section {
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text("DOWNLOADING")
                                .font(.system(size: 13, weight: .bold, design: .monospaced))
                                .foregroundColor(Theme.milAmber)
                            Spacer()
                            if let progress = regionManager.currentTileProgress {
                                Text(progress)
                                    .font(.system(size: 11, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                            }
                        }

                        ProgressView(value: state.progress)
                            .tint(Theme.milGreen)

                        HStack {
                            Text("\(state.downloadedTiles)/\(state.totalTiles) TILES")
                                .font(.system(size: 11, design: .monospaced))
                                .foregroundColor(Theme.milGreenDim)
                            Spacer()
                            Button("CANCEL") {
                                regionManager.cancelDownload()
                            }
                            .buttonStyle(MilPresetButton(isActive: false))
                            .foregroundColor(Theme.milRed)
                        }
                    }
                } header: {
                    Text("ACTIVE DOWNLOAD")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(1)
                }
            }

            // Region list
            Section {
                ForEach(SRTMRegionManager.regions) { region in
                    regionRow(region)
                }
            } header: {
                Text("US REGIONS")
                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                    .tracking(1)
            } footer: {
                Text("Tile counts are approximate. Ocean and void tiles are skipped automatically during download.")
                    .font(.system(size: 11, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim)
            }
        }
        .scrollContentBackground(.hidden)
        .background(Theme.darkBackground)
        .navigationTitle("TERRAIN DATA")
        .navigationBarTitleDisplayMode(.inline)
        .toolbarColorScheme(.dark, for: .navigationBar)
        .tint(Theme.milGreen)
    }

    // MARK: - Region Row

    @ViewBuilder
    private func regionRow(_ region: SRTMRegionManager.Region) -> some View {
        let state = regionManager.regionStates[region.id] ?? SRTMRegionManager.RegionState(
            totalTiles: region.tiles.count,
            downloadedTiles: 0
        )

        VStack(alignment: .leading, spacing: 6) {
            // Region name + states
            HStack {
                VStack(alignment: .leading, spacing: 2) {
                    Text(region.name)
                        .font(.system(size: 13, weight: .bold, design: .monospaced))
                        .foregroundColor(state.isComplete ? Theme.milGreen : .white)
                    Text(region.description)
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }

                Spacer()

                if state.isComplete {
                    Image(systemName: "checkmark.circle.fill")
                        .foregroundColor(Theme.milGreen)
                        .font(.system(size: 16))
                } else if state.isDownloading {
                    ProgressView()
                        .tint(Theme.milAmber)
                        .scaleEffect(0.8)
                }
            }

            // Stats row
            HStack {
                Text("\(state.downloadedTiles)/\(state.totalTiles) TILES")
                    .font(.system(size: 11, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim)

                if state.sizeOnDiskMB > 0 {
                    Text("•")
                        .foregroundColor(Theme.milGreenDim)
                    Text(formatStorage(state.sizeOnDiskMB))
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundColor(Theme.milGreenDim)
                }

                Spacer()

                // Action button
                if state.isDownloading {
                    // No action — cancel is in the progress section above
                } else if state.isComplete {
                    Button("DELETE") {
                        regionManager.deleteRegion(region)
                    }
                    .buttonStyle(MilPresetButton(isActive: false))
                    .foregroundColor(Theme.milRed)
                } else if regionManager.activeDownloadRegion == nil {
                    Button("DOWNLOAD") {
                        regionManager.downloadRegion(region)
                    }
                    .buttonStyle(MilPresetButton(isActive: false))
                }
            }

            // Progress bar for partially downloaded
            if state.downloadedTiles > 0 && !state.isComplete && !state.isDownloading {
                ProgressView(value: state.progress)
                    .tint(Theme.milAmber)
            }
        }
        .padding(.vertical, 4)
    }

    // MARK: - Helpers

    private func formatStorage(_ mb: Double) -> String {
        if mb < 1 {
            return "< 1 MB"
        } else if mb < 1024 {
            return String(format: "%.0f MB", mb)
        } else {
            return String(format: "%.1f GB", mb / 1024)
        }
    }
}
