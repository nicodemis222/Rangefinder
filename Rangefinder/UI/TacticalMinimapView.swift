//
//  TacticalMinimapView.swift
//  Rangefinder
//
//  Compact tactical minimap — bottom-left of camera display.
//  Shows user position, heading direction, DEM tile coverage status,
//  and ray-cast hit point. Always visible when GPS is available.
//
//  UX design:
//  ┌──────────────────┐
//  │ ● DEM 5/9        │  ← tile coverage badge
//  │                   │
//  │       ╱╲          │  ← heading cone (50° wedge)
//  │      ● ──── ●    │  ← user dot → hit point
//  │                   │
//  └──────────────────┘
//
//  - Position: bottom-left, always-on with GPS fix
//  - Size: 330×240 pt (large, readable at arm's length)
//  - Dynamic zoom: tight (~1.7km) when idle, expands to show hit
//  - Tap → opens SRTM tile download sheet
//  - DEM badge: green "DEM 9/9", amber "DEM 3/9", red "NO DEM"
//

import SwiftUI
import MapKit
import CoreLocation

struct TacticalMinimapView: View {
    let userCoordinate: CLLocationCoordinate2D
    let headingDegrees: Double
    let hitCoordinate: CLLocationCoordinate2D?

    @State private var mapPosition: MapCameraPosition = .automatic

    var body: some View {
        ZStack(alignment: .topLeading) {
            // Satellite base map (no user interaction — display only)
            Map(position: $mapPosition, interactionModes: []) {
                // User position with heading cone
                Annotation("", coordinate: userCoordinate) {
                    UserHeadingMarker(headingDegrees: headingDegrees)
                }

                // DEM hit point (when ranging terrain)
                if let hit = hitCoordinate {
                    Annotation("", coordinate: hit) {
                        Circle()
                            .fill(Theme.milRed)
                            .frame(width: 10, height: 10)
                    }

                    // Range line from user to hit
                    MapPolyline(coordinates: [userCoordinate, hit])
                        .stroke(Theme.milAmber.opacity(0.6), lineWidth: 2.5)
                }
            }
            .mapStyle(.imagery(elevation: .flat))
            .mapControlVisibility(.hidden)

            // DEM tile coverage badge (top-left corner of minimap)
            demCoverageBadge
                .padding(6)

            // Download prompt — centered overlay when tiles are missing
            if needsTileDownload {
                downloadPrompt
            }
        }
        .frame(width: 330, height: 240)
        .clipShape(RoundedRectangle(cornerRadius: Theme.hudCornerRadius))
        .overlay(
            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                .stroke(Theme.hudBorder.opacity(0.6), lineWidth: Theme.borderWidth)
        )
        .overlay(
            // Corner bracket marks — tactical HUD motif
            GeometryReader { geo in
                Theme.cornerBrackets(in: geo.frame(in: .local), length: 16, inset: 0)
                    .stroke(Theme.milGreen.opacity(0.4), lineWidth: 1.5)
            }
        )
        .shadow(color: .black.opacity(0.5), radius: 6)
        .contentShape(Rectangle())
        .onAppear { updateCamera() }
        .onChange(of: userCoordinate.latitude) { _, _ in updateCamera() }
        .onChange(of: hitCoordinate?.latitude) { _, _ in updateCamera() }
    }

    // MARK: - DEM Coverage Badge

    private var demCoverageBadge: some View {
        let coverage = computeTileCoverage()
        return HStack(spacing: 5) {
            Circle()
                .fill(coverage.color)
                .frame(width: 10, height: 10)
            Text(coverage.label)
                .font(.system(size: 13, weight: .bold, design: .monospaced))
                .foregroundColor(coverage.color)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(Theme.panelBackground.opacity(0.85))
        .cornerRadius(4)
    }

    // MARK: - Download Prompt

    /// True when fewer than 9 tiles are installed — terrain ranging is degraded.
    private var needsTileDownload: Bool {
        let coverage = computeTileCoverage()
        return coverage.color != Theme.milGreen
    }

    /// Centered tap-to-download banner shown over the map when DEM tiles are missing.
    private var downloadPrompt: some View {
        VStack(spacing: 0) {
            Spacer()
            HStack {
                Spacer()
                HStack(spacing: 8) {
                    Image(systemName: "square.and.arrow.down")
                        .font(.system(size: 14, weight: .bold))
                    Text("TAP TO DOWNLOAD TERRAIN")
                        .font(.system(size: 12, weight: .bold, design: .monospaced))
                }
                .foregroundColor(Theme.milAmber)
                .padding(.horizontal, 14)
                .padding(.vertical, 8)
                .background(
                    RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                        .fill(Color.black.opacity(0.75))
                        .overlay(
                            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                                .stroke(Theme.milAmber.opacity(0.6), lineWidth: 1)
                        )
                )
                Spacer()
            }
            .padding(.bottom, 12)
        }
    }

    // MARK: - Tile Coverage Computation

    /// Check the 3×3 grid of SRTM tiles surrounding the user's position.
    /// Returns a coverage summary for the badge.
    private func computeTileCoverage() -> (label: String, color: Color) {
        let baseLat = Int(floor(userCoordinate.latitude))
        let baseLon = Int(floor(userCoordinate.longitude))

        var installed = 0
        let total = 9

        for dLat in -1...1 {
            for dLon in -1...1 {
                let key = srtmTileKey(lat: baseLat + dLat, lon: baseLon + dLon)
                if isTileOnDisk(key) { installed += 1 }
            }
        }

        if installed == total {
            return ("DEM \(installed)/\(total)", Theme.milGreen)
        } else if installed > 0 {
            return ("DEM \(installed)/\(total)", Theme.milAmber)
        } else {
            return ("NO DEM", Theme.milRed)
        }
    }

    private func srtmTileKey(lat: Int, lon: Int) -> String {
        let latPrefix = lat >= 0 ? "N" : "S"
        let lonPrefix = lon >= 0 ? "E" : "W"
        return String(format: "%@%02d%@%03d", latPrefix, abs(lat), lonPrefix, abs(lon))
    }

    private func isTileOnDisk(_ key: String) -> Bool {
        guard let documentsURL = FileManager.default.urls(
            for: .documentDirectory, in: .userDomainMask
        ).first else { return false }
        let fileURL = documentsURL.appendingPathComponent("SRTM/\(key).hgt")
        return FileManager.default.fileExists(atPath: fileURL.path)
    }

    // MARK: - Map Camera

    private func updateCamera() {
        if let hit = hitCoordinate {
            // Show both user and hit point with padding
            let latDiff = abs(userCoordinate.latitude - hit.latitude)
            let lonDiff = abs(userCoordinate.longitude - hit.longitude)
            let span = max(latDiff, lonDiff) * 2.5 + 0.003
            let center = CLLocationCoordinate2D(
                latitude: (userCoordinate.latitude + hit.latitude) / 2,
                longitude: (userCoordinate.longitude + hit.longitude) / 2
            )
            mapPosition = .region(MKCoordinateRegion(
                center: center,
                span: MKCoordinateSpan(latitudeDelta: span, longitudeDelta: span)
            ))
        } else {
            // No hit — show surrounding area (~1.7km)
            mapPosition = .region(MKCoordinateRegion(
                center: userCoordinate,
                span: MKCoordinateSpan(latitudeDelta: 0.015, longitudeDelta: 0.015)
            ))
        }
    }
}

// MARK: - User Heading Marker

/// Green dot with directional cone showing camera heading.
/// Drawn as a Map Annotation at the user's GPS position.
private struct UserHeadingMarker: View {
    let headingDegrees: Double

    var body: some View {
        ZStack {
            // Heading cone — 50° FOV wedge
            HeadingConeShape(headingDegrees: headingDegrees, coneAngle: 50)
                .fill(Theme.milGreen.opacity(0.25))
                .frame(width: 60, height: 60)

            HeadingConeShape(headingDegrees: headingDegrees, coneAngle: 50)
                .stroke(Theme.milGreen.opacity(0.5), lineWidth: 1)
                .frame(width: 60, height: 60)

            // Center dot
            Circle()
                .fill(Theme.milGreen)
                .frame(width: 12, height: 12)
                .overlay(
                    Circle()
                        .stroke(Color.white, lineWidth: 1.5)
                )
        }
    }
}

// MARK: - Heading Cone Shape

/// Pie-slice shape representing the camera's field-of-view direction.
/// Heading 0° = north (up), clockwise.
private struct HeadingConeShape: Shape {
    let headingDegrees: Double
    let coneAngle: Double

    func path(in rect: CGRect) -> Path {
        var path = Path()
        let center = CGPoint(x: rect.midX, y: rect.midY)
        let radius = min(rect.width, rect.height) / 2

        // Convert from compass heading (0°=N, CW) to SwiftUI angle (0°=3 o'clock, CW)
        // Subtract 90° to rotate from east-reference to north-reference
        let startAngle = Angle(degrees: headingDegrees - coneAngle / 2 - 90)
        let endAngle = Angle(degrees: headingDegrees + coneAngle / 2 - 90)

        path.move(to: center)
        path.addArc(
            center: center,
            radius: radius,
            startAngle: startAngle,
            endAngle: endAngle,
            clockwise: false
        )
        path.closeSubpath()

        return path
    }
}
