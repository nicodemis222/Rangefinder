//
//  MapPiPView.swift
//  Rangefinder
//
//  Picture-in-picture satellite map showing the DEM ray-cast hit point.
//  Displays: user location (blue dot), hit coordinate (red pin),
//  line connecting them. Sized for bottom-corner overlay.
//

import SwiftUI
import MapKit
import CoreLocation

struct MapPiPView: View {
    let userCoordinate: CLLocationCoordinate2D
    let hitCoordinate: CLLocationCoordinate2D?
    let headingDegrees: Double

    @State private var mapPosition: MapCameraPosition = .automatic

    var body: some View {
        Map(position: $mapPosition) {
            // User location marker
            Annotation("", coordinate: userCoordinate) {
                Circle()
                    .fill(Color.blue)
                    .frame(width: 8, height: 8)
                    .overlay(
                        Circle()
                            .stroke(Color.white, lineWidth: 1.5)
                    )
            }

            // Hit point marker
            if let hit = hitCoordinate {
                Annotation("", coordinate: hit) {
                    Image(systemName: "mappin.circle.fill")
                        .font(.system(size: 14))
                        .foregroundColor(Theme.milRed)
                }

                // Line from user to hit
                MapPolyline(coordinates: [userCoordinate, hit])
                    .stroke(Theme.milAmber, lineWidth: 2)
            }
        }
        .mapStyle(.imagery(elevation: .realistic))
        .mapControlVisibility(.hidden)
        .frame(width: 140, height: 100)
        .clipShape(RoundedRectangle(cornerRadius: Theme.hudCornerRadius))
        .overlay(
            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                .stroke(Theme.hudBorder.opacity(0.6), lineWidth: Theme.borderWidth)
        )
        .shadow(color: .black.opacity(0.5), radius: 4)
        .onAppear {
            updateCamera()
        }
        .onChange(of: hitCoordinate?.latitude) { _, _ in
            updateCamera()
        }
    }

    // MARK: - Camera

    private func updateCamera() {
        let center: CLLocationCoordinate2D
        let span: Double

        if let hit = hitCoordinate {
            // Center between user and hit
            center = CLLocationCoordinate2D(
                latitude: (userCoordinate.latitude + hit.latitude) / 2,
                longitude: (userCoordinate.longitude + hit.longitude) / 2
            )
            // Span to fit both points with padding
            let latDiff = abs(userCoordinate.latitude - hit.latitude)
            let lonDiff = abs(userCoordinate.longitude - hit.longitude)
            span = max(latDiff, lonDiff) * 1.8 + 0.002 // Minimum span
        } else {
            center = userCoordinate
            span = 0.005
        }

        mapPosition = .region(MKCoordinateRegion(
            center: center,
            span: MKCoordinateSpan(latitudeDelta: span, longitudeDelta: span)
        ))
    }
}
