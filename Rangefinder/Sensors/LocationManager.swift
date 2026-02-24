//
//  LocationManager.swift
//  Rangefinder
//
//  GPS + barometric altimeter provider for DEM ray-casting.
//  Wraps CLLocationManager for lat/lon/altitude with accuracy,
//  and CMAltimeter for barometric altitude with sub-meter precision.
//
//  Barometric altitude (CMAbsoluteAltitudeData):
//  - Fuses GPS + barometric pressure for 1-5m vertical accuracy
//  - vs GPS-only vertical accuracy of 10-30m
//  - Directly improves DEM ray-casting observer altitude
//  - Falls back to GPS altitude when barometer unavailable
//

import Foundation
import CoreLocation
import CoreMotion
import os

@MainActor
class LocationManager: NSObject, ObservableObject {

    // MARK: - Published State

    @Published var coordinate: CLLocationCoordinate2D = CLLocationCoordinate2D(latitude: 0, longitude: 0)
    @Published var altitude: Double = 0           // meters above sea level (WGS84)
    @Published var horizontalAccuracy: Double = -1 // meters, -1 = invalid
    @Published var verticalAccuracy: Double = -1   // meters, -1 = invalid
    @Published var isAuthorized: Bool = false
    @Published var hasValidFix: Bool = false

    // MARK: - Barometric Altitude

    /// Best available altitude — prefers barometric when accuracy is better than GPS.
    @Published var bestAltitude: Double = 0

    /// Vertical accuracy of bestAltitude (meters).
    @Published var bestVerticalAccuracy: Double = -1

    /// Barometric altitude from CMAltimeter (meters above sea level).
    @Published var barometricAltitude: Double = 0

    /// Barometric vertical accuracy (meters). Typically 1-5m.
    @Published var barometricAccuracy: Double = -1

    /// Relative altitude change since CMAltimeter started (meters).
    /// Extremely precise (~0.1m) but no absolute reference.
    @Published var relativeAltitude: Double = 0

    /// Whether barometric altitude data is available and active.
    @Published var isBarometerActive: Bool = false

    /// Source of the current best altitude.
    @Published var altitudeSource: AltitudeSource = .gps

    enum AltitudeSource: String {
        case gps = "GPS"
        case barometric = "BARO"
        case fused = "FUSED"  // When both contribute
    }

    // MARK: - Private

    private let locationManager = CLLocationManager()
    private let altimeter = CMAltimeter()
    private var gpsAltitude: Double = 0
    private var gpsVerticalAccuracy: Double = -1

    // MARK: - Init

    override init() {
        super.init()
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.distanceFilter = 5.0  // Update every 5m of movement
        updateAuthorizationStatus(locationManager.authorizationStatus)
    }

    // MARK: - Lifecycle

    func requestAuthorization() {
        locationManager.requestWhenInUseAuthorization()
    }

    func startUpdates() {
        guard isAuthorized else {
            requestAuthorization()
            return
        }
        locationManager.startUpdatingLocation()
        startBarometricUpdates()
        Logger.sensors.info("Location updates started")
    }

    func stopUpdates() {
        locationManager.stopUpdatingLocation()
        stopBarometricUpdates()
        Logger.sensors.info("Location updates stopped")
    }

    // MARK: - Barometric Altimeter

    private func startBarometricUpdates() {
        // Relative altitude (extremely precise, ~0.1m resolution)
        if CMAltimeter.isRelativeAltitudeAvailable() {
            altimeter.startRelativeAltitudeUpdates(to: .main) { [weak self] data, error in
                guard let self = self, let data = data else {
                    if let error = error {
                        Logger.sensors.error("Relative altimeter error: \(error.localizedDescription)")
                    }
                    return
                }
                Task { @MainActor in
                    self.relativeAltitude = data.relativeAltitude.doubleValue
                }
            }
        }

        // Absolute altitude (fused GPS + barometer, 1-5m accuracy)
        if CMAltimeter.isAbsoluteAltitudeAvailable() {
            altimeter.startAbsoluteAltitudeUpdates(to: .main) { [weak self] data, error in
                guard let self = self, let data = data else {
                    if let error = error {
                        Logger.sensors.error("Absolute altimeter error: \(error.localizedDescription)")
                    }
                    return
                }
                Task { @MainActor in
                    self.barometricAltitude = data.altitude
                    self.barometricAccuracy = data.accuracy
                    self.isBarometerActive = true
                    self.updateBestAltitude()
                }
            }
            Logger.sensors.info("Barometric altimeter started (absolute + relative)")
        } else {
            Logger.sensors.warning("Absolute altitude not available on this device")
        }
    }

    private func stopBarometricUpdates() {
        altimeter.stopRelativeAltitudeUpdates()
        altimeter.stopAbsoluteAltitudeUpdates()
        isBarometerActive = false
        Logger.sensors.info("Barometric altimeter stopped")
    }

    /// Select the best altitude source based on current accuracy.
    private func updateBestAltitude() {
        let gpsValid = gpsVerticalAccuracy > 0
        let baroValid = isBarometerActive && barometricAccuracy > 0

        if baroValid && gpsValid {
            // Both available — use barometric if it's more accurate
            if barometricAccuracy < gpsVerticalAccuracy {
                bestAltitude = barometricAltitude
                bestVerticalAccuracy = barometricAccuracy
                altitudeSource = .barometric
            } else {
                bestAltitude = gpsAltitude
                bestVerticalAccuracy = gpsVerticalAccuracy
                altitudeSource = .gps
            }
        } else if baroValid {
            bestAltitude = barometricAltitude
            bestVerticalAccuracy = barometricAccuracy
            altitudeSource = .barometric
        } else if gpsValid {
            bestAltitude = gpsAltitude
            bestVerticalAccuracy = gpsVerticalAccuracy
            altitudeSource = .gps
        }

        // Also update the primary altitude property for backward compatibility
        altitude = bestAltitude
        verticalAccuracy = bestVerticalAccuracy
    }

    // MARK: - Authorization

    private func updateAuthorizationStatus(_ status: CLAuthorizationStatus) {
        switch status {
        case .authorizedWhenInUse, .authorizedAlways:
            isAuthorized = true
        default:
            isAuthorized = false
        }
    }
}

// MARK: - CLLocationManagerDelegate

extension LocationManager: CLLocationManagerDelegate {

    nonisolated func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        let status = manager.authorizationStatus
        Task { @MainActor in
            updateAuthorizationStatus(status)
            if isAuthorized {
                startUpdates()
            }
        }
    }

    nonisolated func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }
        Task { @MainActor in
            coordinate = location.coordinate
            horizontalAccuracy = location.horizontalAccuracy
            hasValidFix = location.horizontalAccuracy >= 0 && location.horizontalAccuracy < 100

            // Store GPS altitude separately for comparison with barometric
            gpsAltitude = location.altitude
            gpsVerticalAccuracy = location.verticalAccuracy

            // Update best altitude (picks barometric if more accurate)
            updateBestAltitude()

            if hasValidFix {
                let altSrc = altitudeSource.rawValue
                Logger.sensors.debug("GPS: \(String(format: "%.6f", location.coordinate.latitude)), \(String(format: "%.6f", location.coordinate.longitude)) alt=\(String(format: "%.1f", self.bestAltitude))m(\(altSrc)) ±H\(String(format: "%.1f", location.horizontalAccuracy))m ±V\(String(format: "%.1f", self.bestVerticalAccuracy))m")
            }
        }
    }

    nonisolated func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        Logger.sensors.error("Location error: \(error.localizedDescription)")
    }
}
