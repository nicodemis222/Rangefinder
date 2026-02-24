//
//  InclinationManager.swift
//  Rangefinder
//
//  CMMotionManager-based pitch, angular velocity, and heading tracking.
//  Provides device inclination for range correction, angular velocity
//  for motion-aware smoothing, and true-north heading for DEM ray-casting.
//
//  Pitch convention (portrait, phone held upright aiming at horizon):
//    0°  = aiming level at the horizon
//    +°  = tilted up (aiming above horizon)
//    -°  = tilted down (aiming below horizon)
//
//  CMMotionManager reports attitude.pitch ≈ π/2 when phone is vertical,
//  so we subtract π/2 to get intuitive "aim angle from horizontal."
//  ARKit eulerAngles.x ≈ 0 when camera faces horizon, which is already correct.
//
//  Heading: Uses xTrueNorthZVertical reference frame which fuses gyro,
//  accelerometer, and magnetometer for compass-quality heading (1-5° accuracy).
//  attitude.yaw gives rotation around gravity axis relative to true north.
//

import Foundation
import CoreMotion
import os

@MainActor
class InclinationManager: ObservableObject {

    // MARK: - Published State

    @Published var pitchDegrees: Double = 0
    @Published var pitchRadians: Double = 0
    @Published var headingDegrees: Double = 0  // True-north heading (0-360, 0=N, 90=E)
    @Published var angularVelocity: Double = 0  // rad/s (magnitude of rotation rate)
    @Published var motionState: MotionState = .stationary

    // MARK: - Private

    private let motionManager = CMMotionManager()
    private let updateInterval: TimeInterval = 1.0 / 60.0  // 60 Hz
    private var usingCMMotion = false

    // Angular velocity smoothing
    private var angularVelocityHistory: [Double] = []
    private let angularVelocityWindow = 5

    // MARK: - Start/Stop

    func startUpdates() {
        guard motionManager.isDeviceMotionAvailable else {
            Logger.sensors.warning("Device motion not available")
            return
        }

        motionManager.deviceMotionUpdateInterval = updateInterval
        // xTrueNorthZVertical: fuses gyro + accel + magnetometer for true-north heading.
        // Pitch and roll are identical to xArbitraryZVertical — only yaw changes
        // to be referenced to magnetic/true north instead of arbitrary startup orientation.
        motionManager.startDeviceMotionUpdates(
            using: .xTrueNorthZVertical,
            to: .main
        ) { [weak self] motion, error in
            guard let self = self, let motion = motion else {
                if let error = error {
                    Logger.sensors.error("Motion update error: \(error.localizedDescription)")
                }
                return
            }

            self.usingCMMotion = true

            // CMMotionManager attitude.pitch:
            //   Phone flat on table face-up: pitch ≈ 0
            //   Phone vertical (portrait, aiming at horizon): pitch ≈ π/2
            //   Phone tilted up past vertical: pitch > π/2
            //
            // We want: 0° = aiming level, +° = tilted up, -° = tilted down
            // So subtract π/2: vertical aiming at horizon → 0
            let rawPitch = motion.attitude.pitch
            let aimPitch = rawPitch - .pi / 2.0

            self.pitchRadians = aimPitch
            self.pitchDegrees = aimPitch * 180.0 / .pi

            // True-north heading from yaw:
            // attitude.yaw with xTrueNorthZVertical: 0 = north, increases clockwise
            // But CMMotionManager yaw is counterclockwise-positive, so negate and normalize
            let rawYaw = motion.attitude.yaw  // radians, counterclockwise from north
            var heading = -rawYaw * 180.0 / .pi  // Convert to degrees, clockwise
            if heading < 0 { heading += 360.0 }
            if heading >= 360.0 { heading -= 360.0 }
            self.headingDegrees = heading

            // Angular velocity (magnitude of rotation rate)
            let rotRate = motion.rotationRate
            let magnitude = sqrt(rotRate.x * rotRate.x + rotRate.y * rotRate.y + rotRate.z * rotRate.z)

            // Smooth angular velocity
            self.angularVelocityHistory.append(magnitude)
            if self.angularVelocityHistory.count > self.angularVelocityWindow {
                self.angularVelocityHistory.removeFirst()
            }
            self.angularVelocity = self.angularVelocityHistory.reduce(0, +) / Double(self.angularVelocityHistory.count)

            // Classify motion state
            self.motionState = MotionState(angularVelocity: self.angularVelocity)
        }

        Logger.sensors.info("Inclination manager started (xTrueNorthZVertical)")
    }

    func stopUpdates() {
        motionManager.stopDeviceMotionUpdates()
        usingCMMotion = false
        Logger.sensors.info("Inclination manager stopped")
    }

    // MARK: - AR Frame Fallback

    /// Update pitch from AR frame euler angles (backup when CMMotionManager is not active).
    /// ARKit eulerAngles.x ≈ 0 when camera faces horizon, so no offset needed.
    /// Note: heading is NOT available from AR frame fallback.
    func updateFromARFrame(pitchRadians: Float) {
        guard !usingCMMotion else { return }
        self.pitchRadians = Double(pitchRadians)
        self.pitchDegrees = Double(pitchRadians) * 180.0 / .pi
    }
}
