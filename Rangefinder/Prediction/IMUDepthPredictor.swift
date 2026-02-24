//
//  IMUDepthPredictor.swift
//  Rangefinder
//
//  Extrapolates depth between neural inference frames using IMU data.
//
//  Core insight from Visual-Inertial Odometry research: between depth
//  inference frames (which take 50-150ms), the phone may have moved.
//  IMU accelerometer data at 60Hz tells us HOW it moved, so we can
//  predict how depth should change.
//
//  If the user walks 0.5m forward during a 100ms inference gap,
//  depth should decrease by ~0.5m. Without IMU compensation, the
//  depth reading "jumps" when the next frame arrives. With it,
//  the reading smoothly tracks the motion.
//
//  Uses ARKit's camera transform delta when available (more accurate
//  than raw accelerometer integration which drifts), falling back
//  to accelerometer-based estimation.
//

import Foundation
import simd

struct IMUDepthPredictor {

    // MARK: - State

    /// Last known camera position from ARKit (world coordinates)
    private var lastCameraPosition: SIMD3<Float>?

    /// Last camera forward direction (unit vector, world coordinates)
    private var lastCameraForward: SIMD3<Float>?

    /// Timestamp of last camera pose update
    private var lastPoseTimestamp: TimeInterval = 0

    /// Accumulated forward displacement since last depth measurement
    private var accumulatedForwardDisplacement: Double = 0

    /// Velocity estimate from recent camera motion (m/s along camera axis)
    private var forwardVelocity: Double = 0

    /// Whether we have a valid pose history
    private var isInitialized: Bool { lastCameraPosition != nil }

    // MARK: - Configuration

    /// Maximum displacement to accumulate (prevents runaway during long inference gaps)
    private let maxDisplacement: Double = 3.0

    /// Velocity smoothing factor (EMA alpha)
    private let velocityAlpha: Double = 0.3

    // MARK: - Update Camera Pose

    /// Feed a new camera pose from ARKit.
    /// Call this every ARFrame (30-60 Hz) — it's cheap, just stores the transform.
    ///
    /// - Parameters:
    ///   - cameraTransform: 4x4 camera-to-world transform from ARFrame
    ///   - timestamp: Frame timestamp
    mutating func updateCameraPose(
        cameraTransform: simd_float4x4,
        timestamp: TimeInterval
    ) {
        // Extract position (translation column of the 4x4 matrix)
        let position = SIMD3<Float>(
            cameraTransform.columns.3.x,
            cameraTransform.columns.3.y,
            cameraTransform.columns.3.z
        )

        // Extract forward direction (negative Z axis of camera in world space)
        // In ARKit, camera looks along -Z
        let forward = -SIMD3<Float>(
            cameraTransform.columns.2.x,
            cameraTransform.columns.2.y,
            cameraTransform.columns.2.z
        )

        if let lastPos = lastCameraPosition, lastCameraForward != nil {
            let dt = timestamp - lastPoseTimestamp
            guard dt > 0.001, dt < 1.0 else {
                lastCameraPosition = position
                lastCameraForward = forward
                lastPoseTimestamp = timestamp
                return
            }

            // Compute displacement along the CURRENT camera forward axis
            let displacement3D = position - lastPos
            let forwardDisplacement = Double(simd_dot(displacement3D, forward))

            // Accumulate for depth prediction
            accumulatedForwardDisplacement += forwardDisplacement

            // Clamp accumulated displacement
            accumulatedForwardDisplacement = max(-maxDisplacement,
                min(maxDisplacement, accumulatedForwardDisplacement))

            // Update velocity estimate (smoothed)
            let instantVelocity = forwardDisplacement / dt
            forwardVelocity = velocityAlpha * instantVelocity + (1 - velocityAlpha) * forwardVelocity
        }

        lastCameraPosition = position
        lastCameraForward = forward
        lastPoseTimestamp = timestamp
    }

    // MARK: - Depth Prediction

    /// Get the predicted depth adjustment since the last depth measurement.
    ///
    /// If the camera moved forward by 0.5m since the last depth reading,
    /// this returns -0.5 (depth decreased because we got closer).
    ///
    /// - Returns: Depth adjustment in meters (negative = got closer)
    var depthAdjustment: Double {
        // Camera moving forward → depth decreases
        return -accumulatedForwardDisplacement
    }

    /// Current forward velocity estimate (m/s).
    /// Positive = camera moving toward target.
    var currentForwardVelocity: Double {
        return forwardVelocity
    }

    /// Apply prediction to a base depth reading.
    ///
    /// - Parameter baseDepth: The last measured depth in meters
    /// - Returns: Predicted current depth accounting for camera motion
    func predictDepth(from baseDepth: Double) -> Double {
        let predicted = baseDepth + depthAdjustment
        return max(0.1, predicted)
    }

    // MARK: - Measurement Consumed

    /// Call when a new depth measurement arrives.
    /// Resets the accumulated displacement so prediction starts fresh.
    mutating func onNewMeasurement() {
        accumulatedForwardDisplacement = 0
    }

    // MARK: - Reset

    mutating func reset() {
        lastCameraPosition = nil
        lastCameraForward = nil
        lastPoseTimestamp = 0
        accumulatedForwardDisplacement = 0
        forwardVelocity = 0
    }
}
