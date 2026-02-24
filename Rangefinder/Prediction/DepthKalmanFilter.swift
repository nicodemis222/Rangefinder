//
//  DepthKalmanFilter.swift
//  Rangefinder
//
//  1D Kalman filter for depth estimation with velocity state.
//
//  State vector: [depth, velocity] where velocity = d(depth)/dt
//  This predicts depth forward between inference frames and smooths
//  transitions (especially LiDAR→neural handover at ~8-14 yards).
//
//  Key properties:
//  - Predicts depth BETWEEN inference frames (fills latency gaps)
//  - Smooths source transitions without lag (Kalman gain adapts)
//  - Tracks rate-of-change so moving targets are followed, not lagged
//  - Process noise adapts to motion state (stationary vs panning)
//
//  Based on classical Kalman filter formulation from Matthies et al.
//  (CMU, "Kalman Filter-based Algorithms for Estimating Depth from
//  Image Sequences") adapted for single-point real-time ranging.
//

import Foundation

struct DepthKalmanFilter {

    // MARK: - State

    /// Current state estimate: [depth (meters), velocity (m/s)]
    private var x: (depth: Double, velocity: Double) = (0, 0)

    /// Error covariance matrix (2x2, stored as 4 elements)
    /// [P00, P01]
    /// [P10, P11]
    private var P: (p00: Double, p01: Double, p10: Double, p11: Double) = (1, 0, 0, 1)

    /// Last update timestamp
    private var lastTimestamp: TimeInterval = 0

    /// Whether filter has been initialized
    private var isInitialized = false

    // MARK: - Configuration

    /// Process noise — how much we expect depth to change per second².
    /// Higher = trust measurements more, lower = trust prediction more.
    /// Adapts based on motion state.
    private var processNoiseBase: Double = 0.5

    /// Measurement noise — how noisy the fused depth reading is.
    /// Lower confidence → higher measurement noise.
    private var measurementNoiseBase: Double = 0.3

    /// Minimum dt to avoid numerical issues
    private let minDt: Double = 0.001

    /// Maximum dt before we reset (e.g. app was backgrounded)
    private let maxDt: Double = 2.0

    // MARK: - Predict

    /// Predict state forward by dt seconds.
    /// Call this between measurement updates to get predicted depth.
    ///
    /// - Parameter timestamp: Current time
    /// - Returns: Predicted depth in meters, or nil if not initialized
    mutating func predict(at timestamp: TimeInterval) -> Double? {
        guard isInitialized else { return nil }

        let dt = timestamp - lastTimestamp
        guard dt >= minDt else { return x.depth }

        if dt > maxDt {
            // Too long — just return current estimate, don't extrapolate
            return x.depth
        }

        // State prediction: x' = F * x
        // F = [1, dt]
        //     [0,  1]
        let predictedDepth = x.depth + x.velocity * dt
        // Velocity stays the same (constant velocity model)

        // Don't update internal state for pure prediction calls —
        // that happens in update(). Just return the predicted value.
        return max(0.1, predictedDepth)
    }

    /// Predict with IMU-informed velocity adjustment.
    /// Uses device forward acceleration to correct depth velocity.
    ///
    /// - Parameters:
    ///   - timestamp: Current time
    ///   - forwardAcceleration: Device acceleration along camera axis (m/s²),
    ///     positive = moving toward target (depth should decrease)
    /// - Returns: Predicted depth in meters
    mutating func predictWithIMU(
        at timestamp: TimeInterval,
        forwardAcceleration: Double
    ) -> Double? {
        guard isInitialized else { return nil }

        let dt = timestamp - lastTimestamp
        guard dt >= minDt, dt < maxDt else {
            return dt >= minDt ? x.depth : x.depth
        }

        // Depth change from device motion:
        // If device moves forward by dx, depth decreases by dx
        // v_correction = -forwardAcceleration * dt  (velocity change)
        // But clamp to prevent runaway extrapolation
        let velocityCorrection = -forwardAcceleration * dt
        let clampedCorrection = max(-2.0, min(2.0, velocityCorrection))

        let adjustedVelocity = x.velocity + clampedCorrection
        let predictedDepth = x.depth + adjustedVelocity * dt

        return max(0.1, predictedDepth)
    }

    // MARK: - Update

    /// Update the filter with a new depth measurement.
    ///
    /// - Parameters:
    ///   - measurement: Fused depth in meters
    ///   - confidence: Measurement confidence (0-1), affects measurement noise
    ///   - motionState: Device motion state, affects process noise
    ///   - timestamp: Measurement timestamp
    /// - Returns: Filtered depth estimate
    @discardableResult
    mutating func update(
        measurement: Double,
        confidence: Float,
        motionState: MotionState,
        timestamp: TimeInterval
    ) -> Double {
        guard measurement > 0.01 else { return x.depth }

        // First measurement: initialize directly
        guard isInitialized else {
            x = (depth: measurement, velocity: 0)
            P = (p00: 1.0, p01: 0, p10: 0, p11: 1.0)
            lastTimestamp = timestamp
            isInitialized = true
            return measurement
        }

        let dt = timestamp - lastTimestamp
        guard dt >= minDt else { return x.depth }

        // Reset if too much time elapsed
        if dt > maxDt {
            x = (depth: measurement, velocity: 0)
            P = (p00: 1.0, p01: 0, p10: 0, p11: 1.0)
            lastTimestamp = timestamp
            return measurement
        }

        // --- PREDICT STEP ---

        // State prediction: x' = F * x
        let predictedDepth = x.depth + x.velocity * dt
        let predictedVelocity = x.velocity

        // Process noise adapts to motion state
        let q = processNoise(motionState: motionState, dt: dt)

        // Covariance prediction: P' = F * P * F^T + Q
        // F = [1, dt; 0, 1]
        // Q = [q*dt^3/3, q*dt^2/2; q*dt^2/2, q*dt]
        let qDt3over3 = q * dt * dt * dt / 3.0
        let qDt2over2 = q * dt * dt / 2.0
        let qDt = q * dt

        let Pp00 = P.p00 + dt * P.p10 + dt * P.p01 + dt * dt * P.p11 + qDt3over3
        let Pp01 = P.p01 + dt * P.p11 + qDt2over2
        let Pp10 = P.p10 + dt * P.p11 + qDt2over2
        let Pp11 = P.p11 + qDt

        // --- UPDATE STEP ---

        // Measurement noise adapts to confidence
        let R = measurementNoise(confidence: confidence, depth: measurement)

        // Innovation (measurement residual)
        let y = measurement - predictedDepth

        // Innovation covariance: S = H * P' * H^T + R
        // H = [1, 0] (we only measure depth, not velocity)
        let S = Pp00 + R

        guard S > 1e-10 else {
            lastTimestamp = timestamp
            return x.depth
        }

        // Kalman gain: K = P' * H^T / S
        let K0 = Pp00 / S  // gain for depth
        let K1 = Pp10 / S  // gain for velocity

        // Updated state: x = x' + K * y
        let updatedDepth = predictedDepth + K0 * y
        let updatedVelocity = predictedVelocity + K1 * y

        // Updated covariance: P = (I - K*H) * P'
        let newP00 = (1.0 - K0) * Pp00
        let newP01 = (1.0 - K0) * Pp01
        let newP10 = Pp10 - K1 * Pp00
        let newP11 = Pp11 - K1 * Pp01

        // Store updated state
        x = (depth: max(0.1, updatedDepth), velocity: updatedVelocity)
        P = (p00: newP00, p01: newP01, p10: newP10, p11: newP11)
        lastTimestamp = timestamp

        return x.depth
    }

    // MARK: - Accessors

    /// Current depth estimate
    var currentDepth: Double { x.depth }

    /// Current depth velocity (m/s, positive = getting farther)
    var currentVelocity: Double { x.velocity }

    /// Estimation uncertainty (standard deviation in meters)
    var uncertainty: Double { sqrt(max(0, P.p00)) }

    /// Whether the filter is tracking
    var isTracking: Bool { isInitialized }

    // MARK: - Reset

    mutating func reset() {
        x = (0, 0)
        P = (1, 0, 0, 1)
        lastTimestamp = 0
        isInitialized = false
    }

    // MARK: - Adaptive Noise

    /// Process noise scales with motion state:
    /// - Stationary: very low noise (trust prediction strongly — user wants stability)
    /// - Panning: high noise (scene is changing, trust measurements)
    /// - Tracking: medium noise
    private func processNoise(motionState: MotionState, dt: Double) -> Double {
        switch motionState {
        case .stationary:
            return processNoiseBase * 0.1   // Was 0.3 — more aggressive stability
        case .tracking:
            return processNoiseBase * 1.0
        case .panning:
            return processNoiseBase * 5.0
        }
    }

    /// Measurement noise inversely proportional to confidence.
    /// Crucially, noise scales with distance² for inverse-depth models because
    /// the calibration transform (scale / disparity + shift) amplifies small
    /// disparity changes quadratically at long range.
    ///
    /// Example: at 200m, a 1% disparity change → ~4m depth swing.
    /// At 500m, the same 1% change → ~25m swing.
    private func measurementNoise(confidence: Float, depth: Double) -> Double {
        let confFactor = 1.0 / max(0.05, Double(confidence))

        // Distance-dependent noise: grows quadratically with range
        // This reflects the physics of inverse-depth calibration:
        //   σ_depth ∝ (depth²) × σ_disparity
        let distanceFactor: Double
        if depth < 5 {
            distanceFactor = 0.3           // LiDAR range — very reliable
        } else if depth < 15 {
            distanceFactor = 1.5           // Transition zone — more noise
        } else if depth < 50 {
            distanceFactor = 1.0           // Neural sweet spot
        } else if depth < 100 {
            distanceFactor = 2.0           // Neural mid-range
        } else if depth < 200 {
            // Quadratic scaling kicks in: 100m→4, 200m→16
            let normalized = depth / 50.0  // 2.0 at 100m, 4.0 at 200m
            distanceFactor = normalized * normalized
        } else {
            // Beyond 200m: heavy noise — trust Kalman prediction more
            let normalized = depth / 50.0  // 4.0 at 200m, 10.0 at 500m
            distanceFactor = min(100.0, normalized * normalized)
        }

        return measurementNoiseBase * confFactor * distanceFactor
    }
}
