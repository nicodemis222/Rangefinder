//
//  CameraManager.swift
//  Rangefinder
//
//  ARKit-based camera session providing frames, LiDAR depth, and intrinsics.
//
//  Uses ARWorldTrackingConfiguration with sceneDepth for unified
//  camera + LiDAR pipeline. Publishes frame data via Combine.
//

import ARKit
import AVFoundation
import Combine
import os

/// Bundle of per-frame data from the camera + sensors.
/// Marked @unchecked Sendable because CVPixelBuffer is thread-safe for reading.
struct FrameData: @unchecked Sendable {
    let capturedImage: CVPixelBuffer
    let intrinsics: CameraIntrinsics
    let lidarDepthMap: CVPixelBuffer?
    let lidarConfidenceMap: CVPixelBuffer?
    let pitchRadians: Float
    let timestamp: TimeInterval
    let cameraPose: simd_float4x4
}

@MainActor
class CameraManager: NSObject, ObservableObject {
    // MARK: - Published

    @Published var isSessionRunning = false
    @Published var sessionError: String?

    // MARK: - Frame Publisher (nonisolated for ARSessionDelegate callback performance)

    nonisolated(unsafe) let frameSubject = PassthroughSubject<FrameData, Never>()

    // MARK: - ARKit Session

    let arSession = ARSession()
    private(set) var arView: ARSCNView?

    // MARK: - Zoom

    private var captureDevice: AVCaptureDevice?

    // MARK: - Setup

    func setupSession() {
        arSession.delegate = self

        let config = ARWorldTrackingConfiguration()

        // Enable LiDAR depth
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics = [.smoothedSceneDepth]
            Logger.camera.info("Scene depth enabled (smoothed)")
        } else if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics = [.sceneDepth]
            Logger.camera.info("Scene depth enabled (raw)")
        } else {
            Logger.camera.warning("Scene depth not supported on this device")
        }

        // Prefer highest resolution video format
        if let format = ARWorldTrackingConfiguration.supportedVideoFormats
            .filter({ $0.captureDevicePosition == .back })
            .sorted(by: { $0.imageResolution.width > $1.imageResolution.width })
            .first {
            config.videoFormat = format
            Logger.camera.info("Video format: \(Int(format.imageResolution.width))x\(Int(format.imageResolution.height))")
        }

        arSession.run(config, options: [.resetTracking, .removeExistingAnchors])
        isSessionRunning = true

        // Get the capture device for zoom control
        if let device = AVCaptureDevice.default(.builtInTripleCamera, for: .video, position: .back)
            ?? AVCaptureDevice.default(.builtInDualWideCamera, for: .video, position: .back)
            ?? AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) {
            captureDevice = device
            Logger.camera.info("Capture device: \(device.localizedName)")
        }
    }

    func pauseSession() {
        arSession.pause()
        isSessionRunning = false
    }

    func resumeSession() {
        guard let config = arSession.configuration else { return }
        arSession.run(config)
        isSessionRunning = true
    }

    // MARK: - Zoom Control

    func setZoomFactor(_ factor: CGFloat) {
        guard let device = captureDevice else { return }
        do {
            try device.lockForConfiguration()
            let clamped = min(max(factor, CGFloat(device.minAvailableVideoZoomFactor)),
                             CGFloat(device.maxAvailableVideoZoomFactor))
            device.videoZoomFactor = clamped
            device.unlockForConfiguration()
        } catch {
            Logger.camera.error("Zoom error: \(error.localizedDescription)")
        }
    }

    /// Creates and returns an ARSCNView for embedding in SwiftUI
    func makeARView() -> ARSCNView {
        let view = ARSCNView()
        view.session = arSession
        view.automaticallyUpdatesLighting = false
        view.rendersCameraGrain = false
        view.rendersMotionBlur = false
        // Hide all SceneKit rendering - we just want the camera feed
        view.scene = SCNScene()
        self.arView = view
        return view
    }
}

// MARK: - ARSessionDelegate

extension CameraManager: ARSessionDelegate {
    nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let capturedImage = frame.capturedImage
        let width = CVPixelBufferGetWidth(capturedImage)
        let height = CVPixelBufferGetHeight(capturedImage)

        let intrinsics = CameraIntrinsics(
            from: frame.camera.intrinsics,
            width: width,
            height: height
        )

        // Extract LiDAR depth (smoothed preferred)
        let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth
        let lidarDepth = depthData?.depthMap
        let lidarConfidence = depthData?.confidenceMap

        // Device pitch from AR camera euler angles
        let pitch = frame.camera.eulerAngles.x

        let frameData = FrameData(
            capturedImage: capturedImage,
            intrinsics: intrinsics,
            lidarDepthMap: lidarDepth,
            lidarConfidenceMap: lidarConfidence,
            pitchRadians: pitch,
            timestamp: frame.timestamp,
            cameraPose: frame.camera.transform
        )

        frameSubject.send(frameData)
    }

    nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
        Logger.camera.error("AR session failed: \(error.localizedDescription)")
        Task { @MainActor in
            self.sessionError = error.localizedDescription
            self.isSessionRunning = false
        }
    }

    nonisolated func sessionWasInterrupted(_ session: ARSession) {
        Logger.camera.warning("AR session interrupted")
        Task { @MainActor in
            self.isSessionRunning = false
        }
    }

    nonisolated func sessionInterruptionEnded(_ session: ARSession) {
        Logger.camera.info("AR session interruption ended")
        Task { @MainActor in
            self.isSessionRunning = true
        }
    }
}
