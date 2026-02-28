//
//  CameraManager.swift
//  Rangefinder
//
//  ARKit-based camera session providing frames, LiDAR depth, and intrinsics.
//
//  Uses ARWorldTrackingConfiguration with sceneDepth for unified
//  camera + LiDAR pipeline. Publishes frame data via Combine.
//
//  Zoom: Uses ARWorldTrackingConfiguration.configurableCaptureDeviceForPrimaryCamera
//  to get the actual AVCaptureDevice ARKit uses internally. Setting videoZoomFactor
//  on this device triggers real hardware lens switching (0.5x ultrawide → 1x main →
//  5x telephoto) and ISP-accelerated digital zoom. The ARSCNView renders the
//  hardware-zoomed camera feed at native quality — no pixelating view transforms.
//  LiDAR depth maps always cover the full wide-angle FOV regardless of zoom.
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
    /// Current optical+digital zoom factor (1.0 = no zoom).
    /// Used by depth map sampling to convert crosshair screen position
    /// to the correct coordinate in the wide-angle LiDAR depth map.
    let zoomFactor: CGFloat
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

    /// ARKit's configurable capture device — the ACTUAL device ARKit uses.
    /// Obtained via ARWorldTrackingConfiguration.configurableCaptureDeviceForPrimaryCamera.
    /// Setting videoZoomFactor on this device controls hardware lens switching
    /// and the ISP's ML-enhanced digital zoom. Marked nonisolated(unsafe) because
    /// we read from the nonisolated ARSessionDelegate callback.
    nonisolated(unsafe) private var captureDevice: AVCaptureDevice?

    /// Current logical zoom factor set by ZoomController.
    /// Tracks the user's intended zoom for the ranging pipeline (depth map
    /// coordinate mapping, confidence adjustments at extreme zoom).
    nonisolated(unsafe) var currentZoomFactor: CGFloat = 1.0

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

        // Get ARKit's actual configurable capture device for hardware zoom.
        // This is the REAL device ARKit uses — setting videoZoomFactor on it
        // controls lens switching (0.5x/1x/5x) and ISP digital zoom.
        // The ARSCNView renders the zoomed feed at native quality.
        if let device = ARWorldTrackingConfiguration.configurableCaptureDeviceForPrimaryCamera {
            captureDevice = device
            Logger.camera.info("ARKit capture device: \(device.localizedName), zoom \(device.minAvailableVideoZoomFactor)–\(device.maxAvailableVideoZoomFactor)")
        } else {
            // Fallback for non-configurable devices
            if let device = AVCaptureDevice.default(.builtInTripleCamera, for: .video, position: .back)
                ?? AVCaptureDevice.default(.builtInDualWideCamera, for: .video, position: .back)
                ?? AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) {
                captureDevice = device
                Logger.camera.warning("Using fallback capture device: \(device.localizedName)")
            }
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
        // Store the logical zoom factor for the ranging pipeline
        // (depth map coordinate mapping, confidence adjustments).
        currentZoomFactor = factor

        // Set hardware zoom on ARKit's configurable capture device.
        // This triggers real lens switching (0.5x ultrawide → 1x main → 5x telephoto)
        // and ISP-accelerated digital zoom. The ARSCNView renders the zoomed feed
        // at native quality — no pixelating view transforms needed.
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
            cameraPose: frame.camera.transform,
            zoomFactor: currentZoomFactor
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
