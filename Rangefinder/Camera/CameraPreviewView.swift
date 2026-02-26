//
//  CameraPreviewView.swift
//  Rangefinder
//
//  UIViewRepresentable wrapper for ARSCNView camera preview.
//
//  Visual zoom is implemented via CGAffineTransform scaling on the view itself.
//  ARKit ignores AVCaptureDevice.videoZoomFactor — its camera feed and depth
//  maps always cover the full wide-angle FOV. This is actually desirable:
//  the ranging engine operates on unzoomed data (where LiDAR calibration is
//  valid), while the user sees a zoomed preview that matches the reticle.
//
//  At 1x zoom the view is unscaled. At 5x the center 20% is magnified to
//  fill the screen. The reticle (FFPReticleView) independently scales its
//  mil-space geometry with the same zoom factor, keeping crosshair and
//  camera image aligned.
//

import SwiftUI
import ARKit

struct CameraPreviewView: UIViewRepresentable {
    let cameraManager: CameraManager
    /// Current zoom factor — drives the visual scale transform.
    var zoomFactor: CGFloat

    func makeUIView(context: Context) -> ARSCNView {
        let view = cameraManager.makeARView()
        view.contentMode = .scaleAspectFill
        view.clipsToBounds = true
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        // Apply visual zoom as a scale transform on the camera preview.
        // This crops into the center of the wide-angle feed, giving the
        // user a true zoomed view without affecting the underlying ARKit
        // pipeline (depth maps, intrinsics, neural model input all stay
        // at full wide-angle FOV).
        let clampedZoom = max(1.0, zoomFactor)
        let newTransform = CGAffineTransform(scaleX: clampedZoom, y: clampedZoom)

        // Only animate if the change is significant (avoids micro-jitter)
        if abs(uiView.transform.a - clampedZoom) > 0.01 {
            UIView.animate(withDuration: 0.08, delay: 0, options: [.curveEaseOut, .beginFromCurrentState]) {
                uiView.transform = newTransform
            }
        }
    }
}
