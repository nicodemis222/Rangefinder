//
//  CameraPreviewView.swift
//  Rangefinder
//
//  UIViewRepresentable wrapper for ARSCNView camera preview.
//
//  Zoom is handled entirely by hardware — CameraManager sets videoZoomFactor
//  on ARKit's configurableCaptureDeviceForPrimaryCamera, which triggers real
//  lens switching (0.5x ultrawide → 1x main → 5x telephoto) and ISP-accelerated
//  digital zoom. The ARSCNView renders the hardware-zoomed feed at native quality.
//
//  LiDAR depth maps always cover the full wide-angle FOV regardless of zoom.
//  The ranging engine uses FrameData.zoomFactor for coordinate mapping.
//

import SwiftUI
import ARKit

struct CameraPreviewView: UIViewRepresentable {
    let cameraManager: CameraManager

    func makeUIView(context: Context) -> ARSCNView {
        let view = cameraManager.makeARView()
        view.contentMode = .scaleAspectFill
        view.clipsToBounds = true
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        // Hardware zoom is applied directly to the capture device by CameraManager.
        // No view transforms needed — ARSCNView renders the zoomed feed natively.
    }
}
