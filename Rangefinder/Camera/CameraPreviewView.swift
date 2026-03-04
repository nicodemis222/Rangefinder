//
//  CameraPreviewView.swift
//  Rangefinder
//
//  UIViewRepresentable wrapper for ARSCNView camera preview.
//  Fixed at 1x main lens for maximum ranging confidence.
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
        // Fixed 1x — no zoom transforms needed.
    }
}
