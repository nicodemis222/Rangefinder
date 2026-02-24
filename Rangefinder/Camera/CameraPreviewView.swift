//
//  CameraPreviewView.swift
//  Rangefinder
//
//  UIViewRepresentable wrapper for ARSCNView camera preview.
//

import SwiftUI
import ARKit

struct CameraPreviewView: UIViewRepresentable {
    let cameraManager: CameraManager

    func makeUIView(context: Context) -> ARSCNView {
        let view = cameraManager.makeARView()
        view.contentMode = .scaleAspectFill
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        // No dynamic updates needed
    }
}
