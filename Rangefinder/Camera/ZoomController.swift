//
//  ZoomController.swift
//  Rangefinder
//
//  Manages pinch-to-zoom and lens switching for iPhone 17 Pro Max.
//

import Foundation
import AVFoundation
import Combine
import os

enum CameraLens: String, CaseIterable {
    case ultrawide = "0.5x"
    case main = "1x"
    case telephoto = "5x"
}

@MainActor
class ZoomController: ObservableObject {
    // MARK: - Published

    @Published var zoomFactor: CGFloat = 1.0
    @Published var activeLens: CameraLens = .main

    // MARK: - Configuration

    private let minZoom: CGFloat = 0.5
    private let maxZoom: CGFloat = 25.0

    // iPhone 17 Pro Max lens breakpoints
    private let ultrawideMax: CGFloat = 0.99
    private let mainMax: CGFloat = 4.99
    private let telephoteStart: CGFloat = 5.0

    /// Base horizontal FOV at 1x zoom (main camera, degrees)
    private let baseFOVDegrees: Double = AppConfiguration.mainCameraHFOV

    // MARK: - Pinch State

    private var pinchBaseZoom: CGFloat = 1.0

    // MARK: - Computed Properties

    /// Current horizontal FOV in degrees accounting for zoom
    var currentFOVDegrees: Double {
        baseFOVDegrees / Double(zoomFactor)
    }

    /// Current horizontal FOV in milliradians
    var currentFOVMils: Double {
        currentFOVDegrees * 17.453292519943
    }

    /// Effective focal length in mm (approximate)
    var effectiveFocalLengthMM: Double {
        24.0 * Double(zoomFactor) // 24mm is main camera equiv
    }

    // MARK: - Zoom Control

    func handlePinchBegan() {
        pinchBaseZoom = zoomFactor
    }

    func handlePinchChanged(scale: CGFloat) {
        let newZoom = pinchBaseZoom * scale
        setZoom(newZoom)
    }

    func setZoom(_ factor: CGFloat) {
        let clamped = min(maxZoom, max(minZoom, factor))
        zoomFactor = clamped

        // Update active lens indicator
        if clamped < 1.0 {
            activeLens = .ultrawide
        } else if clamped < telephoteStart {
            activeLens = .main
        } else {
            activeLens = .telephoto
        }
    }

    /// Snaps to a preset lens zoom level
    func snapToLens(_ lens: CameraLens) {
        switch lens {
        case .ultrawide:
            setZoom(0.5)
        case .main:
            setZoom(1.0)
        case .telephoto:
            setZoom(5.0)
        }
    }

    /// Pixels per mil for the current zoom at given screen width
    func pixelsPerMil(screenWidth: CGFloat) -> CGFloat {
        let fovMils = currentFOVMils
        guard fovMils > 0 else { return 1.0 }
        return screenWidth / CGFloat(fovMils)
    }
}
