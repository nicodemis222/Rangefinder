//
//  PerformanceMonitor.swift
//  Rangefinder
//
//  FPS, latency, and thermal state tracking.
//

import Foundation
import os

@MainActor
class PerformanceMonitor: ObservableObject {
    @Published var currentFPS: Double = 0
    @Published var thermalState: ProcessInfo.ThermalState = .nominal

    private var frameTimestamps: [CFAbsoluteTime] = []
    private let fpsWindow: Int = 30
    private nonisolated(unsafe) var thermalObserver: NSObjectProtocol?

    init() {
        setupThermalMonitoring()
    }

    deinit {
        if let observer = thermalObserver {
            NotificationCenter.default.removeObserver(observer)
        }
    }

    func recordFrame() {
        let now = CFAbsoluteTimeGetCurrent()
        frameTimestamps.append(now)

        // Keep window
        if frameTimestamps.count > fpsWindow {
            frameTimestamps.removeFirst()
        }

        // Calculate FPS
        if frameTimestamps.count >= 2 {
            let elapsed = frameTimestamps.last! - frameTimestamps.first!
            guard elapsed > 0 else { return }
            currentFPS = Double(frameTimestamps.count - 1) / elapsed
        }
    }

    private func setupThermalMonitoring() {
        thermalState = ProcessInfo.processInfo.thermalState
        thermalObserver = NotificationCenter.default.addObserver(
            forName: ProcessInfo.thermalStateDidChangeNotification,
            object: nil,
            queue: .main
        ) { [weak self] _ in
            Task { @MainActor in
                self?.thermalState = ProcessInfo.processInfo.thermalState
            }
        }
    }

    var thermalDescription: String {
        switch thermalState {
        case .nominal: return "Normal"
        case .fair: return "Warm"
        case .serious: return "Hot"
        case .critical: return "Critical"
        @unknown default: return "Unknown"
        }
    }

    var shouldThrottle: Bool {
        thermalState == .serious || thermalState == .critical
    }
}
