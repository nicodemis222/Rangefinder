//
//  Logger.swift
//  Rangefinder
//
//  Centralized logging with os.Logger categories.
//

import Foundation
import os

extension Logger {
    private static let subsystem = "com.mildotrangefinder"

    static let camera = Logger(subsystem: subsystem, category: "Camera")
    static let depth = Logger(subsystem: subsystem, category: "Depth")
    static let neural = Logger(subsystem: subsystem, category: "Neural")
    static let ranging = Logger(subsystem: subsystem, category: "Ranging")
    static let fusion = Logger(subsystem: subsystem, category: "Fusion")
    static let calibration = Logger(subsystem: subsystem, category: "Calibration")
    static let reticle = Logger(subsystem: subsystem, category: "Reticle")
    static let sensors = Logger(subsystem: subsystem, category: "Sensors")
    static let ui = Logger(subsystem: subsystem, category: "UI")
    static let detection = Logger(subsystem: subsystem, category: "Detection")
    static let performance = Logger(subsystem: subsystem, category: "Performance")
    static let terrain = Logger(subsystem: subsystem, category: "Terrain")
}
