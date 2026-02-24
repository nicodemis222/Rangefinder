//
//  KnownObjectDatabase.swift
//  Rangefinder
//
//  Database of real-world object sizes for pinhole-model ranging.
//  Data from ANSUR II, EPA, MUTCD, USFWS, and building codes.
//

import Foundation

// MARK: - Object Type

enum ObjectType: String, CaseIterable {
    case human
    case vehicle
    case wildlife
    case structure
    case sign
}

// MARK: - Known Object

struct KnownObject {
    let objectType: ObjectType
    let label: String
    let displayName: String
    let sizeMeters: Double       // Primary dimension
    let isHeight: Bool           // true = height, false = width
    let sizeVariability: Float   // 0-1 (std dev as fraction of mean)
    let reliabilityWeight: Float // 0-1
    let expectedAspectRatio: Float // W/H

    var adjustedReliability: Float {
        reliabilityWeight * (1.0 - sizeVariability)
    }
}

// MARK: - Database

final class KnownObjectDatabase: @unchecked Sendable {
    static let shared = KnownObjectDatabase()

    private var objects: [String: KnownObject] = [:]
    private var fuzzyCache: [String: KnownObject?] = [:]
    private let lock = NSLock()

    private init() {
        loadDatabase()
    }

    func lookup(_ label: String) -> KnownObject? {
        // Exact match first
        if let obj = objects[label.lowercased()] {
            return obj
        }
        // Fuzzy match
        lock.lock()
        defer { lock.unlock() }
        if let cached = fuzzyCache[label.lowercased()] {
            return cached
        }
        let result = fuzzyMatch(label)
        fuzzyCache[label.lowercased()] = result
        return result
    }

    private func fuzzyMatch(_ label: String) -> KnownObject? {
        let lower = label.lowercased()
        let aliases: [String: String] = [
            "pedestrian": "person",
            "man": "person",
            "woman": "person",
            "child": "person",
            "car": "sedan",
            "automobile": "sedan",
            "truck": "pickup_truck",
            "pickup": "pickup_truck",
            "suv": "suv",
            "motorbike": "motorcycle",
            "bike": "bicycle",
            "deer": "deer",
            "elk": "elk",
            "dog": "dog",
            "cat": "cat",
            "stop sign": "stop_sign",
            "traffic sign": "speed_limit_sign",
        ]
        if let mapped = aliases[lower], let obj = objects[mapped] {
            return obj
        }
        // Partial match
        for (key, obj) in objects {
            if lower.contains(key) || key.contains(lower) {
                return obj
            }
        }
        return nil
    }

    // MARK: - Data

    private func loadDatabase() {
        let entries: [KnownObject] = [
            // Humans
            KnownObject(objectType: .human, label: "person", displayName: "Person", sizeMeters: 1.74, isHeight: true, sizeVariability: 0.10, reliabilityWeight: 0.85, expectedAspectRatio: 0.35),

            // Vehicles
            KnownObject(objectType: .vehicle, label: "sedan", displayName: "Sedan", sizeMeters: 1.48, isHeight: true, sizeVariability: 0.08, reliabilityWeight: 0.80, expectedAspectRatio: 2.8),
            KnownObject(objectType: .vehicle, label: "suv", displayName: "SUV", sizeMeters: 1.78, isHeight: true, sizeVariability: 0.10, reliabilityWeight: 0.78, expectedAspectRatio: 2.2),
            KnownObject(objectType: .vehicle, label: "pickup_truck", displayName: "Pickup Truck", sizeMeters: 1.85, isHeight: true, sizeVariability: 0.10, reliabilityWeight: 0.78, expectedAspectRatio: 2.5),
            KnownObject(objectType: .vehicle, label: "bus", displayName: "Bus", sizeMeters: 3.2, isHeight: true, sizeVariability: 0.05, reliabilityWeight: 0.85, expectedAspectRatio: 3.0),
            KnownObject(objectType: .vehicle, label: "motorcycle", displayName: "Motorcycle", sizeMeters: 1.15, isHeight: true, sizeVariability: 0.12, reliabilityWeight: 0.65, expectedAspectRatio: 1.4),
            KnownObject(objectType: .vehicle, label: "bicycle", displayName: "Bicycle", sizeMeters: 1.05, isHeight: true, sizeVariability: 0.08, reliabilityWeight: 0.60, expectedAspectRatio: 1.6),

            // Wildlife
            KnownObject(objectType: .wildlife, label: "deer", displayName: "White-tail Deer", sizeMeters: 1.0, isHeight: true, sizeVariability: 0.15, reliabilityWeight: 0.65, expectedAspectRatio: 1.5),
            KnownObject(objectType: .wildlife, label: "elk", displayName: "Elk", sizeMeters: 1.5, isHeight: true, sizeVariability: 0.12, reliabilityWeight: 0.70, expectedAspectRatio: 1.6),
            KnownObject(objectType: .wildlife, label: "bear", displayName: "Black Bear", sizeMeters: 0.9, isHeight: true, sizeVariability: 0.20, reliabilityWeight: 0.55, expectedAspectRatio: 1.2),
            KnownObject(objectType: .wildlife, label: "dog", displayName: "Dog", sizeMeters: 0.55, isHeight: true, sizeVariability: 0.30, reliabilityWeight: 0.40, expectedAspectRatio: 1.3),
            KnownObject(objectType: .wildlife, label: "cat", displayName: "Cat", sizeMeters: 0.30, isHeight: true, sizeVariability: 0.15, reliabilityWeight: 0.35, expectedAspectRatio: 1.4),

            // Signs (MUTCD standard dimensions)
            KnownObject(objectType: .sign, label: "stop_sign", displayName: "Stop Sign", sizeMeters: 0.762, isHeight: false, sizeVariability: 0.02, reliabilityWeight: 0.95, expectedAspectRatio: 1.0),
            KnownObject(objectType: .sign, label: "speed_limit_sign", displayName: "Speed Limit Sign", sizeMeters: 0.610, isHeight: false, sizeVariability: 0.02, reliabilityWeight: 0.93, expectedAspectRatio: 0.75),
            KnownObject(objectType: .sign, label: "yield_sign", displayName: "Yield Sign", sizeMeters: 0.914, isHeight: true, sizeVariability: 0.02, reliabilityWeight: 0.93, expectedAspectRatio: 1.15),

            // Structures
            KnownObject(objectType: .structure, label: "door", displayName: "Standard Door", sizeMeters: 2.032, isHeight: true, sizeVariability: 0.03, reliabilityWeight: 0.90, expectedAspectRatio: 0.42),
            KnownObject(objectType: .structure, label: "fire_hydrant", displayName: "Fire Hydrant", sizeMeters: 0.61, isHeight: true, sizeVariability: 0.05, reliabilityWeight: 0.88, expectedAspectRatio: 0.4),
            KnownObject(objectType: .structure, label: "utility_pole", displayName: "Utility Pole", sizeMeters: 10.7, isHeight: true, sizeVariability: 0.10, reliabilityWeight: 0.70, expectedAspectRatio: 0.02),
        ]

        for entry in entries {
            objects[entry.label] = entry
        }
    }
}
