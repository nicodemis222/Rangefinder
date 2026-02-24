//
//  Measurement+Extensions.swift
//  Rangefinder
//
//  Range formatting utilities.
//

import Foundation

extension Measurement where UnitType == UnitLength {
    /// Format as yards with appropriate precision
    var formattedYards: String {
        let yards = converted(to: .yards).value
        if yards < 10 {
            return String(format: "%.1f", yards)
        } else {
            return String(format: "%.0f", yards)
        }
    }

    /// Format as meters with appropriate precision
    var formattedMeters: String {
        let meters = converted(to: .meters).value
        if meters < 10 {
            return String(format: "%.1f", meters)
        } else {
            return String(format: "%.0f", meters)
        }
    }

    /// Format with the given unit preference
    func formatted(unit: UnitLength) -> String {
        if unit == .yards {
            return formattedYards
        } else {
            return formattedMeters
        }
    }

    /// Unit abbreviation string
    static func unitAbbreviation(_ unit: UnitLength) -> String {
        switch unit {
        case .yards: return "YDS"
        case .meters: return "M"
        default: return unit.symbol.uppercased()
        }
    }
}

extension Double {
    /// Convert meters to yards
    var metersToYards: Double { self * 1.09361 }

    /// Convert yards to meters
    var yardsToMeters: Double { self * 0.9144 }
}
