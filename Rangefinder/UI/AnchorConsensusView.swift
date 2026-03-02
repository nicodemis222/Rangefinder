//
//  AnchorConsensusView.swift
//  Rangefinder
//
//  When spatial coherence is .centerSuspect, shows a secondary range readout
//  from the anchor median — an alternative estimate from high-confidence
//  peripheral readings.
//

import SwiftUI

struct AnchorConsensusView: View {
    let anchorMedianMeters: Double
    let displayUnit: UnitLength

    var body: some View {
        HStack(spacing: 4) {
            Text("ANCHOR AVG")
                .font(.system(size: 8, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milAmber.opacity(0.7))

            Text(formattedDistance)
                .font(.system(size: 12, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milAmber)

            Text(unitLabel)
                .font(.system(size: 8, weight: .medium, design: .monospaced))
                .foregroundColor(Theme.milAmber.opacity(0.6))
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 4)
        .background(
            Capsule()
                .fill(Theme.panelBackground.opacity(0.8))
                .overlay(
                    Capsule()
                        .stroke(Theme.milAmber.opacity(0.5), lineWidth: 1.0)
                )
        )
        .shadow(color: .black.opacity(0.5), radius: 3, x: 0, y: 1)
    }

    private var formattedDistance: String {
        let value = Measurement(
            value: anchorMedianMeters,
            unit: UnitLength.meters
        ).converted(to: displayUnit).value
        if value < 10 {
            return String(format: "%.1f", value)
        } else {
            return String(format: "%.0f", value)
        }
    }

    private var unitLabel: String {
        displayUnit == .yards ? "YDS" : "M"
    }
}
