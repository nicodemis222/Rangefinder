//
//  RangeDisplayView.swift
//  Rangefinder
//
//  Large monospaced range readout with uncertainty band.
//

import SwiftUI

struct RangeDisplayView: View {
    let range: RangeOutput
    let displayUnit: UnitLength

    var body: some View {
        VStack(spacing: 4) {
            if range.isValid {
                // Main range number
                Text(formattedRange)
                    .font(Theme.rangeDisplayFont)
                    .foregroundColor(Theme.confidenceColor(for: range.confidence))
                    .shadow(color: .black, radius: 4)
                    .contentTransition(.numericText())
                    .animation(.easeInOut(duration: 0.15), value: formattedRange)

                // Unit label
                Text(unitLabel)
                    .font(.system(size: 14, weight: .medium, design: .monospaced))
                    .foregroundColor(.white.opacity(0.7))

                // Uncertainty band
                if range.uncertainty.converted(to: .meters).value > 0.5 {
                    Text("\u{00B1}\(formattedUncertainty)")
                        .font(.system(size: 13, weight: .regular, design: .monospaced))
                        .foregroundColor(.white.opacity(0.5))
                }
            } else {
                Text("---")
                    .font(Theme.rangeDisplayFont)
                    .foregroundColor(.white.opacity(0.3))

                Text(unitLabel)
                    .font(.system(size: 14, weight: .medium, design: .monospaced))
                    .foregroundColor(.white.opacity(0.3))
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 10)
        .background(
            RoundedRectangle(cornerRadius: 12)
                .fill(Color.black.opacity(0.6))
        )
    }

    // MARK: - Formatting

    private var formattedRange: String {
        let value = range.adjustedRange.converted(to: displayUnit).value
        if value < 10 {
            return String(format: "%.1f", value)
        } else {
            return String(format: "%.0f", value)
        }
    }

    private var formattedUncertainty: String {
        let value = range.uncertainty.converted(to: displayUnit).value
        if value < 1 {
            return String(format: "%.1f", value)
        } else {
            return String(format: "%.0f", value)
        }
    }

    private var unitLabel: String {
        displayUnit == .yards ? "YDS" : "M"
    }
}
