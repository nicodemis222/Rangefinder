//
//  BackgroundRangeChip.swift
//  Rangefinder
//
//  Compact chip showing the background (alternate hypothesis) range.
//  When semantic selection picks DEM as primary, the background might
//  show neural depth for foreground context — and vice versa.
//
//  Displayed in the bottom HUD zone, above the source blend bar.
//

import SwiftUI

struct BackgroundRangeChip: View {
    let backgroundRange: RangeOutput
    let displayUnit: UnitLength

    var body: some View {
        if backgroundRange.isValid {
            HStack(spacing: 4) {
                // "BG" prefix to distinguish from primary
                Text("BG")
                    .font(.system(size: 9, weight: .bold, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim.opacity(0.5))

                // Source icon
                Image(systemName: backgroundRange.primarySource.icon)
                    .font(.system(size: 9, weight: .medium))
                    .foregroundColor(Theme.sourceColor(backgroundRange.primarySource).opacity(0.7))

                // Source label
                Text(backgroundRange.primarySource.shortName)
                    .font(.system(size: 9, weight: .bold, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim)

                // Range value
                Text(formattedRange)
                    .font(.system(size: 12, weight: .medium, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim.opacity(0.8))

                // Unit
                Text(unitLabel)
                    .font(.system(size: 9, weight: .medium, design: .monospaced))
                    .foregroundColor(Theme.milGreenDim.opacity(0.5))
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(
                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                    .fill(Theme.panelBackground.opacity(0.7))
                    .overlay(
                        RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                            .stroke(Theme.hudBorder.opacity(0.3), lineWidth: Theme.borderWidth)
                    )
            )
        }
    }

    // MARK: - Formatting

    private var formattedRange: String {
        let value = backgroundRange.adjustedRange.converted(to: displayUnit).value
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
