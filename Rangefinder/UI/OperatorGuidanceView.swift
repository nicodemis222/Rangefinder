//
//  OperatorGuidanceView.swift
//  Rangefinder
//
//  Live-view operator guidance overlay.
//
//  Displays real-time coaching hints based on device motion, GPS quality,
//  calibration state, and environmental conditions. Modeled on military
//  fire-control HUD advisory displays (AN/PSQ-23 STORM status bar,
//  Kestrel 5700 solution confidence indicators).
//
//  Layout: positioned between the range readout and the reticle center,
//  in the top HUD data zone. Hints appear as brief tactical status chips
//  with icon + terse message, auto-dismissing after conditions clear.
//
//  ┌──────────────────────────────────────┐
//  │ [MAG 1x] [.308 Z100] [ELEV +2°] [≡]│
//  │         ● 124 YDS  ±3              │
//  │         ▲ 1.2 MIL HIGH             │
//  │    ┌─────────────────────┐          │
//  │    │ ◉ STABILIZED  ▓▓▓░ │ ← THIS   │
//  │    └─────────────────────┘          │
//  │            ╋ reticle                │
//  │       ▓▓▓▓▓░░░ LIDAR  AI           │
//  └──────────────────────────────────────┘
//

import SwiftUI

// MARK: - Operator Guidance Overlay

struct OperatorGuidanceView: View {
    @ObservedObject var engine: OperatorGuidanceEngine

    var body: some View {
        VStack(spacing: 4) {
            // Stability bar (always visible when ranging)
            StabilityBar(
                level: engine.stabilityLevel,
                percent: engine.stabilityPercent,
                isCapturePrimed: engine.isCapturePrimed
            )

            // Active hint chips (max 2)
            ForEach(engine.activeHints, id: \.self) { hint in
                GuidanceHintChip(hint: hint)
                    .transition(.asymmetric(
                        insertion: .opacity.combined(with: .move(edge: .top)),
                        removal: .opacity
                    ))
            }
        }
        .animation(.easeInOut(duration: 0.3), value: engine.activeHints)
        .animation(.easeInOut(duration: 0.2), value: engine.stabilityLevel)
    }
}

// MARK: - Stability Bar

/// Compact horizontal bar showing device stability level.
/// Fills left-to-right: red → amber → green as stability improves.
/// Pulses green when capture window is detected (respiratory pause).
struct StabilityBar: View {
    let level: StabilityLevel
    let percent: Float
    let isCapturePrimed: Bool

    @State private var pulsePhase = false

    var body: some View {
        HStack(spacing: 6) {
            // Stability icon
            Image(systemName: stabilityIcon)
                .font(.system(size: 10, weight: .medium))
                .foregroundColor(stabilityColor)
                .frame(width: 12)

            // Bar
            GeometryReader { geometry in
                ZStack(alignment: .leading) {
                    // Background track
                    RoundedRectangle(cornerRadius: 2)
                        .fill(Theme.panelBackground)
                        .frame(height: 4)

                    // Fill
                    RoundedRectangle(cornerRadius: 2)
                        .fill(stabilityColor)
                        .frame(width: geometry.size.width * CGFloat(percent), height: 4)
                        .opacity(isCapturePrimed ? (pulsePhase ? 1.0 : 0.5) : 0.8)
                }
            }
            .frame(height: 4)

            // Level label
            Text(level.description)
                .font(.system(size: 9, weight: .bold, design: .monospaced))
                .foregroundColor(stabilityColor.opacity(0.7))
                .frame(width: 60, alignment: .trailing)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 4)
        .background(
            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                .fill(Theme.panelBackground.opacity(0.8))
                .overlay(
                    RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                        .stroke(stabilityColor.opacity(0.3), lineWidth: Theme.borderWidth)
                )
        )
        .frame(width: 180)
        .onAppear {
            withAnimation(.easeInOut(duration: 0.8).repeatForever(autoreverses: true)) {
                pulsePhase = true
            }
        }
    }

    private var stabilityIcon: String {
        if isCapturePrimed { return "circle.circle" }
        switch level {
        case .unstable:  return "exclamationmark.triangle"
        case .marginal:  return "hand.raised"
        case .adequate:  return "minus.circle"
        case .good:      return "checkmark.circle"
        case .excellent: return "lock.fill"
        }
    }

    private var stabilityColor: Color {
        if isCapturePrimed { return Theme.milGreen }
        switch level {
        case .unstable:  return Theme.milRed
        case .marginal:  return Theme.milAmber
        case .adequate:  return Theme.milAmber.opacity(0.8)
        case .good:      return Theme.milGreen.opacity(0.8)
        case .excellent: return Theme.milGreen
        }
    }
}

// MARK: - Guidance Hint Chip

/// Single-line tactical advisory chip: icon + terse message.
struct GuidanceHintChip: View {
    let hint: GuidanceHint

    var body: some View {
        HStack(spacing: 5) {
            Image(systemName: hint.icon)
                .font(.system(size: 10, weight: .medium))
                .foregroundColor(hintColor)
                .frame(width: 12)

            Text(hint.message)
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundColor(hintColor.opacity(0.85))
                .lineLimit(1)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 4)
        .background(
            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                .fill(Theme.panelBackground.opacity(0.8))
                .overlay(
                    RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                        .stroke(hintColor.opacity(0.3), lineWidth: Theme.borderWidth)
                )
        )
    }

    private var hintColor: Color {
        switch hint.severity {
        case .positive: return Theme.milGreen
        case .caution:  return Theme.milAmber
        case .warning:  return Theme.confidenceLow
        }
    }
}
