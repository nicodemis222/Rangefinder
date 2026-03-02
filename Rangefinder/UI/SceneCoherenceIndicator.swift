//
//  SceneCoherenceIndicator.swift
//  Rangefinder
//
//  Small status chip showing spatial coherence of multi-point readings.
//  Replaces the old semantic decision label in the bottom HUD zone.
//

import SwiftUI

struct SceneCoherenceIndicator: View {
    let coherence: SpatialCoherence
    @State private var isPulsing = false

    var body: some View {
        HStack(spacing: 4) {
            Circle()
                .fill(indicatorColor)
                .frame(width: 6, height: 6)
                .scaleEffect(isPulsing && coherence == .centerSuspect ? 1.3 : 1.0)

            Text(indicatorText)
                .font(.system(size: 9, weight: .bold, design: .monospaced))
                .foregroundColor(indicatorColor.opacity(0.85))
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(
            Capsule()
                .fill(Theme.panelBackground.opacity(0.7))
                .overlay(
                    Capsule()
                        .stroke(indicatorColor.opacity(0.4), lineWidth: Theme.borderWidth)
                )
        )
        .onChange(of: coherence) { _, newValue in
            if newValue == .centerSuspect {
                withAnimation(.easeInOut(duration: 0.6).repeatForever(autoreverses: true)) {
                    isPulsing = true
                }
            } else {
                isPulsing = false
            }
        }
        .onAppear {
            if coherence == .centerSuspect {
                withAnimation(.easeInOut(duration: 0.6).repeatForever(autoreverses: true)) {
                    isPulsing = true
                }
            }
        }
    }

    private var indicatorText: String {
        switch coherence {
        case .allCoherent: return "VERIFIED"
        case .centerSuspect: return "RANGE SUSPECT"
        case .mixed: return "PARTIAL"
        case .insufficient: return "SCANNING"
        case .unknown: return "---"
        }
    }

    private var indicatorColor: Color {
        switch coherence {
        case .allCoherent: return Theme.milGreen
        case .centerSuspect: return Theme.milAmber
        case .mixed: return Theme.milAmber
        case .insufficient: return Theme.milGreenDim
        case .unknown: return Theme.milGreenDim
        }
    }
}
