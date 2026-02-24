//
//  ConfidenceBadge.swift
//  Rangefinder
//
//  Color ring with 0.5s hysteresis to prevent color flickering.
//

import SwiftUI

struct ConfidenceBadge: View {
    let confidence: Float
    let source: DepthSource

    // Hysteresis: don't flicker between colors
    @State private var displayedColor: Color = Theme.confidenceHigh

    var body: some View {
        HStack(spacing: 6) {
            // Confidence ring
            Circle()
                .fill(displayedColor)
                .frame(width: 10, height: 10)
                .overlay(
                    Circle()
                        .stroke(displayedColor.opacity(0.5), lineWidth: 2)
                        .frame(width: 16, height: 16)
                )

            // Source icon
            Image(systemName: source.icon)
                .font(.system(size: 12))
                .foregroundColor(.white.opacity(0.7))
        }
        .onChange(of: confidence) { _, newValue in
            // Delayed color transition for hysteresis
            let targetColor = Theme.confidenceColor(for: newValue)
            withAnimation(.easeInOut(duration: 0.5)) {
                displayedColor = targetColor
            }
        }
        .onAppear {
            displayedColor = Theme.confidenceColor(for: confidence)
        }
    }
}
