//
//  StadiametricBracketOverlay.swift
//  Rangefinder
//
//  Transparent overlay with two draggable horizontal lines (brackets)
//  for stadiametric ranging. The user aligns the top and bottom brackets
//  with the top and bottom of a known-size target. The pixel distance
//  between brackets feeds the pinhole formula: R = (knownSize * f) / pixelSize.
//
//  Visual style: thin amber horizontal lines spanning the full width,
//  with small drag handles at the center.
//

import SwiftUI

struct StadiametricBracketOverlay: View {
    /// Callback with the pixel distance between brackets.
    let onPixelSizeChanged: (Double) -> Void

    @State private var topOffset: CGFloat = -80
    @State private var bottomOffset: CGFloat = 80

    var body: some View {
        GeometryReader { geometry in
            let centerY = geometry.size.height / 2

            ZStack {
                // Top bracket line
                bracketLine
                    .position(x: geometry.size.width / 2, y: centerY + topOffset)
                    .gesture(
                        DragGesture()
                            .onChanged { value in
                                let newOffset = value.location.y - centerY
                                // Constrain: top must be above bottom
                                if newOffset < bottomOffset - 20 {
                                    topOffset = newOffset
                                    reportPixelSize(viewHeight: geometry.size.height)
                                }
                            }
                    )

                // Bottom bracket line
                bracketLine
                    .position(x: geometry.size.width / 2, y: centerY + bottomOffset)
                    .gesture(
                        DragGesture()
                            .onChanged { value in
                                let newOffset = value.location.y - centerY
                                // Constrain: bottom must be below top
                                if newOffset > topOffset + 20 {
                                    bottomOffset = newOffset
                                    reportPixelSize(viewHeight: geometry.size.height)
                                }
                            }
                    )

                // Connecting line (vertical, between brackets)
                Path { path in
                    path.move(to: CGPoint(x: geometry.size.width / 2, y: centerY + topOffset))
                    path.addLine(to: CGPoint(x: geometry.size.width / 2, y: centerY + bottomOffset))
                }
                .stroke(Theme.milAmber.opacity(0.3), style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
                .allowsHitTesting(false)

                // Pixel size label (centered between brackets)
                let midY = centerY + (topOffset + bottomOffset) / 2
                Text(String(format: "%.0f px", abs(bottomOffset - topOffset)))
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundColor(Theme.milAmber.opacity(0.7))
                    .position(x: geometry.size.width / 2 + 40, y: midY)
                    .allowsHitTesting(false)
            }
        }
        .onAppear {
            // Report initial pixel size
            onPixelSizeChanged(Double(abs(bottomOffset - topOffset)))
        }
    }

    // MARK: - Bracket Line

    private var bracketLine: some View {
        ZStack {
            // Full-width thin line
            Rectangle()
                .fill(Theme.milAmber.opacity(0.6))
                .frame(height: 1)

            // Center drag handle
            RoundedRectangle(cornerRadius: 2)
                .fill(Theme.milAmber)
                .frame(width: 30, height: 6)
        }
        .frame(maxWidth: .infinity)
        .contentShape(Rectangle().size(width: .infinity, height: 30))
    }

    // MARK: - Report

    private func reportPixelSize(viewHeight: CGFloat) {
        let pixelDistance = abs(bottomOffset - topOffset)
        onPixelSizeChanged(Double(pixelDistance))
    }
}
