//
//  HoldoverIndicator.swift
//  Rangefinder
//
//  Tactical holdover direction indicator.
//  Shows amber triangle + holdover values above or below the crosshair
//  depending on whether the shooter needs to hold high or low.
//
//  Displays all three units: mils (primary), inches/cm (secondary).
//

import SwiftUI

// MARK: - Holdover Indicator

/// Amber triangle indicator showing holdover direction and amount.
/// Positioned above (hold high) or below (hold low) the crosshair.
struct HoldoverIndicator: View {
    let holdover: HoldoverResult?

    // Triangle dimensions
    private let triangleWidth: CGFloat = 18
    private let triangleHeight: CGFloat = 12
    private let offsetFromCenter: CGFloat = 8

    var body: some View {
        if let holdover = holdover, holdover.isSignificant {
            VStack(spacing: 0) {
                if holdover.holdHigh {
                    holdHighView(holdover)
                    Spacer()
                } else {
                    Spacer()
                    holdLowView(holdover)
                }
            }
        }
    }

    // MARK: - Hold High (triangle pointing DOWN toward crosshair, above center)

    private func holdHighView(_ holdover: HoldoverResult) -> some View {
        VStack(spacing: 2) {
            Spacer()

            // Holdover value in mils (primary)
            Text(holdover.displayMils)
                .font(.system(size: 14, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milAmber)

            HStack(spacing: 3) {
                Text("MIL")
                    .font(.system(size: 9, weight: .bold, design: .monospaced))
                Text(holdover.directionText)
                    .font(.system(size: 9, weight: .bold, design: .monospaced))
            }
            .foregroundColor(Theme.milAmber.opacity(0.8))

            // Secondary: inches or cm
            Text("\(holdover.displayValue) \(holdover.unitLabel)")
                .font(.system(size: 9, weight: .medium, design: .monospaced))
                .foregroundColor(Theme.milAmber.opacity(0.6))

            // Triangle pointing DOWN (toward crosshair)
            TriangleShape(pointingUp: false)
                .fill(Theme.milAmber)
                .frame(width: triangleWidth, height: triangleHeight)
                .shadow(color: .black.opacity(0.6), radius: 2, x: 0, y: 1)

            Spacer()
                .frame(height: offsetFromCenter)
        }
        .shadow(color: .black.opacity(0.5), radius: 3)
    }

    // MARK: - Hold Low (triangle pointing UP toward crosshair, below center)

    private func holdLowView(_ holdover: HoldoverResult) -> some View {
        VStack(spacing: 2) {
            Spacer()
                .frame(height: offsetFromCenter)

            // Triangle pointing UP (toward crosshair)
            TriangleShape(pointingUp: true)
                .fill(Theme.milAmber)
                .frame(width: triangleWidth, height: triangleHeight)
                .shadow(color: .black.opacity(0.6), radius: 2, x: 0, y: -1)

            HStack(spacing: 3) {
                Text("MIL")
                    .font(.system(size: 9, weight: .bold, design: .monospaced))
                Text(holdover.directionText)
                    .font(.system(size: 9, weight: .bold, design: .monospaced))
            }
            .foregroundColor(Theme.milAmber.opacity(0.8))

            // Holdover value in mils (primary)
            Text(holdover.displayMils)
                .font(.system(size: 14, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milAmber)

            // Secondary: inches or cm
            Text("\(holdover.displayValue) \(holdover.unitLabel)")
                .font(.system(size: 9, weight: .medium, design: .monospaced))
                .foregroundColor(Theme.milAmber.opacity(0.6))

            Spacer()
        }
        .shadow(color: .black.opacity(0.5), radius: 3)
    }
}

// MARK: - Triangle Shape

struct TriangleShape: Shape {
    let pointingUp: Bool

    func path(in rect: CGRect) -> Path {
        var path = Path()

        if pointingUp {
            path.move(to: CGPoint(x: rect.midX, y: rect.minY))
            path.addLine(to: CGPoint(x: rect.maxX, y: rect.maxY))
            path.addLine(to: CGPoint(x: rect.minX, y: rect.maxY))
            path.closeSubpath()
        } else {
            path.move(to: CGPoint(x: rect.minX, y: rect.minY))
            path.addLine(to: CGPoint(x: rect.maxX, y: rect.minY))
            path.addLine(to: CGPoint(x: rect.midX, y: rect.maxY))
            path.closeSubpath()
        }

        return path
    }
}
