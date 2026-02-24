//
//  Theme.swift
//  Rangefinder
//
//  Tactical fire-control color palette and typography.
//  Inspired by MIL-STD-3009 NVIS-compatible displays,
//  AN/PEQ thermal optics, and DAGR/Kestrel devices.
//

import SwiftUI

struct Theme {
    // MARK: - Primary Colors (Phosphor Green / Amber / Red)

    /// Phosphor green — NVG-compatible primary HUD color
    static let milGreen = Color(red: 0.0, green: 0.78, blue: 0.2)
    /// Amber warning — DAGR-style caution color
    static let milAmber = Color(red: 0.9, green: 0.7, blue: 0.0)
    /// Alert red — MASTER CAUTION
    static let milRed = Color(red: 0.85, green: 0.1, blue: 0.1)

    // MARK: - Dimmed Variants

    static let milGreenDim = milGreen.opacity(0.5)
    static let milAmberDim = milAmber.opacity(0.4)
    static let milRedDim = milRed.opacity(0.4)

    // MARK: - HUD Border / Accent

    /// Subtle green border on panels — CRT bezel effect
    static let hudBorder = Color(red: 0.0, green: 0.55, blue: 0.15)

    // MARK: - Legacy Aliases (for files not yet updated)

    static let primaryGreen = milGreen
    static let primaryCyan = milGreen  // Cyan → Green in mil palette
    static let primaryOrange = milAmber
    static let primaryRed = milRed

    // MARK: - Reticle Colors

    /// Default reticle: phosphor green (military standard)
    static let reticleDefault = milGreen
    static let reticleGreen = Color(red: 0.0, green: 1.0, blue: 0.0)
    static let reticleRed = Color(red: 1.0, green: 0.0, blue: 0.0)
    static let reticleAmber = milAmber
    static let reticlePurple = Color(red: 0.6, green: 0.1, blue: 0.95)

    // MARK: - Depth Zone Colors (reticle bracket overlay)

    /// Near depth zone — warm amber (foreground occluder)
    static let depthZoneNear = milAmber
    /// Far depth zone — cool cyan-teal (background terrain)
    static let depthZoneFar = Color(red: 0.0, green: 0.7, blue: 0.8)

    // MARK: - Backgrounds

    /// Near-black with green tint — panel fill
    static let panelBackground = Color(red: 0.02, green: 0.04, blue: 0.02)
    /// Hard black HUD background
    static let hudBackground = Color.black.opacity(0.75)
    /// Dark background for full-screen
    static let darkBackground = Color(red: 0.02, green: 0.03, blue: 0.02)
    /// Card background — slightly lighter dark green
    static let cardBackground = Color(red: 0.04, green: 0.06, blue: 0.04)

    // MARK: - Confidence Colors (Mil-Spec)

    static let confidenceHigh = milGreen
    static let confidenceMedium = milAmber
    static let confidenceLow = Color(red: 0.9, green: 0.4, blue: 0.0)  // Deep amber
    static let confidenceVeryLow = milRed

    static func confidenceColor(for confidence: Float) -> Color {
        switch confidence {
        case 0.7...: return confidenceHigh
        case 0.4..<0.7: return confidenceMedium
        case 0.2..<0.4: return confidenceLow
        default: return confidenceVeryLow
        }
    }

    // MARK: - Source Blend Colors

    static func sourceColor(_ source: DepthSource) -> Color {
        switch source {
        case .lidar: return milGreen
        case .neural: return milAmber
        case .geometric: return Color(red: 0.0, green: 0.6, blue: 0.7)  // Cyan/teal (sensor/IMU)
        case .demRaycast: return Color(red: 0.9, green: 0.5, blue: 0.1)  // Orange/amber (terrain)
        case .objectSize: return Color(red: 0.5, green: 0.5, blue: 0.5)  // Gray
        case .semantic: return milGreen
        case .stadiametric: return Color(red: 0.7, green: 0.3, blue: 0.9)  // Purple (manual)
        }
    }

    // MARK: - Holdover

    /// Holdover indicator color — amber for ballistic correction
    static let holdoverColor = milAmber

    // MARK: - Typography (Monospaced, Tactical)

    static let rangeDisplayFont = Font.system(size: 48, weight: .bold, design: .monospaced)
    static let rangeFont = Font.system(size: 56, weight: .bold, design: .monospaced)
    static let rangeUnitFont = Font.system(size: 18, weight: .medium, design: .monospaced)
    static let hudFont = Font.system(size: 13, weight: .medium, design: .monospaced)
    static let hudLargeFont = Font.system(size: 16, weight: .semibold, design: .monospaced)
    static let labelFont = Font.system(size: 11, weight: .regular, design: .monospaced)
    static let tutorialTitleFont = Font.system(size: 28, weight: .bold, design: .monospaced)
    static let tutorialBodyFont = Font.system(size: 16, weight: .regular, design: .monospaced)

    // MARK: - Dimensions (Hard Edges)

    /// Minimal corner radius — hard military edges
    static let hudCornerRadius: CGFloat = 4
    static let hudPadding: CGFloat = 10
    static let cardCornerRadius: CGFloat = 4
    /// Panel border width
    static let borderWidth: CGFloat = 1

    // MARK: - Corner Bracket Helpers

    /// Draw corner bracket marks on a RoundedRectangle (tactical HUD motif)
    /// Returns a Path with L-shaped corners for overlay
    static func cornerBrackets(in rect: CGRect, length: CGFloat = 12, inset: CGFloat = 2) -> Path {
        var path = Path()
        let r = rect.insetBy(dx: inset, dy: inset)

        // Top-left ┌
        path.move(to: CGPoint(x: r.minX, y: r.minY + length))
        path.addLine(to: CGPoint(x: r.minX, y: r.minY))
        path.addLine(to: CGPoint(x: r.minX + length, y: r.minY))

        // Top-right ┐
        path.move(to: CGPoint(x: r.maxX - length, y: r.minY))
        path.addLine(to: CGPoint(x: r.maxX, y: r.minY))
        path.addLine(to: CGPoint(x: r.maxX, y: r.minY + length))

        // Bottom-left └
        path.move(to: CGPoint(x: r.minX, y: r.maxY - length))
        path.addLine(to: CGPoint(x: r.minX, y: r.maxY))
        path.addLine(to: CGPoint(x: r.minX + length, y: r.maxY))

        // Bottom-right ┘
        path.move(to: CGPoint(x: r.maxX - length, y: r.maxY))
        path.addLine(to: CGPoint(x: r.maxX, y: r.maxY))
        path.addLine(to: CGPoint(x: r.maxX, y: r.maxY - length))

        return path
    }
}
