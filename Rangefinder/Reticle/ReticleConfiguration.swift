//
//  ReticleConfiguration.swift
//  Rangefinder
//
//  Reticle style, color, and appearance configuration.
//  Defaults to phosphor green (NVG-compatible mil standard).
//
//  Reticle styles:
//  - milDot:       Standard NATO mil-dot with dots at 1-mil intervals + half-mil hashes
//  - simpleCross:  Clean crosshair — no dots/hashes. Maximum clarity. Digital data only.
//  - rangefinder:  Duplex crosshair with center gap and corner brackets (Vectronix/VECTOR style)
//

import SwiftUI

// MARK: - Depth Zone Overlay

/// Lightweight value type carrying depth scene analysis into the reticle.
///
/// Always shows what the depth field sees — not just during bimodal detection.
/// This gives the operator constant awareness of the scene's depth structure:
/// - What the fusion is reading at the crosshair (inner bracket)
/// - What the DEM terrain ray-cast says (outer bracket, when available)
/// - Whether the two disagree (the fundamental ranging problem)
///
/// When DEM and crosshair agree, both brackets converge to the same size.
/// When they disagree (e.g., crosshair reads 42m rocks but DEM reads 1600m cliff),
/// the brackets visually separate, alerting the operator.
struct DepthZoneOverlay: Equatable {
    /// Whether the scene has two distinct depth zones
    let isBimodal: Bool

    /// Current fused depth at crosshair (what the range readout shows)
    let crosshairDepthM: Float

    /// DEM terrain ray-cast distance, if available (0 if no DEM)
    let demDepthM: Float

    /// Whether DEM data is available and valid
    let hasDEM: Bool

    /// Which zone the user has selected (near=1st target, far=last target)
    let activeZone: TargetPriority

    /// Near/far bimodal cluster peaks (only valid when isBimodal)
    let nearPeakM: Float
    let farPeakM: Float

    /// Whether there's a significant disagreement between crosshair and DEM
    var hasDisagreement: Bool {
        guard hasDEM, crosshairDepthM > 1, demDepthM > 1 else { return false }
        let ratio = max(crosshairDepthM, demDepthM) / min(crosshairDepthM, demDepthM)
        return ratio > 2.0
    }

    /// Whether there's useful depth information to display
    var isActive: Bool {
        return crosshairDepthM > 1 || hasDEM
    }

    static let none = DepthZoneOverlay(
        isBimodal: false,
        crosshairDepthM: 0, demDepthM: 0, hasDEM: false,
        activeZone: .far,
        nearPeakM: 0, farPeakM: 0
    )

    /// Format a distance for the depth label: "~40m" or "~1.6km"
    static func formatDepth(_ meters: Float) -> String {
        if meters < 1 { return "--" }
        if meters < 1000 {
            return "~\(Int(meters))m"
        } else {
            return String(format: "~%.1fkm", meters / 1000)
        }
    }
}

// MARK: - Reticle Style

enum ReticleStyle: String, CaseIterable, Identifiable {
    case milDot = "MIL-DOT"
    case simpleCross = "CROSSHAIR"
    case rangefinder = "RANGEFINDER"

    var id: String { rawValue }

    var description: String {
        switch self {
        case .milDot:      return "Standard mil-dot with angular measurement marks"
        case .simpleCross: return "Clean crosshair — maximum target clarity"
        case .rangefinder: return "Duplex crosshair with ranging brackets"
        }
    }
}

struct ReticleConfiguration {
    /// Active reticle style
    var style: ReticleStyle = .milDot

    /// Reticle line color — defaults to phosphor green (military standard)
    var color: Color = Theme.reticleDefault

    /// Fine crosshair line width (points, before pixel density)
    var fineLineWidth: CGFloat = 1.0

    /// Thick outer crosshair line width
    var outerLineWidth: CGFloat = 2.5

    /// Dot fill style
    var dotFilled: Bool = true

    /// Show half-mil hash marks
    var showHalfMilHashes: Bool = true

    /// Show mil number labels at 5-mil marks
    var showMilLabels: Bool = true

    /// Opacity of the reticle
    var opacity: Double = 0.92

    /// Shadow/outline for visibility against bright backgrounds
    var showOutline: Bool = true
    var outlineColor: Color = .black
    var outlineWidth: CGFloat = 0.5

    static let `default` = ReticleConfiguration()

    /// Night vision — phosphor green, high visibility
    static let nightVision = ReticleConfiguration(
        color: Theme.milGreen,
        opacity: 0.92,
        showOutline: true
    )

    /// Daylight — amber for high contrast against foliage/sky
    static let daylight = ReticleConfiguration(
        color: Theme.reticleAmber,
        opacity: 0.90,
        showOutline: true
    )

    /// Emergency — red, maximum visibility
    static let emergency = ReticleConfiguration(
        color: Theme.reticleRed,
        opacity: 0.95,
        showOutline: true
    )

    /// Purple neon — high contrast alternative
    static let purpleNeon = ReticleConfiguration(
        color: Theme.reticlePurple,
        opacity: 0.90,
        showOutline: true
    )

    /// Clean rangefinder — duplex crosshair, no dots
    static let cleanRangefinder = ReticleConfiguration(
        style: .rangefinder,
        color: Theme.milGreen
    )

    /// Minimal crosshair — maximum target clarity
    static let minimal = ReticleConfiguration(
        style: .simpleCross,
        color: Theme.milGreen,
        fineLineWidth: 0.75,
        outerLineWidth: 2.0,
        opacity: 0.85
    )
}
