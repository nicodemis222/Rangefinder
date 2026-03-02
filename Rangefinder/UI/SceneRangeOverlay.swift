//
//  SceneRangeOverlay.swift
//  Rangefinder
//
//  Vertical range ladder overlay for scene-aware multi-point ranging.
//
//  Center pill (larger) at screen center + up to 4 anchor pills on a
//  right-edge vertical ladder. Horizontal leader lines connect each
//  pill to a scene marker showing exactly where depth was sampled.
//
//  The vertical arrangement naturally aligns with ground-plane
//  perspective: near objects at bottom, far objects at top. De-conflict
//  logic prevents pill overlap when multiple anchors share similar
//  Y positions.
//

import SwiftUI

struct SceneRangeOverlay: View {
    let result: SceneRangeResult
    let displayUnit: UnitLength

    // Right-edge ladder layout constants
    private let pillRightInset: CGFloat = 55   // pill center X from right edge
    private let leaderEndInset: CGFloat = 100  // leader line end X from right edge
    private let minPillSpacing: CGFloat = 34   // minimum vertical gap between pills
    private let hudSafeTop: CGFloat = 100      // avoid top HUD zone
    private let hudSafeBottom: CGFloat = 100   // avoid bottom HUD zone

    var body: some View {
        GeometryReader { geometry in
            let size = geometry.size
            let items = layoutAnchors(in: size)
            let pillCenterX = size.width - pillRightInset
            let leaderEndX = size.width - leaderEndInset

            // Layer 1: Leader lines (behind everything)
            ForEach(items, id: \.sample.id) { item in
                LeaderLine(
                    from: screenPosition(item.sample.screenPoint, in: size),
                    to: CGPoint(x: leaderEndX, y: item.displayY),
                    coherence: item.sample.coherenceStatus
                )
            }

            // Layer 2: Scene markers at actual sample points
            ForEach(result.anchorSamples) { sample in
                if sample.estimate.isValid {
                    SceneMarker(coherence: sample.coherenceStatus)
                        .position(screenPosition(sample.screenPoint, in: size))
                }
            }

            // Center scene marker (slightly larger)
            if let center = result.centerSample, center.estimate.isValid {
                SceneMarker(coherence: center.coherenceStatus, isCenter: true)
                    .position(x: size.width * 0.5, y: size.height * 0.5)
            }

            // Layer 3: Anchor pills (right-edge vertical ladder)
            ForEach(items, id: \.sample.id) { item in
                AnchorRangePill(
                    sample: item.sample,
                    displayUnit: displayUnit
                )
                .fixedSize()
                .position(x: pillCenterX, y: item.displayY)
                .transition(.opacity)
                .animation(.easeInOut(duration: 0.3), value: item.displayY)
            }

            // Layer 4: Center pill (drawn last, on top, always at screen center)
            if let center = result.centerSample, center.estimate.isValid {
                CenterRangePill(
                    sample: center,
                    displayUnit: displayUnit
                )
                .position(
                    x: size.width * 0.5,
                    y: size.height * 0.5 - 28
                )
            }
        }
        .allowsHitTesting(false)
    }

    // MARK: - Anchor Layout Engine

    /// Position metadata for each anchor pill on the right-edge ladder.
    private struct LayoutItem {
        let sample: SceneRangeSample
        var displayY: CGFloat
    }

    /// Position anchor pills on the right edge at Y-coordinates matching
    /// their scene sample points. De-conflicts overlapping pills with
    /// minimum spacing while preserving depth ordering.
    private func layoutAnchors(in size: CGSize) -> [LayoutItem] {
        let valid = result.anchorSamples.filter { $0.estimate.isValid }
        guard !valid.isEmpty else { return [] }

        // Raw Y from normalized screen position
        var items = valid.map { sample in
            LayoutItem(
                sample: sample,
                displayY: sample.screenPoint.y * size.height
            )
        }

        // Sort by Y (top to bottom — far to near in ground-plane scenes)
        items.sort { $0.displayY < $1.displayY }

        // Clamp to HUD-safe zone
        let minY = hudSafeTop
        let maxY = size.height - hudSafeBottom

        for i in 0..<items.count {
            items[i].displayY = max(minY, min(maxY, items[i].displayY))
        }

        // Forward pass: push apart overlapping pills
        for i in 1..<items.count {
            if items[i].displayY - items[i - 1].displayY < minPillSpacing {
                items[i].displayY = items[i - 1].displayY + minPillSpacing
            }
        }

        // Backward pass: if overflow, push back up
        if let last = items.last, last.displayY > maxY {
            items[items.count - 1].displayY = maxY
            for i in stride(from: items.count - 2, through: 0, by: -1) {
                if items[i + 1].displayY - items[i].displayY < minPillSpacing {
                    items[i].displayY = items[i + 1].displayY - minPillSpacing
                }
            }
        }

        return items
    }

    private func screenPosition(_ normalized: CGPoint, in size: CGSize) -> CGPoint {
        CGPoint(
            x: normalized.x * size.width,
            y: normalized.y * size.height
        )
    }
}

// MARK: - Center Range Pill

private struct CenterRangePill: View {
    let sample: SceneRangeSample
    let displayUnit: UnitLength

    var body: some View {
        HStack(spacing: 4) {
            // Source icon
            Image(systemName: sourceIcon)
                .font(.system(size: 9, weight: .medium))
                .foregroundColor(sourceColor)

            // Distance — numericText transition for smooth digit changes
            Text(formattedDistance)
                .font(.system(size: 14, weight: .bold, design: .monospaced))
                .foregroundColor(.white)
                .contentTransition(.numericText())
                .animation(.easeInOut(duration: 0.25), value: formattedDistance)

            // Unit
            Text(unitLabel)
                .font(.system(size: 8, weight: .medium, design: .monospaced))
                .foregroundColor(Theme.milGreenDim)

            // Confidence dot
            Circle()
                .fill(Theme.confidenceColor(for: sample.estimate.confidence))
                .frame(width: 6, height: 6)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(
            Capsule()
                .fill(Theme.panelBackground.opacity(0.85))
                .overlay(
                    Capsule()
                        .stroke(borderColor, lineWidth: 1.5)
                )
        )
        .shadow(color: .black.opacity(0.6), radius: 4, x: 0, y: 2)
    }

    private var borderColor: Color {
        switch sample.coherenceStatus {
        case .inconsistent: return Theme.milAmber
        case .coherent: return Theme.milGreen.opacity(0.6)
        case .noData: return Theme.hudBorder.opacity(0.6)
        }
    }

    private var formattedDistance: String {
        let value = Measurement(
            value: sample.estimate.distanceMeters,
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

    private var sourceIcon: String {
        switch sample.estimate.source {
        case .lidar: return "sensor.fill"
        case .neural: return "brain"
        case .geometric: return "angle"
        case .demRaycast: return "mountain.2.fill"
        case .objectSize: return "viewfinder"
        case .stadiametric: return "ruler"
        case .semantic: return "sparkles"
        }
    }

    private var sourceColor: Color {
        Theme.sourceColor(sample.estimate.source)
    }
}

// MARK: - Anchor Range Pill

private struct AnchorRangePill: View {
    let sample: SceneRangeSample
    let displayUnit: UnitLength

    var body: some View {
        HStack(spacing: 3) {
            // Source icon (smaller)
            Image(systemName: sourceIcon)
                .font(.system(size: 7, weight: .medium))
                .foregroundColor(sourceColor)

            // Distance — numericText transition for smooth digit changes
            Text(formattedDistance)
                .font(.system(size: 10, weight: .medium, design: .monospaced))
                .foregroundColor(.white.opacity(0.9))
                .contentTransition(.numericText())
                .animation(.easeInOut(duration: 0.25), value: formattedDistance)

            // Confidence dot (smaller)
            Circle()
                .fill(Theme.confidenceColor(for: sample.estimate.confidence))
                .frame(width: 4, height: 4)
        }
        .padding(.horizontal, 7)
        .padding(.vertical, 3)
        .background(
            Capsule()
                .fill(Theme.panelBackground.opacity(0.75))
                .overlay(
                    Capsule()
                        .stroke(borderColor, lineWidth: 1.0)
                )
        )
        .shadow(color: .black.opacity(0.5), radius: 3, x: 0, y: 1)
    }

    private var borderColor: Color {
        switch sample.coherenceStatus {
        case .coherent: return Theme.milGreen.opacity(0.5)
        case .inconsistent: return Theme.milAmber.opacity(0.7)
        case .noData: return Theme.hudBorder.opacity(0.4)
        }
    }

    private var formattedDistance: String {
        let value = Measurement(
            value: sample.estimate.distanceMeters,
            unit: UnitLength.meters
        ).converted(to: displayUnit).value
        if value < 10 {
            return String(format: "%.1f", value)
        } else {
            return String(format: "%.0f", value)
        }
    }

    private var sourceIcon: String {
        switch sample.estimate.source {
        case .lidar: return "sensor.fill"
        case .neural: return "brain"
        case .geometric: return "angle"
        case .demRaycast: return "mountain.2.fill"
        case .objectSize: return "viewfinder"
        case .stadiametric: return "ruler"
        case .semantic: return "sparkles"
        }
    }

    private var sourceColor: Color {
        Theme.sourceColor(sample.estimate.source)
    }
}

// MARK: - Scene Marker

/// Small crosshair marker at the exact point in the scene where depth
/// was sampled. Shows the user precisely what each anchor pill measures.
private struct SceneMarker: View {
    let coherence: CoherenceStatus
    var isCenter: Bool = false

    private var ringSize: CGFloat { isCenter ? 12 : 8 }
    private var dotSize: CGFloat { isCenter ? 3 : 2 }
    private var lineWidth: CGFloat { isCenter ? 1.0 : 0.75 }

    var body: some View {
        ZStack {
            // Outer ring
            Circle()
                .stroke(markerColor, lineWidth: lineWidth)
                .frame(width: ringSize, height: ringSize)

            // Center dot
            Circle()
                .fill(markerColor)
                .frame(width: dotSize, height: dotSize)
        }
    }

    private var markerColor: Color {
        switch coherence {
        case .coherent: return Theme.milGreen.opacity(0.6)
        case .inconsistent: return Theme.milAmber.opacity(0.7)
        case .noData: return Theme.milGreenDim.opacity(0.3)
        }
    }
}

// MARK: - Leader Line

/// Thin dashed line connecting a scene marker to its right-edge pill.
/// Subtle enough to not obscure the camera feed, but visible enough
/// to show which pill measures which scene point.
private struct LeaderLine: View {
    let from: CGPoint
    let to: CGPoint
    let coherence: CoherenceStatus

    var body: some View {
        Path { path in
            path.move(to: from)
            path.addLine(to: to)
        }
        .stroke(
            lineColor,
            style: StrokeStyle(lineWidth: 0.5, dash: [3, 3])
        )
    }

    private var lineColor: Color {
        switch coherence {
        case .coherent: return Theme.milGreen.opacity(0.20)
        case .inconsistent: return Theme.milAmber.opacity(0.25)
        case .noData: return Theme.milGreenDim.opacity(0.10)
        }
    }
}
