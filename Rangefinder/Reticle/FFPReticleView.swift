//
//  FFPReticleView.swift
//  Rangefinder
//
//  First Focal Plane mil-dot reticle overlay.
//
//  FFP behavior: the reticle scales proportionally with camera zoom.
//  All reticle elements are defined in milliradian (mil) coordinates,
//  then converted to pixels using the current zoom-dependent pixelsPerMil.
//  This means mil measurements are accurate at ANY zoom level.
//

import SwiftUI

struct FFPReticleView: View {
    let zoomFactor: CGFloat
    let configuration: ReticleConfiguration
    var depthZones: DepthZoneOverlay = .none

    var body: some View {
        GeometryReader { geometry in
            Canvas { context, size in
                let center = CGPoint(x: size.width / 2, y: size.height / 2)
                let ppm = ReticleGeometry.pixelsPerMil(
                    screenWidth: size.width,
                    zoomFactor: zoomFactor
                )

                // Draw outline layer first (if enabled)
                if configuration.showOutline {
                    drawReticle(
                        context: context,
                        center: center,
                        ppm: ppm,
                        size: size,
                        color: configuration.outlineColor,
                        lineWidthBoost: configuration.outlineWidth * 2
                    )
                }

                // Draw main reticle
                drawReticle(
                    context: context,
                    center: center,
                    ppm: ppm,
                    size: size,
                    color: configuration.color,
                    lineWidthBoost: 0
                )

                // Draw depth zone brackets (on top of reticle).
                // Always shows scene depth when data is available:
                // - Inner bracket: crosshair depth (what fusion reads)
                // - Outer bracket: DEM terrain distance (when available)
                // When they disagree, brackets visually separate.
                if depthZones.isActive {
                    drawDepthZoneBrackets(
                        context: context,
                        center: center,
                        ppm: ppm,
                        lineWidth: configuration.fineLineWidth
                    )
                }
            }
            .opacity(configuration.opacity)
        }
        .allowsHitTesting(false)
    }

    // MARK: - Drawing

    private func drawReticle(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        size: CGSize,
        color: Color,
        lineWidthBoost: CGFloat
    ) {
        let fineWidth = configuration.fineLineWidth + lineWidthBoost
        let outerWidth = configuration.outerLineWidth + lineWidthBoost

        switch configuration.style {
        case .milDot:
            drawCrosshairs(
                context: context, center: center, ppm: ppm, size: size,
                color: color, fineWidth: fineWidth, outerWidth: outerWidth
            )
            drawMilDots(
                context: context, center: center, ppm: ppm,
                color: color, lineWidthBoost: lineWidthBoost
            )
            if configuration.showHalfMilHashes {
                drawHalfMilHashes(
                    context: context, center: center, ppm: ppm,
                    color: color, lineWidth: fineWidth * 0.8
                )
            }
            drawCenterDot(context: context, center: center, ppm: ppm, color: color)

        case .simpleCross:
            // Clean crosshair — duplex arms + center dot, no marks
            drawCrosshairs(
                context: context, center: center, ppm: ppm, size: size,
                color: color, fineWidth: fineWidth, outerWidth: outerWidth
            )
            drawCenterDot(context: context, center: center, ppm: ppm, color: color)

        case .rangefinder:
            // Duplex crosshair + corner ranging brackets (Vectronix-style)
            drawCrosshairs(
                context: context, center: center, ppm: ppm, size: size,
                color: color, fineWidth: fineWidth, outerWidth: outerWidth
            )
            drawRangingBrackets(
                context: context, center: center, ppm: ppm,
                color: color, lineWidth: fineWidth
            )
            drawCenterDot(context: context, center: center, ppm: ppm, color: color)
        }
    }

    private func drawCrosshairs(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        size: CGSize,
        color: Color,
        fineWidth: CGFloat,
        outerWidth: CGFloat
    ) {
        let gapPx = ReticleGeometry.centerGapMils * ppm
        let fineEndPx = ReticleGeometry.fineExtentMils * ppm
        let outerEndPx = (ReticleGeometry.fineExtentMils + ReticleGeometry.outerExtentMils) * ppm

        // Clamp outer end to screen bounds
        let maxH = size.width / 2
        let maxV = size.height / 2

        // Fine crosshairs (from gap to fineEnd)
        let fineStyle = StrokeStyle(lineWidth: fineWidth, lineCap: .butt)

        // Right
        var path = Path()
        path.move(to: CGPoint(x: center.x + gapPx, y: center.y))
        path.addLine(to: CGPoint(x: center.x + min(fineEndPx, maxH), y: center.y))
        context.stroke(path, with: .color(color), style: fineStyle)

        // Left
        path = Path()
        path.move(to: CGPoint(x: center.x - gapPx, y: center.y))
        path.addLine(to: CGPoint(x: center.x - min(fineEndPx, maxH), y: center.y))
        context.stroke(path, with: .color(color), style: fineStyle)

        // Down
        path = Path()
        path.move(to: CGPoint(x: center.x, y: center.y + gapPx))
        path.addLine(to: CGPoint(x: center.x, y: center.y + min(fineEndPx, maxV)))
        context.stroke(path, with: .color(color), style: fineStyle)

        // Up
        path = Path()
        path.move(to: CGPoint(x: center.x, y: center.y - gapPx))
        path.addLine(to: CGPoint(x: center.x, y: center.y - min(fineEndPx, maxV)))
        context.stroke(path, with: .color(color), style: fineStyle)

        // Thick outer crosshairs (duplex style)
        let outerStyle = StrokeStyle(lineWidth: outerWidth, lineCap: .butt)

        // Right outer
        if fineEndPx < maxH {
            path = Path()
            path.move(to: CGPoint(x: center.x + fineEndPx, y: center.y))
            path.addLine(to: CGPoint(x: center.x + min(outerEndPx, maxH), y: center.y))
            context.stroke(path, with: .color(color), style: outerStyle)
        }

        // Left outer
        if fineEndPx < maxH {
            path = Path()
            path.move(to: CGPoint(x: center.x - fineEndPx, y: center.y))
            path.addLine(to: CGPoint(x: center.x - min(outerEndPx, maxH), y: center.y))
            context.stroke(path, with: .color(color), style: outerStyle)
        }

        // Down outer
        if fineEndPx < maxV {
            path = Path()
            path.move(to: CGPoint(x: center.x, y: center.y + fineEndPx))
            path.addLine(to: CGPoint(x: center.x, y: center.y + min(outerEndPx, maxV)))
            context.stroke(path, with: .color(color), style: outerStyle)
        }

        // Up outer
        if fineEndPx < maxV {
            path = Path()
            path.move(to: CGPoint(x: center.x, y: center.y - fineEndPx))
            path.addLine(to: CGPoint(x: center.x, y: center.y - min(outerEndPx, maxV)))
            context.stroke(path, with: .color(color), style: outerStyle)
        }
    }

    private func drawMilDots(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        color: Color,
        lineWidthBoost: CGFloat
    ) {
        let dotDiameterPx = ReticleGeometry.dotDiameterMils * ppm + lineWidthBoost
        let minDotSize: CGFloat = 2.0

        for milPos in ReticleGeometry.dotPositionsMils {
            let offset = milPos * ppm
            let diameter = max(minDotSize, dotDiameterPx)
            let radius = diameter / 2

            // Horizontal: right and left
            drawDot(context: context,
                    at: CGPoint(x: center.x + offset, y: center.y),
                    radius: radius, color: color)
            drawDot(context: context,
                    at: CGPoint(x: center.x - offset, y: center.y),
                    radius: radius, color: color)

            // Vertical: down and up
            drawDot(context: context,
                    at: CGPoint(x: center.x, y: center.y + offset),
                    radius: radius, color: color)
            drawDot(context: context,
                    at: CGPoint(x: center.x, y: center.y - offset),
                    radius: radius, color: color)
        }
    }

    private func drawDot(
        context: GraphicsContext,
        at point: CGPoint,
        radius: CGFloat,
        color: Color
    ) {
        let rect = CGRect(
            x: point.x - radius,
            y: point.y - radius,
            width: radius * 2,
            height: radius * 2
        )

        if configuration.dotFilled {
            context.fill(Path(ellipseIn: rect), with: .color(color))
        } else {
            context.stroke(
                Path(ellipseIn: rect),
                with: .color(color),
                style: StrokeStyle(lineWidth: 1.0)
            )
        }
    }

    private func drawHalfMilHashes(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        color: Color,
        lineWidth: CGFloat
    ) {
        let hashLen = ReticleGeometry.hashLengthMils * ppm
        let minHashLen: CGFloat = 1.5
        let actualLen = max(minHashLen, hashLen)
        let style = StrokeStyle(lineWidth: lineWidth, lineCap: .butt)

        for milPos in ReticleGeometry.halfMilPositionsMils {
            let offset = milPos * ppm

            // Horizontal axis hashes (perpendicular = vertical tick marks)
            for sign: CGFloat in [-1, 1] {
                let x = center.x + sign * offset
                var path = Path()
                path.move(to: CGPoint(x: x, y: center.y - actualLen))
                path.addLine(to: CGPoint(x: x, y: center.y + actualLen))
                context.stroke(path, with: .color(color), style: style)
            }

            // Vertical axis hashes (perpendicular = horizontal tick marks)
            for sign: CGFloat in [-1, 1] {
                let y = center.y + sign * offset
                var path = Path()
                path.move(to: CGPoint(x: center.x - actualLen, y: y))
                path.addLine(to: CGPoint(x: center.x + actualLen, y: y))
                context.stroke(path, with: .color(color), style: style)
            }
        }
    }

    private func drawCenterDot(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        color: Color
    ) {
        // Small center dot (0.1 mil diameter, min 1.5pt)
        let diameter = max(1.5, 0.1 * ppm)
        let rect = CGRect(
            x: center.x - diameter / 2,
            y: center.y - diameter / 2,
            width: diameter,
            height: diameter
        )
        context.fill(Path(ellipseIn: rect), with: .color(color))
    }

    /// Ranging brackets: L-shaped corner marks forming a 2×2 mil square
    /// around the center. Used in rangefinder-style reticles (Vectronix VECTOR)
    /// for framing the target area and providing a quick angular size reference.
    private func drawRangingBrackets(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        color: Color,
        lineWidth: CGFloat
    ) {
        let bracketSizeMils: CGFloat = 1.0  // 1 mil from center in each direction
        let bracketLenMils: CGFloat = 0.4   // Length of each L-arm
        let offset = bracketSizeMils * ppm
        let armLen = bracketLenMils * ppm
        let style = StrokeStyle(lineWidth: lineWidth, lineCap: .butt)

        // Top-left bracket ┌
        var path = Path()
        path.move(to: CGPoint(x: center.x - offset, y: center.y - offset + armLen))
        path.addLine(to: CGPoint(x: center.x - offset, y: center.y - offset))
        path.addLine(to: CGPoint(x: center.x - offset + armLen, y: center.y - offset))
        context.stroke(path, with: .color(color), style: style)

        // Top-right bracket ┐
        path = Path()
        path.move(to: CGPoint(x: center.x + offset - armLen, y: center.y - offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y - offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y - offset + armLen))
        context.stroke(path, with: .color(color), style: style)

        // Bottom-left bracket └
        path = Path()
        path.move(to: CGPoint(x: center.x - offset, y: center.y + offset - armLen))
        path.addLine(to: CGPoint(x: center.x - offset, y: center.y + offset))
        path.addLine(to: CGPoint(x: center.x - offset + armLen, y: center.y + offset))
        context.stroke(path, with: .color(color), style: style)

        // Bottom-right bracket ┘
        path = Path()
        path.move(to: CGPoint(x: center.x + offset - armLen, y: center.y + offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y + offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y + offset - armLen))
        context.stroke(path, with: .color(color), style: style)
    }

    // MARK: - Depth Zone Brackets (Scene Depth Visualization)

    /// Draw scene depth brackets showing what the rangefinder sees.
    ///
    /// Always-on when depth data is available:
    /// - **Inner bracket** (amber): crosshair/fusion depth — what the range readout shows
    /// - **Outer bracket** (cyan): DEM terrain distance — the terrain ray-cast
    ///
    /// When both agree, brackets are similar size (visual confirmation).
    /// When they disagree (e.g., fusion reads 42m rocks but DEM reads 1600m cliff),
    /// the brackets separate, alerting the operator to a depth conflict.
    ///
    /// In bimodal + far-target mode with DEM confirmation, the outer bracket
    /// becomes solid (active) to show the system is ranging through to terrain.
    private func drawDepthZoneBrackets(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        lineWidth: CGFloat
    ) {
        let hasDisagreement = depthZones.hasDisagreement

        // Inner bracket: crosshair/fusion depth (what the readout shows)
        if depthZones.crosshairDepthM > 1 {
            let isActive = !hasDisagreement || depthZones.activeZone == .near
            drawBracketSet(
                context: context,
                center: center,
                ppm: ppm,
                sizeMils: 0.6,
                armMils: 0.3,
                color: hasDisagreement ? Theme.depthZoneNear : configuration.color,
                lineWidth: isActive ? lineWidth * 1.3 : lineWidth,
                isActive: isActive,
                label: DepthZoneOverlay.formatDepth(depthZones.crosshairDepthM)
            )
        }

        // Outer bracket: DEM terrain distance
        if depthZones.hasDEM && depthZones.demDepthM > 1 {
            let isActive = hasDisagreement && depthZones.activeZone == .far
            drawBracketSet(
                context: context,
                center: center,
                ppm: ppm,
                sizeMils: 1.4,
                armMils: 0.4,
                color: hasDisagreement ? Theme.depthZoneFar : configuration.color.opacity(0.6),
                lineWidth: isActive ? lineWidth * 1.3 : lineWidth,
                isActive: isActive || !hasDisagreement,
                label: hasDisagreement ? DepthZoneOverlay.formatDepth(depthZones.demDepthM) : ""
            )
        }
    }

    /// Draw a single set of 4 L-shaped corner brackets at the specified mil size.
    ///
    /// - Parameters:
    ///   - sizeMils: Distance from center to bracket corner in mils
    ///   - armMils: Length of each L-arm in mils
    ///   - isActive: If true, renders solid; if false, renders dashed + dimmer
    ///   - label: Distance label to show at bottom-right corner
    private func drawBracketSet(
        context: GraphicsContext,
        center: CGPoint,
        ppm: CGFloat,
        sizeMils: CGFloat,
        armMils: CGFloat,
        color: Color,
        lineWidth: CGFloat,
        isActive: Bool,
        label: String
    ) {
        let offset = sizeMils * ppm
        let armLen = armMils * ppm

        let style: StrokeStyle
        if isActive {
            style = StrokeStyle(lineWidth: lineWidth, lineCap: .butt)
        } else {
            style = StrokeStyle(lineWidth: lineWidth, lineCap: .butt, dash: [3, 3])
        }

        let opacity: CGFloat = isActive ? 0.92 : 0.45

        // Apply opacity via context transform
        var ctx = context
        ctx.opacity = opacity

        // Outline for visibility (dark shadow behind brackets)
        let outlineStyle: StrokeStyle
        if isActive {
            outlineStyle = StrokeStyle(lineWidth: lineWidth + 1.5, lineCap: .butt)
        } else {
            outlineStyle = StrokeStyle(lineWidth: lineWidth + 1.0, lineCap: .butt, dash: [3, 3])
        }

        // Draw outline first
        let brackets = buildBracketPath(center: center, offset: offset, armLen: armLen)
        ctx.stroke(brackets, with: .color(.black.opacity(0.6)), style: outlineStyle)

        // Draw colored brackets
        ctx.stroke(brackets, with: .color(color), style: style)

        // Draw depth label at bottom-right bracket corner
        if !label.isEmpty {
            drawDepthLabel(
                context: &ctx,
                text: label,
                position: CGPoint(
                    x: center.x + offset + 4,
                    y: center.y + offset + 2
                ),
                color: color
            )
        }
    }

    /// Build the L-shaped bracket path for all 4 corners.
    private func buildBracketPath(
        center: CGPoint,
        offset: CGFloat,
        armLen: CGFloat
    ) -> Path {
        var path = Path()

        // Top-left ┌
        path.move(to: CGPoint(x: center.x - offset, y: center.y - offset + armLen))
        path.addLine(to: CGPoint(x: center.x - offset, y: center.y - offset))
        path.addLine(to: CGPoint(x: center.x - offset + armLen, y: center.y - offset))

        // Top-right ┐
        path.move(to: CGPoint(x: center.x + offset - armLen, y: center.y - offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y - offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y - offset + armLen))

        // Bottom-left └
        path.move(to: CGPoint(x: center.x - offset, y: center.y + offset - armLen))
        path.addLine(to: CGPoint(x: center.x - offset, y: center.y + offset))
        path.addLine(to: CGPoint(x: center.x - offset + armLen, y: center.y + offset))

        // Bottom-right ┘
        path.move(to: CGPoint(x: center.x + offset - armLen, y: center.y + offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y + offset))
        path.addLine(to: CGPoint(x: center.x + offset, y: center.y + offset - armLen))

        return path
    }

    /// Draw a compact depth label (e.g., "~40m") with dark background pill.
    private func drawDepthLabel(
        context: inout GraphicsContext,
        text: String,
        position: CGPoint,
        color: Color
    ) {
        // Use resolved text for Canvas drawing
        let resolvedText = context.resolve(
            Text(text)
                .font(.system(size: 8, weight: .bold, design: .monospaced))
                .foregroundColor(color)
        )

        let textSize = resolvedText.measure(in: CGSize(width: 100, height: 20))

        // Background pill
        let pillRect = CGRect(
            x: position.x,
            y: position.y,
            width: textSize.width + 4,
            height: textSize.height + 2
        )
        context.fill(
            Path(roundedRect: pillRect, cornerRadius: 2),
            with: .color(.black.opacity(0.7))
        )

        // Text
        context.draw(
            resolvedText,
            at: CGPoint(
                x: pillRect.midX,
                y: pillRect.midY
            ),
            anchor: .center
        )
    }
}
