//
//  AnchorPointSelector.swift
//  Rangefinder
//
//  ML-driven feature point selection for scene-aware multi-point ranging.
//  Picks up to 4 anchor points from data already computed each frame
//  (neural depth map, object detections, bimodal analysis). No extra
//  inference — all strategies reuse existing buffers.
//
//  Strategies:
//  (a) Depth Edge Detection — gradient analysis on neural depth map
//  (b) Object Detection Hits — known-size objects with screen positions
//  (c) Histogram Peak Spatial Mapping — bimodal near/far cluster centroids
//  (d) High-Confidence Regions — low-variance stable depth regions
//

import Foundation
import CoreVideo
import os

/// A candidate anchor point with its selection metadata.
struct AnchorCandidate {
    let point: CGPoint
    let reason: AnchorSelectionReason
    let score: Float
    let estimatedDepthM: Float
}

@MainActor
class AnchorPointSelector {

    // MARK: - Configuration

    private let reselectInterval: Int = AppConfiguration.anchorReselectInterval
    private let gridSize: Int = AppConfiguration.depthEdgeGridSize
    private let maxAnchors: Int = AppConfiguration.maxAnchorCount
    private let minDistFromCenter: Float = AppConfiguration.anchorMinDistanceFromCenter

    // MARK: - State

    private var frameCount = 0
    private var cachedAnchors: [(point: CGPoint, reason: AnchorSelectionReason, score: Float)] = []
    private let smoothingAlpha: Float = AppConfiguration.anchorSmoothingAlpha

    // MARK: - Select Anchors

    /// Pick up to 4 best anchor points from current frame data.
    /// Runs full selection every `reselectInterval` frames; returns cached result otherwise.
    func selectAnchors(
        neuralDepthMap: CVPixelBuffer?,
        objectDetections: [ObjectRangeResult],
        bimodalAnalysis: UnifiedDepthField.BimodalAnalysis
    ) -> [(point: CGPoint, reason: AnchorSelectionReason, score: Float)] {

        frameCount += 1
        guard frameCount % reselectInterval == 0 else { return cachedAnchors }

        var candidates: [AnchorCandidate] = []

        // (a) Depth edge detection
        if let depthMap = neuralDepthMap {
            candidates.append(contentsOf: findDepthEdges(depthMap: depthMap))
        }

        // (b) Object detection positions
        for det in objectDetections {
            let distanceScore = DepthSourceConfidence.object(
                distanceM: Float(det.distanceMeters),
                detectionConfidence: det.confidence
            )
            candidates.append(AnchorCandidate(
                point: det.screenCenter,
                reason: .objectDetection,
                score: distanceScore * 1.5,  // Boost: best long-range anchors
                estimatedDepthM: Float(det.distanceMeters)
            ))
        }

        // (c) Histogram peak spatial mapping
        if let depthMap = neuralDepthMap, bimodalAnalysis.isBimodal {
            candidates.append(contentsOf: findHistogramPeakLocations(
                depthMap: depthMap,
                bimodal: bimodalAnalysis
            ))
        }

        // (d) High-confidence regions
        if let depthMap = neuralDepthMap {
            candidates.append(contentsOf: findHighConfidenceRegions(depthMap: depthMap))
        }

        // Filter candidates too close to center
        candidates = candidates.filter { c in
            let dx = Float(c.point.x) - 0.5
            let dy = Float(c.point.y) - 0.5
            return sqrt(dx * dx + dy * dy) > minDistFromCenter
        }

        // Filter out-of-bounds — top 18% excluded to avoid HUD overlap,
        // right 12% excluded to keep clear of the pill ladder column.
        candidates = candidates.filter { c in
            c.point.x > 0.02 && c.point.x < 0.88
                && c.point.y > 0.18 && c.point.y < 0.88
        }

        // Apply spatial spread and rank
        let selected = applySpatialSpread(candidates: candidates)

        // Apply EMA smoothing: blend new positions with previous if nearby
        let smoothed = selected.map { candidate -> (point: CGPoint, reason: AnchorSelectionReason, score: Float) in
            let newPoint = candidate.point
            // Find closest previous anchor
            if let closest = cachedAnchors.min(by: { distance($0.point, newPoint) < distance($1.point, newPoint) }),
               distance(closest.point, newPoint) < 0.15 {
                // Blend position with EMA
                let alpha = CGFloat(smoothingAlpha)
                let blended = CGPoint(
                    x: alpha * newPoint.x + (1 - alpha) * closest.point.x,
                    y: alpha * newPoint.y + (1 - alpha) * closest.point.y
                )
                return (blended, candidate.reason, candidate.score)
            }
            return (newPoint, candidate.reason, candidate.score)
        }

        cachedAnchors = smoothed
        return cachedAnchors
    }

    // MARK: - Strategy (a): Depth Edge Detection

    /// Find depth discontinuities on a sparse grid.
    /// Sharp edges in the depth map indicate object boundaries — structurally
    /// interesting points where relative depth ordering is most reliable.
    private func findDepthEdges(depthMap: CVPixelBuffer) -> [AnchorCandidate] {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return [] }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let format = CVPixelBufferGetPixelFormatType(depthMap)

        // Sample depth on a sparse grid
        var depthGrid: [[Float?]] = Array(
            repeating: Array(repeating: nil, count: gridSize),
            count: gridSize
        )

        for gy in 0..<gridSize {
            for gx in 0..<gridSize {
                let nx = CGFloat(gx) / CGFloat(gridSize - 1)
                let ny = CGFloat(gy) / CGFloat(gridSize - 1)
                depthGrid[gy][gx] = sampleDepth(
                    baseAddress: baseAddress, width: width, height: height,
                    bytesPerRow: bytesPerRow, format: format,
                    at: CGPoint(x: nx, y: ny)
                )
            }
        }

        // Compute gradient magnitude at each interior grid point
        var edgeCandidates: [AnchorCandidate] = []

        for gy in 1..<(gridSize - 1) {
            for gx in 1..<(gridSize - 1) {
                guard let center = depthGrid[gy][gx], center > 0 else { continue }
                guard let left = depthGrid[gy][gx - 1], left > 0 else { continue }
                guard let right = depthGrid[gy][gx + 1], right > 0 else { continue }
                guard let above = depthGrid[gy - 1][gx], above > 0 else { continue }
                guard let below = depthGrid[gy + 1][gx], below > 0 else { continue }

                // Log-scale gradient to handle depth spanning orders of magnitude
                let logCenter = log(max(center, 0.1))
                let gxGrad = abs(log(max(right, 0.1)) - log(max(left, 0.1)))
                let gyGrad = abs(log(max(below, 0.1)) - log(max(above, 0.1)))
                let gradMag = sqrt(gxGrad * gxGrad + gyGrad * gyGrad)

                // Only consider significant edges (>0.5 in log-space ≈ 1.65x depth ratio)
                guard gradMag > 0.5 else { continue }

                let nx = CGFloat(gx) / CGFloat(gridSize - 1)
                let ny = CGFloat(gy) / CGFloat(gridSize - 1)

                edgeCandidates.append(AnchorCandidate(
                    point: CGPoint(x: nx, y: ny),
                    reason: .depthEdge,
                    score: min(gradMag, 3.0),  // Cap to avoid domination by extreme edges
                    estimatedDepthM: center
                ))
            }
        }

        // Return top 4 by gradient magnitude
        return Array(edgeCandidates.sorted { $0.score > $1.score }.prefix(4))
    }

    // MARK: - Strategy (c): Histogram Peak Spatial Mapping

    /// Map bimodal near/far peaks to spatial positions in the depth map.
    /// Find the centroid of pixels belonging to each depth cluster.
    private func findHistogramPeakLocations(
        depthMap: CVPixelBuffer,
        bimodal: UnifiedDepthField.BimodalAnalysis
    ) -> [AnchorCandidate] {
        guard bimodal.isBimodal else { return [] }

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return [] }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let format = CVPixelBufferGetPixelFormatType(depthMap)

        let midpoint = (bimodal.nearPeakM + bimodal.farPeakM) / 2.0

        // Sparse 8x8 grid sampling to find cluster centroids
        var nearSumX: Float = 0, nearSumY: Float = 0, nearCount: Float = 0
        var farSumX: Float = 0, farSumY: Float = 0, farCount: Float = 0

        let sampleGrid = 8
        for gy in 0..<sampleGrid {
            for gx in 0..<sampleGrid {
                let nx = CGFloat(gx) / CGFloat(sampleGrid - 1)
                let ny = CGFloat(gy) / CGFloat(sampleGrid - 1)

                guard let d = sampleDepth(
                    baseAddress: baseAddress, width: width, height: height,
                    bytesPerRow: bytesPerRow, format: format,
                    at: CGPoint(x: nx, y: ny)
                ), d > 0, !d.isNaN else { continue }

                if d < midpoint {
                    nearSumX += Float(nx)
                    nearSumY += Float(ny)
                    nearCount += 1
                } else {
                    farSumX += Float(nx)
                    farSumY += Float(ny)
                    farCount += 1
                }
            }
        }

        var results: [AnchorCandidate] = []

        if nearCount >= 3 {
            results.append(AnchorCandidate(
                point: CGPoint(x: CGFloat(nearSumX / nearCount), y: CGFloat(nearSumY / nearCount)),
                reason: .histogramPeak,
                score: 0.8 * min(nearCount / Float(sampleGrid * sampleGrid), 0.5) * 2.0,
                estimatedDepthM: bimodal.nearPeakM
            ))
        }
        if farCount >= 3 {
            results.append(AnchorCandidate(
                point: CGPoint(x: CGFloat(farSumX / farCount), y: CGFloat(farSumY / farCount)),
                reason: .histogramPeak,
                score: 0.9 * min(farCount / Float(sampleGrid * sampleGrid), 0.5) * 2.0,
                estimatedDepthM: bimodal.farPeakM
            ))
        }

        return results
    }

    // MARK: - Strategy (d): High-Confidence Regions

    /// Find low-variance depth regions — stable readings make good anchors.
    private func findHighConfidenceRegions(depthMap: CVPixelBuffer) -> [AnchorCandidate] {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return [] }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let format = CVPixelBufferGetPixelFormatType(depthMap)

        // Evaluate 4x4 macro blocks (each ~25% of frame)
        let blockSize = 4
        var blockCandidates: [AnchorCandidate] = []

        for by in 0..<blockSize {
            for bx in 0..<blockSize {
                // Sample a 3x3 sub-grid within each macro block
                var samples: [Float] = []
                for sy in 0..<3 {
                    for sx in 0..<3 {
                        let nx = CGFloat(bx) / CGFloat(blockSize) + CGFloat(sx) / CGFloat(blockSize * 3) + 0.05
                        let ny = CGFloat(by) / CGFloat(blockSize) + CGFloat(sy) / CGFloat(blockSize * 3) + 0.05
                        if let d = sampleDepth(
                            baseAddress: baseAddress, width: width, height: height,
                            bytesPerRow: bytesPerRow, format: format,
                            at: CGPoint(x: min(0.98, nx), y: min(0.98, ny))
                        ), d > 0, !d.isNaN {
                            samples.append(d)
                        }
                    }
                }

                guard samples.count >= 5 else { continue }

                // Compute coefficient of variation
                let mean = samples.reduce(0, +) / Float(samples.count)
                let variance = samples.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Float(samples.count)
                let cv = sqrt(variance) / max(mean, 0.01)

                // Low CV = stable region = good anchor (but not zero — that's sky)
                guard cv > 0.01 && cv < 0.15 else { continue }

                let centerX = (CGFloat(bx) + 0.5) / CGFloat(blockSize)
                let centerY = (CGFloat(by) + 0.5) / CGFloat(blockSize)

                // Score: inversely proportional to CV, boosted by being farther from center
                let distFromCenter = sqrt(pow(Float(centerX) - 0.5, 2) + pow(Float(centerY) - 0.5, 2))
                let score = (1.0 - cv * 5.0) * 0.6 * (0.5 + distFromCenter)

                blockCandidates.append(AnchorCandidate(
                    point: CGPoint(x: centerX, y: centerY),
                    reason: .highConfidence,
                    score: max(0.1, score),
                    estimatedDepthM: mean
                ))
            }
        }

        return Array(blockCandidates.sorted { $0.score > $1.score }.prefix(4))
    }

    // MARK: - Spatial Spread Enforcement

    /// Ensure anchors are vertically distributed for the range ladder.
    /// Uses Y-band enforcement (4 horizontal bands, max 1 per band) to
    /// maximize vertical coverage. Candidates are scored with a Y-diversity
    /// bonus: anchors far from already-selected points score higher.
    private func applySpatialSpread(candidates: [AnchorCandidate]) -> [AnchorCandidate] {
        guard !candidates.isEmpty else { return [] }

        // Sort by score descending
        let sorted = candidates.sorted { $0.score > $1.score }
        var selected: [AnchorCandidate] = []
        var occupiedBands: Set<Int> = []

        for candidate in sorted {
            guard selected.count < maxAnchors else { break }

            let band = yBandFor(candidate.point)

            // Primary: enforce 1 anchor per Y-band for vertical diversity
            if occupiedBands.contains(band) {
                // Allow if Y-distance from all selected is significant
                let minYDist = selected.map { abs(Float(candidate.point.y - $0.point.y)) }.min() ?? 1.0
                guard minYDist > 0.15 else { continue }
            }

            // Secondary: ensure minimum spatial distance from all selected
            if !selected.isEmpty {
                let minDist = selected.map { distance(candidate.point, $0.point) }.min() ?? 1.0
                guard minDist > 0.12 else { continue }
            }

            selected.append(candidate)
            occupiedBands.insert(band)
        }

        return selected
    }

    /// Y-band index (0-3) for vertical spread enforcement.
    /// Divides the frame into 4 horizontal bands of equal height.
    private func yBandFor(_ point: CGPoint) -> Int {
        min(3, max(0, Int(point.y * 4.0)))
    }

    /// Euclidean distance between two normalized points.
    private func distance(_ a: CGPoint, _ b: CGPoint) -> Float {
        let dx = Float(a.x - b.x)
        let dy = Float(a.y - b.y)
        return sqrt(dx * dx + dy * dy)
    }

    // MARK: - Depth Sampling

    private func sampleDepth(
        baseAddress: UnsafeMutableRawPointer,
        width: Int, height: Int,
        bytesPerRow: Int,
        format: OSType,
        at point: CGPoint
    ) -> Float? {
        let px = min(width - 1, max(0, Int(point.x * CGFloat(width))))
        let py = min(height - 1, max(0, Int(point.y * CGFloat(height))))
        let rowPtr = baseAddress + py * bytesPerRow

        switch format {
        case kCVPixelFormatType_DepthFloat16,
             kCVPixelFormatType_OneComponent16Half:
            let ptr = rowPtr.assumingMemoryBound(to: UInt16.self)
            return float16ToFloat(ptr[px])
        default:
            let ptr = rowPtr.assumingMemoryBound(to: Float.self)
            let val = ptr[px]
            return val.isNaN || val <= 0 ? nil : val
        }
    }

    private func float16ToFloat(_ value: UInt16) -> Float {
        let sign = (value & 0x8000) >> 15
        let exponent = (value & 0x7C00) >> 10
        let mantissa = value & 0x03FF
        if exponent == 0 {
            if mantissa == 0 { return sign == 0 ? 0.0 : -0.0 }
            var m = Float(mantissa) / 1024.0
            m *= pow(2.0, -14.0)
            return sign == 0 ? m : -m
        }
        if exponent == 0x1F {
            return mantissa == 0 ? (sign == 0 ? .infinity : -.infinity) : .nan
        }
        let f32Exponent = Int(exponent) - 15 + 127
        let f32Bits = UInt32(sign) << 31 | UInt32(f32Exponent) << 23 | UInt32(mantissa) << 13
        return Float(bitPattern: f32Bits)
    }
}
