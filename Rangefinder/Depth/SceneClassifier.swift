//
//  SceneClassifier.swift
//  Rangefinder
//
//  Lightweight scene classification at the crosshair point using the
//  neural depth map. Classifies what the user is aiming at to route
//  fusion confidence intelligently.
//
//  Instead of a separate ML model (2.3MB+ for DeepLabV3), this uses
//  the neural depth map that we're already computing — zero additional
//  model cost, ~0.1ms per frame.
//
//  Classifications and their fusion effects:
//  - sky:       Neural depth confidence → 0 (depth is meaningless for sky)
//  - ground:    Geometric confidence boosted (flat ground assumption valid)
//  - structure: Geometric confidence suppressed (camera height assumption invalid)
//  - unknown:   No modification (default fusion weights)
//
//  Detection heuristics:
//  - Sky: depth at crosshair is at or near the maximum depth value
//    in the depth map, AND the depth is uniform in a large patch
//  - Ground: smooth depth gradient from bottom to top of frame,
//    crosshair depth increases monotonically upward
//  - Structure: sharp depth discontinuity near crosshair (edge of building)
//

import Foundation
import CoreVideo
import os

/// Scene class at the crosshair point.
enum CrosshairSceneClass: String {
    case sky = "SKY"           // Aiming at sky — neural depth meaningless
    case ground = "GND"        // Aiming at ground plane — geometric valid
    case structure = "STRUCT"  // Aiming at vertical structure — suppress geometric
    case unknown = "UNK"       // Can't determine — use default fusion
}

@MainActor
class SceneClassifier {

    // MARK: - Output

    /// Current scene classification at crosshair.
    var currentClass: CrosshairSceneClass = .unknown

    /// Confidence in the classification (0-1).
    var classConfidence: Float = 0

    // MARK: - Configuration

    /// How often to reclassify (every N frames). Light computation so can run frequently.
    private let classifyInterval = 3

    /// Frame counter for rate limiting.
    private var frameCount = 0

    // MARK: - Classify

    /// Classify the scene at the crosshair using the neural depth map.
    ///
    /// - Parameters:
    ///   - depthMap: Neural depth map (any pixel format: Float32 or Float16)
    ///   - crosshairPoint: Normalized screen point (0-1)
    ///   - pitchDegrees: Device pitch (negative = looking down)
    func classify(
        depthMap: CVPixelBuffer,
        crosshairPoint: CGPoint = CGPoint(x: 0.5, y: 0.5),
        pitchDegrees: Double
    ) {
        frameCount += 1
        guard frameCount % classifyInterval == 0 else { return }

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let formatType = CVPixelBufferGetPixelFormatType(depthMap)

        // Sample depth at crosshair and surrounding points
        let centerDepth = sampleDepth(
            baseAddress: baseAddress, width: width, height: height,
            bytesPerRow: bytesPerRow, format: formatType,
            at: crosshairPoint
        )

        guard let center = centerDepth, center > 0, !center.isNaN else {
            currentClass = .unknown
            classConfidence = 0
            return
        }

        // Sample a 5-point cross pattern around crosshair
        let offset: CGFloat = 0.08  // 8% of frame
        let samples: [(CGPoint, String)] = [
            (CGPoint(x: crosshairPoint.x, y: crosshairPoint.y - offset), "above"),
            (CGPoint(x: crosshairPoint.x, y: crosshairPoint.y + offset), "below"),
            (CGPoint(x: crosshairPoint.x - offset, y: crosshairPoint.y), "left"),
            (CGPoint(x: crosshairPoint.x + offset, y: crosshairPoint.y), "right"),
        ]

        var depthValues: [Float] = [center]
        for (point, _) in samples {
            let clamped = CGPoint(
                x: min(1, max(0, point.x)),
                y: min(1, max(0, point.y))
            )
            if let d = sampleDepth(
                baseAddress: baseAddress, width: width, height: height,
                bytesPerRow: bytesPerRow, format: formatType,
                at: clamped
            ), d > 0, !d.isNaN {
                depthValues.append(d)
            }
        }

        // Get global depth statistics (sample a sparse grid for speed)
        let globalStats = computeGlobalStats(
            baseAddress: baseAddress, width: width, height: height,
            bytesPerRow: bytesPerRow, format: formatType
        )

        // --- Sky Detection ---
        // Sky produces max-depth (or near-max) values with very low local variance
        let isSkyCandidate = detectSky(
            centerDepth: center,
            localDepths: depthValues,
            globalMax: globalStats.max,
            globalMedian: globalStats.median,
            pitchDegrees: pitchDegrees
        )

        if isSkyCandidate {
            currentClass = .sky
            classConfidence = 0.85
            return
        }

        // --- Structure Detection ---
        // Sharp depth discontinuities near crosshair indicate structure edges
        let isStructure = detectStructure(
            centerDepth: center,
            localDepths: depthValues,
            pitchDegrees: pitchDegrees
        )

        if isStructure {
            currentClass = .structure
            classConfidence = 0.7
            return
        }

        // --- Ground Detection ---
        // Ground plane shows monotonic depth increase from bottom to top of frame
        let isGround = detectGround(
            centerDepth: center,
            aboveDepth: depthValues.count > 1 ? depthValues[1] : nil,
            belowDepth: depthValues.count > 2 ? depthValues[2] : nil,
            pitchDegrees: pitchDegrees
        )

        if isGround {
            currentClass = .ground
            classConfidence = 0.65
            return
        }

        // Default
        currentClass = .unknown
        classConfidence = 0
    }

    // MARK: - Detection Heuristics

    /// Sky: depth near global maximum with very low local variance.
    /// Neural models assign maximum depth to sky regions.
    private func detectSky(
        centerDepth: Float,
        localDepths: [Float],
        globalMax: Float,
        globalMedian: Float,
        pitchDegrees: Double
    ) -> Bool {
        // Sky only possible when looking upward or near level
        guard pitchDegrees > -10.0 else { return false }

        // Center depth should be near global max (>80% of max)
        guard centerDepth > globalMax * 0.80 else { return false }

        // Local variance should be very low (sky is uniform)
        let mean = localDepths.reduce(0, +) / Float(localDepths.count)
        let variance = localDepths.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Float(localDepths.count)
        let cv = sqrt(variance) / max(mean, 0.01)  // Coefficient of variation

        // If the local patch is very uniform (cv < 5%), it's likely sky.
        // Additionally check that center is significantly deeper than global median,
        // unless the entire frame is uniform (cv of global stats would be ~0).
        guard cv < 0.05 else { return false }

        // If the entire depth map is near-uniform (center ≈ median ≈ max),
        // AND variance is near zero, that's a sky-dominated frame.
        let globalRatio = globalMax - globalMedian
        let isNearlyUniformGlobal = globalRatio < globalMax * 0.10
        let isCenterNearMax = centerDepth > globalMax * 0.90

        if isNearlyUniformGlobal && isCenterNearMax {
            return true  // Entire frame is sky (or very far uniform background)
        }

        // Normal scene: center depth should be significantly deeper than median
        // (sky is the farthest region, other content is closer)
        return centerDepth > globalMedian * 1.5
    }

    /// Structure: sharp depth edges near crosshair.
    /// Buildings/walls create strong depth discontinuities.
    private func detectStructure(
        centerDepth: Float,
        localDepths: [Float],
        pitchDegrees: Double
    ) -> Bool {
        guard localDepths.count >= 3 else { return false }

        // Check for large depth jumps in neighboring samples
        var maxRatio: Float = 1.0
        for depth in localDepths {
            let ratio = depth > centerDepth ? depth / centerDepth : centerDepth / depth
            maxRatio = max(maxRatio, ratio)
        }

        // Strong depth discontinuity (>2× depth change in 8% of frame)
        // AND not looking steeply down (which naturally has depth gradients)
        return maxRatio > 2.0 && pitchDegrees > -30.0
    }

    /// Ground: smooth depth gradient increasing upward.
    /// Ground plane creates a characteristic depth gradient from
    /// near (bottom of frame) to far (top of frame).
    private func detectGround(
        centerDepth: Float,
        aboveDepth: Float?,
        belowDepth: Float?,
        pitchDegrees: Double
    ) -> Bool {
        // Ground detection only makes sense when looking downward
        guard pitchDegrees < -3.0 else { return false }

        guard let above = aboveDepth, let below = belowDepth else { return false }

        // Ground plane: depth increases going up in the frame
        // (farther away toward horizon) and decreases going down
        // (closer, toward your feet)
        let gradientCorrect = above > centerDepth && centerDepth > below

        // The gradient should be smooth (no big jumps)
        let aboveRatio = above / centerDepth
        let belowRatio = centerDepth / below
        let smoothGradient = aboveRatio < 2.0 && belowRatio < 2.0

        return gradientCorrect && smoothGradient
    }

    // MARK: - Depth Sampling Utilities

    private struct GlobalStats {
        let min: Float
        let max: Float
        let median: Float
    }

    /// Compute sparse global statistics of the depth map.
    /// Samples a 8×8 grid (64 points) for speed.
    private func computeGlobalStats(
        baseAddress: UnsafeMutableRawPointer,
        width: Int, height: Int,
        bytesPerRow: Int,
        format: OSType
    ) -> GlobalStats {
        var values: [Float] = []
        values.reserveCapacity(64)

        let gridSize = 8
        for gy in 0..<gridSize {
            for gx in 0..<gridSize {
                let nx = CGFloat(gx) / CGFloat(gridSize - 1)
                let ny = CGFloat(gy) / CGFloat(gridSize - 1)
                if let d = sampleDepth(
                    baseAddress: baseAddress, width: width, height: height,
                    bytesPerRow: bytesPerRow, format: format,
                    at: CGPoint(x: nx, y: ny)
                ), d > 0, !d.isNaN {
                    values.append(d)
                }
            }
        }

        guard !values.isEmpty else {
            return GlobalStats(min: 0, max: 0, median: 0)
        }

        values.sort()
        return GlobalStats(
            min: values.first!,
            max: values.last!,
            median: values[values.count / 2]
        )
    }

    /// Sample a single depth value from the depth map.
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
            return ptr[px]
        }
    }

    /// Convert Float16 (as UInt16) to Float32.
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
