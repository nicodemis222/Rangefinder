//
//  LiDARDepthProvider.swift
//  Rangefinder
//
//  Extracts depth at crosshair from ARKit sceneDepth.
//  Provides median-filtered sampling for stable readings.
//

import Foundation
import CoreVideo
import os

struct LiDARDepthProvider {

    /// Sample LiDAR depth at a normalized screen position.
    ///
    /// Uses median filtering over a small patch for robustness against noise.
    /// Returns nil if no valid LiDAR data at that point.
    static func sampleDepth(
        from depthMap: CVPixelBuffer,
        at normalizedPoint: CGPoint,
        patchRadius: Int = 2
    ) -> Float? {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return nil }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)

        let centerX = Int(normalizedPoint.x * CGFloat(width))
        let centerY = Int(normalizedPoint.y * CGFloat(height))

        var samples: [Float] = []
        samples.reserveCapacity((2 * patchRadius + 1) * (2 * patchRadius + 1))

        let formatType = CVPixelBufferGetPixelFormatType(depthMap)
        let isFloat16 = (formatType == kCVPixelFormatType_DepthFloat16)

        for dy in -patchRadius...patchRadius {
            for dx in -patchRadius...patchRadius {
                let x = centerX + dx
                let y = centerY + dy
                guard x >= 0, x < width, y >= 0, y < height else { continue }

                let rowPtr = baseAddress + y * bytesPerRow

                let depth: Float
                if isFloat16 {
                    let ptr = rowPtr.assumingMemoryBound(to: UInt16.self)
                    depth = float16ToFloat32(ptr[x])
                } else {
                    let ptr = rowPtr.assumingMemoryBound(to: Float.self)
                    depth = ptr[x]
                }

                // LiDAR valid range: 0.1m to ~12m (iPhone 16/17 Pro Max in good light)
                if depth > 0.1 && depth < 12.0 && !depth.isNaN {
                    samples.append(depth)
                }
            }
        }

        guard !samples.isEmpty else { return nil }

        // Median for robustness
        samples.sort()
        return samples[samples.count / 2]
    }

    /// Sample LiDAR confidence at a normalized screen position.
    static func sampleConfidence(
        from confidenceMap: CVPixelBuffer,
        at normalizedPoint: CGPoint
    ) -> Float {
        CVPixelBufferLockBaseAddress(confidenceMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(confidenceMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(confidenceMap) else { return 0 }

        let width = CVPixelBufferGetWidth(confidenceMap)
        let height = CVPixelBufferGetHeight(confidenceMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(confidenceMap)

        let x = Int(normalizedPoint.x * CGFloat(width))
        let y = Int(normalizedPoint.y * CGFloat(height))
        guard x >= 0, x < width, y >= 0, y < height else { return 0 }

        let rowPtr = baseAddress + y * bytesPerRow
        let ptr = rowPtr.assumingMemoryBound(to: UInt8.self)
        let rawConfidence = ptr[x]

        // ARKit confidence: 0 = low, 1 = medium, 2 = high
        switch rawConfidence {
        case 2: return 0.98
        case 1: return 0.7
        default: return 0.3
        }
    }

    // MARK: - Float16 Conversion

    private static func float16ToFloat32(_ value: UInt16) -> Float {
        let sign = (value & 0x8000) >> 15
        let exponent = (value & 0x7C00) >> 10
        let mantissa = value & 0x03FF

        if exponent == 0 {
            if mantissa == 0 { return sign == 0 ? 0.0 : -0.0 }
            // Subnormal
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
