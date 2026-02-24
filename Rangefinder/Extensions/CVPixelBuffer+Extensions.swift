//
//  CVPixelBuffer+Extensions.swift
//  Rangefinder
//
//  Depth sampling and format helpers for CVPixelBuffer.
//

import CoreVideo
import Foundation

extension CVPixelBuffer {
    /// Samples a Float32 depth value at a normalized point (0-1).
    func sampleDepthFloat32(at normalizedPoint: CGPoint) -> Float? {
        CVPixelBufferLockBaseAddress(self, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(self, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(self) else { return nil }

        let width = CVPixelBufferGetWidth(self)
        let height = CVPixelBufferGetHeight(self)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(self)

        let x = Int(normalizedPoint.x * CGFloat(width))
        let y = Int(normalizedPoint.y * CGFloat(height))

        guard x >= 0, x < width, y >= 0, y < height else { return nil }

        let rowPtr = baseAddress + y * bytesPerRow
        let depth = rowPtr.assumingMemoryBound(to: Float.self)[x]

        guard depth > 0.01, depth < 10000.0, !depth.isNaN, !depth.isInfinite else {
            return nil
        }

        return depth
    }

    /// Samples depth using median of a small region for robustness.
    func sampleDepthMedian(at normalizedPoint: CGPoint, radiusNormalized: CGFloat = 0.02) -> Float? {
        CVPixelBufferLockBaseAddress(self, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(self, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(self) else { return nil }

        let width = CVPixelBufferGetWidth(self)
        let height = CVPixelBufferGetHeight(self)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(self)
        let pixelFormat = CVPixelBufferGetPixelFormatType(self)

        let centerX = Int(normalizedPoint.x * CGFloat(width))
        let centerY = Int(normalizedPoint.y * CGFloat(height))
        let radiusPx = Int(CGFloat(min(width, height)) * radiusNormalized)

        let startX = max(0, centerX - radiusPx)
        let endX = min(width - 1, centerX + radiusPx)
        let startY = max(0, centerY - radiusPx)
        let endY = min(height - 1, centerY + radiusPx)

        let isFloat16 = pixelFormat == kCVPixelFormatType_OneComponent16Half ||
                        pixelFormat == kCVPixelFormatType_DepthFloat16

        var samples: [Float] = []
        samples.reserveCapacity((endX - startX + 1) * (endY - startY + 1) / 4)

        for y in stride(from: startY, through: endY, by: 2) {
            let rowPtr = baseAddress + y * bytesPerRow
            for x in stride(from: startX, through: endX, by: 2) {
                let depth: Float
                if isFloat16 {
                    let float16Ptr = rowPtr.assumingMemoryBound(to: UInt16.self)
                    depth = float16ToFloat32(float16Ptr[x])
                } else {
                    depth = rowPtr.assumingMemoryBound(to: Float.self)[x]
                }

                if depth > 0.01, depth < 10000.0, !depth.isNaN, !depth.isInfinite {
                    samples.append(depth)
                }
            }
        }

        guard samples.count >= 3 else { return nil }

        samples.sort()
        return samples[samples.count / 2]
    }

    /// Samples LiDAR confidence at a normalized point. Returns 0-1.
    func sampleConfidence(at normalizedPoint: CGPoint) -> Float? {
        CVPixelBufferLockBaseAddress(self, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(self, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(self) else { return nil }

        let width = CVPixelBufferGetWidth(self)
        let height = CVPixelBufferGetHeight(self)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(self)

        let x = Int(normalizedPoint.x * CGFloat(width))
        let y = Int(normalizedPoint.y * CGFloat(height))

        guard x >= 0, x < width, y >= 0, y < height else { return nil }

        let rowPtr = baseAddress + y * bytesPerRow
        let value = rowPtr.assumingMemoryBound(to: UInt8.self)[x]

        // ARConfidenceLevel: 0=low, 1=medium, 2=high
        switch value {
        case 0: return 0.3
        case 1: return 0.7
        case 2: return 0.95
        default: return 0.5
        }
    }
}

// MARK: - Float16 Conversion

private func float16ToFloat32(_ value: UInt16) -> Float {
    if value == 0 { return 0.0 }
    if value == 0x8000 { return -0.0 }

    let sign = (value & 0x8000) >> 15
    let exponent = (value & 0x7C00) >> 10
    let mantissa = value & 0x03FF

    let result: Float
    if exponent == 0 {
        result = Float(mantissa) / 1024.0 * pow(2.0, -14.0)
    } else if exponent == 31 {
        result = mantissa == 0 ? .infinity : .nan
    } else {
        result = (1.0 + Float(mantissa) / 1024.0) * pow(2.0, Float(Int(exponent) - 15))
    }

    return sign == 1 ? -result : result
}
