//
//  MetricDepthEstimator.swift
//  Rangefinder
//
//  Single-shot depth estimation using DepthAnythingV2.
//  Runs on CPU+NeuralEngine at ~15-20 FPS.
//

import Foundation
@preconcurrency import Vision
import CoreML
import CoreImage
import Metal
import os

private struct SendablePixelBuffer: @unchecked Sendable {
    let buffer: CVPixelBuffer
}

class MetricDepthEstimator: @unchecked Sendable {

    // MARK: - State

    private(set) var isLoaded = false
    private(set) var latency: TimeInterval = 0

    let inputSize: Int = 518
    let minDepthClip: Float = 0.1
    let maxDepthClip: Float = 1000.0

    // MARK: - Private

    private var model: VNCoreMLModel?
    private var mlModel: MLModel?

    private let processingQueue = DispatchQueue(
        label: "com.mildotrangefinder.metricdepth",
        qos: .userInitiated
    )

    private lazy var ciContext: CIContext = {
        if let device = MTLCreateSystemDefaultDevice() {
            return CIContext(mtlDevice: device, options: [
                .cacheIntermediates: false,
                .priorityRequestLow: false
            ])
        }
        return CIContext(options: [.cacheIntermediates: false])
    }()

    private var depthBufferPool: CVPixelBufferPool?
    private var confidenceBufferPool: CVPixelBufferPool?

    // Throttling
    private var lastInferenceTime: CFAbsoluteTime = 0
    var minInferenceInterval: CFAbsoluteTime = 0.066 // ~15 FPS
    private var cachedResult: MetricDepthResult?
    private var isInferring = false

    // Latency tracking
    private var latencyHistory: [TimeInterval] = []
    private let maxLatencyHistory = 10

    // One-time output format logging
    private var hasLoggedOutputFormat = false

    // MARK: - Init

    init() {}

    // MARK: - Model Loading

    func loadModel() async throws {
        let modelNames = [
            "DepthAnythingV2SmallF16",
            "DepthPro_Lite",
            "Metric3D_Small",
            "ZoeDepth_NK"
        ]

        var modelURL: URL?
        for name in modelNames {
            if let url = Bundle.main.url(forResource: name, withExtension: "mlmodelc") ??
                        Bundle.main.url(forResource: name, withExtension: "mlpackage") {
                modelURL = url
                Logger.neural.info("Found metric depth model: \(name)")
                break
            }
        }

        guard let url = modelURL else {
            Logger.neural.warning("No metric depth model found — using stub")
            isLoaded = true
            return
        }

        let config = MLModelConfiguration()
        config.computeUnits = .cpuAndNeuralEngine
        if #available(iOS 16.0, *) {
            config.allowLowPrecisionAccumulationOnGPU = true
        }

        let loadedModel = try await MLModel.load(contentsOf: url, configuration: config)
        let visionModel = try VNCoreMLModel(for: loadedModel)

        self.mlModel = loadedModel
        self.model = visionModel
        self.isLoaded = true

        Logger.neural.info("Metric depth model loaded")
    }

    // MARK: - Depth Estimation

    func estimateDepth(
        from pixelBuffer: CVPixelBuffer,
        intrinsics: CameraIntrinsics
    ) async throws -> MetricDepthResult {
        let startTime = Date()

        guard isLoaded, let model = model else {
            return createStubResult(for: pixelBuffer, intrinsics: intrinsics, startTime: startTime)
        }

        // Throttle
        let now = CFAbsoluteTimeGetCurrent()
        if now - lastInferenceTime < minInferenceInterval, let cached = cachedResult {
            return cached
        }
        if isInferring, let cached = cachedResult {
            return cached
        }

        isInferring = true
        defer { isInferring = false }

        let capturedModel = model
        let capturedCIContext = ciContext
        let capturedIntrinsics = intrinsics
        let capturedPixelBuffer = SendablePixelBuffer(buffer: pixelBuffer)

        let result: MetricDepthResult = try await withCheckedThrowingContinuation { continuation in
            processingQueue.async { [weak self] in
                guard let self = self else {
                    continuation.resume(throwing: MilDotError.depthModelFailed("Estimator deallocated"))
                    return
                }

                let request = VNCoreMLRequest(model: capturedModel) { [weak self] request, error in
                    if let error = error {
                        continuation.resume(throwing: MilDotError.depthModelFailed(error.localizedDescription))
                        return
                    }
                    guard let self = self else {
                        continuation.resume(throwing: MilDotError.depthModelFailed("Deallocated"))
                        return
                    }
                    do {
                        let result = try self.parseResults(
                            request.results,
                            intrinsics: capturedIntrinsics,
                            startTime: startTime
                        )
                        continuation.resume(returning: result)
                    } catch {
                        continuation.resume(throwing: error)
                    }
                }

                request.imageCropAndScaleOption = .scaleFill

                let handler = VNImageRequestHandler(
                    cvPixelBuffer: capturedPixelBuffer.buffer,
                    orientation: .up,
                    options: [.ciContext: capturedCIContext]
                )

                do {
                    try handler.perform([request])
                } catch {
                    continuation.resume(throwing: MilDotError.depthModelFailed(error.localizedDescription))
                }
            }
        }

        lastInferenceTime = CFAbsoluteTimeGetCurrent()
        cachedResult = result
        return result
    }

    /// Last valid depth result for calibration use.
    var lastDepthResult: MetricDepthResult? { cachedResult }

    // MARK: - Result Parsing

    private func parseResults(
        _ results: [Any]?,
        intrinsics: CameraIntrinsics,
        startTime: Date
    ) throws -> MetricDepthResult {
        let inferenceTime = Date().timeIntervalSince(startTime)
        updateLatency(inferenceTime)

        if let observation = results?.first as? VNPixelBufferObservation {
            let buf = observation.pixelBuffer
            if !hasLoggedOutputFormat {
                hasLoggedOutputFormat = true
                let fmt = CVPixelBufferGetPixelFormatType(buf)
                let w = CVPixelBufferGetWidth(buf)
                let h = CVPixelBufferGetHeight(buf)
                Logger.neural.info("Model output: VNPixelBufferObservation \(w)x\(h) pixelFormat=\(fmt)")
            }
            return parsePixelBufferOutput(buf, intrinsics: intrinsics, inferenceTime: inferenceTime)
        }

        if let observation = results?.first as? VNCoreMLFeatureValueObservation,
           let multiArray = observation.featureValue.multiArrayValue {
            if !hasLoggedOutputFormat {
                hasLoggedOutputFormat = true
                let shape = multiArray.shape.map { $0.intValue }
                Logger.neural.info("Model output: MLMultiArray shape=\(shape) dataType=\(multiArray.dataType.rawValue)")
            }
            return try parseMultiArrayOutput(multiArray, intrinsics: intrinsics, inferenceTime: inferenceTime)
        }

        // Log what we DID get
        if let results = results {
            Logger.neural.error("Unknown result type: \(results.map { type(of: $0) })")
        }

        throw MilDotError.depthModelFailed("Could not parse model output")
    }

    private func parsePixelBufferOutput(
        _ buffer: CVPixelBuffer,
        intrinsics: CameraIntrinsics,
        inferenceTime: TimeInterval
    ) -> MetricDepthResult {
        let (minD, maxD) = computeDepthRange(buffer)
        return MetricDepthResult(
            depthMap: buffer,
            confidenceMap: nil,
            minDepth: minD,
            maxDepth: maxD,
            intrinsics: intrinsics,
            timestamp: Date(),
            inferenceTime: inferenceTime
        )
    }

    private func parseMultiArrayOutput(
        _ multiArray: MLMultiArray,
        intrinsics: CameraIntrinsics,
        inferenceTime: TimeInterval
    ) throws -> MetricDepthResult {
        guard let buffer = multiArrayToPixelBuffer(multiArray) else {
            throw MilDotError.depthModelFailed("MultiArray conversion failed")
        }
        return parsePixelBufferOutput(buffer, intrinsics: intrinsics, inferenceTime: inferenceTime)
    }

    // MARK: - Depth Sampling

    func sampleDepth(from depthMap: CVPixelBuffer, at position: CGPoint) -> Float? {
        return sampleDepthPatch(from: depthMap, at: position, radius: 2)
    }

    /// Sample a patch of pixels around the given position and return the
    /// median value. This eliminates per-pixel noise that causes oscillation
    /// at long range where disparity values are very small.
    ///
    /// - Parameters:
    ///   - depthMap: The depth/disparity pixel buffer
    ///   - position: Normalized 0-1 position
    ///   - radius: Patch radius in pixels (e.g. 2 → 5×5 patch)
    /// - Returns: Median of valid pixel values in the patch
    func sampleDepthPatch(from depthMap: CVPixelBuffer, at position: CGPoint, radius: Int) -> Float? {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return nil }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let formatType = CVPixelBufferGetPixelFormatType(depthMap)

        let cx = min(width - 1, max(0, Int(position.x * CGFloat(width))))
        let cy = min(height - 1, max(0, Int(position.y * CGFloat(height))))

        // Collect valid values from a (2*radius+1) × (2*radius+1) patch
        var values: [Float] = []
        values.reserveCapacity((2 * radius + 1) * (2 * radius + 1))

        for dy in -radius...radius {
            for dx in -radius...radius {
                let px = cx + dx
                let py = cy + dy
                guard px >= 0, px < width, py >= 0, py < height else { continue }
                let val = readPixelValue(baseAddress: baseAddress, bytesPerRow: bytesPerRow,
                                          x: px, y: py, formatType: formatType)
                if !val.isNaN && !val.isInfinite && val > 0 {
                    values.append(val)
                }
            }
        }

        guard !values.isEmpty else { return nil }

        // Return median (robust to outliers)
        values.sort()
        return values[values.count / 2]
    }

    /// Read a single pixel value, handling Float32, Float16, and UInt8 pixel formats.
    private func readPixelValue(baseAddress: UnsafeMutableRawPointer, bytesPerRow: Int,
                                 x: Int, y: Int, formatType: OSType) -> Float {
        let rowPtr = baseAddress + y * bytesPerRow
        switch formatType {
        case kCVPixelFormatType_DepthFloat16,
             kCVPixelFormatType_OneComponent16Half:
            let ptr = rowPtr.assumingMemoryBound(to: UInt16.self)
            return float16ToFloat32(ptr[x])
        case kCVPixelFormatType_OneComponent8:
            let ptr = rowPtr.assumingMemoryBound(to: UInt8.self)
            return Float(ptr[x]) / 255.0
        default:
            // Float32 (most common for CoreML depth outputs)
            let ptr = rowPtr.assumingMemoryBound(to: Float.self)
            return ptr[x]
        }
    }

    /// Convert Float16 (stored as UInt16) to Float32.
    private func float16ToFloat32(_ value: UInt16) -> Float {
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

    // MARK: - Utility

    private func computeDepthRange(_ buffer: CVPixelBuffer) -> (Float, Float) {
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(buffer) else {
            return (minDepthClip, maxDepthClip)
        }

        let width = CVPixelBufferGetWidth(buffer)
        let height = CVPixelBufferGetHeight(buffer)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
        let formatType = CVPixelBufferGetPixelFormatType(buffer)

        var minVal = Float.greatestFiniteMagnitude
        var maxVal: Float = -Float.greatestFiniteMagnitude

        for y in stride(from: 0, to: height, by: 4) {
            for x in stride(from: 0, to: width, by: 4) {
                let depth = readPixelValue(baseAddress: baseAddress, bytesPerRow: bytesPerRow,
                                            x: x, y: y, formatType: formatType)
                if !depth.isNaN && !depth.isInfinite && depth > 0 {
                    minVal = min(minVal, depth)
                    maxVal = max(maxVal, depth)
                }
            }
        }

        if minVal == Float.greatestFiniteMagnitude {
            return (0.0, 0.0) // No valid values found
        }

        return (minVal, maxVal)
    }

    private func multiArrayToPixelBuffer(_ multiArray: MLMultiArray) -> CVPixelBuffer? {
        let shape = multiArray.shape.map { $0.intValue }
        guard shape.count >= 2 else { return nil }

        let height = shape[shape.count - 2]
        let width = shape[shape.count - 1]

        var pixelBuffer: CVPixelBuffer?
        let attrs: [String: Any] = [
            kCVPixelBufferIOSurfacePropertiesKey as String: [:] as [String: Any]
        ]
        CVPixelBufferCreate(kCFAllocatorDefault, width, height,
                           kCVPixelFormatType_OneComponent32Float,
                           attrs as CFDictionary, &pixelBuffer)

        guard let buffer = pixelBuffer else { return nil }

        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(buffer) else { return nil }

        let dstPtr = baseAddress.assumingMemoryBound(to: Float.self)
        let totalPixels = width * height

        // Handle different MLMultiArray data types
        switch multiArray.dataType {
        case .float32:
            memcpy(baseAddress, multiArray.dataPointer, totalPixels * MemoryLayout<Float>.size)
        case .float16:
            // DepthAnythingV2SmallF16 outputs Float16 — convert to Float32
            let srcPtr = multiArray.dataPointer.assumingMemoryBound(to: UInt16.self)
            for i in 0..<totalPixels {
                dstPtr[i] = float16ToFloat32(srcPtr[i])
            }
            Logger.neural.debug("Converted Float16 MLMultiArray (\(width)x\(height)) to Float32")
        case .double:
            let srcPtr = multiArray.dataPointer.assumingMemoryBound(to: Double.self)
            for i in 0..<totalPixels {
                dstPtr[i] = Float(srcPtr[i])
            }
        case .int32:
            let srcPtr = multiArray.dataPointer.assumingMemoryBound(to: Int32.self)
            for i in 0..<totalPixels {
                dstPtr[i] = Float(srcPtr[i])
            }
        @unknown default:
            memcpy(baseAddress, multiArray.dataPointer, totalPixels * MemoryLayout<Float>.size)
        }

        return buffer
    }

    private func createStubResult(
        for pixelBuffer: CVPixelBuffer,
        intrinsics: CameraIntrinsics,
        startTime: Date
    ) -> MetricDepthResult {
        let size = 384
        var depthBuffer: CVPixelBuffer?
        let attrs: [String: Any] = [
            kCVPixelBufferIOSurfacePropertiesKey as String: [:] as [String: Any]
        ]
        CVPixelBufferCreate(kCFAllocatorDefault, size, size,
                           kCVPixelFormatType_OneComponent32Float,
                           attrs as CFDictionary, &depthBuffer)

        if let buffer = depthBuffer {
            CVPixelBufferLockBaseAddress(buffer, [])
            if let base = CVPixelBufferGetBaseAddress(buffer) {
                let ptr = base.assumingMemoryBound(to: Float.self)
                for y in 0..<size {
                    for x in 0..<size {
                        let dx = Float(x - size/2) / Float(size)
                        let dy = Float(y - size/2) / Float(size)
                        ptr[y * size + x] = 10.0 + sqrt(dx*dx + dy*dy) * 90.0
                    }
                }
            }
            CVPixelBufferUnlockBaseAddress(buffer, [])
        }

        return MetricDepthResult(
            depthMap: depthBuffer!,
            confidenceMap: nil,
            minDepth: 10.0,
            maxDepth: 100.0,
            intrinsics: intrinsics,
            timestamp: Date(),
            inferenceTime: Date().timeIntervalSince(startTime)
        )
    }

    private func updateLatency(_ time: TimeInterval) {
        latencyHistory.append(time)
        if latencyHistory.count > maxLatencyHistory {
            latencyHistory.removeFirst()
        }
        latency = latencyHistory.reduce(0, +) / Double(latencyHistory.count)
    }
}
