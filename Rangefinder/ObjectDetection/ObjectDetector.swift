//
//  ObjectDetector.swift
//  Rangefinder
//
//  YOLO + VNRecognizeAnimalsRequest object detection.
//  Feeds detections to ObjectRangeEstimator for pinhole ranging.
//

import Foundation
@preconcurrency import Vision
import CoreML
import CoreVideo
import os

struct DetectedObject: @unchecked Sendable {
    let label: String
    let confidence: Float
    let boundingBox: CGRect       // Vision coordinates (origin bottom-left, normalized)
    let screenBoundingBox: CGRect // Screen coordinates (origin top-left, normalized 0-1)
    let timestamp: Date
}

class ObjectDetector: @unchecked Sendable {

    // MARK: - State

    private(set) var isLoaded = false
    private var model: VNCoreMLModel?

    // Throttling
    private var lastDetectionTime: CFAbsoluteTime = 0
    var minDetectionInterval: CFAbsoluteTime = 0.2 // 5 FPS
    private var cachedDetections: [DetectedObject] = []
    private var isDetecting = false

    private let processingQueue = DispatchQueue(
        label: "com.mildotrangefinder.objectdetection",
        qos: .userInitiated
    )

    // MARK: - Init

    init() {}

    // MARK: - Model Loading

    func loadModel() async throws {
        let modelNames = [
            "SniperScope_Detector",
            "YOLOv8n",
            "YOLOv5s"
        ]

        var modelURL: URL?
        for name in modelNames {
            if let url = Bundle.main.url(forResource: name, withExtension: "mlmodelc") ??
                        Bundle.main.url(forResource: name, withExtension: "mlpackage") {
                modelURL = url
                Logger.detection.info("Found detection model: \(name)")
                break
            }
        }

        guard let url = modelURL else {
            Logger.detection.warning("No detection model found — object detection disabled")
            return
        }

        let config = MLModelConfiguration()
        config.computeUnits = .cpuAndNeuralEngine

        let loadedModel = try await MLModel.load(contentsOf: url, configuration: config)
        let visionModel = try VNCoreMLModel(for: loadedModel)
        self.model = visionModel
        self.isLoaded = true

        Logger.detection.info("Object detection model loaded")
    }

    // MARK: - Detection

    func detect(in pixelBuffer: CVPixelBuffer) async throws -> [DetectedObject] {
        // Throttle
        let now = CFAbsoluteTimeGetCurrent()
        if now - lastDetectionTime < minDetectionInterval {
            return cachedDetections
        }
        if isDetecting { return cachedDetections }

        isDetecting = true
        defer { isDetecting = false }

        var allDetections: [DetectedObject] = []

        // YOLO model detection
        if let model = model {
            let yoloDetections = try await runVisionModel(model, on: pixelBuffer)
            allDetections.append(contentsOf: yoloDetections)
        }

        // Built-in animal recognition (no model needed)
        let animalDetections = try await runAnimalRecognition(on: pixelBuffer)
        allDetections.append(contentsOf: animalDetections)

        lastDetectionTime = CFAbsoluteTimeGetCurrent()
        cachedDetections = allDetections
        return allDetections
    }

    // MARK: - Vision Model

    private func runVisionModel(
        _ model: VNCoreMLModel,
        on pixelBuffer: CVPixelBuffer
    ) async throws -> [DetectedObject] {
        try await withCheckedThrowingContinuation { continuation in
            processingQueue.async {
                let request = VNCoreMLRequest(model: model) { request, error in
                    if let error = error {
                        continuation.resume(throwing: MilDotError.objectDetectionFailed(error.localizedDescription))
                        return
                    }

                    guard let results = request.results as? [VNRecognizedObjectObservation] else {
                        continuation.resume(returning: [])
                        return
                    }

                    let detections = results.compactMap { obs -> DetectedObject? in
                        guard let topLabel = obs.labels.first,
                              topLabel.confidence > 0.4 else { return nil }

                        let visionBox = obs.boundingBox
                        let screenBox = CGRect(
                            x: visionBox.origin.x,
                            y: 1.0 - visionBox.origin.y - visionBox.height,
                            width: visionBox.width,
                            height: visionBox.height
                        )

                        return DetectedObject(
                            label: topLabel.identifier,
                            confidence: topLabel.confidence,
                            boundingBox: visionBox,
                            screenBoundingBox: screenBox,
                            timestamp: Date()
                        )
                    }

                    continuation.resume(returning: detections)
                }

                request.imageCropAndScaleOption = .scaleFill

                let handler = VNImageRequestHandler(
                    cvPixelBuffer: pixelBuffer,
                    orientation: .up,
                    options: [:]
                )

                do {
                    try handler.perform([request])
                } catch {
                    continuation.resume(throwing: MilDotError.objectDetectionFailed(error.localizedDescription))
                }
            }
        }
    }

    // MARK: - Built-in Animal Recognition

    private func runAnimalRecognition(on pixelBuffer: CVPixelBuffer) async throws -> [DetectedObject] {
        try await withCheckedThrowingContinuation { continuation in
            processingQueue.async {
                let request = VNRecognizeAnimalsRequest { request, error in
                    if let error = error {
                        continuation.resume(returning: [])
                        Logger.detection.warning("Animal recognition failed: \(error.localizedDescription)")
                        return
                    }

                    guard let results = request.results as? [VNRecognizedObjectObservation] else {
                        continuation.resume(returning: [])
                        return
                    }

                    let detections = results.compactMap { obs -> DetectedObject? in
                        guard let topLabel = obs.labels.first,
                              topLabel.confidence > 0.5 else { return nil }

                        let visionBox = obs.boundingBox
                        let screenBox = CGRect(
                            x: visionBox.origin.x,
                            y: 1.0 - visionBox.origin.y - visionBox.height,
                            width: visionBox.width,
                            height: visionBox.height
                        )

                        return DetectedObject(
                            label: topLabel.identifier,
                            confidence: topLabel.confidence,
                            boundingBox: visionBox,
                            screenBoundingBox: screenBox,
                            timestamp: Date()
                        )
                    }

                    continuation.resume(returning: detections)
                }

                let handler = VNImageRequestHandler(
                    cvPixelBuffer: pixelBuffer,
                    orientation: .up,
                    options: [:]
                )

                do {
                    try handler.perform([request])
                } catch {
                    continuation.resume(returning: [])
                }
            }
        }
    }
}
