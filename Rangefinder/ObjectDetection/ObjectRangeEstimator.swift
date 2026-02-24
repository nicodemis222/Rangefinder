//
//  ObjectRangeEstimator.swift
//  Rangefinder
//
//  Pinhole camera model ranging from detected objects with known sizes.
//  distance = (knownSize × focalLength) / pixelSize
//

import Foundation
import os

struct ObjectRangeEstimator {

    /// Estimate range to detected objects using the pinhole camera model.
    ///
    /// For each detection with a known real-world size:
    /// distance = (realSize × focalLength) / apparentPixelSize
    static func estimateRanges(
        detections: [DetectedObject],
        intrinsics: CameraIntrinsics,
        imageWidth: Int,
        imageHeight: Int
    ) -> [ObjectRangeResult] {
        let db = KnownObjectDatabase.shared
        var results: [ObjectRangeResult] = []

        for detection in detections {
            guard let knownObject = db.lookup(detection.label) else { continue }

            // Get apparent size in pixels
            let apparentPixelSize: Double
            let focalLength: Double

            if knownObject.isHeight {
                // Use vertical dimension
                apparentPixelSize = Double(detection.screenBoundingBox.height) * Double(imageHeight)
                focalLength = Double(intrinsics.focalLength.y)
            } else {
                // Use horizontal dimension
                apparentPixelSize = Double(detection.screenBoundingBox.width) * Double(imageWidth)
                focalLength = Double(intrinsics.focalLength.x)
            }

            guard apparentPixelSize > 5 else { continue } // Too small to measure

            // Pinhole model: distance = (realSize × focalLength) / pixelSize
            let distance = (knownObject.sizeMeters * focalLength) / apparentPixelSize

            // Confidence based on detection confidence × object reliability
            let confidence = detection.confidence * knownObject.adjustedReliability

            // Screen center of bounding box
            let screenCenter = CGPoint(
                x: detection.screenBoundingBox.midX,
                y: detection.screenBoundingBox.midY
            )

            guard distance > 0.5, distance < 2000 else { continue }

            results.append(ObjectRangeResult(
                label: knownObject.displayName,
                distanceMeters: distance,
                confidence: confidence,
                screenCenter: screenCenter,
                boundingBox: detection.screenBoundingBox
            ))
        }

        return results
    }
}
