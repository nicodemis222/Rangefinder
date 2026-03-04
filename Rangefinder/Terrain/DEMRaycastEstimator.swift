//
//  DEMRaycastEstimator.swift
//  Rangefinder
//
//  Terrain-aware range estimation via terrain visibility scanning.
//
//  Scans terrain along the device heading using GPS + altitude + IMU pitch
//  to find where the line of sight intersects the terrain surface.
//
//  This is the most accurate method for medium-to-long range targets
//  on terrain (50-2000m), especially on sloped ground where the
//  geometric ground-plane model fails catastrophically.
//
//  Algorithm (Terrain Visibility Scan — 3 strategies):
//  1. Ray origin: GPS (lat, lon, altitude)
//  2. March HORIZONTALLY along heading in 30m steps
//  3. At each step, run three detection strategies:
//     S1: LOS Intersection — compute LOS altitude at camera pitch, detect
//         where it crosses below terrain. Handles direct terrain hits.
//     S2: Viewshed Pitch Matching — find visible terrain whose elevation
//         angle matches camera center pitch (≥200m, tight 3° tolerance cap).
//     S3: FOV Terrain Feature Detection — find prominent terrain features
//         (mountains/cliffs rising >50m above base) visible anywhere within
//         the camera's vertical FOV (±15° from center pitch). This catches
//         mountains in the upper frame when center pitch hits flat ground.
//  4. Select best: terrain feature > far viewshed > far LOS > near LOS
//
//  KEY IMPROVEMENT over simple ray march:
//  The old approach cast a 3D ray at the pitch angle. On flat terrain at
//  negative pitch, the ray would go underground at ~eyeHeight/tan(pitch)
//  (e.g. 14m at -7.9°) and NEVER re-emerge — missing mountains at 1600m.
//  The new horizontal march + viewshed approach decouples the march direction
//  from the pitch angle, finding ALL visible terrain intersections.
//

import Foundation
import CoreLocation
import os

/// Result from DEM ray-terrain intersection.
struct DEMRaycastEstimate {
    let distanceMeters: Float    // Horizontal distance to intersection
    let confidence: Float        // 0-1 based on GPS/heading accuracy + distance
    let terrainElevation: Float  // Elevation at intersection point (m ASL)
    let headingDeg: Float        // Heading used for the ray (true north)
    let gpsAccuracy: Float       // Horizontal GPS accuracy at time of estimate
    let hitCoordinate: CLLocationCoordinate2D  // Lat/lon of the terrain intersection
}

@MainActor
class DEMRaycastEstimator {

    // MARK: - Dependencies

    let tileCache: SRTMTileCache

    // MARK: - Configuration

    /// Maximum ray march distance (meters).
    private let maxRayDistance: Float = 2000.0

    /// Ray step size (meters) — matches SRTM ~30m resolution.
    private let stepSize: Float = 30.0

    /// Minimum distance before returning a result (meters).
    private let minDistance: Float = 20.0

    /// Bisection refinement iterations after initial intersection detection.
    private let bisectionIterations = 5

    // MARK: - Rate Limiting

    private var lastEstimate: DEMRaycastEstimate?
    private var lastHeading: Double = -999
    private var lastPitch: Double = -999
    private var lastQueryTime: CFAbsoluteTime = 0

    /// Minimum time between DEM queries (seconds).
    let queryInterval: CFAbsoluteTime = 0.5  // 2 Hz max

    /// Vertical accuracy for current estimate (set in estimate(), used in terrainScan()).
    private var currentVerticalAccuracy: Float = -1

    // MARK: - Init

    init(tileCache: SRTMTileCache) {
        self.tileCache = tileCache
    }

    // MARK: - Estimate

    /// Cast a ray from the device into the terrain and find the intersection.
    ///
    /// - Parameters:
    ///   - coordinate: Device GPS position
    ///   - altitude: Best available altitude above sea level (meters, prefers barometric)
    ///   - pitchRadians: Device pitch (0=level, negative=down)
    ///   - headingDegrees: True-north heading (0=N, 90=E, 180=S, 270=W)
    ///   - horizontalAccuracy: GPS horizontal accuracy (meters)
    ///   - verticalAccuracy: Altitude accuracy (meters). Barometric: 1-5m, GPS-only: 10-30m.
    /// - Returns: DEM estimate, or nil if no intersection found
    func estimate(
        coordinate: CLLocationCoordinate2D,
        altitude: Double,
        pitchRadians: Double,
        headingDegrees: Double,
        horizontalAccuracy: Double,
        verticalAccuracy: Double = -1
    ) async -> DEMRaycastEstimate? {

        // Rate limiting: reuse last result if heading/pitch haven't changed much
        let now = CFAbsoluteTimeGetCurrent()
        if now - lastQueryTime < queryInterval,
           abs(headingDegrees - lastHeading) < 1.0,
           abs(pitchRadians - lastPitch) < 0.5 * .pi / 180.0,  // 0.5 degrees
           let cached = lastEstimate {
            return cached
        }

        // Allow rays from steep downward to moderately upward.
        // Looking UP at mountains is valid — the terrain scan marches forward
        // and finds terrain whose elevation angle matches the camera pitch.
        // Only reject rays pointing steeply upward (>30°) where hitting terrain
        // is unlikely (aiming at sky). Level rays (0°) are fine — if terrain
        // rises ahead (mountain), the viewshed will find it; if not, the scan
        // returns nil naturally after maxRayDistance.
        let pitchDegrees = pitchRadians * 180.0 / .pi
        guard pitchDegrees < 30.0 else { return nil }  // >30° above horizontal: aiming at sky

        // Need valid GPS
        guard horizontalAccuracy > 0, horizontalAccuracy < 100 else { return nil }

        // Store vertical accuracy for confidence computation in terrainScan
        currentVerticalAccuracy = Float(verticalAccuracy)

        // Heading in radians for horizontal march direction
        let headingRad = headingDegrees * .pi / 180.0

        // Horizontal march direction components (no pitch — march along ground)
        let dEast = sin(headingRad)    // per horizontal meter
        let dNorth = cos(headingRad)   // per horizontal meter

        // Ensure SRTM tile is available. Without it, each scan step requires
        // an individual EPQS HTTP request (slow, US-only, needs connectivity).
        // SRTM tiles provide instant elevation lookups for the entire corridor.
        if await !tileCache.hasTile(for: coordinate) {
            // Try auto-downloading the SRTM tile (runs once, persists to disk)
            if await !tileCache.hasAttemptedDownload {
                let downloaded = await tileCache.downloadTile(for: coordinate)
                if downloaded {
                    Logger.terrain.info("DEM: SRTM tile auto-downloaded for current location")
                }
            }

            // If still no tile, fall back to EPQS corridor pre-fetch
            if await !tileCache.hasTile(for: coordinate) {
                let fetchCount = await tileCache.prefetchCorridor(
                    origin: coordinate,
                    dEast: dEast,
                    dNorth: dNorth,
                    maxDistance: maxRayDistance,
                    stepSize: stepSize
                )
                if fetchCount == 0 {
                    // All points already cached — heading/pitch update
                    // on the same general corridor. Scan will use cached data.
                }
            }
        }

        // Scan terrain along heading and find intersection with line of sight
        let result = await terrainScan(
            origin: coordinate,
            originAltitude: altitude,
            cameraPitchRadians: pitchRadians,
            headingRad: headingRad,
            headingDeg: Float(headingDegrees),
            gpsAccuracy: Float(horizontalAccuracy)
        )

        // Cache result
        lastQueryTime = now
        lastHeading = headingDegrees
        lastPitch = pitchRadians
        lastEstimate = result

        return result
    }

    // MARK: - Terrain Visibility Scan

    /// Scan terrain along the heading using three complementary strategies:
    ///
    /// **Strategy 1: Line-of-Sight (LOS) Intersection**
    /// March HORIZONTALLY along heading. At each step, compute the LOS altitude
    /// (observer + d × tan(pitch)) and compare to terrain. Detect transitions
    /// where LOS crosses below terrain.
    ///
    /// **Strategy 2: Viewshed Pitch Matching (≥200m, tight tolerance)**
    /// Find visible terrain whose elevation angle matches camera center pitch.
    /// Tolerance capped at 3° to prevent flat-ground false matches (the old
    /// uncapped tolerance produced 131-164 yard false readings on flat desert).
    ///
    /// **Strategy 3: FOV Terrain Feature Detection**
    /// Find prominent terrain features (>50m rise above base level) whose
    /// elevation angle falls within the camera's vertical FOV (±15° from center).
    /// This is the KEY strategy for mountain ranging: when the camera center
    /// pitch (-5°) hits flat ground at 21m, mountains visible in the upper
    /// part of the frame (at +5° to +10° elevation angle) are still detected.
    ///
    /// **Selection:** terrain feature > far viewshed > far LOS > near LOS.
    private func terrainScan(
        origin: CLLocationCoordinate2D,
        originAltitude: Double,
        cameraPitchRadians: Double,
        headingRad: Double,
        headingDeg: Float,
        gpsAccuracy: Float
    ) async -> DEMRaycastEstimate? {

        // Meters per degree at this latitude
        let metersPerDegLat = 111_320.0  // roughly constant
        let metersPerDegLon = 111_320.0 * cos(origin.latitude * .pi / 180.0)

        guard metersPerDegLon > 1000 else { return nil }  // Sanity check (near poles)

        // --- ALTITUDE CORRECTION ---
        // GPS/barometric altitude can be 5-30m off from SRTM terrain level.
        // If the observer appears to be below or at SRTM terrain, the LOS
        // immediately detects a false intersection. Fix: query terrain at
        // origin and ensure we start at least 2m above it (eye height).
        var effectiveAltitude = originAltitude
        if let originTerrainElev = await tileCache.elevation(at: origin) {
            let aboveTerrain = originAltitude - originTerrainElev
            if aboveTerrain < 2.0 {
                effectiveAltitude = originTerrainElev + 2.0
                Logger.terrain.debug("DEM altitude correction: GPS=\(String(format: "%.1f", originAltitude))m SRTM=\(String(format: "%.1f", originTerrainElev))m → using \(String(format: "%.1f", effectiveAltitude))m")
            }
        }

        // Horizontal march direction (no pitch component — march along ground)
        let dEast = sin(headingRad)    // per horizontal meter
        let dNorth = cos(headingRad)   // per horizontal meter

        // LOS altitude: observer_alt + d * tan(pitch)
        // tan(pitch) is negative when looking down, positive when looking up
        let tanPitch = tan(cameraPitchRadians)

        // Camera pitch in degrees for viewshed matching
        let cameraPitchDeg = cameraPitchRadians * 180.0 / .pi

        let numSteps = Int(maxRayDistance / stepSize)

        // --- Strategy 1: LOS intersection tracking ---
        var prevLOSAbove = true
        var prevD: Float = 0
        var nearFieldHit: (distance: Float, elevation: Double, confidence: Float)?   // <100m
        var farFieldHit: (distance: Float, elevation: Double, confidence: Float)?    // >=100m

        // --- Strategy 2: Viewshed pitch matching (tightened tolerance) ---
        var maxElevAngle: Double = -90.0   // Viewshed horizon (nothing visible yet)
        var bestViewshedMatch: (distance: Float, elevation: Double, angleDiff: Double, confidence: Float)?

        // --- Strategy 3: FOV-aware terrain feature detection ---
        // The camera has a vertical FOV of ~30° (±15° from center). Mountains
        // visible in the upper part of the frame may have a very different
        // elevation angle than the center pitch. Strategy 3 finds prominent
        // terrain features (>50m rise) anywhere within the camera FOV.
        let halfFOVDeg: Double = 15.0   // Half of camera vertical FOV
        let fovMinPitch = cameraPitchDeg - halfFOVDeg
        let fovMaxPitch = cameraPitchDeg + halfFOVDeg
        var baseTerrainElev: Double?     // Ground level near observer
        var bestTerrainFeature: (distance: Float, elevation: Double, elevAngle: Double, confidence: Float)?

        for step in 1...numSteps {
            let d = Float(step) * stepSize
            let horizontalDist = Double(d)

            // Horizontal position along heading
            let lat = origin.latitude + (dNorth * horizontalDist) / metersPerDegLat
            let lon = origin.longitude + (dEast * horizontalDist) / metersPerDegLon
            let coord = CLLocationCoordinate2D(latitude: lat, longitude: lon)

            // Query terrain elevation
            guard let terrainElev = await tileCache.elevation(at: coord) else {
                continue  // No data at this point, keep marching
            }

            // Establish base terrain level from early readings (first 90m)
            if baseTerrainElev == nil && d <= 90.0 {
                baseTerrainElev = terrainElev
            }

            // ===== STRATEGY 1: LOS Intersection =====
            // Line-of-sight altitude at this horizontal distance
            let losAlt = effectiveAltitude + horizontalDist * tanPitch

            let isLOSAbove = losAlt > terrainElev

            if !isLOSAbove && prevLOSAbove && d >= minDistance {
                // LOS crossed below terrain between prevD and d
                // Bisection refinement for precision
                let refined = await bisectionRefineLOS(
                    origin: origin,
                    originAltitude: effectiveAltitude,
                    dEast: dEast,
                    dNorth: dNorth,
                    tanPitch: tanPitch,
                    dLow: prevD,
                    dHigh: d,
                    metersPerDegLat: metersPerDegLat,
                    metersPerDegLon: metersPerDegLon
                )

                let confidence = computeConfidence(
                    distance: refined.distance,
                    gpsAccuracy: gpsAccuracy,
                    headingAccuracy: 5.0,
                    verticalAccuracy: currentVerticalAccuracy
                )

                if confidence > 0.05 {
                    if refined.distance >= 100.0 {
                        // Far-field: prefer the farthest intersection
                        if let existing = farFieldHit {
                            if refined.distance > existing.distance {
                                farFieldHit = (refined.distance, refined.elevation, confidence)
                            }
                        } else {
                            farFieldHit = (refined.distance, refined.elevation, confidence)
                        }
                    } else if nearFieldHit == nil {
                        // Near-field: keep only the first (closest flat-ground hit)
                        nearFieldHit = (refined.distance, refined.elevation, confidence)
                    }
                }
            }

            prevLOSAbove = isLOSAbove
            prevD = d

            // ===== Shared: elevation angle + visibility =====
            let elevAngleRad = atan2(terrainElev - effectiveAltitude, horizontalDist)
            let elevAngleDeg = elevAngleRad * 180.0 / .pi

            let isVisible = elevAngleDeg >= maxElevAngle
            if isVisible {
                maxElevAngle = elevAngleDeg
            }

            // ===== STRATEGY 2: Viewshed Pitch Matching (tightened) =====
            // Only at ≥200m (close range is handled by LiDAR/geometric).
            // Cap SRTM tolerance at 3° to prevent flat-ground false matches.
            if isVisible && d >= 200.0 {
                let angleDiff = abs(elevAngleDeg - cameraPitchDeg)

                let srtmToleranceDeg = atan2(10.0, horizontalDist) * 180.0 / .pi
                let cappedSrtmTolerance = min(srtmToleranceDeg, 3.0)
                let matchTolerance = max(cappedSrtmTolerance + 0.5, 1.0)

                if angleDiff < matchTolerance {
                    let confidence = computeConfidence(
                        distance: d,
                        gpsAccuracy: gpsAccuracy,
                        headingAccuracy: 5.0,
                        verticalAccuracy: currentVerticalAccuracy
                    )

                    if confidence > 0.05 {
                        if let existing = bestViewshedMatch {
                            let isFarther = d > existing.distance + stepSize
                            let isTighterMatch = d >= existing.distance - stepSize && angleDiff < existing.angleDiff
                            if isFarther || isTighterMatch {
                                bestViewshedMatch = (d, terrainElev, angleDiff, confidence)
                            }
                        } else {
                            bestViewshedMatch = (d, terrainElev, angleDiff, confidence)
                        }
                    }
                }
            }

            // ===== STRATEGY 3: FOV Terrain Feature Detection =====
            // Find prominent terrain features (mountains/cliffs) visible within
            // the camera's vertical FOV. The camera center pitch may be looking
            // at flat ground while mountains are visible in the upper frame.
            //
            // Conditions:
            // - ≥200m (skip near-field)
            // - Terrain has risen ≥50m above base level (real feature, not flat)
            // - Elevation angle is within camera FOV bounds
            // - Terrain is visible (not occluded by closer terrain)
            if isVisible && d >= 200.0, let base = baseTerrainElev {
                let terrainRise = terrainElev - base
                let isFeature = terrainRise > 50.0
                let isInFOV = elevAngleDeg >= fovMinPitch && elevAngleDeg <= fovMaxPitch

                if isFeature && isInFOV {
                    let confidence = computeConfidence(
                        distance: d,
                        gpsAccuracy: gpsAccuracy,
                        headingAccuracy: 5.0,
                        verticalAccuracy: currentVerticalAccuracy
                    )

                    if confidence > 0.05 {
                        // Prefer the farthest feature still within camera FOV.
                        // Mountains extend across the FOV; the farthest visible
                        // point on the cliff face is the best range estimate.
                        if let existing = bestTerrainFeature {
                            if d > existing.distance {
                                bestTerrainFeature = (d, terrainElev, elevAngleDeg, confidence)
                            }
                        } else {
                            bestTerrainFeature = (d, terrainElev, elevAngleDeg, confidence)
                        }
                    }
                }
            }
        }

        // ===== RESULT SELECTION =====
        // Priority: terrain feature > far-field viewshed > far-field LOS > near-field
        //
        // Strategy 3 (terrain feature) wins because it handles the critical case
        // where the camera center pitch hits flat ground but mountains are visible
        // in the upper frame — the most common far-ranging scenario.
        let result: (distance: Float, elevation: Double, confidence: Float)?

        if let feature = bestTerrainFeature {
            // FOV terrain feature: mountain/cliff detected within camera FOV
            result = (feature.distance, feature.elevation, feature.confidence)
            Logger.terrain.debug("DEM terrain scan: using FOV terrain feature at \(String(format: "%.1f", feature.distance))m (elev angle=\(String(format: "%.1f", feature.elevAngle))° rise=\(String(format: "%.0f", feature.elevation - (baseTerrainElev ?? 0)))m)")
        } else if let viewshed = bestViewshedMatch, viewshed.distance >= 200.0 {
            // Far-field viewshed match: pitch-matched terrain at distance
            result = (viewshed.distance, viewshed.elevation, viewshed.confidence)
            Logger.terrain.debug("DEM terrain scan: using viewshed match at \(String(format: "%.1f", viewshed.distance))m (angle diff=\(String(format: "%.2f", viewshed.angleDiff))°)")
        } else if let far = farFieldHit {
            // Far-field LOS intersection: standard terrain intersection
            result = far
            Logger.terrain.debug("DEM terrain scan: using far-field LOS at \(String(format: "%.1f", far.distance))m")
        } else if let viewshed = bestViewshedMatch {
            // Near-field viewshed match (200m+ after tightening)
            result = (viewshed.distance, viewshed.elevation, viewshed.confidence)
            Logger.terrain.debug("DEM terrain scan: using near-field viewshed at \(String(format: "%.1f", viewshed.distance))m")
        } else if let near = nearFieldHit {
            // Near-field LOS: flat ground graze (only result available)
            result = near
            Logger.terrain.debug("DEM terrain scan: using near-field LOS at \(String(format: "%.1f", near.distance))m")
        } else {
            return nil
        }

        guard let best = result else { return nil }

        // Compute hit coordinate
        let hitLat = origin.latitude + (dNorth * Double(best.distance)) / metersPerDegLat
        let hitLon = origin.longitude + (dEast * Double(best.distance)) / metersPerDegLon
        let hitCoord = CLLocationCoordinate2D(latitude: hitLat, longitude: hitLon)

        let estimate = DEMRaycastEstimate(
            distanceMeters: best.distance,
            confidence: best.confidence,
            terrainElevation: Float(best.elevation),
            headingDeg: headingDeg,
            gpsAccuracy: gpsAccuracy,
            hitCoordinate: hitCoord
        )

        Logger.terrain.debug("DEM terrain scan: \(String(format: "%.1f", best.distance))m heading=\(String(format: "%.1f", headingDeg))° terrainElev=\(String(format: "%.1f", best.elevation))m conf=\(String(format: "%.2f", best.confidence))")

        return estimate
    }

    // MARK: - Bisection Refinement

    private struct RefinementResult {
        let distance: Float
        let elevation: Double
    }

    /// Refine the LOS-terrain intersection between dLow (LOS above terrain)
    /// and dHigh (LOS below terrain) using horizontal distance parameterization.
    private func bisectionRefineLOS(
        origin: CLLocationCoordinate2D,
        originAltitude: Double,
        dEast: Double,
        dNorth: Double,
        tanPitch: Double,
        dLow: Float,
        dHigh: Float,
        metersPerDegLat: Double,
        metersPerDegLon: Double
    ) async -> RefinementResult {

        var lo = dLow
        var hi = dHigh
        var lastElev = 0.0

        for _ in 0..<bisectionIterations {
            let mid = (lo + hi) / 2.0
            let horizontalDist = Double(mid)

            let eastM = dEast * horizontalDist
            let northM = dNorth * horizontalDist
            let losAlt = originAltitude + horizontalDist * tanPitch

            let lat = origin.latitude + northM / metersPerDegLat
            let lon = origin.longitude + eastM / metersPerDegLon

            let coord = CLLocationCoordinate2D(latitude: lat, longitude: lon)
            if let terrainElev = await tileCache.elevation(at: coord) {
                lastElev = terrainElev
                if losAlt > terrainElev {
                    lo = mid  // LOS still above terrain
                } else {
                    hi = mid  // LOS below terrain
                }
            } else {
                // No data — assume intersection is closer to last known point
                hi = mid
            }
        }

        return RefinementResult(distance: (lo + hi) / 2.0, elevation: lastElev)
    }

    // MARK: - Confidence

    /// Compute confidence based on GPS accuracy, heading accuracy, and vertical accuracy.
    ///
    /// IMPORTANT: Distance-dependent weighting is NOT applied here. It's handled
    /// by `DepthSourceConfidence.demRaycast()` in the fusion pipeline. Applying it
    /// here would double-penalize distance, crushing DEM weight at long range.
    /// The old formula `distanceFactor = 1/(1 + d/500)` × gpsFactor × altFactor ×
    /// headingFactor produced confidence ~0.10 at 1600m, which when multiplied by
    /// the confidence curve's ~0.72 gave final weight ~0.07 — far too low for
    /// what is actually the MOST accurate source at that range.
    private func computeConfidence(
        distance: Float,
        gpsAccuracy: Float,
        headingAccuracy: Float,
        verticalAccuracy: Float = -1
    ) -> Float {
        // GPS accuracy factor (horizontal position)
        let gpsFactor: Float
        if gpsAccuracy < 5.0 {
            gpsFactor = 0.90
        } else if gpsAccuracy < 10.0 {
            gpsFactor = 0.75
        } else if gpsAccuracy < 15.0 {
            gpsFactor = 0.55
        } else if gpsAccuracy < 30.0 {
            gpsFactor = 0.35
        } else {
            gpsFactor = 0.15
        }

        // Vertical accuracy factor (altitude precision)
        // Barometric altimeter gives 1-5m, GPS-only gives 10-30m.
        // Better altitude = more precise ray origin = higher confidence.
        let altFactor: Float
        if verticalAccuracy > 0 && verticalAccuracy < 3.0 {
            altFactor = 1.0       // Barometric: excellent
        } else if verticalAccuracy > 0 && verticalAccuracy < 10.0 {
            altFactor = 0.90      // Good barometric or excellent GPS
        } else if verticalAccuracy > 0 && verticalAccuracy < 20.0 {
            altFactor = 0.75      // Typical GPS vertical
        } else {
            altFactor = 0.65      // Poor or unknown
        }

        // Heading accuracy factor
        // At 100m with 5° error: lateral error = 100*tan(5°) ≈ 8.7m
        // At 500m with 5° error: lateral error ≈ 43m
        let headingFactor: Float
        if headingAccuracy < 5.0 {
            headingFactor = 1.0
        } else if headingAccuracy < 10.0 {
            headingFactor = 0.80
        } else {
            headingFactor = 0.55
        }

        return gpsFactor * altFactor * headingFactor
    }

    // MARK: - Reset

    func reset() {
        lastEstimate = nil
        lastHeading = -999
        lastPitch = -999
        lastQueryTime = 0
    }
}
