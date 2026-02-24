//
//  DEMRaycastEstimator.swift
//  Rangefinder
//
//  Terrain-aware range estimation via DEM ray-casting.
//
//  Casts a ray from the device's GPS position + altitude using IMU
//  pitch and true-north heading into the SRTM elevation model.
//  Where the ray intersects the terrain surface = range to target.
//
//  This is the most accurate method for medium-to-long range targets
//  on terrain (50-2000m), especially on sloped ground where the
//  geometric ground-plane model fails catastrophically.
//
//  Algorithm:
//  1. Ray origin: GPS (lat, lon, altitude)
//  2. Ray direction: heading (yaw) + pitch from InclinationManager
//  3. Convert to local ENU (East-North-Up) coordinate frame
//  4. March ray in 30m steps, querying SRTM elevation at each step
//  5. When ray drops below terrain → bisection refinement
//  6. Return horizontal distance to intersection
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

    /// Vertical accuracy for current estimate (set in estimate(), used in marchRay()).
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
        // Looking UP at mountains is valid — the ray marches forward and upward
        // until it intersects the rising terrain slope. The ray math handles this
        // correctly: dUp becomes positive for upward pitch, and the ray intersects
        // the mountain when terrain elevation exceeds ray altitude.
        // Only reject rays pointing steeply upward (>30°) where hitting terrain
        // is unlikely (aiming at sky). Level rays (0°) are fine — if terrain
        // rises ahead (mountain), the ray will intersect it; if not, marchRay
        // returns nil naturally after maxRayDistance.
        let pitchDegrees = pitchRadians * 180.0 / .pi
        guard pitchDegrees < 30.0 else { return nil }  // >30° above horizontal: aiming at sky

        // Need valid GPS
        guard horizontalAccuracy > 0, horizontalAccuracy < 100 else { return nil }

        // Store vertical accuracy for confidence computation in marchRay
        currentVerticalAccuracy = Float(verticalAccuracy)

        // Compute ray direction in ENU (East-North-Up) frame
        let pitchBelowHorizontal = -pitchRadians  // positive angle below horizon
        let headingRad = headingDegrees * .pi / 180.0

        // Horizontal distance per unit ray length
        let cosPitch = cos(pitchBelowHorizontal)
        let sinPitch = sin(pitchBelowHorizontal)

        // Ray direction components in ENU:
        // East  = sin(heading) * cos(pitch)
        // North = cos(heading) * cos(pitch)
        // Up    = -sin(pitch)  (ray goes down)
        let dEast = sin(headingRad) * cosPitch
        let dNorth = cos(headingRad) * cosPitch
        let dUp = -sinPitch

        // Pre-fetch elevation corridor if no SRTM tile is available.
        // Without this, each of the ~66 ray march steps would make an individual
        // HTTP request to USGS EPQS, taking 15-30 seconds total. Pre-fetching
        // grabs all unique grid points in parallel (~20 concurrent requests)
        // and caches them so the ray march hits only local cache.
        if await !tileCache.hasTile(for: coordinate) {
            let fetchCount = await tileCache.prefetchCorridor(
                origin: coordinate,
                dEast: dEast,
                dNorth: dNorth,
                maxDistance: maxRayDistance,
                stepSize: stepSize
            )
            if fetchCount == 0 {
                // All points were already cached — this is a heading/pitch update
                // on the same general corridor. The ray march will use cached data.
            }
        }

        // March ray and find terrain intersection
        let result = await marchRay(
            origin: coordinate,
            originAltitude: altitude,
            dEast: dEast,
            dNorth: dNorth,
            dUp: dUp,
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

    // MARK: - Ray Marching

    /// March the ray through the terrain, checking elevation at each step.
    ///
    /// KEY DESIGN: We look for the FARTHEST significant terrain intersection, not
    /// the first. When the user is looking at distant mountains over nearby flat
    /// ground, the ray may graze flat terrain 30-50m ahead before hitting the
    /// mountain at 1600m. The old single-intersection approach would stop at 30m.
    ///
    /// We detect "significant" terrain by tracking elevation gain. A mountain
    /// intersection (terrain rising 50+ meters above the ray's starting plane)
    /// always wins over a flat-ground graze.
    private func marchRay(
        origin: CLLocationCoordinate2D,
        originAltitude: Double,
        dEast: Double,
        dNorth: Double,
        dUp: Double,
        headingDeg: Float,
        gpsAccuracy: Float
    ) async -> DEMRaycastEstimate? {

        // Meters per degree at this latitude
        let metersPerDegLat = 111_320.0  // roughly constant
        let metersPerDegLon = 111_320.0 * cos(origin.latitude * .pi / 180.0)

        guard metersPerDegLon > 1000 else { return nil }  // Sanity check (near poles)

        // --- ALTITUDE CORRECTION ---
        // GPS/barometric altitude can be 5-30m off from SRTM terrain level.
        // If the observer appears to be below or at SRTM terrain, the ray
        // immediately detects a false intersection. Fix: query terrain at
        // origin and ensure we start at least 2m above it (eye height).
        var effectiveAltitude = originAltitude
        if let originTerrainElev = await tileCache.elevation(at: origin) {
            let aboveTerrain = originAltitude - originTerrainElev
            if aboveTerrain < 2.0 {
                // Observer altitude is at or below SRTM terrain — altitude error.
                // Snap to terrain + 2m eye height.
                effectiveAltitude = originTerrainElev + 2.0
                Logger.terrain.debug("DEM altitude correction: GPS=\(String(format: "%.1f", originAltitude))m SRTM=\(String(format: "%.1f", originTerrainElev))m → using \(String(format: "%.1f", effectiveAltitude))m")
            }
        }

        var prevAboveTerrain = true
        var prevT: Float = 0
        let numSteps = Int(maxRayDistance / stepSize)

        // Track the best (farthest significant) intersection
        var bestIntersection: (distance: Float, elevation: Double, confidence: Float)?

        // Track terrain along the ray for significance detection
        let originTerrainBase = await tileCache.elevation(at: origin) ?? effectiveAltitude

        for step in 1...numSteps {
            let t = Float(step) * stepSize
            let horizontalDist = Double(t)

            // Position along ray in meters (ENU)
            let eastM = dEast * horizontalDist
            let northM = dNorth * horizontalDist
            let upM = dUp * horizontalDist

            // Convert ENU offset to lat/lon
            let lat = origin.latitude + northM / metersPerDegLat
            let lon = origin.longitude + eastM / metersPerDegLon
            let rayAlt = effectiveAltitude + upM

            // Query terrain elevation
            let coord = CLLocationCoordinate2D(latitude: lat, longitude: lon)
            guard let terrainElev = await tileCache.elevation(at: coord) else {
                continue  // No data at this point, keep marching
            }

            let isAboveTerrain = rayAlt > terrainElev

            if !isAboveTerrain && prevAboveTerrain && t >= minDistance {
                // Ray crossed below terrain between prevT and t
                // Bisection refinement for precision
                let refinedResult = await bisectionRefine(
                    origin: origin,
                    originAltitude: effectiveAltitude,
                    dEast: dEast,
                    dNorth: dNorth,
                    dUp: dUp,
                    tLow: prevT,
                    tHigh: t,
                    metersPerDegLat: metersPerDegLat,
                    metersPerDegLon: metersPerDegLon
                )

                let finalDist = refinedResult.distance
                let finalElev = refinedResult.elevation

                // Compute confidence
                let confidence = computeConfidence(
                    distance: finalDist,
                    gpsAccuracy: gpsAccuracy,
                    headingAccuracy: 5.0,
                    verticalAccuracy: currentVerticalAccuracy
                )

                guard confidence > 0.05 else {
                    prevAboveTerrain = isAboveTerrain
                    prevT = t
                    continue
                }

                // Is this a "significant" terrain feature?
                // Terrain that rises well above the observer's base elevation
                // (mountains, cliffs) is more significant than flat ground grazes.
                let terrainRise = finalElev - originTerrainBase
                let isSignificantTerrain = terrainRise > 30.0  // 30m+ above origin = mountain/cliff

                if let existing = bestIntersection {
                    // Prefer significant terrain over flat ground
                    let existingRise = existing.elevation - originTerrainBase
                    let existingSignificant = existingRise > 30.0

                    if isSignificantTerrain && !existingSignificant {
                        // New hit is a mountain, old was flat ground → replace
                        bestIntersection = (finalDist, finalElev, confidence)
                    } else if isSignificantTerrain && existingSignificant {
                        // Both significant → keep the farther one (user is looking at distant mountain)
                        if finalDist > existing.distance {
                            bestIntersection = (finalDist, finalElev, confidence)
                        }
                    }
                    // If new hit is flat ground and old is mountain → keep mountain
                    // If both flat ground → keep first one
                } else {
                    bestIntersection = (finalDist, finalElev, confidence)
                }
            }

            prevAboveTerrain = isAboveTerrain
            prevT = t
        }

        // Return the best intersection found
        guard let best = bestIntersection else {
            return nil  // No intersection within maxRayDistance
        }

        // Compute hit coordinate from the best intersection
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

        Logger.terrain.debug("DEM raycast: \(String(format: "%.1f", best.distance))m heading=\(String(format: "%.1f", headingDeg))° terrainElev=\(String(format: "%.1f", best.elevation))m conf=\(String(format: "%.2f", best.confidence))")

        return estimate
    }

    // MARK: - Bisection Refinement

    private struct RefinementResult {
        let distance: Float
        let elevation: Double
    }

    /// Refine the intersection point between tLow (above terrain) and tHigh (below terrain).
    private func bisectionRefine(
        origin: CLLocationCoordinate2D,
        originAltitude: Double,
        dEast: Double,
        dNorth: Double,
        dUp: Double,
        tLow: Float,
        tHigh: Float,
        metersPerDegLat: Double,
        metersPerDegLon: Double
    ) async -> RefinementResult {

        var lo = tLow
        var hi = tHigh
        var lastElev = 0.0

        for _ in 0..<bisectionIterations {
            let mid = (lo + hi) / 2.0
            let horizontalDist = Double(mid)

            let eastM = dEast * horizontalDist
            let northM = dNorth * horizontalDist
            let upM = dUp * horizontalDist

            let lat = origin.latitude + northM / metersPerDegLat
            let lon = origin.longitude + eastM / metersPerDegLon
            let rayAlt = originAltitude + upM

            let coord = CLLocationCoordinate2D(latitude: lat, longitude: lon)
            if let terrainElev = await tileCache.elevation(at: coord) {
                lastElev = terrainElev
                if rayAlt > terrainElev {
                    lo = mid  // Still above terrain
                } else {
                    hi = mid  // Below terrain
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
