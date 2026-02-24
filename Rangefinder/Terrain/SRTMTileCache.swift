//
//  SRTMTileCache.swift
//  Rangefinder
//
//  SRTM HGT tile parser + USGS EPQS online elevation fallback.
//
//  SRTM (Shuttle Radar Topography Mission) data provides ~30m resolution
//  global elevation data. HGT files are 3601×3601 signed Int16 big-endian,
//  covering 1° × 1° tiles.
//
//  When no local HGT tile is available, falls back to USGS EPQS
//  (Elevation Point Query Service) which provides 10m NED resolution
//  for the US, free, no API key required.
//

import Foundation
import CoreLocation
import os

/// Thread-safe elevation data provider with LRU tile cache.
actor SRTMTileCache {

    // MARK: - Types

    /// A loaded SRTM tile with its elevation grid.
    private struct Tile {
        let key: String          // e.g. "N37W122"
        let data: Data           // Raw HGT data (3601×3601 × 2 bytes)
        let baseLat: Int         // South edge latitude (integer degrees)
        let baseLon: Int         // West edge longitude (integer degrees)
        var lastAccess: Date     // For LRU eviction
    }

    // MARK: - Constants

    /// SRTM 1-arc-second tiles are 3601 × 3601 posts.
    private static let tileSize = 3601
    /// Expected file size: 3601 * 3601 * 2 bytes = 25,934,402
    private static let expectedFileSize = tileSize * tileSize * 2
    /// Void/no-data marker in SRTM
    private static let voidValue: Int16 = -32768

    // MARK: - Cache

    private var tiles: [String: Tile] = [:]
    private let maxCachedTiles = 3

    /// Cache for USGS EPQS results (keyed by rounded lat/lon).
    private var epqsCache: [String: Double] = [:]

    // MARK: - Public API

    /// Get elevation at a coordinate, using SRTM tiles if available,
    /// falling back to USGS EPQS API.
    func elevation(at coord: CLLocationCoordinate2D) async -> Double? {
        // Try SRTM tile first
        if let hgtElevation = elevationFromHGT(at: coord) {
            return hgtElevation
        }

        // Fall back to USGS EPQS (online)
        return await elevationFromEPQS(at: coord)
    }

    // MARK: - SRTM HGT

    /// Query elevation from a loaded/loadable HGT tile.
    private func elevationFromHGT(at coord: CLLocationCoordinate2D) -> Double? {
        let key = tileKey(for: coord)

        // Check cache
        if var tile = tiles[key] {
            tile.lastAccess = Date()
            tiles[key] = tile
            return sampleTile(tile, at: coord)
        }

        // Try to load from bundle or Documents
        if let tile = loadTile(key: key) {
            return sampleTile(tile, at: coord)
        }

        return nil
    }

    /// Compute the SRTM tile key for a coordinate.
    /// Convention: tile N37W122 covers lat 37→38, lon -122→-121.
    private func tileKey(for coord: CLLocationCoordinate2D) -> String {
        let lat = Int(floor(coord.latitude))
        let lon = Int(floor(coord.longitude))
        let latPrefix = lat >= 0 ? "N" : "S"
        let lonPrefix = lon >= 0 ? "E" : "W"
        let latAbs = abs(lat)
        let lonAbs = abs(lon)
        return String(format: "%@%02d%@%03d", latPrefix, latAbs, lonPrefix, lonAbs)
    }

    /// Load an HGT tile from app bundle or Documents directory.
    private func loadTile(key: String) -> Tile? {
        let filename = "\(key).hgt"

        // Check app bundle first
        var data: Data?
        if let bundlePath = Bundle.main.path(forResource: key, ofType: "hgt") {
            data = try? Data(contentsOf: URL(fileURLWithPath: bundlePath))
        }

        // Check Documents directory
        if data == nil {
            let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first
            if let fileURL = documentsURL?.appendingPathComponent("SRTM/\(filename)") {
                data = try? Data(contentsOf: fileURL)
            }
        }

        guard let tileData = data else { return nil }

        // Validate file size
        guard tileData.count == Self.expectedFileSize else {
            Logger.terrain.warning("HGT file \(key) has wrong size: \(tileData.count) (expected \(Self.expectedFileSize))")
            return nil
        }

        // Parse lat/lon from key
        let latSign: Int = key.hasPrefix("N") ? 1 : -1
        let lonSign: Int = key.contains("E") ? 1 : -1
        let latStr = key.dropFirst(1).prefix(2)
        let lonPart = key.contains("E") ? key.components(separatedBy: "E").last! : key.components(separatedBy: "W").last!
        guard let baseLat = Int(latStr), let baseLon = Int(lonPart) else { return nil }

        let tile = Tile(
            key: key,
            data: tileData,
            baseLat: latSign * baseLat,
            baseLon: lonSign * baseLon,
            lastAccess: Date()
        )

        // Cache with LRU eviction
        if tiles.count >= maxCachedTiles {
            evictLRU()
        }
        tiles[key] = tile

        Logger.terrain.info("Loaded SRTM tile: \(key)")
        return tile
    }

    /// Sample elevation from a loaded tile with bilinear interpolation.
    private func sampleTile(_ tile: Tile, at coord: CLLocationCoordinate2D) -> Double? {
        // Fractional position within the tile (0.0 to 1.0)
        let fracLat = coord.latitude - Double(tile.baseLat)
        let fracLon = coord.longitude - Double(tile.baseLon)

        guard fracLat >= 0, fracLat <= 1.0, fracLon >= 0, fracLon <= 1.0 else { return nil }

        // Convert to grid indices
        // Row 0 = north edge (baseLat + 1), row 3600 = south edge (baseLat)
        let rowF = (1.0 - fracLat) * Double(Self.tileSize - 1)
        let colF = fracLon * Double(Self.tileSize - 1)

        let row0 = min(Self.tileSize - 2, Int(floor(rowF)))
        let col0 = min(Self.tileSize - 2, Int(floor(colF)))
        let row1 = row0 + 1
        let col1 = col0 + 1

        let rowFrac = Float(rowF - Double(row0))
        let colFrac = Float(colF - Double(col0))

        // Read 4 corner elevations
        guard let e00 = readElevation(tile: tile, row: row0, col: col0),
              let e01 = readElevation(tile: tile, row: row0, col: col1),
              let e10 = readElevation(tile: tile, row: row1, col: col0),
              let e11 = readElevation(tile: tile, row: row1, col: col1) else {
            return nil
        }

        // Bilinear interpolation
        let top = e00 * (1 - colFrac) + e01 * colFrac
        let bottom = e10 * (1 - colFrac) + e11 * colFrac
        let elevation = top * (1 - rowFrac) + bottom * rowFrac

        return Double(elevation)
    }

    /// Read a single elevation value from the tile at (row, col).
    private func readElevation(tile: Tile, row: Int, col: Int) -> Float? {
        let index = (row * Self.tileSize + col) * 2
        guard index + 1 < tile.data.count else { return nil }

        // Big-endian Int16
        let hi = Int16(tile.data[index])
        let lo = Int16(tile.data[index + 1])
        let value = (hi << 8) | (lo & 0xFF)

        guard value != Self.voidValue else { return nil }
        return Float(value)
    }

    /// Evict the least recently used tile.
    private func evictLRU() {
        guard let oldest = tiles.min(by: { $0.value.lastAccess < $1.value.lastAccess }) else { return }
        tiles.removeValue(forKey: oldest.key)
        Logger.terrain.debug("Evicted SRTM tile: \(oldest.key)")
    }

    // MARK: - USGS EPQS Fallback

    /// Query USGS Elevation Point Query Service for elevation.
    /// Free, no API key, 10m NED resolution (US coverage).
    private func elevationFromEPQS(at coord: CLLocationCoordinate2D) async -> Double? {
        // Check cache (rounded to ~100m grid)
        let cacheKey = String(format: "%.3f,%.3f", coord.latitude, coord.longitude)
        if let cached = epqsCache[cacheKey] {
            return cached
        }

        let urlString = String(format:
            "https://epqs.nationalmap.gov/v1/json?x=%.6f&y=%.6f&wkid=4326&units=Meters",
            coord.longitude, coord.latitude
        )

        guard let url = URL(string: urlString) else { return nil }

        do {
            let (data, response) = try await URLSession.shared.data(from: url)

            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                Logger.terrain.warning("EPQS request failed for \(cacheKey)")
                return nil
            }

            // Parse JSON response: {"value": 123.45, ...}
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let value = json["value"] as? Double {
                // -1000000 is EPQS's "no data" marker
                guard value > -999999 else { return nil }
                epqsCache[cacheKey] = value
                Logger.terrain.debug("EPQS elevation at \(cacheKey): \(String(format: "%.1f", value))m")
                return value
            }
        } catch {
            Logger.terrain.error("EPQS request error: \(error.localizedDescription)")
        }

        return nil
    }

    // MARK: - Corridor Pre-fetch for Ray Marching

    /// Pre-fetch elevation data along a ray corridor using concurrent EPQS requests.
    /// This is essential when no SRTM tiles are available — instead of 66 sequential
    /// HTTP requests during ray marching, we pre-fetch all unique grid points in parallel.
    ///
    /// - Parameters:
    ///   - origin: Starting coordinate
    ///   - dEast: East component of ray direction (per meter)
    ///   - dNorth: North component of ray direction (per meter)
    ///   - maxDistance: Maximum distance to pre-fetch (meters)
    ///   - stepSize: Step size along the ray (meters)
    /// - Returns: Number of points successfully fetched
    func prefetchCorridor(
        origin: CLLocationCoordinate2D,
        dEast: Double,
        dNorth: Double,
        maxDistance: Float,
        stepSize: Float
    ) async -> Int {

        let metersPerDegLat = 111_320.0
        let metersPerDegLon = 111_320.0 * cos(origin.latitude * .pi / 180.0)
        guard metersPerDegLon > 1000 else { return 0 }

        // Collect unique EPQS grid points along the ray
        var uniqueKeys: Set<String> = []
        var pointsToFetch: [(key: String, coord: CLLocationCoordinate2D)] = []

        let numSteps = Int(maxDistance / stepSize)
        for step in 0...numSteps {
            let t = Double(Float(step) * stepSize)
            let lat = origin.latitude + (dNorth * t) / metersPerDegLat
            let lon = origin.longitude + (dEast * t) / metersPerDegLon

            let cacheKey = String(format: "%.3f,%.3f", lat, lon)
            if !uniqueKeys.contains(cacheKey) && epqsCache[cacheKey] == nil {
                uniqueKeys.insert(cacheKey)
                pointsToFetch.append((key: cacheKey, coord: CLLocationCoordinate2D(latitude: lat, longitude: lon)))
            }
        }

        guard !pointsToFetch.isEmpty else {
            // All points already cached
            return 0
        }

        Logger.terrain.info("EPQS corridor pre-fetch: \(pointsToFetch.count) unique points for \(String(format: "%.0f", maxDistance))m ray")

        // Fetch all points concurrently with a concurrency limit
        let maxConcurrent = 8
        var fetched = 0

        // Process in batches to avoid overwhelming the server
        for batchStart in stride(from: 0, to: pointsToFetch.count, by: maxConcurrent) {
            let batchEnd = min(batchStart + maxConcurrent, pointsToFetch.count)
            let batch = Array(pointsToFetch[batchStart..<batchEnd])

            await withTaskGroup(of: (String, Double?).self) { group in
                for point in batch {
                    group.addTask { [self] in
                        let elevation = await self.elevationFromEPQS(at: point.coord)
                        return (point.key, elevation)
                    }
                }

                for await (key, elevation) in group {
                    if elevation != nil {
                        fetched += 1
                    }
                    // Note: elevationFromEPQS already caches successful results
                }
            }
        }

        Logger.terrain.info("EPQS corridor pre-fetch complete: \(fetched)/\(pointsToFetch.count) points")
        return fetched
    }

    // MARK: - Cache Management

    /// Clear all cached tiles and EPQS results.
    func clearCache() {
        tiles.removeAll()
        epqsCache.removeAll()
    }

    /// Check if a tile is available (either cached, in bundle, or in Documents).
    func hasTile(for coord: CLLocationCoordinate2D) -> Bool {
        let key = tileKey(for: coord)
        if tiles[key] != nil { return true }

        // Check bundle
        if Bundle.main.path(forResource: key, ofType: "hgt") != nil { return true }

        // Check Documents
        let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first
        if let fileURL = documentsURL?.appendingPathComponent("SRTM/\(key).hgt") {
            return FileManager.default.fileExists(atPath: fileURL.path)
        }

        return false
    }
}
