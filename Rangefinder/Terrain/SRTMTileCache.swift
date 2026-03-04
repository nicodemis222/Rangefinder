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

    // MARK: - SRTM Tile Auto-Download

    /// Track which tiles are currently being downloaded to avoid duplicate requests.
    private var downloadingTiles: Set<String> = []

    /// Track tiles that failed to download (don't retry until cleared).
    private var failedTiles: Set<String> = []

    /// Whether auto-download has been triggered for the current session.
    private(set) var hasAttemptedDownload: Bool = false

    /// Download an SRTM HGT tile for the given coordinate from a public source.
    ///
    /// Uses the USGS SRTM GL1 data via a public CDN. Files are ~26MB compressed,
    /// stored permanently in the Documents/SRTM directory for offline use.
    ///
    /// Returns true if the tile was successfully downloaded and is now available.
    @discardableResult
    func downloadTile(for coord: CLLocationCoordinate2D) async -> Bool {
        let key = tileKey(for: coord)
        hasAttemptedDownload = true

        // Already cached or loading
        guard tiles[key] == nil else { return true }
        guard !downloadingTiles.contains(key) else { return false }
        guard !failedTiles.contains(key) else { return false }

        // Already on disk?
        if hasTileOnDisk(key: key) {
            // loadTile will cache it
            if loadTile(key: key) != nil {
                return true
            }
        }

        downloadingTiles.insert(key)
        defer { downloadingTiles.remove(key) }

        Logger.terrain.info("Downloading SRTM tile: \(key) (~26MB)")

        // Try multiple sources in order of reliability:
        // 1. USGS EarthExplorer CDN (e3 = SRTM GL1 v003)
        // 2. OpenData AWS mirror
        let sources = [
            "https://e4ftl01.cr.usgs.gov/MEASURES/SRTMGL1.003/2000.02.11/\(key).SRTMGL1.hgt.zip",
            "https://elevation-tiles-prod.s3.amazonaws.com/skadi/\(key.prefix(3))/\(key).hgt.gz",
        ]

        for sourceURL in sources {
            if let data = await downloadAndExtract(urlString: sourceURL, key: key) {
                // Validate
                guard data.count == Self.expectedFileSize else {
                    Logger.terrain.warning("Downloaded \(key) has wrong size: \(data.count)")
                    continue
                }

                // Save to Documents/SRTM for offline use
                if saveToDisk(data: data, key: key) {
                    // Load into cache
                    if loadTile(key: key) != nil {
                        Logger.terrain.info("SRTM tile \(key) downloaded and cached")
                        return true
                    }
                }
            }
        }

        // All sources failed
        failedTiles.insert(key)
        Logger.terrain.warning("Failed to download SRTM tile: \(key)")
        return false
    }

    /// Download and extract an SRTM tile from a URL (handles .zip and .gz).
    private func downloadAndExtract(urlString: String, key: String) async -> Data? {
        guard let url = URL(string: urlString) else { return nil }

        do {
            let (data, response) = try await URLSession.shared.data(from: url)

            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                let status = (response as? HTTPURLResponse)?.statusCode ?? -1
                Logger.terrain.debug("SRTM download HTTP \(status) from \(urlString)")
                return nil
            }

            // Check if the data is compressed
            if urlString.hasSuffix(".gz") {
                // gzip decompression
                return decompressGzip(data)
            } else if urlString.hasSuffix(".zip") {
                // ZIP extraction — find the .hgt file inside
                return extractHGTFromZip(data, key: key)
            } else {
                // Assume raw HGT
                return data
            }
        } catch {
            Logger.terrain.debug("SRTM download error: \(error.localizedDescription)")
            return nil
        }
    }

    /// Decompress gzip data using Foundation's built-in decompression.
    private func decompressGzip(_ data: Data) -> Data? {
        // Use NSData's decompression for gzip
        do {
            let decompressed = try (data as NSData).decompressed(using: .zlib)
            return decompressed as Data
        } catch {
            // Try raw inflate (skip gzip header)
            // gzip files have a 10-byte header, try skipping it
            if data.count > 18 && data[0] == 0x1f && data[1] == 0x8b {
                // Find the start of the deflate stream (after gzip header)
                var offset = 10
                let flags = data[3]
                if flags & 0x04 != 0 { // FEXTRA
                    let xlen = Int(data[10]) | (Int(data[11]) << 8)
                    offset += 2 + xlen
                }
                if flags & 0x08 != 0 { // FNAME - skip null-terminated string
                    while offset < data.count && data[offset] != 0 { offset += 1 }
                    offset += 1
                }
                if flags & 0x10 != 0 { // FCOMMENT - skip null-terminated string
                    while offset < data.count && data[offset] != 0 { offset += 1 }
                    offset += 1
                }
                if flags & 0x02 != 0 { offset += 2 } // FHCRC

                if offset < data.count - 8 {
                    let deflateData = data[offset..<(data.count - 8)]
                    do {
                        let decompressed = try (deflateData as NSData).decompressed(using: .zlib)
                        return decompressed as Data
                    } catch {
                        Logger.terrain.debug("Gzip inflate failed: \(error.localizedDescription)")
                    }
                }
            }
            Logger.terrain.debug("Gzip decompression failed: \(error.localizedDescription)")
            return nil
        }
    }

    /// Extract the HGT file from a ZIP archive.
    /// Uses a minimal ZIP parser (no third-party dependencies).
    private func extractHGTFromZip(_ zipData: Data, key: String) -> Data? {
        // Simple ZIP file parser: find local file headers, look for .hgt file
        let targetName = "\(key).hgt"
        var offset = 0

        while offset + 30 < zipData.count {
            // Local file header signature: PK\x03\x04
            guard zipData[offset] == 0x50,
                  zipData[offset + 1] == 0x4B,
                  zipData[offset + 2] == 0x03,
                  zipData[offset + 3] == 0x04 else {
                break
            }

            let compressionMethod = UInt16(zipData[offset + 8]) | (UInt16(zipData[offset + 9]) << 8)
            let compressedSize = Int(UInt32(zipData[offset + 18]) | (UInt32(zipData[offset + 19]) << 8) |
                                     (UInt32(zipData[offset + 20]) << 16) | (UInt32(zipData[offset + 21]) << 24))
            let uncompressedSize = Int(UInt32(zipData[offset + 22]) | (UInt32(zipData[offset + 23]) << 8) |
                                       (UInt32(zipData[offset + 24]) << 16) | (UInt32(zipData[offset + 25]) << 24))
            let fileNameLen = Int(UInt16(zipData[offset + 26]) | (UInt16(zipData[offset + 27]) << 8))
            let extraLen = Int(UInt16(zipData[offset + 28]) | (UInt16(zipData[offset + 29]) << 8))

            let fileNameStart = offset + 30
            let fileNameEnd = fileNameStart + fileNameLen
            guard fileNameEnd <= zipData.count else { break }

            let fileName = String(data: zipData[fileNameStart..<fileNameEnd], encoding: .utf8) ?? ""
            let dataStart = fileNameEnd + extraLen

            if fileName.hasSuffix(".hgt") || fileName.contains(targetName) {
                guard dataStart + compressedSize <= zipData.count else { break }
                let fileData = zipData[dataStart..<(dataStart + compressedSize)]

                if compressionMethod == 0 {
                    // Stored (no compression)
                    return Data(fileData)
                } else if compressionMethod == 8 {
                    // Deflate
                    do {
                        let decompressed = try (fileData as NSData).decompressed(using: .zlib)
                        let result = decompressed as Data
                        if result.count == uncompressedSize || result.count == Self.expectedFileSize {
                            return result
                        }
                    } catch {
                        Logger.terrain.debug("ZIP deflate failed: \(error.localizedDescription)")
                    }
                }
            }

            offset = dataStart + compressedSize
        }

        Logger.terrain.debug("HGT file not found in ZIP archive")
        return nil
    }

    /// Save HGT data to Documents/SRTM directory for persistent offline access.
    private func saveToDisk(data: Data, key: String) -> Bool {
        guard let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else {
            return false
        }

        let srtmDir = documentsURL.appendingPathComponent("SRTM")

        // Create directory if needed
        do {
            try FileManager.default.createDirectory(at: srtmDir, withIntermediateDirectories: true)
        } catch {
            Logger.terrain.error("Failed to create SRTM directory: \(error.localizedDescription)")
            return false
        }

        let fileURL = srtmDir.appendingPathComponent("\(key).hgt")

        do {
            try data.write(to: fileURL)
            Logger.terrain.info("Saved SRTM tile to disk: \(key) (\(data.count) bytes)")
            return true
        } catch {
            Logger.terrain.error("Failed to save SRTM tile: \(error.localizedDescription)")
            return false
        }
    }

    /// Check if a tile file exists on disk (without loading it).
    private func hasTileOnDisk(key: String) -> Bool {
        // Check bundle
        if Bundle.main.path(forResource: key, ofType: "hgt") != nil { return true }

        // Check Documents
        let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first
        if let fileURL = documentsURL?.appendingPathComponent("SRTM/\(key).hgt") {
            return FileManager.default.fileExists(atPath: fileURL.path)
        }

        return false
    }

    /// Reset download failure tracking (e.g., when connectivity changes).
    func resetDownloadFailures() {
        failedTiles.removeAll()
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
        return hasTileOnDisk(key: key)
    }

    /// Get the tile key for a coordinate (exposed for download UI).
    func tileKeyForCoordinate(_ coord: CLLocationCoordinate2D) -> String {
        return tileKey(for: coord)
    }
}
