//
//  SRTMRegionManager.swift
//  Rangefinder
//
//  SRTM tile download manager — downloads 1-arc-second elevation tiles
//  from AWS S3 (elevation-tiles-prod) organized by US region.
//
//  Tiles are gzip-compressed HGT files (~7-12MB each), decompressed to
//  25,934,402 bytes (3601×3601 × Int16 big-endian). Stored in
//  Documents/SRTM/ for use by SRTMTileCache.
//

import Foundation
import CoreLocation
import Compression
import os

@MainActor
class SRTMRegionManager: ObservableObject {

    // MARK: - Types

    struct Region: Identifiable {
        let id: String
        let name: String
        let description: String
        let latRange: ClosedRange<Int>
        let lonRange: ClosedRange<Int>   // Negative for west

        /// All tile keys in this region's bounding box.
        var tiles: [String] {
            var keys: [String] = []
            for lat in latRange {
                for lon in lonRange {
                    let latPrefix = lat >= 0 ? "N" : "S"
                    let lonPrefix = lon >= 0 ? "E" : "W"
                    keys.append(String(format: "%@%02d%@%03d", latPrefix, abs(lat), lonPrefix, abs(lon)))
                }
            }
            return keys
        }
    }

    struct RegionState {
        var totalTiles: Int
        var downloadedTiles: Int
        var isDownloading: Bool = false
        var error: String?
        var sizeOnDiskMB: Double = 0

        var isComplete: Bool { downloadedTiles >= totalTiles && totalTiles > 0 }
        var progress: Double {
            guard totalTiles > 0 else { return 0 }
            return Double(downloadedTiles) / Double(totalTiles)
        }
    }

    // MARK: - Published State

    @Published var regionStates: [String: RegionState] = [:]
    @Published var activeDownloadRegion: String?
    @Published var currentTileProgress: String?
    @Published var installedTileCount: Int = 0

    // MARK: - Private

    private var downloadTask: Task<Void, Never>?
    private let srtmDirectory: URL

    /// Expected decompressed HGT file size.
    nonisolated(unsafe) private static let expectedFileSize = 3601 * 3601 * 2  // 25,934,402

    /// AWS S3 base URL for SRTM tiles (free, no auth).
    nonisolated(unsafe) private static let baseURL = "https://s3.amazonaws.com/elevation-tiles-prod/skadi"

    /// Maximum concurrent tile downloads.
    nonisolated(unsafe) private static let maxConcurrentDownloads = 3

    // MARK: - Region Definitions

    static let regions: [Region] = [
        Region(
            id: "pacific_nw",
            name: "PACIFIC NW",
            description: "WA, OR",
            latRange: 42...48,
            lonRange: -125...(-117)
        ),
        Region(
            id: "california",
            name: "CALIFORNIA",
            description: "CA",
            latRange: 32...42,
            lonRange: -125...(-114)
        ),
        Region(
            id: "southwest",
            name: "SOUTHWEST",
            description: "UT, AZ, NM, CO",
            latRange: 31...42,
            lonRange: -115...(-103)
        ),
        Region(
            id: "rocky_mtn",
            name: "ROCKY MTN",
            description: "MT, ID, WY",
            latRange: 42...49,
            lonRange: -117...(-104)
        ),
        Region(
            id: "great_plains_n",
            name: "GREAT PLAINS N",
            description: "ND, SD, NE, KS, MN, IA",
            latRange: 36...49,
            lonRange: -104...(-89)
        ),
        Region(
            id: "great_plains_s",
            name: "GREAT PLAINS S",
            description: "TX, OK, AR, LA",
            latRange: 25...37,
            lonRange: -107...(-89)
        ),
        Region(
            id: "great_lakes",
            name: "GREAT LAKES",
            description: "WI, MI, IL, IN, OH",
            latRange: 37...47,
            lonRange: -93...(-80)
        ),
        Region(
            id: "southeast",
            name: "SOUTHEAST",
            description: "VA, NC, SC, GA, FL, AL, MS, TN, KY, WV",
            latRange: 24...39,
            lonRange: -92...(-75)
        ),
        Region(
            id: "northeast",
            name: "NORTHEAST",
            description: "ME, NH, VT, MA, NY, NJ, PA, DE, MD",
            latRange: 38...47,
            lonRange: -80...(-67)
        )
    ]

    // MARK: - Init

    init() {
        let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        srtmDirectory = documentsURL.appendingPathComponent("SRTM", isDirectory: true)

        // Ensure SRTM directory exists
        try? FileManager.default.createDirectory(at: srtmDirectory, withIntermediateDirectories: true)

        // Initialize region states
        for region in Self.regions {
            regionStates[region.id] = RegionState(
                totalTiles: region.tiles.count,
                downloadedTiles: 0
            )
        }

        // Scan for existing tiles
        scanInstalledTiles()
    }

    // MARK: - Public API

    /// Scan Documents/SRTM/ for existing HGT files and update region states.
    func scanInstalledTiles() {
        let fileManager = FileManager.default

        // Get all .hgt files
        let installedKeys: Set<String>
        if let contents = try? fileManager.contentsOfDirectory(atPath: srtmDirectory.path) {
            installedKeys = Set(contents.compactMap { filename -> String? in
                guard filename.hasSuffix(".hgt") else { return nil }
                return String(filename.dropLast(4))  // Remove .hgt
            })
        } else {
            installedKeys = []
        }

        installedTileCount = installedKeys.count

        // Update each region's state
        for region in Self.regions {
            let regionTiles = Set(region.tiles)
            let downloaded = regionTiles.intersection(installedKeys).count

            // Compute size on disk
            var sizeMB: Double = 0
            for key in regionTiles.intersection(installedKeys) {
                let fileURL = srtmDirectory.appendingPathComponent("\(key).hgt")
                if let attrs = try? fileManager.attributesOfItem(atPath: fileURL.path),
                   let fileSize = attrs[.size] as? UInt64 {
                    sizeMB += Double(fileSize) / (1024 * 1024)
                }
            }

            regionStates[region.id] = RegionState(
                totalTiles: region.tiles.count,
                downloadedTiles: downloaded,
                sizeOnDiskMB: sizeMB
            )
        }

        Logger.terrain.info("SRTM scan: \(installedKeys.count) tiles installed")
    }

    /// Download all tiles for a region.
    func downloadRegion(_ region: Region) {
        guard activeDownloadRegion == nil else { return }

        activeDownloadRegion = region.id
        regionStates[region.id]?.isDownloading = true
        regionStates[region.id]?.error = nil

        downloadTask = Task {
            await performDownload(region: region)
        }
    }

    /// Cancel the active download.
    func cancelDownload() {
        downloadTask?.cancel()
        downloadTask = nil

        if let regionId = activeDownloadRegion {
            regionStates[regionId]?.isDownloading = false
        }
        activeDownloadRegion = nil
        currentTileProgress = nil
    }

    /// Delete all tiles for a region.
    func deleteRegion(_ region: Region) {
        let fileManager = FileManager.default

        for key in region.tiles {
            let fileURL = srtmDirectory.appendingPathComponent("\(key).hgt")
            try? fileManager.removeItem(at: fileURL)
        }

        scanInstalledTiles()
        Logger.terrain.info("Deleted SRTM region: \(region.name)")
    }

    /// Find the best-fitting region for a coordinate.
    /// When bounding boxes overlap (border tiles), picks the region where
    /// the coordinate is most centrally located.
    func regionForCoordinate(_ coord: CLLocationCoordinate2D) -> Region? {
        let lat = Int(floor(coord.latitude))
        let lon = Int(floor(coord.longitude))

        let matching = Self.regions.filter { region in
            region.latRange.contains(lat) && region.lonRange.contains(lon)
        }

        guard !matching.isEmpty else { return nil }
        if matching.count == 1 { return matching.first }

        // Multiple matches — pick the region where the point is most central.
        // Use normalized distance from center of bounding box (lower = more central).
        return matching.min { a, b in
            let aMidLat = Double(a.latRange.lowerBound + a.latRange.upperBound) / 2.0
            let aMidLon = Double(a.lonRange.lowerBound + a.lonRange.upperBound) / 2.0
            let aDistSq = pow(Double(lat) - aMidLat, 2) + pow(Double(lon) - aMidLon, 2)

            let bMidLat = Double(b.latRange.lowerBound + b.latRange.upperBound) / 2.0
            let bMidLon = Double(b.lonRange.lowerBound + b.lonRange.upperBound) / 2.0
            let bDistSq = pow(Double(lat) - bMidLat, 2) + pow(Double(lon) - bMidLon, 2)

            return aDistSq < bDistSq
        }
    }

    /// Total storage used by all SRTM tiles (MB).
    func totalStorageMB() -> Double {
        regionStates.values.reduce(0) { $0 + $1.sizeOnDiskMB }
    }

    // MARK: - Download Implementation

    private func performDownload(region: Region) async {
        let fileManager = FileManager.default
        let tilesToDownload = region.tiles.filter { key in
            let fileURL = srtmDirectory.appendingPathComponent("\(key).hgt")
            if fileManager.fileExists(atPath: fileURL.path) {
                // Verify file size
                if let attrs = try? fileManager.attributesOfItem(atPath: fileURL.path),
                   let size = attrs[.size] as? UInt64,
                   size == UInt64(Self.expectedFileSize) {
                    return false  // Already downloaded and valid
                }
            }
            return true
        }

        if tilesToDownload.isEmpty {
            await MainActor.run {
                regionStates[region.id]?.isDownloading = false
                activeDownloadRegion = nil
                currentTileProgress = nil
            }
            scanInstalledTiles()
            return
        }

        Logger.terrain.info("SRTM download: \(tilesToDownload.count) tiles for \(region.name)")

        var completedCount = 0
        let totalCount = tilesToDownload.count

        // Process in batches of maxConcurrentDownloads
        for batchStart in stride(from: 0, to: tilesToDownload.count, by: Self.maxConcurrentDownloads) {
            guard !Task.isCancelled else { break }

            let batchEnd = min(batchStart + Self.maxConcurrentDownloads, tilesToDownload.count)
            let batch = Array(tilesToDownload[batchStart..<batchEnd])

            await withTaskGroup(of: (String, Bool).self) { group in
                for tileKey in batch {
                    group.addTask {
                        let success = await self.downloadTile(key: tileKey)
                        return (tileKey, success)
                    }
                }

                for await (tileKey, success) in group {
                    completedCount += 1
                    if success {
                        Logger.terrain.debug("Downloaded tile: \(tileKey)")
                    }

                    // Update progress on main actor
                    let current = completedCount
                    await MainActor.run {
                        self.currentTileProgress = "\(tileKey) (\(current)/\(totalCount))"
                        self.regionStates[region.id]?.downloadedTiles = (self.regionStates[region.id]?.downloadedTiles ?? 0) + (success ? 1 : 0)
                    }
                }
            }
        }

        await MainActor.run {
            regionStates[region.id]?.isDownloading = false
            activeDownloadRegion = nil
            currentTileProgress = nil
        }

        scanInstalledTiles()

        Logger.terrain.info("SRTM download complete: \(region.name) - \(completedCount)/\(totalCount) tiles")
    }

    /// Download and decompress a single SRTM tile.
    private nonisolated func downloadTile(key: String) async -> Bool {
        // Construct URL: e.g. N37/N37W114.hgt.gz
        let latBand = String(key.prefix(3))  // "N37"
        let urlString = "\(Self.baseURL)/\(latBand)/\(key).hgt.gz"

        guard let url = URL(string: urlString) else { return false }

        do {
            let (data, response) = try await URLSession.shared.data(from: url)

            guard let httpResponse = response as? HTTPURLResponse else { return false }

            // 404 = ocean/no-data tile — not an error
            if httpResponse.statusCode == 404 || httpResponse.statusCode == 403 {
                return false
            }

            guard httpResponse.statusCode == 200 else {
                Logger.terrain.warning("SRTM download HTTP \(httpResponse.statusCode) for \(key)")
                return false
            }

            // Decompress gzip
            guard let decompressed = decompressGzip(data) else {
                Logger.terrain.warning("SRTM decompression failed for \(key)")
                return false
            }

            // Validate size
            guard decompressed.count == Self.expectedFileSize else {
                Logger.terrain.warning("SRTM tile \(key) wrong size: \(decompressed.count) (expected \(Self.expectedFileSize))")
                return false
            }

            // Write to Documents/SRTM/
            let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
            let srtmDir = documentsURL.appendingPathComponent("SRTM", isDirectory: true)
            try? FileManager.default.createDirectory(at: srtmDir, withIntermediateDirectories: true)

            let fileURL = srtmDir.appendingPathComponent("\(key).hgt")
            try decompressed.write(to: fileURL)

            return true
        } catch {
            if !Task.isCancelled {
                Logger.terrain.error("SRTM download error for \(key): \(error.localizedDescription)")
            }
            return false
        }
    }

    // MARK: - Gzip Decompression

    /// Decompress gzip data using Apple's Compression framework.
    private nonisolated func decompressGzip(
        _ data: Data
    ) -> Data? {
        // Gzip files have a header — we need to strip it to get raw DEFLATE data.
        // Gzip header: 1F 8B 08 ... (minimum 10 bytes)
        guard data.count > 18,
              data[data.startIndex] == 0x1F,
              data[data.startIndex + 1] == 0x8B else {
            return nil
        }

        // Find the start of the DEFLATE payload.
        // Standard gzip header is 10 bytes, but optional fields may extend it.
        var offset = 10
        let flags = data[data.startIndex + 3]

        // FEXTRA
        if flags & 0x04 != 0 {
            guard offset + 2 <= data.count else { return nil }
            let extraLen = Int(data[data.startIndex + offset]) | (Int(data[data.startIndex + offset + 1]) << 8)
            offset += 2 + extraLen
        }

        // FNAME
        if flags & 0x08 != 0 {
            while offset < data.count && data[data.startIndex + offset] != 0 { offset += 1 }
            offset += 1  // Skip null terminator
        }

        // FCOMMENT
        if flags & 0x10 != 0 {
            while offset < data.count && data[data.startIndex + offset] != 0 { offset += 1 }
            offset += 1
        }

        // FHCRC
        if flags & 0x02 != 0 {
            offset += 2
        }

        guard offset < data.count - 8 else { return nil }  // Need at least trailer (8 bytes)

        // Extract DEFLATE payload (everything between header and 8-byte trailer)
        let deflateData = data[data.startIndex + offset ..< data.endIndex - 8]

        // Decompress using Compression framework (ZLIB wrapping handles raw DEFLATE)
        let sourceSize = deflateData.count
        let destinationSize = Self.expectedFileSize + 1024  // Slight padding

        let decompressed = deflateData.withUnsafeBytes { sourceBuffer -> Data? in
            guard let sourcePtr = sourceBuffer.baseAddress?.assumingMemoryBound(to: UInt8.self) else {
                return nil
            }

            let destinationBuffer = UnsafeMutablePointer<UInt8>.allocate(capacity: destinationSize)
            defer { destinationBuffer.deallocate() }

            let decodedSize = compression_decode_buffer(
                destinationBuffer, destinationSize,
                sourcePtr, sourceSize,
                nil,
                COMPRESSION_ZLIB
            )

            guard decodedSize > 0 else { return nil }
            return Data(bytes: destinationBuffer, count: decodedSize)
        }

        return decompressed
    }
}
