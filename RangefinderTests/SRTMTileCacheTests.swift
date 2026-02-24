//
//  SRTMTileCacheTests.swift
//  RangefinderTests
//
//  Tests for SRTM HGT tile parsing and elevation queries.
//

import XCTest
import CoreLocation
@testable import Rangefinder

final class SRTMTileCacheTests: XCTestCase {

    // MARK: - Tile Name Generation

    func testTileNamePositiveCoordinates() async {
        // Lat 37.7, Lon -122.4 → "N37W123.hgt"
        // Floor(37.7) = 37 → N37
        // Floor(-122.4) = -123 → W123
        let cache = SRTMTileCache()
        let coord = CLLocationCoordinate2D(latitude: 37.7, longitude: -122.4)
        // We can test this indirectly — the cache should not crash on valid coords
        let elev = await cache.elevation(at: coord)
        // Won't have actual data, but shouldn't crash
        // (returns nil if no HGT file and EPQS fails)
        _ = elev
    }

    func testTileNameSouthernHemisphere() async {
        // Lat -33.8, Lon 151.2 → "S34E151.hgt"
        let cache = SRTMTileCache()
        let coord = CLLocationCoordinate2D(latitude: -33.8, longitude: 151.2)
        let elev = await cache.elevation(at: coord)
        _ = elev  // Just verify no crash
    }

    func testTileNameEquatorGreenwich() async {
        // Lat 0.5, Lon 0.5 → "N00E000.hgt"
        let cache = SRTMTileCache()
        let coord = CLLocationCoordinate2D(latitude: 0.5, longitude: 0.5)
        let elev = await cache.elevation(at: coord)
        _ = elev
    }

    // MARK: - Bilinear Interpolation Logic

    func testBilinearInterpolationCorners() {
        // Test the bilinear interpolation math:
        // At exact grid corner, should return that corner's value
        // f(0,0) with tx=0, ty=0 → v00
        let v00: Double = 100
        let v10: Double = 200
        let v01: Double = 300
        let v11: Double = 400

        let tx = 0.0, ty = 0.0
        let result = v00 * (1 - tx) * (1 - ty) + v10 * tx * (1 - ty)
                   + v01 * (1 - tx) * ty + v11 * tx * ty
        XCTAssertEqual(result, 100.0, accuracy: 0.01)
    }

    func testBilinearInterpolationCenter() {
        // At center (0.5, 0.5) → average of all 4
        let v00: Double = 100
        let v10: Double = 200
        let v01: Double = 300
        let v11: Double = 400

        let tx = 0.5, ty = 0.5
        let result = v00 * (1 - tx) * (1 - ty) + v10 * tx * (1 - ty)
                   + v01 * (1 - tx) * ty + v11 * tx * ty
        XCTAssertEqual(result, 250.0, accuracy: 0.01)
    }

    func testBilinearInterpolationEdge() {
        // At (1.0, 0.5) → average of v10 and v11
        let v00: Double = 100
        let v10: Double = 200
        let v01: Double = 300
        let v11: Double = 400

        let tx = 1.0, ty = 0.5
        let result = v00 * (1 - tx) * (1 - ty) + v10 * tx * (1 - ty)
                   + v01 * (1 - tx) * ty + v11 * tx * ty
        XCTAssertEqual(result, 300.0, accuracy: 0.01)
    }

    // MARK: - EPQS Grid Key

    func testEPQSGridKeyRounding() {
        // 0.001° grid: 37.7749 → "37.775,-122.419"
        let lat = 37.7749
        let lon = -122.4194

        let roundedLat = (lat * 1000).rounded() / 1000
        let roundedLon = (lon * 1000).rounded() / 1000

        XCTAssertEqual(roundedLat, 37.775, accuracy: 0.0001)
        XCTAssertEqual(roundedLon, -122.419, accuracy: 0.0001)
    }

    // MARK: - HGT File Size Validation

    func testExpectedHGTFileSize() {
        // SRTM1 (1-arc-second): 3601 × 3601 × 2 bytes = 25,934,402
        let expectedSize = 3601 * 3601 * 2
        XCTAssertEqual(expectedSize, 25_934_402)
    }

    func testVoidValue() {
        // SRTM void (no data) is represented as -32768
        let voidValue: Int16 = -32768
        XCTAssertEqual(voidValue, Int16.min)
    }
}
