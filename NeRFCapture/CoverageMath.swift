//
//  CoverageMath.swift  (Otterly Spike 2 — guidance prototype)
//
//  Pure viewing-pose-coverage scoring. NO ARKit / UIKit imports, so it compiles on the macOS
//  command line (`swiftc CoverageMath.swift coverage_selftest.swift`) for a sign/geometry self-test
//  before any device time — same discipline as mac_capture_selfcheck.swift.
//
//  This is the offline-VALIDATED metric from coverage_meter.py ported verbatim:
//    - each surface voxel records which AZIMUTH bins (12 x 30 deg) a camera viewed it from,
//    - "green" once that azimuth span >= SPAN_OK (the locked quality signal = wider viewing-pose coverage),
//    - next-angle = midpoint of the widest missing azimuth wedge over the under-covered surface.
//
import Foundation

enum CoverageMath {
    static let nAz = 12
    static let degPerBin = 360.0 / Double(nAz)
    static let spanOK = 90.0
    static let lowOccFrac = 0.15        // an azimuth bin is "missing" when occupancy <= this * peak bin

    /// Azimuth bin [0, nAz) for an angle in degrees (any sign).
    static func azBin(_ degrees: Double) -> Int {
        let a = degrees.truncatingRemainder(dividingBy: 360.0)
        let pos = a < 0 ? a + 360.0 : a
        return min(nAz - 1, Int(pos / degPerBin))
    }

    /// Azimuth span covered = 360 - largest empty gap among occupied bins. Monotone in #occupied bins,
    /// so a voxel that is green stays green as more bins fill (the basis of the incremental green count).
    static func span(_ occ: UInt16) -> Double {
        var angs: [Double] = []
        for b in 0..<nAz where (occ & (UInt16(1) << b)) != 0 { angs.append(Double(b) * degPerBin) }
        if angs.isEmpty { return 0 }
        if angs.count == 1 { return degPerBin }
        var maxGap = 0.0
        for i in 0..<angs.count {
            let next = (i + 1 < angs.count) ? angs[i + 1] : angs[0] + 360.0
            maxGap = max(maxGap, next - angs[i])
        }
        return 360.0 - maxGap
    }

    static func isGreen(_ occ: UInt16) -> Bool { span(occ) >= spanOK }

    /// Longest circular run of `true` over nAz bins -> (startBin, length).
    static func longestMissingRun(_ missing: [Bool]) -> (start: Int, length: Int) {
        precondition(missing.count == nAz)
        var bestLen = 0, bestStart = 0, run = 0, start = 0
        for i in 0..<(2 * nAz) {
            if missing[i % nAz] {
                if run == 0 { start = i }
                run += 1
                if run > bestLen && run <= nAz { bestLen = run; bestStart = start }
            } else {
                run = 0
            }
        }
        return (bestStart % nAz, bestLen)
    }

    /// Recommended orbit azimuth (deg, world x-z) = midpoint of the widest missing wedge.
    /// `underHist[b]` = number of under-covered voxels that have been seen from azimuth bin b.
    /// Returns nil when fully covered or the gap is diffuse (under-covered surface seen from all sides).
    static func nextAngle(underHist: [Double]) -> (azimuth: Double, gapDeg: Double)? {
        precondition(underHist.count == nAz)
        guard let peak = underHist.max(), peak > 0 else { return nil }
        let missing = underHist.map { $0 <= lowOccFrac * peak }
        let (start, length) = longestMissingRun(missing)
        if length == 0 { return nil }
        let az = ((Double(start) + Double(length) / 2.0).truncatingRemainder(dividingBy: Double(nAz))) * degPerBin
        return (az, Double(length) * degPerBin)
    }

    // --- auto-capture GATE cells (capture POLICY, NOT the quality metric) -------------------------
    // The gate fires per (camera orbit sector x elevation band) cell, so it (a) does not stop after one
    // horizontal orbit and (b) requires HEIGHT variation (the locked protocol's "vary height"). These
    // are separate from the azimuth-only green quality metric above, which stays as validated.
    // v1.2 (2026-06-01): range raised from [-15, 75] to [15, 90] so the LOW band is reachable for
    // small floor objects. Old range needed camera within ~27cm of the floor at 1m horizontal to
    // trigger inner band — physically blocked by the floor for any bottle-sized object.
    // New 3-band split: low [15, 40), mid [40, 65), high [65, 90].
    static let elevMinDeg = 15.0      // elevation banding range floor (camera elevation above object centre)
    static let elevSpanDeg = 75.0     // span covered by the bands (3 bands x 25 deg over [15, 90])

    static func orbitSector(azimuthDeg: Double, bins: Int) -> Int {
        let a = azimuthDeg.truncatingRemainder(dividingBy: 360.0)
        let pos = a < 0 ? a + 360.0 : a
        return min(bins - 1, Int(pos / (360.0 / Double(bins))))
    }

    static func elevBand(elevDeg: Double, bands: Int) -> Int {
        let w = elevSpanDeg / Double(bands)
        let i = Int(floor((elevDeg - elevMinDeg) / w))
        return max(0, min(bands - 1, i))
    }
}
