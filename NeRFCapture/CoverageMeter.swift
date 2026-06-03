//
//  CoverageMeter.swift  (Otterly — guidance prototype, v2)
//
//  Live, on-device REGION-centric + QUALITY-GATED coverage. Consumes what a phone has live (ARKit pose
//  + LiDAR depth + per-pixel confidence) and maintains, incrementally, per-surface-voxel viewing-angle
//  coverage over the WHOLE space (no object crop). Drives the guidance HUD (quality % + next-angle arrow)
//  and the quality-gated auto-shutter (fire when a steady frame ADDS new viewing-angle coverage).
//
//  v2 (2026-06-02) — replaces the v1.x device-orientation "center-disk" gate (orbit-sector x elevation-band)
//  with the validated parallax signal driving capture directly. Changes vs v1.3 (codex B-gate REVISE,
//  all points incorporated):
//    - WHOLE-HOUSE: removed the 1 m object-crop -> every observed surface voxel is tracked.
//    - VOXEL ADMISSION: a voxel counts (denominator / green / informative) only after >= admitObs DISTINCT
//      frames have seen it -> a single noisy/edge/sparse LiDAR sample can no longer create a fake region.
//    - CONFIDENCE FILTER: only depth pixels with ARKit confidence >= confMin are used.
//    - BEARING from the observed HIT POINT (validated) = angle-AT-region (true triangulation angle), not
//      device orientation. One bearing per voxel per frame; across frames these accumulate into the span.
//    - INFORMATIVE: update() reports whether this frame added a new azimuth bin to >= minInformativeVoxels
//      under-covered admitted voxels -> the caller fires the shutter only on such frames (no redundant shots).
//  Steadiness (sharp-frame) gating lives in the caller (ARViewModel): only steady frames reach update().
//
//  THREADING: NOT thread-safe; all methods must be called from one serial queue (ARViewModel.coverageQueue).
//
import Foundation
import simd

/// Per-frame ingest result.
struct CoverageUpdate {
    let informative: Bool   // frame added a new azimuth bin to >= minInformativeVoxels under-covered admitted voxels
    let valid: Bool         // had usable (confidence-filtered) depth this frame
}

final class CoverageMeter {
    // --- region grid (whole-house) ---
    static let voxel: Float = 0.10          // surface voxel size (m). Whole-house: coarser than the 0.05 object
                                            //   metric for voxel-count headroom; still fine guidance granularity
                                            //   and the validated span>=90 relation is granularity-robust. TUNE.
    static let pixStride = 4                // subsample depth pixels (256x192 -> ~3k pts/frame)
    // --- codex B-gate hardening ---
    static let admitObs: UInt8 = 3          // a voxel counts only after this many DISTINCT frames observed it
    static let confMin: UInt8 = 1           // keep depth pixels with ARKit confidence >= medium (0=low,1=med,2=high)
    static let minInformativeVoxels = 12    // a frame is "informative" (-> auto-capture eligible) when it adds a
                                            //   new azimuth bin to at least this many under-covered admitted voxels
    // --- perf ---
    static let naThrottle = 5               // recompute next-angle every Nth update (~10Hz/5 = ~2Hz)

    // azimuth bitmask (which 30-deg wedges have viewed this voxel) + distinct-frame count.
    private struct Vox { var occ: UInt16; var frames: UInt8 }
    private var bins: [SIMD3<Int32>: Vox] = [:]
    private var greenSet: Set<SIMD3<Int32>> = []   // admitted voxels whose azimuth span >= spanOK
    private(set) var greenCount = 0                // green among admitted
    private(set) var admittedCount = 0             // voxels with frames >= admitObs

    private var naCache: (azimuth: Double, gapDeg: Double)?
    private var sinceNA = 0
    private var seenThisFrame = Set<SIMD3<Int32>>()   // per-frame scratch (reused; single-queue)

    var admittedVoxels: Int { admittedCount }
    var greenFrac: Float { admittedCount == 0 ? 0 : Float(greenCount) / Float(admittedCount) }
    func cachedNextAngle() -> (azimuth: Double, gapDeg: Double)? { naCache }

    /// Ingest one STEADY ARKit frame (the caller only forwards frames that pass the steadiness gate).
    /// `conf` is the per-pixel confidence (same dims as depth); nil/short -> accept all pixels.
    @discardableResult
    func update(depth: [Float], conf: [UInt8]?, dw: Int, dh: Int,
                intrinsics K: simd_float3x3, rgbSize: CGSize, camera c2w: simd_float4x4) -> CoverageUpdate {
        guard dw > 0, dh > 0, rgbSize.width > 0, rgbSize.height > 0, depth.count >= dw * dh else {
            return CoverageUpdate(informative: false, valid: false)
        }
        let camPos = SIMD3<Float>(c2w.columns.3.x, c2w.columns.3.y, c2w.columns.3.z)
        let R = simd_float3x3(SIMD3(c2w.columns.0.x, c2w.columns.0.y, c2w.columns.0.z),
                              SIMD3(c2w.columns.1.x, c2w.columns.1.y, c2w.columns.1.z),
                              SIMD3(c2w.columns.2.x, c2w.columns.2.y, c2w.columns.2.z))
        let sx = Float(dw) / Float(rgbSize.width), sy = Float(dh) / Float(rgbSize.height)
        let fx = K[0, 0] * sx, fy = K[1, 1] * sy, cx = K[2, 0] * sx, cy = K[2, 1] * sy
        let hasConf = (conf?.count ?? 0) >= dw * dh

        seenThisFrame.removeAll(keepingCapacity: true)
        var informativeHits = 0
        var anyValid = false

        var v = 0
        while v < dh {
            var u = 0
            while u < dw {
                let idx = v * dw + u
                let z = depth[idx]
                if z.isFinite, z > 0, !hasConf || conf![idx] >= CoverageMeter.confMin {
                    anyValid = true
                    let xc = (Float(u) + 0.5 - cx) / fx * z
                    let yc = -(Float(v) + 0.5 - cy) / fy * z
                    let world = R * SIMD3<Float>(xc, yc, -z) + camPos
                    let vi = SIMD3<Int32>(Int32(floor(world.x / CoverageMeter.voxel)),
                                          Int32(floor(world.y / CoverageMeter.voxel)),
                                          Int32(floor(world.z / CoverageMeter.voxel)))
                    // one bearing + one frame-count per voxel per frame (dedup within the frame)
                    if seenThisFrame.insert(vi).inserted {
                        let azDeg = Double(atan2(camPos.z - world.z, camPos.x - world.x)) * 180.0 / .pi
                        if addObservation(vi: vi, bin: CoverageMath.azBin(azDeg)) { informativeHits += 1 }
                    }
                }
                u += CoverageMeter.pixStride
            }
            v += CoverageMeter.pixStride
        }
        if !anyValid { return CoverageUpdate(informative: false, valid: false) }

        sinceNA += 1
        if sinceNA >= CoverageMeter.naThrottle { naCache = computeNextAngle(); sinceNA = 0 }

        return CoverageUpdate(informative: informativeHits >= CoverageMeter.minInformativeVoxels, valid: true)
    }

    /// Record one frame's observation of a voxel. Returns true iff this frame added a NEW azimuth bin to an
    /// admitted, not-yet-green voxel (a genuinely informative contribution toward that region's coverage).
    @discardableResult
    private func addObservation(vi: SIMD3<Int32>, bin: Int) -> Bool {
        let mask = UInt16(1) << bin
        guard var vox = bins[vi] else {
            bins[vi] = Vox(occ: mask, frames: 1)     // first sighting (admitObs >= 2 -> not admitted yet)
            return false
        }
        let wasGreen = greenSet.contains(vi)
        let wasAdmitted = vox.frames >= CoverageMeter.admitObs
        if vox.frames < UInt8.max { vox.frames += 1 }
        let nowAdmitted = vox.frames >= CoverageMeter.admitObs
        let isNewBin = (vox.occ & mask) == 0
        if isNewBin { vox.occ |= mask }
        bins[vi] = vox

        if nowAdmitted && !wasAdmitted { admittedCount += 1 }
        // (re)evaluate green when newly admitted (occ may already span >= spanOK from pre-admission frames)
        // OR when a new bin was added to an already-admitted voxel.
        if nowAdmitted && !wasGreen && (isNewBin || !wasAdmitted) && CoverageMath.isGreen(vox.occ) {
            greenSet.insert(vi); greenCount += 1
        }
        return nowAdmitted && isNewBin && !wasGreen
    }

    /// Widest missing azimuth wedge over the UNDER-covered admitted surface. O(voxels) — call throttled.
    private func computeNextAngle() -> (azimuth: Double, gapDeg: Double)? {
        var hist = [Double](repeating: 0, count: CoverageMath.nAz)
        for (vi, vox) in bins where vox.frames >= CoverageMeter.admitObs && !greenSet.contains(vi) {
            for b in 0..<CoverageMath.nAz where (vox.occ & (UInt16(1) << b)) != 0 { hist[b] += 1 }
        }
        return CoverageMath.nextAngle(underHist: hist)
    }

    /// Test-only: simulate one steady frame observing voxel `vi` from azimuth `azDeg` (bypasses depth
    /// back-projection + per-frame dedup). Exercises the admission / green / informative bookkeeping in
    /// coverage_selftest.swift on a Mac with no device. Each call = one distinct frame for that voxel.
    @discardableResult
    func _testObserve(_ vi: SIMD3<Int32>, _ azDeg: Double) -> Bool {
        return addObservation(vi: vi, bin: CoverageMath.azBin(azDeg))
    }
}
