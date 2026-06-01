//
//  CoverageMeter.swift  (Otterly Spike 2 — guidance prototype, v1.1)
//
//  Live port of the offline-validated coverage_meter.py. Consumes the SAME inputs a phone has live
//  (ARKit pose + LiDAR depth) and maintains, incrementally, the per-surface viewing-pose coverage.
//  Drives the guidance HUD (green-fraction + heatmap-from-above + next-angle arrow) and the
//  coverage-gated auto-shutter.
//
//  v1.1 device-feedback fixes (Andy, 2026-05-31):
//    - GATE is now (orbit-sector x elevation-band) cells, not azimuth-only: capture no longer stops
//      after one horizontal orbit (was 24 frames) AND height variation is required/tracked. The GATE
//      is capture POLICY; the green QUALITY metric stays azimuth-only as validated (CoverageMath).
//    - next-angle is recomputed at ~naThrottle (not every frame) and cached -> removes the growing
//      O(voxels) per-frame cost that made the live view stutter over time.
//    - update() takes a copied depth [Float] (the caller extracts it from the ARFrame on the AR queue,
//      then runs the meter on a background serial queue) -> the heavy work is off the render path.
//
//  THREADING: NOT thread-safe; all methods must be called from one serial queue (ARViewModel.coverageQueue).
//
import Foundation
import simd

struct GateState {
    let azBin: Int
    let elevBand: Int
    let isNew: Bool          // this (sector x band) cell has not been captured from yet
    let valid: Bool
}

final class CoverageMeter {
    // --- geometry / quality-metric constants (match coverage_meter.py) ---
    static let voxel: Float = 0.05         // surface voxel size (m)
    // v1.3 (2026-06-01): cropR tightened from 2.5 to 1.0 m. The locked validated metric is the
    // azimuth-span green count over the CROP SPHERE; 2.5 m worked for room-scale capture but for
    // handheld object capture it pulled in floor/wall voxels that only get scanned from a narrow
    // azimuth wedge, inflating the denominator and pinning coverage % at ~10-15% even when the
    // gate cells were largely filled. 1.0 m still covers a generous bottle/box-sized object.
    static let cropR: Float = 1.0          // object-crop radius around running look-at centre (m)
    static let pixStride = 4               // subsample depth pixels (256x192 -> ~3k pts/frame)
    // --- auto-capture gate grid (capture policy; INDEPENDENT of the green metric's own 12 azimuth bins) ---
    static let orbitBins = 36              // camera-azimuth sectors (10 deg). 36 x 3 = 108 max cells: lands a
                                           // fully-covered capture in the empirically-proven ~100-130 frame
                                           // range (locked finding: well-SPREAD frames matter more than raw
                                           // count, so this is a ceiling, not a target). Tunable.
    static let elevBins = 3                // elevation bands (low / mid / high) — the height dimension
    static let warmupFrames = 30           // v1.3: collect elev samples for ~3 s (10 Hz) to learn the
                                           // user's actual handheld elev envelope, then split bands by
                                           // observed P33/P66 — fixes "outer ring unreachable" across
                                           // arbitrary object heights (floor / table-top / shelf). During
                                           // warmup the gate is invalid so auto-capture defers.
    // --- perf ---
    static let naThrottle = 5              // recompute next-angle every Nth update (~10Hz/5 = ~2Hz)

    // --- voxel store (quality metric, azimuth-only — unchanged from validated Spike 1) ---
    private var bins: [SIMD3<Int32>: UInt16] = [:]
    private var greenSet: Set<SIMD3<Int32>> = []
    private(set) var greenCount = 0

    // --- running object centre = mean per-frame look-at point ---
    private var lookSum = SIMD3<Float>(repeating: 0)
    private var lookN: Float = 0
    var center: SIMD3<Float> { lookN > 0 ? lookSum / lookN : SIMD3<Float>(repeating: 0) }

    // --- auto-capture gate: which (orbit sector x elevation band) cells captured ---
    // (qualify the static refs: an instance stored-property default cannot use unqualified static names)
    private var captured = [Bool](repeating: false, count: CoverageMeter.orbitBins * CoverageMeter.elevBins)

    // --- next-angle cache (throttled) ---
    private var naCache: (azimuth: Double, gapDeg: Double)?
    private var sinceNA = 0

    // --- v1.3 adaptive elev bands: edges = [P33, P66] of warmup samples; nil while warming up ---
    private var elevSamples: [Double] = []
    private var elevEdges: [Double]?

    var voxelCount: Int { bins.count }
    var greenFrac: Float { bins.isEmpty ? 0 : Float(greenCount) / Float(bins.count) }
    func capturedCellsFlat() -> [Bool] { captured }
    func cachedNextAngle() -> (azimuth: Double, gapDeg: Double)? { naCache }
    var isWarmingUp: Bool { elevEdges == nil }
    var warmupProgress: Float {
        Float(min(elevSamples.count, CoverageMeter.warmupFrames)) / Float(CoverageMeter.warmupFrames)
    }

    func markCellCaptured(az: Int, elev: Int) {
        let i = az * CoverageMeter.elevBins + elev
        if i >= 0 && i < captured.count { captured[i] = true }
    }

    /// Ingest one ARKit frame (depth already copied to `depth` by the caller). Returns the current
    /// gate cell + whether it is new (the firing POLICY — interval, cap, tracking — lives in the caller).
    @discardableResult
    func update(depth: [Float], dw: Int, dh: Int, intrinsics K: simd_float3x3,
                rgbSize: CGSize, camera c2w: simd_float4x4) -> GateState {
        let camPos = SIMD3<Float>(c2w.columns.3.x, c2w.columns.3.y, c2w.columns.3.z)
        let R = simd_float3x3(SIMD3(c2w.columns.0.x, c2w.columns.0.y, c2w.columns.0.z),
                              SIMD3(c2w.columns.1.x, c2w.columns.1.y, c2w.columns.1.z),
                              SIMD3(c2w.columns.2.x, c2w.columns.2.y, c2w.columns.2.z))
        let forward = R * SIMD3<Float>(0, 0, -1)
        guard dw > 0, dh > 0, rgbSize.width > 0, rgbSize.height > 0, depth.count >= dw * dh else {
            return GateState(azBin: 0, elevBand: 0, isNew: false, valid: false)   // no usable depth -> not a gate frame
        }
        let sx = Float(dw) / Float(rgbSize.width), sy = Float(dh) / Float(rgbSize.height)
        let fx = K[0, 0] * sx, fy = K[1, 1] * sy, cx = K[2, 0] * sx, cy = K[2, 1] * sy

        // Pass A: strided valid samples + mean valid depth (this frame's look-at point)
        var us: [Int] = [], vs: [Int] = []; var zs: [Float] = []
        var depthSum: Float = 0
        var v = 0
        while v < dh {
            var u = 0
            while u < dw {
                let z = depth[v * dw + u]
                if z.isFinite && z > 0 { us.append(u); vs.append(v); zs.append(z); depthSum += z }
                u += CoverageMeter.pixStride
            }
            v += CoverageMeter.pixStride
        }
        if zs.isEmpty { return GateState(azBin: 0, elevBand: 0, isNew: false, valid: false) }  // no valid depth pixels
        lookSum += camPos + forward * (depthSum / Float(zs.count)); lookN += 1
        let C = center
        let cropR2 = CoverageMeter.cropR * CoverageMeter.cropR

        // Pass B: backproject -> object-crop -> per-voxel azimuth update (the validated quality metric)
        for i in 0..<zs.count {
            let z = zs[i]
            let xc = (Float(us[i]) + 0.5 - cx) / fx * z
            let yc = -(Float(vs[i]) + 0.5 - cy) / fy * z
            let world = R * SIMD3<Float>(xc, yc, -z) + camPos
            if simd_length_squared(world - C) > cropR2 { continue }
            let vi = SIMD3<Int32>(Int32(floor(world.x / CoverageMeter.voxel)),
                                  Int32(floor(world.y / CoverageMeter.voxel)),
                                  Int32(floor(world.z / CoverageMeter.voxel)))
            let azDeg = Double(atan2(camPos.z - world.z, camPos.x - world.x)) * 180.0 / .pi
            addObservation(vi: vi, bin: CoverageMath.azBin(azDeg))
        }

        // throttled next-angle (avoids the growing O(voxels) per-frame scan)
        sinceNA += 1
        if sinceNA >= CoverageMeter.naThrottle { naCache = computeNextAngle(); sinceNA = 0 }

        return gateState(camPos: camPos)
    }

    /// Incremental occupancy + green count. green is monotone -> once green, skip (O(touched voxels)).
    private func addObservation(vi: SIMD3<Int32>, bin: Int) {
        let mask = UInt16(1) << bin
        if let occ = bins[vi] {
            if greenSet.contains(vi) || (occ & mask) != 0 { return }
            let merged = occ | mask
            bins[vi] = merged
            if CoverageMath.isGreen(merged) { greenSet.insert(vi); greenCount += 1 }
        } else {
            bins[vi] = mask
            if CoverageMath.isGreen(mask) { greenSet.insert(vi); greenCount += 1 }
        }
    }

    /// Current gate cell = camera orbit sector (azimuth) x elevation band (height) around the centre.
    /// v1.3: elevation banding is ADAPTIVE — first `warmupFrames` samples define band edges (P33, P66
    /// of observed elev), so the bands always span the user's actual envelope (no "physically unreachable
    /// outer ring" for any object height). Gate is invalid during warmup; auto-capture defers.
    private func gateState(camPos: SIMD3<Float>) -> GateState {
        guard lookN > 0 else { return GateState(azBin: 0, elevBand: 0, isNew: false, valid: false) }
        let C = center
        let dx = camPos.x - C.x, dz = camPos.z - C.z, dy = camPos.y - C.y
        let horiz = (dx * dx + dz * dz).squareRoot()
        let azDeg = Double(atan2(dz, dx)) * 180.0 / .pi
        let elevDeg = Double(atan2(dy, max(horiz, 1e-4))) * 180.0 / .pi
        let az = CoverageMath.orbitSector(azimuthDeg: azDeg, bins: CoverageMeter.orbitBins)

        if elevEdges == nil {
            // warmup: collect elev samples, lock band edges once we have enough
            elevSamples.append(elevDeg)
            if elevSamples.count >= CoverageMeter.warmupFrames {
                let s = elevSamples.sorted()
                let last = s.count - 1
                let i33 = max(0, min(last, Int(Double(last) * 0.33)))
                let i66 = max(0, min(last, Int(Double(last) * 0.66)))
                elevEdges = [s[i33], s[i66]]
            }
            return GateState(azBin: az, elevBand: 0, isNew: false, valid: false)   // no auto-capture during warmup
        }
        let edges = elevEdges!
        let el = elevDeg < edges[0] ? 0 : (elevDeg < edges[1] ? 1 : 2)
        return GateState(azBin: az, elevBand: el, isNew: !captured[az * CoverageMeter.elevBins + el], valid: true)
    }

    /// Widest missing azimuth wedge over the UNDER-covered surface. O(voxels) — call throttled.
    private func computeNextAngle() -> (azimuth: Double, gapDeg: Double)? {
        var hist = [Double](repeating: 0, count: CoverageMath.nAz)
        for (vi, occ) in bins where !greenSet.contains(vi) {
            for b in 0..<CoverageMath.nAz where (occ & (UInt16(1) << b)) != 0 { hist[b] += 1 }
        }
        return CoverageMath.nextAngle(underHist: hist)
    }
}
