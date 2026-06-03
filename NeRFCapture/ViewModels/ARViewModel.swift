//
//  ARViewModel.swift
//  NeRFCapture
//
//  Created by Jad Abou-Chakra on 13/7/2022.
//  Otterly Spike 2 (v1.1): live CoverageMeter + coverage-gated auto-shutter, run OFF the AR render
//  path on a background serial queue (search "Otterly").
//

import Foundation
import Zip
import Combine
import ARKit
import RealityKit
import simd

enum AppError : Error {
    case projectAlreadyExists
    case manifestInitializationFailed
}

class ARViewModel : NSObject, ARSessionDelegate, ObservableObject {
    @Published var appState = AppState()
    var session: ARSession? = nil
    var arView: ARView? = nil
//    let frameSubject = PassthroughSubject<ARFrame, Never>()
    var cancellables = Set<AnyCancellable>()
    let datasetWriter: DatasetWriter
    let ddsWriter: DDSWriter

    // --- Otterly v2: region+quality coverage + quality-gated auto-capture ---
    private var coverageMeter = CoverageMeter()
    private let coverageQueue = DispatchQueue(label: "com.yuandypei.otterly.coverage")  // serial; meter lives here
    private let coverageHz = 10.0          // throttle the live meter (60fps frames -> ~10 updates/s)
    private let autoMinInterval = 0.4      // min seconds between auto-captures (anti-blur / thermal)
    private let autoMaxFrames = 300        // hard cap on auto-captured frames per session
    private var lastCoverageT: TimeInterval = 0     // AR-queue throttle clock
    private var lastAutoCaptureT: TimeInterval = 0  // coverageQueue only
    private var autoCaptured = 0                    // coverageQueue only
    // steady gate: only frames slow enough to be sharp count toward coverage / auto-capture. Starting
    // thresholds -> calibrate on real capture (lights on, hand-held walk).
    private static let angVelMax: Float = 0.30      // rad/s (~17 deg/s) max camera rotation to count as steady
    private static let linVelMax: Float = 0.40      // m/s   max camera translation to count as steady
    private var prevC2W: simd_float4x4? = nil       // AR queue only: previous processed-frame pose (for velocity)
    private var prevPoseT: TimeInterval = 0         // AR queue only

    init(datasetWriter: DatasetWriter, ddsWriter: DDSWriter) {
        self.datasetWriter = datasetWriter
        self.ddsWriter = ddsWriter
        super.init()
        self.setupObservers()
        self.ddsWriter.setupDDS()
    }

    func setupObservers() {
        datasetWriter.$writerState.sink { x in
            self.appState.writerState = x
            if x == .SessionStarted { self.resetCoverage() }   // Otterly: fresh meter per capture session
        }.store(in: &cancellables)
        datasetWriter.$currentFrameCounter.sink { x in self.appState.numFrames = x }.store(in: &cancellables)
        ddsWriter.$peers.sink {x in self.appState.ddsPeers = UInt32(x)}.store(in: &cancellables)

        $appState
            .map(\.appMode)
            .prepend(appState.appMode)
            .removeDuplicates()
            .sink { x in
                switch x {
                case .Offline:
                    print("Changed to offline")
                case .Online:
                    print("Changed to online")
                }
            }
            .store(in: &cancellables)
    }

    // Otterly Spike 2 ---------------------------------------------------------
    private func resetCoverage() {
        coverageQueue.async {
            self.coverageMeter = CoverageMeter()
            self.autoCaptured = 0
            self.lastAutoCaptureT = 0
        }
        appState.sessionStart = Date()
        appState.autoCaptureCount = 0
        appState.coverageGreenFrac = 0
        appState.coverageVoxels = 0
        appState.nextAngleAz = nil
        appState.nextAngleGap = 0
        appState.holdSteady = false
        // prevC2W / prevPoseT are AR-queue state; they self-heal on the next frame (the dt>0.5s guard in
        // session(_:didUpdate:) treats the post-reset gap as not-steady), so they are intentionally NOT reset here.
    }

    /// Copy a DepthFloat32 pixel buffer into a Swift [Float] on the AR queue (buffers are only valid now),
    /// so the heavy meter work can run later on the background queue without touching the ARFrame.
    private static func copyDepth(_ pb: CVPixelBuffer) -> ([Float], Int, Int) {
        CVPixelBufferLockBaseAddress(pb, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pb, .readOnly) }
        let w = CVPixelBufferGetWidth(pb), h = CVPixelBufferGetHeight(pb)
        guard let base = CVPixelBufferGetBaseAddress(pb), w > 0, h > 0 else { return ([], 0, 0) }
        let rowBytes = CVPixelBufferGetBytesPerRow(pb)
        var out = [Float](repeating: 0, count: w * h)
        for r in 0..<h {
            let src = base.advanced(by: r * rowBytes).assumingMemoryBound(to: Float.self)
            for c in 0..<w { out[r * w + c] = src[c] }
        }
        return (out, w, h)
    }

    /// Copy the LiDAR confidence map (UInt8 per pixel: 0=low,1=med,2=high) on the AR queue.
    /// Same dims as the depth map; used to drop low-confidence depth before it pollutes coverage.
    private static func copyConfidence(_ pb: CVPixelBuffer) -> [UInt8] {
        CVPixelBufferLockBaseAddress(pb, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pb, .readOnly) }
        let w = CVPixelBufferGetWidth(pb), h = CVPixelBufferGetHeight(pb)
        guard let base = CVPixelBufferGetBaseAddress(pb), w > 0, h > 0 else { return [] }
        let rowBytes = CVPixelBufferGetBytesPerRow(pb)
        var out = [UInt8](repeating: 0, count: w * h)
        for r in 0..<h {
            let src = base.advanced(by: r * rowBytes).assumingMemoryBound(to: UInt8.self)
            for c in 0..<w { out[r * w + c] = src[c] }
        }
        return out
    }
    // -------------------------------------------------------------------------

    func createARConfiguration() -> ARWorldTrackingConfiguration {
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        if type(of: configuration).supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics = .sceneDepth
        }
        return configuration
    }

    func resetWorldOrigin() {
        session?.pause()
        let config = createARConfiguration()
        session?.run(config, options: [.resetTracking])
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // --- Otterly Spike 2 (v1.1): extract on the AR queue, run the meter off the render path -----
        let t = frame.timestamp
        guard t - lastCoverageT >= 1.0 / coverageHz else { return }   // ~10Hz throttle (AR queue)
        lastCoverageT = t
        guard appState.appMode == .Offline, appState.writerState == .SessionStarted,
              let depthMap = frame.sceneDepth?.depthMap else { return }

        let (depth, dw, dh) = Self.copyDepth(depthMap)   // cheap copy; ARFrame buffers valid only now
        guard dw > 0, dh > 0 else { return }
        let conf = frame.sceneDepth?.confidenceMap.map(Self.copyConfidence)   // nil if unavailable -> accept all
        let K = frame.camera.intrinsics
        let rgb = frame.camera.imageResolution
        let c2w = frame.camera.transform
        var trackingNormal = false
        if case .normal = frame.camera.trackingState { trackingNormal = true }
        let enabled = appState.autoCapture   // Bool read; tolerate one-frame staleness (prototype)

        // steady gate (AR queue, serial): rotation + translation speed vs the previous processed frame.
        // Only steady frames are sharp enough to count toward coverage / be auto-captured (kills motion blur).
        var steady = false
        if let p = prevC2W, prevPoseT > 0 {
            let dt = Float(t - prevPoseT)
            if dt > 1e-3, dt < 0.5 {     // ignore stale gaps (paused/restarted session, dropped frames)
                let cur = c2w.columns.3, prv = p.columns.3
                let linVel = simd_length(SIMD3<Float>(cur.x - prv.x, cur.y - prv.y, cur.z - prv.z)) / dt
                let q0 = simd_quatf(simd_float3x3(SIMD3(p.columns.0.x, p.columns.0.y, p.columns.0.z),
                                                  SIMD3(p.columns.1.x, p.columns.1.y, p.columns.1.z),
                                                  SIMD3(p.columns.2.x, p.columns.2.y, p.columns.2.z)))
                let q1 = simd_quatf(simd_float3x3(SIMD3(c2w.columns.0.x, c2w.columns.0.y, c2w.columns.0.z),
                                                  SIMD3(c2w.columns.1.x, c2w.columns.1.y, c2w.columns.1.z),
                                                  SIMD3(c2w.columns.2.x, c2w.columns.2.y, c2w.columns.2.z)))
                let dotq = min(Float(1), abs(simd_dot(q0.vector, q1.vector)))
                let angVel = Float(2.0 * acos(Double(dotq))) / dt   // radians/s
                steady = angVel < ARViewModel.angVelMax && linVel < ARViewModel.linVelMax
            }
        }
        prevC2W = c2w; prevPoseT = t

        coverageQueue.async {
            // Only a steady frame with NORMAL tracking has a trustworthy pose -> anything else would
            // pollute the voxel bearings, so skip the meter entirely. (Show "稳住" only for motion blur,
            // not for a tracking drop-out.)
            guard steady, trackingNormal else {
                DispatchQueue.main.async { self.appState.holdSteady = !steady }
                return
            }
            let upd = self.coverageMeter.update(depth: depth, conf: conf, dw: dw, dh: dh,
                                                intrinsics: K, rgbSize: rgb, camera: c2w)
            // quality-gated auto-capture: fire only when a STEADY frame adds enough NEW viewing-angle
            // coverage to under-covered regions (upd.informative), with anti-blur interval + hard cap.
            // (tracking is already guaranteed normal by the guard above.)
            var fired = false
            if enabled, upd.informative,
               t - self.lastAutoCaptureT >= self.autoMinInterval,
               self.autoCaptured < self.autoMaxFrames {
                self.lastAutoCaptureT = t
                self.autoCaptured += 1
                fired = true
            }
            let gf = self.coverageMeter.greenFrac
            let vox = self.coverageMeter.admittedVoxels
            let na = self.coverageMeter.cachedNextAngle()
            let n = self.autoCaptured
            DispatchQueue.main.async {
                self.appState.holdSteady = false
                self.appState.coverageGreenFrac = gf
                self.appState.coverageVoxels = vox
                self.appState.nextAngleAz = na?.azimuth
                self.appState.nextAngleGap = na?.gapDeg ?? 0
                if fired {
                    // NOTE: writes session.currentFrame (a hair later than the gated frame t). At handheld
                    // speeds the pose drift is sub-voxel; retaining the exact ARFrame is discouraged by ARKit.
                    if let f = self.session?.currentFrame { self.datasetWriter.writeFrameToDisk(frame: f) }
                    self.appState.autoCaptureCount = n
                }
            }
        }
        // ------------------------------------------------------------------------------------------
    }

    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        self.appState.trackingState = trackingStateToString(camera.trackingState)
    }
}
