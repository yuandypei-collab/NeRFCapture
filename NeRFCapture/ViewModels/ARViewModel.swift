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

    // --- Otterly Spike 2: guidance coverage + auto-capture ---
    private var coverageMeter = CoverageMeter()
    private let coverageQueue = DispatchQueue(label: "com.yuandypei.otterly.coverage")  // serial; meter lives here
    private let coverageHz = 10.0          // throttle the live meter (60fps frames -> ~10 updates/s)
    private let autoMinInterval = 0.4      // min seconds between auto-captures (anti-blur / thermal)
    private let autoMaxFrames = 300        // hard cap on auto-captured frames per session
    private var lastCoverageT: TimeInterval = 0     // AR-queue throttle clock
    private var lastAutoCaptureT: TimeInterval = 0  // coverageQueue only
    private var autoCaptured = 0                    // coverageQueue only

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
        appState.capturedCells = Array(repeating: false, count: CoverageMeter.orbitBins * CoverageMeter.elevBins)
        appState.nextAngleAz = nil
        appState.nextAngleGap = 0
        appState.calibratingHeight = true
        appState.calibrationProgress = 0
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
        let K = frame.camera.intrinsics
        let rgb = frame.camera.imageResolution
        let c2w = frame.camera.transform
        var trackingNormal = false
        if case .normal = frame.camera.trackingState { trackingNormal = true }
        let enabled = appState.autoCapture   // Bool read; tolerate one-frame staleness (prototype)

        coverageQueue.async {
            let g = self.coverageMeter.update(depth: depth, dw: dw, dh: dh, intrinsics: K,
                                              rgbSize: rgb, camera: c2w)
            // coverage-gated auto-capture: fire only on a NEW (orbit-sector x elevation-band) cell (adds
            // viewing-angle / height diversity), with anti-blur interval + cap. Decide + mark BEFORE reading
            // capturedCells so the published HUD grid reflects this capture immediately (no one-frame lag).
            var fired = false
            if enabled, g.isNew, g.valid, trackingNormal,
               t - self.lastAutoCaptureT >= self.autoMinInterval,
               self.autoCaptured < self.autoMaxFrames {
                self.lastAutoCaptureT = t
                self.coverageMeter.markCellCaptured(az: g.azBin, elev: g.elevBand)
                self.autoCaptured += 1
                fired = true
            }
            let gf = self.coverageMeter.greenFrac
            let vox = self.coverageMeter.voxelCount
            let cells = self.coverageMeter.capturedCellsFlat()
            let na = self.coverageMeter.cachedNextAngle()
            let n = self.autoCaptured
            let calibrating = self.coverageMeter.isWarmingUp
            let calibProgress = self.coverageMeter.warmupProgress
            DispatchQueue.main.async {
                self.appState.coverageGreenFrac = gf
                self.appState.coverageVoxels = vox
                self.appState.capturedCells = cells
                self.appState.nextAngleAz = na?.azimuth
                self.appState.nextAngleGap = na?.gapDeg ?? 0
                self.appState.calibratingHeight = calibrating
                self.appState.calibrationProgress = calibProgress
                if fired {
                    // NOTE: writes session.currentFrame (a hair later than the gated frame t). At handheld
                    // speeds the pose drift is sub-sector; retaining the exact ARFrame is discouraged by ARKit.
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
