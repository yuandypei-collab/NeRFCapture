//
//  AppState.swift
//  NeRFCapture
//
//  Created by Jad Abou-Chakra on 13/7/2022.
//  Otterly Spike 2: added guidance-coverage + auto-capture fields (marked below).
//

import Foundation
import Metal
import MetalKit

enum AppMode: Int, Codable {
    case Online
    case Offline
}

struct AppState {
    var appMode: AppMode = .Online
    var writerState: DatasetWriter.SessionState = .SessionNotStarted

    var trackingState = ""
    var projectName = ""
    var numFrames = 0
    var supportsDepth = false
//    var stream = false

    var ddsPeers: UInt32 = 0
    var ddsReady = false

    // --- Otterly guidance prototype (Spike 2) — published live from CoverageMeter ---
    var coverageGreenFrac: Float = 0          // fraction of object surface seen from wide-enough angles
    var coverageVoxels: Int = 0               // observed surface voxels (debug / sanity)
    // gate grid captured-from flags, flat index = orbitSector * elevBins + elevBand.
    // Size sourced from CoverageMeter (single source of truth) so it can't drift from the meter's grid.
    var capturedCells: [Bool] = Array(repeating: false, count: CoverageMeter.orbitBins * CoverageMeter.elevBins)
    var nextAngleAz: Double? = nil            // recommended orbit azimuth (world x-z, deg); nil = covered/diffuse
    var nextAngleGap: Double = 0              // width of the missing wedge (deg)
    var autoCapture: Bool = true              // coverage-gated auto-shutter on/off (Andy's choice: ON by default)
    var autoCaptureCount: Int = 0             // frames auto-captured this session
    var sessionStart: Date? = nil             // for the elapsed-time display
    // v1.3 adaptive elev-band warmup status (drives HUD hint; auto-capture defers until done)
    var calibratingHeight: Bool = true
    var calibrationProgress: Float = 0        // 0..1
}

struct AppSettings: Codable {
    var zipDataset = true
    var startingAppMode = AppMode.Online
}



struct MetalState {
    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!

    var sharedUniformBuffer: MTLBuffer!
    var imagePlaneVertexBuffer: MTLBuffer!

    var capturedImagePipelineState: MTLRenderPipelineState!
    var capturedImageTextureY: CVMetalTexture?
    var capturedImageTextureCbCr: CVMetalTexture?
    var capturedImageTextureCache: CVMetalTextureCache!
}
