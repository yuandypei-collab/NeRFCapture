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

    // --- Otterly guidance prototype (v2) — published live from CoverageMeter ---
    var coverageGreenFrac: Float = 0          // fraction of admitted surface seen from wide-enough angles (quality %)
    var coverageVoxels: Int = 0               // admitted (>=3-frame) whole-house surface voxels (debug / sanity)
    var nextAngleAz: Double? = nil            // recommended orbit azimuth (world x-z, deg); nil = covered/diffuse
    var nextAngleGap: Double = 0              // width of the missing wedge (deg)
    var autoCapture: Bool = true              // quality-gated auto-shutter on/off (Andy's choice: ON by default)
    var autoCaptureCount: Int = 0             // frames auto-captured this session
    var sessionStart: Date? = nil             // for the elapsed-time display
    var holdSteady: Bool = false              // phone moving too fast for a clean capture -> HUD "稳住"
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
