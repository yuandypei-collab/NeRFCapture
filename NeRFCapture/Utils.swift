//
//  Utils.swift
//  NeRFCapture
//
//  Created by Jad Abou-Chakra on 13/7/2022.
//

import Foundation
import ARKit
import ImageIO

func trackingStateToString(_ trackingState: ARCamera.TrackingState) -> String {
        switch trackingState {
            case .notAvailable: return "Not Available"
            case .normal: return "Tracking Normal"
            case .limited(.excessiveMotion): return "Excessive Motion"
            case .limited(.initializing): return "Tracking Initializing"
            case .limited(.insufficientFeatures): return  "Insufficient Features"
            default: return "Unknown"
        }
}

func tupleFromTransform(_ t: matrix_float4x4) -> (Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float) {
    let tuple = (t.columns.0.x, t.columns.0.y, t.columns.0.z, t.columns.0.w,
        t.columns.1.x, t.columns.1.y, t.columns.1.z, t.columns.1.w,
        t.columns.2.x, t.columns.2.y, t.columns.2.z, t.columns.2.w,
        t.columns.3.x, t.columns.3.y, t.columns.3.z, t.columns.3.w
    )
    return tuple
}

func arrayFromTransform(_ transform: matrix_float4x4) -> [[Float]] {
    var array: [[Float]] = Array(repeating: Array(repeating:Float(), count: 4), count: 4)
    array[0] = [transform.columns.0.x, transform.columns.1.x, transform.columns.2.x, transform.columns.3.x]
    array[1] = [transform.columns.0.y, transform.columns.1.y, transform.columns.2.y, transform.columns.3.y]
    array[2] = [transform.columns.0.z, transform.columns.1.z, transform.columns.2.z, transform.columns.3.z]
    array[3] = [transform.columns.0.w, transform.columns.1.w, transform.columns.2.w, transform.columns.3.w]
    return array
}

func arrayFromTransform(_ transform: matrix_float3x3) -> [[Float]] {
    var array: [[Float]] = Array(repeating: Array(repeating:Float(), count: 3), count: 3)
    array[0] = [transform.columns.0.x, transform.columns.1.x, transform.columns.2.x]
    array[1] = [transform.columns.0.y, transform.columns.1.y, transform.columns.2.y]
    array[2] = [transform.columns.0.z, transform.columns.1.z, transform.columns.2.z]
    return array
}

func pixelBufferToUIImage(pixelBuffer: CVPixelBuffer) -> UIImage {
    let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
    let context = CIContext(options: nil)
    let cgImage = context.createCGImage(ciImage, from: ciImage.extent)
    let uiImage = UIImage(cgImage: cgImage!)
    return uiImage
}

// Encode an ARKit sceneDepth Float32 (metres) buffer as a 16-bit single-channel PNG
// in millimetres at the buffer's NATIVE resolution. No resize, no normalisation,
// no 8-bit quantisation. The prior path (pixelBufferToUIImage + resizeImageTo +
// pngData) silently produced 8-bit RGBA at UIScreen 3x — depth saturated to garbage.
// Invalid samples (NaN, Inf, ≤0) map to 0; values ≥65.535 m clamp to 65535.
func depthFloat32BufferToMM16PNGData(_ pixelBuffer: CVPixelBuffer) -> Data? {
    CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
    defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

    let width = CVPixelBufferGetWidth(pixelBuffer)
    let height = CVPixelBufferGetHeight(pixelBuffer)
    let srcBytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
    guard let baseRaw = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }

    var mmBuffer = [UInt16](repeating: 0, count: width * height)
    for y in 0..<height {
        let rowPtr = baseRaw.advanced(by: y * srcBytesPerRow).assumingMemoryBound(to: Float32.self)
        let rowBase = y * width
        for x in 0..<width {
            let meters = rowPtr[x]
            if meters.isFinite && meters > 0 {
                let mm = meters * 1000.0
                mmBuffer[rowBase + x] = mm >= 65535.0 ? 65535 : UInt16(mm)
            }
            // else leaves 0 (sentinel for invalid)
        }
    }

    let outputData = NSMutableData()
    let success: Bool = mmBuffer.withUnsafeBufferPointer { ptr -> Bool in
        guard let bytes = ptr.baseAddress else { return false }
        let length = ptr.count * MemoryLayout<UInt16>.size
        guard let provider = CGDataProvider(data: NSData(bytes: bytes, length: length)) else { return false }
        let bitmapInfo = CGBitmapInfo(
            rawValue: CGImageAlphaInfo.none.rawValue | CGBitmapInfo.byteOrder16Little.rawValue
        )
        guard let cgImage = CGImage(
            width: width,
            height: height,
            bitsPerComponent: 16,
            bitsPerPixel: 16,
            bytesPerRow: width * 2,
            space: CGColorSpaceCreateDeviceGray(),
            bitmapInfo: bitmapInfo,
            provider: provider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        ) else { return false }
        guard let dest = CGImageDestinationCreateWithData(outputData, "public.png" as CFString, 1, nil) else { return false }
        CGImageDestinationAddImage(dest, cgImage, nil)
        return CGImageDestinationFinalize(dest)
    }
    return success ? (outputData as Data) : nil
}

func getDocumentsDirectory() -> URL {
    let paths = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)
    return paths[0]
}


class YUVToRGBFilter {
    
    var device: MTLDevice
    var defaultLib: MTLLibrary?
    var shader: MTLFunction?
    var commandQueue: MTLCommandQueue?
    var commandEncoder: MTLComputeCommandEncoder?
    var pipelineState: MTLComputePipelineState?
    var width: UInt32 = 0
    var height: UInt32 = 0
    let threadsPerBlock = MTLSize(width: 16, height: 16, depth: 1)
    
    var capturedImagePipelineState: MTLRenderPipelineState!
    var capturedImageTextureY: CVMetalTexture?
    var capturedImageTextureCbCr: CVMetalTexture?
    var capturedImageTextureCache: CVMetalTextureCache!
    var rgbBuffer: MTLBuffer!
    
    init() {
        self.device = MTLCreateSystemDefaultDevice()!
        self.defaultLib = self.device.makeDefaultLibrary()
        self.shader = self.defaultLib?.makeFunction(name: "yuv2rgb_kernel")
        self.commandQueue = self.device.makeCommandQueue()
        
        // Create captured image texture cache
        var textureCache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(nil, nil, self.device, nil, &textureCache)
        self.capturedImageTextureCache = textureCache
        
        if let shader = self.shader {
            do {
                try self.pipelineState = self.device.makeComputePipelineState(function: shader)
            } catch {
                fatalError("unable to make compute pipeline")
            }
        }
        else {
            fatalError("unable to make compute pipeline")
        }
    }
    
    func getBlockDimensions() -> MTLSize {
        let blockWidth = Int(width) / self.threadsPerBlock.width
        let blockHeight = Int(height) / self.threadsPerBlock.height
        return MTLSizeMake(blockWidth, blockHeight, 1)
    }
    
    
    func createTexture(fromPixelBuffer pixelBuffer: CVPixelBuffer, pixelFormat: MTLPixelFormat, planeIndex: Int) -> CVMetalTexture? {
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
        var texture: CVMetalTexture? = nil
        let status = CVMetalTextureCacheCreateTextureFromImage(nil, capturedImageTextureCache, pixelBuffer, nil, pixelFormat, width, height, planeIndex, &texture)
        if status != kCVReturnSuccess {
            texture = nil
        }
        return texture
    }
    
    func updateCapturedImageTextures(frame: ARFrame) {
        // Create two textures (Y and CbCr) from the provided frame's captured image
        let pixelBuffer = frame.capturedImage
        if (CVPixelBufferGetPlaneCount(pixelBuffer) < 2) {
            return
        }
        capturedImageTextureY = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.r8Unorm, planeIndex:0)
        capturedImageTextureCbCr = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.rg8Unorm, planeIndex:1)
        
        let w = Int(frame.camera.imageResolution.width)
        let h = Int(frame.camera.imageResolution.height)
        if(w != self.width || h != self.height) {
            rgbBuffer = device.makeBuffer(length: w*h*3, options: .storageModeShared)
        }
        width = UInt32(w)
        height = UInt32(h)

    }
    
    func applyFilter(frame:ARFrame) {
        updateCapturedImageTextures(frame: frame)
        guard let buffer = self.commandQueue?.makeCommandBuffer(), let encoder = buffer.makeComputeCommandEncoder() else {
            return;
        }
        encoder.setComputePipelineState(self.pipelineState!)
        encoder.setTextures([CVMetalTextureGetTexture(capturedImageTextureY!), CVMetalTextureGetTexture(capturedImageTextureCbCr!)], range: 0..<2)
        encoder.setBuffer(rgbBuffer, offset: 0, index: 0)
        encoder.dispatchThreadgroups(self.getBlockDimensions(), threadsPerThreadgroup: threadsPerBlock)
        encoder.endEncoding()
        buffer.commit()
        buffer.waitUntilCompleted()
    }
    
}
