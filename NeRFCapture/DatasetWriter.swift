//
//  DatasetWriter.swift
//  NeRFCapture
//
//  Created by Jad Abou-Chakra on 11/1/2023.
//

import Foundation
import ARKit
import Zip

extension UIImage {
    func resizeImageTo(size: CGSize) -> UIImage? {
        UIGraphicsBeginImageContextWithOptions(size, false, 0.0)
        self.draw(in: CGRect(origin: CGPoint.zero, size: size))
        let resizedImage = UIGraphicsGetImageFromCurrentImageContext()!
        UIGraphicsEndImageContext()
        return resizedImage
    }
}

class DatasetWriter {
    
    enum SessionState {
        case SessionNotStarted
        case SessionStarted
    }
    
    var manifest = Manifest()
    var projectName = ""
    var projectDir = getDocumentsDirectory()
    var useDepthIfAvailable = true
    
    @Published var currentFrameCounter = 0
    @Published var writerState = SessionState.SessionNotStarted
    
    func projectExists(_ projectDir: URL) -> Bool {
        var isDir: ObjCBool = true
        return FileManager.default.fileExists(atPath: projectDir.absoluteString, isDirectory: &isDir)
    }
    
    func initializeProject() throws {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "YYMMddHHmmss"
        projectName = dateFormatter.string(from: Date())
        projectDir = getDocumentsDirectory()
            .appendingPathComponent(projectName)
        if projectExists(projectDir) {
            throw AppError.projectAlreadyExists
        }
        do {
            try FileManager.default.createDirectory(at: projectDir.appendingPathComponent("images"), withIntermediateDirectories: true)
        }
        catch {
            print(error)
        }
        
        manifest = Manifest()
        
        // The first frame will set these properly
        manifest.w = 0
        manifest.h = 0
        
        // These don't matter since every frame will redefine them
        manifest.flX = 1.0
        manifest.flY =  1.0
        manifest.cx =  320
        manifest.cy =  240
        
        // Depth PNGs are 16-bit single-channel millimetres at the LiDAR's native 256x192
        // — metres = pixel * depthIntegerScale. The previous 1.0 only made sense for the
        // (broken) 8-bit RGBA depth viz path; consumers reading that as metric would have
        // been off by 1000x even ignoring the saturation bug.
        manifest.depthIntegerScale = 0.001
        currentFrameCounter = 0
        writerState = .SessionStarted
    }
    
    func clean() {
        guard case .SessionStarted = writerState else { return; }
        writerState = .SessionNotStarted
        DispatchQueue.global().async {
            do {
                try FileManager.default.removeItem(at: self.projectDir)
            }
            catch {
                print("Could not cleanup project files")
            }
        }
    }
    
    func finalizeProject(zip: Bool = true) {
        writerState = .SessionNotStarted
        let manifest_path = getDocumentsDirectory()
            .appendingPathComponent(projectName)
            .appendingPathComponent("transforms.json")

        writeManifestToPath(path: manifest_path)
        // Snapshot before async — guard against initializeProject() landing a new value
        // mid-finalize and our cleanup nuking the wrong directory.
        let dirToFinalize = self.projectDir
        let nameToFinalize = self.projectName
        DispatchQueue.global().async {
            if zip {
                // Day-1 observation: Zip.quickZipFiles silently failed on ~5/8 captures,
                // leaving a partial .zip (no end-of-central-directory) that `unzip -t`
                // rejects. The original do/catch swallowed the error as "Could not zip"
                // with no detail and offered no recovery path. Now: surface the actual
                // error, retry once after cleaning the partial output, and on final
                // failure delete the partial .zip + preserve the project dir so
                // devicectl can pull the raw files.
                let zipURL = getDocumentsDirectory().appendingPathComponent("\(nameToFinalize).zip")
                var zipOK = false
                for attempt in 1...2 {
                    if FileManager.default.fileExists(atPath: zipURL.path) {
                        try? FileManager.default.removeItem(at: zipURL)
                    }
                    do {
                        let _ = try Zip.quickZipFiles([dirToFinalize], fileName: nameToFinalize)
                        zipOK = true
                        break
                    } catch {
                        print("NeRFCapture: Zip attempt \(attempt)/2 failed: \(error.localizedDescription)")
                    }
                }
                if !zipOK {
                    if FileManager.default.fileExists(atPath: zipURL.path) {
                        try? FileManager.default.removeItem(at: zipURL)
                    }
                    print("NeRFCapture: zip failed both attempts. Project dir preserved at \(dirToFinalize.path)")
                    print("NeRFCapture: recover via `xcrun devicectl device copy from --source Documents/\(nameToFinalize) ...`, then `zip -qr` locally.")
                    return
                }
            }
            do {
                try FileManager.default.removeItem(at: dirToFinalize)
            } catch {
                print("NeRFCapture: zip OK but project-dir cleanup failed at \(dirToFinalize.path): \(error.localizedDescription)")
            }
        }
    }
    
    func getCurrentFrameName() -> String {
        let frameName = String(currentFrameCounter)
        return frameName
    }
    
    func getFrameMetadata(_ frame: ARFrame, withDepth: Bool = false, withConfidence: Bool = false) -> Manifest.Frame {
        let frameName = getCurrentFrameName()
        let filePath = "images/\(frameName)"
        let depthPath = "images/\(frameName).depth.png"
        let confidencePath = "images/\(frameName).confidence.png"
        let manifest_frame = Manifest.Frame(
            filePath: filePath,
            depthPath: withDepth ? depthPath : nil,
            confidencePath: withConfidence ? confidencePath : nil,
            transformMatrix: arrayFromTransform(frame.camera.transform),
            timestamp: frame.timestamp,
            flX:  frame.camera.intrinsics[0, 0],
            flY:  frame.camera.intrinsics[1, 1],
            cx:  frame.camera.intrinsics[2, 0],
            cy:  frame.camera.intrinsics[2, 1],
            w: Int(frame.camera.imageResolution.width),
            h: Int(frame.camera.imageResolution.height)
        )
        return manifest_frame
    }
    
    func writeManifestToPath(path: URL) {
        let encoder = JSONEncoder()
        encoder.keyEncodingStrategy = .convertToSnakeCase
        encoder.outputFormatting = .withoutEscapingSlashes
        if let encoded = try? encoder.encode(manifest) {
            do {
                try encoded.write(to: path)
            } catch {
                print(error)
            }
        }
    }
    
    func writeFrameToDisk(frame: ARFrame, useDepthIfAvailable: Bool = true) {
        let frameName =  "\(getCurrentFrameName()).png"
        let depthFrameName =  "\(getCurrentFrameName()).depth.png"
        let confidenceFrameName =  "\(getCurrentFrameName()).confidence.png"
        let baseDir = projectDir
            .appendingPathComponent("images")
        let fileName = baseDir
            .appendingPathComponent(frameName)
        let depthFileName = baseDir
            .appendingPathComponent(depthFrameName)
        let confidenceFileName = baseDir
            .appendingPathComponent(confidenceFrameName)

        if manifest.w == 0 {
            manifest.w = Int(frame.camera.imageResolution.width)
            manifest.h = Int(frame.camera.imageResolution.height)
            manifest.flX =  frame.camera.intrinsics[0, 0]
            manifest.flY =  frame.camera.intrinsics[1, 1]
            manifest.cx =  frame.camera.intrinsics[2, 0]
            manifest.cy =  frame.camera.intrinsics[2, 1]
        }

        let useDepth = frame.sceneDepth != nil && useDepthIfAvailable
        let useConfidence = useDepth && frame.sceneDepth!.confidenceMap != nil

        let frameMetadata = getFrameMetadata(frame, withDepth: useDepth, withConfidence: useConfidence)
        let rgbBuffer = pixelBufferToUIImage(pixelBuffer: frame.capturedImage)
        // Native 256x192, 16-bit single-channel mm. The old pixelBufferToUIImage + resizeImageTo
        // + pngData path quantised Float32 metres to 8-bit RGBA and screen-3x upscaled it,
        // saturating depth to ~255 garbage. The metric path now reads the Float32 buffer
        // directly and writes a 16-bit grayscale PNG at the LiDAR's native resolution.
        let depthPNGData = useDepth ? depthFloat32BufferToMM16PNGData(frame.sceneDepth!.depthMap) : nil
        // Native 256x192 — bilinear resize would corrupt raw 0/1/2 ARConfidenceLevel values.
        let confidenceBuffer = useConfidence ? pixelBufferToUIImage(pixelBuffer: frame.sceneDepth!.confidenceMap!) : nil

        DispatchQueue.global().async {
            do {
                let rgbData = rgbBuffer.pngData()
                try rgbData?.write(to: fileName)
                if useDepth, let depthData = depthPNGData {
                    try depthData.write(to: depthFileName)
                }
                if useConfidence {
                    let confidenceData = confidenceBuffer!.pngData()
                    try confidenceData?.write(to: confidenceFileName)
                }
            }
            catch {
                print(error)
            }
            DispatchQueue.main.async {
                self.manifest.frames.append(frameMetadata)
            }
        }
        currentFrameCounter += 1
    }
}
