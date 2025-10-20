//
//  MotionKit.swift
//  AcceloCube
//
//  Created by Weeraphot Bumbaugh on 10/6/25.
//
import Foundation
import SceneKit
import CoreMotion
import SwiftUI
import simd
import Combine
import QuartzCore

struct MotionConfig {
    var sampleHz: Double = 60
    var smoothing: Double = 0.2         // 0..1 (low-pass factor)
    var damping: Double = 0.02          // 0..0.2 per-tick velocity damping
    var maxSpeed: Float = 5.0           // m/s
    var maxRange: Float = 2.0           // meters (position clamp)
    var loggingEnabled: Bool = false
}

@MainActor
final class MotionVM: ObservableObject {
    @Published var cfg = MotionConfig()
    @Published var quat: simd_quatf = simd_quatf(angle: 0, axis: SIMD3<Float>(0,1,0))
    @Published var pos: SIMD3<Float> = .zero
    @Published var status: String = "Idle"
    @Published var sampleLatencyMs: Double = 0
    @Published var usingDeviceMotion: Bool = false
    @Published var lastAttitude: CMQuaternion? = nil
    @Published var trailPoints: [SIMD3<Float>] = []   // for SceneKit trail
    @Published var autoResumeOnActive: Bool = true
    @Published var hudAuth: String = "unknown"   // Motion & Fitness authorization snapshot
    @Published var hudQueue: String = "op-queue" // which queue we're using
    @Published var useHighPass: Bool = false
    private var lpAccel: SIMD3<Float> = .zero   // low-pass accumulator for accel
    private let trailCap = 128                        // max points to keep
    private let trailMinStep: Float = 0.01            // append only if moved ≥ 1cm
    private var hasCalibrated = false
    
    // Lifecycle
    private let mgr = CMMotionManager()
    private let queue = OperationQueue()
    
    // Kinematics
    private var v: SIMD3<Float> = .zero
    private var lastTimestamp: Double?
    
    // Calibration: neutral orientation alignment
    private var neutralInv: simd_quatf = simd_quatf(angle: 0, axis: SIMD3<Float>(0,1,0))
    
    // Logging
    private var logger: CSVLogger? = nil
    
    init() {
        queue.name = "MotionVM.queue"
        queue.qualityOfService = .userInteractive
        refreshAuthHUD()
    }

    private func refreshAuthHUD() {
        // Proxy for Motion & Fitness permission
        if #available(iOS 11.0, *) {
            switch CMMotionActivityManager.authorizationStatus() {
            case .authorized: hudAuth = "authorized"
            case .denied:     hudAuth = "denied"
            case .restricted: hudAuth = "restricted"
            case .notDetermined: hudAuth = "notDetermined"
            @unknown default: hudAuth = "unknown"
            }
        } else {
            hudAuth = "n/a"
        }
        hudQueue = queue.underlyingQueue?.label ?? "op-queue"
    }
    
    func start() {
        refreshAuthHUD()
        guard mgr.isDeviceMotionAvailable else {
            status = "DeviceMotion unavailable"
            usingDeviceMotion = false
            return
        }
        stop()
        usingDeviceMotion = true
        mgr.deviceMotionUpdateInterval = 1.0 / max(1.0, cfg.sampleHz)
        lastTimestamp = nil
        v = .zero
        hasCalibrated = false            // reset first-run calibration
        status = "Starting..."
        if cfg.loggingEnabled {
            logger = CSVLogger(filename: "accelocube_log.csv")
            logger?.writeHeaderIfNeeded()
        } else {
            logger = nil
        }

        // Start fused DeviceMotion in a stable reference frame
        mgr.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: queue, withHandler: { [weak self] dm, err in
            guard let self = self else { return }
            // Hop to the main actor to touch @Published and other actor-isolated state
            Task { @MainActor in
                self.handleDeviceMotion(dm: dm, err: err)
            }
        })
    }

    // MARK: - Split-out handler (runs on the main actor)
    private func handleDeviceMotion(dm: CMDeviceMotion?, err: Error?) {
        guard let dm = dm else {
            if let err = err {
                self.status = "Error: \(err.localizedDescription)"
            }
            return
        }

        // Compute dt (seconds) from the monotonic timestamp; first frame gets 0
        let now = CACurrentMediaTime()
        let ts = dm.timestamp
        let dt: Double = (self.lastTimestamp.map { max(0, ts - $0) }) ?? 0
        self.lastTimestamp = ts

        // Read the device attitude as a quaternion (this is in device coordinates)
        let aq = dm.attitude.quaternion
        let qDevice = simd_quatf(ix: Float(aq.x), iy: Float(aq.y), iz: Float(aq.z), r: Float(aq.w))

        // First-run calibration: treat current orientation as "neutral" once
        if !self.hasCalibrated {
            self.neutralInv = qDevice.inverse   // store inverse so neutral * current = identity
            self.hasCalibrated = true
        }

        // Apply calibration so orientation is in neutral/world frame
        let qCal = self.neutralInv * qDevice

        let ua = SIMD3<Float>(Float(dm.userAcceleration.x),
                              Float(dm.userAcceleration.y),
                              Float(dm.userAcceleration.z))
        let aWorld = qCal.act(ua)
        
        // Low-pass accel with α = cfg.smoothing; then optional high-pass
        let alpha = Float(min(max(self.cfg.smoothing, 0.0), 0.98)) // 0…0.98
        lpAccel = alpha * lpAccel + (1 - alpha) * aWorld
        let accForIntegration = self.useHighPass ? (aWorld - lpAccel) : aWorld

        // Integrate acceleration → velocity with simple damping and speed clamp
        var vNew = self.v + accForIntegration * Float(dt)
        if !(vNew.x.isFinite && vNew.y.isFinite && vNew.z.isFinite) { vNew = .zero } // NaN/Inf guard
        let speed = length(vNew)
        if speed > self.cfg.maxSpeed { vNew = normalize(vNew) * self.cfg.maxSpeed } // cap speed
        vNew *= max(0, 1.0 - Float(self.cfg.damping))                               // apply damping

        var pNew = self.pos + vNew * Float(dt)
        pNew = simd_clamp(pNew,
                          SIMD3<Float>(repeating: -self.cfg.maxRange),
                          SIMD3<Float>(repeating:  self.cfg.maxRange))

        // Guard quaternion before using it, protect against NaN
        func isFinite(_ q: simd_quatf) -> Bool {
            q.real.isFinite && q.imag.x.isFinite && q.imag.y.isFinite && q.imag.z.isFinite
        }
        let qSafe = isFinite(qCal) ? qCal : simd_quatf(angle: 0, axis: SIMD3<Float>(0,1,0))
        let qSmoothed = simd_slerp(self.quat, qSafe, 1 - alpha)

        // Publish to the UI (we're already on main)
        self.lastAttitude = aq
        self.quat = qSmoothed
        self.v = vNew
        self.pos = pNew

        // Trail buffer (append only if moved ≥ 1cm; cap length)
        if let last = self.trailPoints.last {
            if simd_distance(last, pNew) >= self.trailMinStep { self.trailPoints.append(pNew) }
        } else {
            self.trailPoints.append(pNew)
        }
        if self.trailPoints.count > self.trailCap {
            self.trailPoints.removeFirst(self.trailPoints.count - self.trailCap)
        }

        self.sampleLatencyMs = (CACurrentMediaTime() - now) * 1000.0
        self.status = "OK \(Int(self.cfg.sampleHz)) Hz | v=\(String(format: "%.2f", length(vNew))) m/s | pos=\(self.formatVec(pNew)) m"

        if let logger = self.logger, dt > 0 {
            logger.writeRow(timestamp: ts,
                            q: qSmoothed,
                            userAccel: ua,
                            pos: pNew)
        }
    }
    
    func stop() {
        if mgr.isDeviceMotionActive { mgr.stopDeviceMotionUpdates() }
        usingDeviceMotion = false
        status = "Stopped"
    }
    
    func toggle() { usingDeviceMotion ? stop() : start() }
    
    func calibrateNeutral(currentAttitude: CMQuaternion?) {
        guard let cq = currentAttitude else { return }
        let q = simd_quatf(ix: Float(cq.x), iy: Float(cq.y), iz: Float(cq.z), r: Float(cq.w))
        neutralInv = q.inverse
    }
    
    func applySampleRate() {
        if usingDeviceMotion { start() }
    }
    
    private func formatVec(_ v: SIMD3<Float>) -> String { String(format: "%.2f, %.2f, %.2f", v.x, v.y, v.z) }
    
    // Make current attitude the new neutral
    func calibrate() {
        guard let aq = lastAttitude else { return }
        let q = simd_quatf(ix: Float(aq.x), iy: Float(aq.y), iz: Float(aq.z), r: Float(aq.w))
        neutralInv = q.inverse
        // keep velocity/position unless user explicitly re-centers
        status = "Calibrated"
    }

    // Zero velocity and position (recenters translation)
    func recenter() {
        v = .zero
        pos = .zero
        status = "Re-centered"
    }

    // Restart motion if running so slider/toggles take effect live
    func applyConfigChange() {
        if usingDeviceMotion { start() }
    }
}

private extension SIMD3 where Scalar == Float {
    var allFinite: Bool { x.isFinite && y.isFinite && z.isFinite }
}

final class CSVLogger {
    private let url: URL
    private var wroteHeader = false
    private let fm = FileManager.default
    
    init?(filename: String) {
        do {
            let docs = try fm.url(for: .documentDirectory, in: .userDomainMask, appropriateFor: nil, create: true)
            url = docs.appendingPathComponent(filename)
        } catch { return nil }
    }
    
    func writeHeaderIfNeeded() {
        guard wroteHeader == false else { return }
        let header = "timestamp,qx,qy,qz,qw,ax,ay,az,px,py,pz\n"
        append(text: header)
        wroteHeader = true
    }
    
    func writeRow(timestamp: Double, q: simd_quatf, userAccel: SIMD3<Float>, pos: SIMD3<Float>) {
        let row = String(format: "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                         timestamp, q.imag.x, q.imag.y, q.imag.z, q.real,
                         userAccel.x, userAccel.y, userAccel.z,
                         pos.x, pos.y, pos.z)
        append(text: row)
    }
    
    private func append(text: String) {
        guard let data = text.data(using: .utf8) else { return }
        if FileManager.default.fileExists(atPath: url.path) {
            if let handle = try? FileHandle(forWritingTo: url) {
                try? handle.seekToEnd()
                try? handle.write(contentsOf: data)
                try? handle.close()
            }
        } else {
            try? data.write(to: url)
        }
    }
}

struct SceneViewBridge: UIViewRepresentable {
    @ObservedObject var vm: MotionVM
    
    func makeUIView(context: Context) -> SCNView {
        let view = SCNView()
        view.scene = makeScene()
        view.allowsCameraControl = true
        view.autoenablesDefaultLighting = false
        view.backgroundColor = .black
        context.coordinator.cubeNode = view.scene?.rootNode.childNode(withName: "cube", recursively: true)
        context.coordinator.cameraNode = view.scene?.rootNode.childNode(withName: "camera", recursively: true)
        context.coordinator.trailNode = view.scene?.rootNode.childNode(withName: "trail", recursively: true)
        return view
    }
    
    func updateUIView(_ view: SCNView, context: Context) {
        guard let cube = context.coordinator.cubeNode else { return }
        let q = vm.quat
        cube.orientation = SCNQuaternion(q.imag.x, q.imag.y, q.imag.z, q.real)
        cube.position = SCNVector3(vm.pos.x, vm.pos.y, vm.pos.z)

        // Rebuild trail when points change
        if let trailNode = context.coordinator.trailNode {
            context.coordinator.rebuildTrail(on: trailNode, with: vm.trailPoints)
        }
    }
    func makeCoordinator() -> Coordinator { Coordinator() }
    
    final class Coordinator {
        var cubeNode: SCNNode?
        var cameraNode: SCNNode?
        var trailNode: SCNNode?
        
        // simple throttle state
        private var lastTrailBuild: CFTimeInterval = 0
        private let minTrailInterval: CFTimeInterval = 1.0 / 30.0  // ~30 Hz

        // Build a polyline from points (fades tail via vertex colors)
        func rebuildTrail(on node: SCNNode, with pts: [SIMD3<Float>]) {
            // throttle to ~30 Hz
            let now = CACurrentMediaTime()
            if now - lastTrailBuild < minTrailInterval { return }
            lastTrailBuild = now

            guard pts.count >= 2 else {
                node.geometry = nil
                return
            }

            // Vertices
            let scnVerts: [SCNVector3] = pts.map { SCNVector3($0.x, $0.y, $0.z) }
            let vertexSource = SCNGeometrySource(vertices: scnVerts)

            // Indices to draw as a connected line strip (pair-wise segments)
            // SceneKit lacks "line strip", so build N-1 segments: (0,1), (1,2), ...
            var indices = [UInt32]()
            indices.reserveCapacity((scnVerts.count - 1) * 2)
            for i in 0..<(scnVerts.count - 1) {
                indices.append(UInt32(i))
                indices.append(UInt32(i + 1))
            }
            let indexData = Data(bytes: indices, count: indices.count * MemoryLayout<UInt32>.size)
            let element = SCNGeometryElement(data: indexData,
                                             primitiveType: .line,
                                             primitiveCount: scnVerts.count - 1,
                                             bytesPerIndex: MemoryLayout<UInt32>.size)

            // Per-vertex colors to fade the trail (older = more transparent)
            var colors = [SIMD4<Float>]()
            colors.reserveCapacity(scnVerts.count)
            for (i, _) in scnVerts.enumerated() {
                let t = Float(i) / Float(scnVerts.count - 1)  // 0 → 1
                let alpha = max(0.08, t)                      // tip brightest
                colors.append(SIMD4<Float>(0.2, 0.8, 1.0, alpha)) // teal-ish; SceneKit uses RGBA 0..1
            }
            let colorData = Data(bytes: colors, count: colors.count * MemoryLayout<SIMD4<Float>>.size)
            let colorSource = SCNGeometrySource(data: colorData,
                                                semantic: .color,
                                                vectorCount: scnVerts.count,
                                                usesFloatComponents: true,
                                                componentsPerVector: 4,
                                                bytesPerComponent: MemoryLayout<Float>.size,
                                                dataOffset: 0,
                                                dataStride: MemoryLayout<SIMD4<Float>>.size)

            let geom = SCNGeometry(sources: [vertexSource, colorSource], elements: [element])

            // Thin, additive, slightly emissive line
            let mat = SCNMaterial()
            mat.diffuse.contents = UIColor.white
            mat.emission.contents = UIColor.white
            mat.isDoubleSided = true
            mat.blendMode = .add
            mat.lightingModel = .constant
            geom.materials = [mat]

            node.geometry = geom
        }
    }

    private func makeScene() -> SCNScene {
        let scene = SCNScene()

        // Ground
        let floor = SCNFloor()
        floor.firstMaterial?.diffuse.contents = UIColor(white: 0.1, alpha: 1)
        floor.firstMaterial?.roughness.contents = 0.8
        let floorNode = SCNNode(geometry: floor)
        scene.rootNode.addChildNode(floorNode)

        // Cube
        let box = SCNBox(width: 0.5, height: 0.5, length: 0.5, chamferRadius: 0.01)
        let mat = SCNMaterial()
        mat.diffuse.contents = UIColor.systemTeal
        box.materials = [mat]
        let cubeNode = SCNNode(geometry: box)
        cubeNode.name = "cube"
        cubeNode.position = SCNVector3(0, 0.1, 0)
        scene.rootNode.addChildNode(cubeNode)

        // Trail holder node
        let trailNode = SCNNode()
        trailNode.name = "trail"
        scene.rootNode.addChildNode(trailNode)

        // Lights
        let ambient = SCNLight(); ambient.type = .ambient; ambient.intensity = 200
        let ambientNode = SCNNode(); ambientNode.light = ambient
        scene.rootNode.addChildNode(ambientNode)

        let directional = SCNLight(); directional.type = .directional; directional.intensity = 700
        let dirNode = SCNNode(); dirNode.light = directional
        dirNode.eulerAngles = SCNVector3(-Float.pi/3, Float.pi/4, 0)
        scene.rootNode.addChildNode(dirNode)

        // Camera
        let camera = SCNCamera()
        camera.zNear = 0.01; camera.zFar = 100; camera.wantsHDR = true
        let camNode = SCNNode()
        camNode.name = "camera"
        camNode.camera = camera
        camNode.position = SCNVector3(0, 0.5, 2.0)
        camNode.look(at: SCNVector3(0, 0.1, 0))
        scene.rootNode.addChildNode(camNode)

        return scene
    }
}
