//
//  ContentView.swift
//  AcceloCube
//
//  Created by Weeraphot Bumbaugh on 10/6/25.
//
import SwiftUI
import SceneKit

struct ContentView: View {
    @StateObject private var vm = MotionVM()
    
    @Environment(\.scenePhase) private var scenePhase

    var body: some View {
        NavigationStack {
            VStack(spacing: 18) {
                SceneViewBridge(vm: vm)
                    .frame(height: 340)
                    .clipShape(RoundedRectangle(cornerRadius: 16))
                    .overlay(alignment: .topLeading) { statusPill.padding(8) }

                GroupBox("Controls") {
                    VStack(spacing: 14) {
                        HStack {
                            Toggle(isOn: Binding(
                                get: { vm.usingDeviceMotion },
                                set: { on in on ? vm.start() : vm.stop() }
                            )) { Text("Start") }
                            .toggleStyle(.switch)
                            Spacer()
                            Button("Re-Center") { vm.recenter() }
                            Button("Calibrate") { vm.calibrate() }
                        }

                        // Filter input noise
                        // Small = fast & jittery, large = slowly, smooth, laggy
                        LabeledContent("Smoothing") {
                            HStack {
                                Slider(value: $vm.cfg.smoothing, in: 0...0.98, step: 0.02)
                                Text(String(format: "%.2f", vm.cfg.smoothing))
                                    .font(.caption.monospaced()).frame(width: 44, alignment: .trailing)
                            }
                        }
                        .onChange(of: vm.cfg.smoothing) { _, _ in
                            vm.applyConfigChange()
                        }
                        
                        // Air resistance in simulation
                        // Small = Higher drift, large = less drift
                        LabeledContent("Damping") {
                            HStack {
                                Slider(value: $vm.cfg.damping, in: 0...0.2, step: 0.01)
                                Text(String(format: "%.2f", vm.cfg.damping))
                                    .font(.caption.monospaced()).frame(width: 44, alignment: .trailing)
                            }
                        }
                        .onChange(of: vm.cfg.damping) { _, _ in
                            vm.applyConfigChange()
                        }
                        
                        // How often it refreshes
                        // Lower = Less update & power consumption, smoother but slower
                        // Higher = Ultra-responsive but battery hungry
                        LabeledContent("Hz") {
                            HStack {
                                Picker("", selection: $vm.cfg.sampleHz) {
                                    Text("30").tag(30.0)
                                    Text("60").tag(60.0)
                                    Text("100").tag(100.0)
                                }
                                .pickerStyle(.segmented)
                                .frame(width: 180)

                                Slider(value: $vm.cfg.sampleHz, in: 10...120, step: 10)
                                Text("\(Int(vm.cfg.sampleHz))")
                                    .font(.caption.monospaced()).frame(width: 44, alignment: .trailing)
                            }
                        }
                        .onChange(of: vm.cfg.sampleHz) { _, _ in
                            vm.applyConfigChange()
                        }
                        
                        Toggle("Auto-resume on foreground", isOn: $vm.autoResumeOnActive)
                        
                        // Disregard slow-constant gravity like movement and only keep fast changes
                        Toggle("High-pass Accel", isOn: $vm.useHighPass)
                        
                        Toggle("CSV Logging", isOn: Binding(
                            get: { vm.cfg.loggingEnabled },
                            set: { on in
                                vm.cfg.loggingEnabled = on
                                if vm.usingDeviceMotion { vm.start() }
                            }
                        ))
                    }
                }

                // HUD
                VStack(spacing: 4) {
                    Text("rate \(Int(vm.cfg.sampleHz)) Hz | latency \(vm.sampleLatencyMs, specifier: "%.1f") ms")
                        .font(.caption.monospaced())
                    Text("pos \(vm.pos.x, specifier: "%.2f"), \(vm.pos.y, specifier: "%.2f"), \(vm.pos.z, specifier: "%.2f")")
                        .font(.caption.monospaced())
                    Text("auth \(vm.hudAuth) | queue \(vm.hudQueue)")
                        .font(.caption.monospaced())
                }
            }
            .padding()
            .navigationTitle("AcceloCubePro")
            .onChange(of: scenePhase) { _, newPhase in
                switch newPhase {
                case .background, .inactive:
                    if vm.usingDeviceMotion { vm.stop() }
                case .active:
                    if vm.autoResumeOnActive, !vm.usingDeviceMotion { vm.start() }
                @unknown default:
                    break
                }
            }
        }
    }

    private var statusPill: some View {
        HStack(spacing: 8) {
            Circle().fill(vm.usingDeviceMotion ? Color.green : Color.red)
                .frame(width: 12, height: 12)
            Text(vm.status).font(.footnote.monospaced()).padding(.vertical, 8)
        }
        .padding(.horizontal, 12)
        .background(.ultraThinMaterial, in: Capsule())
    }
}


#Preview {
    ContentView()
}
