import Charts
import SpacecraftSim
import SwiftUI

/// The mission-ops screen: 3D attitude view, live telemetry, strip charts,
/// scenario/speed controls, and per-wheel kill buttons for fault injection.
struct DashboardView: View {
    @StateObject private var vm = SimViewModel()

    var body: some View {
        VStack(spacing: 0) {
            toolbar
                .padding(.horizontal, 12)
                .padding(.vertical, 8)
            Divider()
            HSplitView {
                AttitudeSceneView(attitude: vm.engine.estimatedAttitude,
                                  target: vm.engine.targetAttitude)
                    .frame(minWidth: 380, idealWidth: 620)
                ScrollView {
                    rightPanel
                }
                .frame(minWidth: 330, idealWidth: 400)
            }
            Divider()
            chartsRow
                .frame(height: 190)
        }
        .frame(minWidth: 1020, minHeight: 740)
        .preferredColorScheme(.dark)
        .onAppear { vm.start() }
        .onDisappear { vm.stop() }
    }

    // MARK: - Toolbar

    private var toolbar: some View {
        HStack(spacing: 14) {
            Text("MISSION OPS")
                .font(.headline)
                .tracking(3)
            Text("ADCS SIMULATOR")
                .font(.subheadline)
                .foregroundStyle(.secondary)
            if !vm.running {
                Text("HELD")
                    .font(.caption)
                    .bold()
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background(.yellow.opacity(0.2), in: Capsule())
                    .foregroundStyle(.yellow)
            }
            Spacer()
            Picker("Scenario", selection: $vm.scenarioName) {
                ForEach(vm.scenarioNames, id: \.self) { name in
                    Text(name).tag(name)
                }
            }
            .pickerStyle(.segmented)
            .frame(width: 330)
            .onChange(of: vm.scenarioName) { _, new in vm.loadScenario(new) }
            Picker("Speed", selection: $vm.speed) {
                ForEach(vm.speeds, id: \.self) { s in
                    Text("\(Int(s))×").tag(s)
                }
            }
            .pickerStyle(.segmented)
            .frame(width: 160)
            Button(vm.running ? "Hold" : "Resume") { vm.running.toggle() }
            Button("Reset") { vm.reset() }
        }
    }

    // MARK: - Telemetry column

    private func statusColor(_ errDeg: Double) -> Color {
        errDeg < 0.5 ? .green : errDeg < 5 ? .yellow : .red
    }

    private var rightPanel: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(spacing: 10) {
                StatCard(title: "POINTING ERROR",
                         value: String(format: "%.3f°", vm.engine.pointingErrorDeg),
                         color: statusColor(vm.engine.pointingErrorDeg))
                StatCard(title: "SIM TIME",
                         value: String(format: "%.0f s", vm.engine.t))
            }

            PanelSection(title: "BODY RATE — DEG/S")
            HStack(spacing: 10) {
                let w = vm.bodyRatesDegS
                StatCard(title: "ωx", value: String(format: "%+.3f", w.x))
                StatCard(title: "ωy", value: String(format: "%+.3f", w.y))
                StatCard(title: "ωz", value: String(format: "%+.3f", w.z))
            }

            PanelSection(title: "MEKF ESTIMATOR")
            HStack(spacing: 10) {
                StatCard(title: "ATTITUDE ERR",
                         value: String(format: "%.3f°", vm.engine.estimatorErrorDeg))
                StatCard(title: "BIAS ERR",
                         value: String(format: "%.4f°/s", vm.engine.biasErrorDegS))
            }

            PanelSection(title: "CONTROL")
            HStack(spacing: 10) {
                StatCard(title: "TORQUE CMD",
                         value: String(format: "%.4f N·m", vm.engine.lastTorque.norm))
                StatCard(title: "WHEEL SAT",
                         value: String(format: "%.0f%%", vm.engine.wheelSaturation * 100),
                         color: vm.engine.wheelSaturation > 0.9 ? .red : .primary)
            }

            PanelSection(title: "RCS THRUSTERS")
            HStack(spacing: 10) {
                StatCard(title: "PULSES",
                         value: "\(vm.engine.thrusterFirings)")
                StatCard(title: "BURN TIME",
                         value: String(format: "%.1f s", vm.engine.thrusterBurnTime))
                StatCard(title: "SUN",
                         value: vm.engine.sunValid ? "TRACK" : "BLIND",
                         color: vm.engine.sunValid ? .green : .yellow)
            }

            PanelSection(title: "REACTION WHEELS — KILL TO INJECT FAULT (N·m·s)")
            VStack(spacing: 8) {
                ForEach(0..<4, id: \.self) { i in
                    WheelRow(index: i,
                             momentum: vm.engine.wheelMomenta[i],
                             maxMomentum: vm.engine.wheelMaxMomentum,
                             failed: vm.engine.failedWheels.contains(i),
                             onKill: { vm.killWheel(i) })
                }
            }
            .padding(.bottom, 8)
        }
        .padding(12)
    }

    // MARK: - Strip charts

    private var errYMax: Double {
        max(1.0, vm.errHistory.map(\.v).max() ?? 1.0)
    }

    private var rateYMax: Double {
        let m = vm.rateHistory.flatMap { [$0.x, $0.y, $0.z] }.map(abs).max() ?? 1.0
        return max(0.5, m * 1.1)
    }

    private var chartsRow: some View {
        HStack(spacing: 0) {
            VStack(alignment: .leading, spacing: 4) {
                Text("POINTING ERROR — DEG")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .tracking(1)
                Chart(vm.errHistory) { p in
                    LineMark(x: .value("t", p.t), y: .value("err", p.v))
                        .foregroundStyle(.green)
                }
                .chartYScale(domain: 0...errYMax)
                .chartXAxisLabel("sim time (s)")
            }
            .padding(12)
            Divider()
            VStack(alignment: .leading, spacing: 4) {
                Text("BODY RATES — DEG/S")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .tracking(1)
                Chart(vm.rateHistory) { p in
                    LineMark(x: .value("t", p.t), y: .value("wx", p.x))
                        .foregroundStyle(.red)
                    LineMark(x: .value("t", p.t), y: .value("wy", p.y))
                        .foregroundStyle(.green)
                    LineMark(x: .value("t", p.t), y: .value("wz", p.z))
                        .foregroundStyle(.blue)
                }
                .chartYScale(domain: -rateYMax...rateYMax)
                .chartXAxisLabel("sim time (s)")
            }
            .padding(12)
        }
    }
}
