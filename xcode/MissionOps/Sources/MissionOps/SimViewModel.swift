import Combine
import Foundation
import SpacecraftSim

/// Drives a `SimEngine` on a 30 Hz timer and publishes everything the
/// dashboard needs: live state plus rolling history for the strip charts.
final class SimViewModel: ObservableObject {
    struct PlotPoint: Identifiable {
        let id: Int
        let t: Double
        let v: Double
    }
    struct RatePoint: Identifiable {
        let id: Int
        let t: Double
        let x, y, z: Double
    }

    @Published private(set) var engine: SimEngine
    @Published var running = true
    @Published var speed = 10.0  // sim-seconds per real second
    @Published var scenarioName = "nominal"
    @Published private(set) var errHistory: [PlotPoint] = []
    @Published private(set) var rateHistory: [RatePoint] = []

    let scenarioNames = ["nominal", "wheel-failure", "tumble", "thruster-slew"]
    let speeds = [1.0, 10.0, 60.0]

    var deg: Double { Double.pi / 180 }
    var bodyRatesDegS: Vec3 { engine.omega / deg }

    private var timer: Timer?
    private var pointID = 0
    private let tickHz = 30.0
    private let historyCap = 1200

    init() {
        engine = SimEngine(config: builtinScenario("nominal") ?? ScenarioConfig(name: "nominal"))
    }

    func start() {
        timer?.invalidate()
        timer = Timer.scheduledTimer(withTimeInterval: 1.0 / tickHz, repeats: true) { [weak self] _ in
            self?.tick()
        }
    }

    func stop() {
        timer?.invalidate()
        timer = nil
    }

    func loadScenario(_ name: String) {
        guard let config = builtinScenario(name) else { return }
        scenarioName = name
        engine = SimEngine(config: config)
        errHistory = []
        rateHistory = []
        pointID = 0
        running = true
    }

    func reset() {
        loadScenario(scenarioName)
    }

    func killWheel(_ i: Int) {
        engine.killWheel(i)
    }

    // MARK: - Private

    private func tick() {
        guard running else { return }
        let steps = max(1, Int((speed / tickHz) / engine.config.dt))
        for _ in 0..<steps {
            if engine.t >= engine.config.duration {
                running = false
                break
            }
            engine.advance()
        }
        record()
    }

    private func record() {
        pointID += 1
        errHistory.append(PlotPoint(id: pointID, t: engine.t, v: engine.pointingErrorDeg))
        let w = bodyRatesDegS
        rateHistory.append(RatePoint(id: pointID, t: engine.t, x: w.x, y: w.y, z: w.z))
        if errHistory.count > historyCap {
            errHistory.removeFirst(historyCap / 2)
            rateHistory.removeFirst(historyCap / 2)
        }
    }
}
