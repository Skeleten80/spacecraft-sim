import Combine
import Foundation
import SpacecraftSim

/// Read-only engine snapshot fed by telemetry frames. Exposes the same
/// members DashboardView reads from SimEngine, so the view works against a
/// local engine or a network feed.
struct RemoteEngine {
    var estimatedAttitude = Quat.identity
    var targetAttitude = Quat.identity
    var pointingErrorDeg = 0.0
    var t = 0.0
    var estimatorErrorDeg = 0.0
    var biasErrorDegS = 0.0
    var lastTorque = Vec3.zero
    var omega = Vec3.zero
    var wheelSaturation = 0.0
    var thrusterFirings = 0
    var thrusterBurnTime = 0.0
    var sunValid = false
    var wheelMomenta = [0.0, 0.0, 0.0, 0.0]
    var wheelMaxMomentum = 1.0
    var failedWheels = Set<Int>()

    mutating func update(from f: TelemetryFrame) {
        func quat(_ a: [Double]) -> Quat {
            guard a.count == 4 else { return .identity }
            return Quat(x: a[0], y: a[1], z: a[2], w: a[3])
        }
        estimatedAttitude = quat(f.estimatedAttitude)
        targetAttitude = quat(f.targetAttitude)
        pointingErrorDeg = f.pointErrDeg
        t = f.t
        estimatorErrorDeg = f.estErrDeg
        biasErrorDegS = f.biasErrDegS
        // The view only reads lastTorque.norm; carry the magnitude.
        lastTorque = Vec3(f.torqueNorm, 0, 0)
        omega = f.omega.count == 3 ? Vec3(f.omega[0], f.omega[1], f.omega[2]) : .zero
        wheelSaturation = f.wheelSaturation
        thrusterFirings = f.thrusterFirings
        thrusterBurnTime = f.thrusterBurnTime
        sunValid = f.sunValid
        if f.wheelMomenta.count == 4 { wheelMomenta = f.wheelMomenta }
        wheelMaxMomentum = f.wheelMaxMomentum
        failedWheels = Set(f.failedWheels)
    }
}

/// Dashboard view-model driven by a remote sim (TelemetryClient) instead of
/// a local SimEngine. Its published surface mirrors SimViewModel, so
/// DashboardView only needs its `@StateObject` line changed to use it.
///
/// Run the far end first: `spacecraft-cli serve <scenario> --port 9001`
/// (Mini 1, "the spacecraft"). This is Mini 2, "mission control".
final class RemoteViewModel: ObservableObject {
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

    @Published private(set) var engine = RemoteEngine()
    @Published var running = true
    @Published var speed = 1.0 {
        didSet { try? client?.sendCommand(.setSpeed(speed)) }
    }
    @Published var scenarioName = "nominal"
    @Published private(set) var connected = false
    @Published private(set) var error: String?
    @Published private(set) var errHistory: [PlotPoint] = []
    @Published private(set) var rateHistory: [RatePoint] = []

    let scenarioNames = ["nominal", "wheel-failure", "tumble", "thruster-slew"]
    let speeds = [1.0, 10.0, 60.0]

    var bodyRatesDegS: Vec3 { engine.omega / (Double.pi / 180) }

    /// Where the sim is serving. Set before start().
    var host = "127.0.0.1"
    var port: UInt16 = 9001

    private var client: TelemetryClient?
    private var pointID = 0
    private let historyCap = 1200

    /// Connect if needed; the server streams on arrival. Called by the view's
    /// onAppear, mirroring SimViewModel.start().
    func start() {
        if client == nil { connect() }
    }

    func stop() {
        client?.disconnect()
        client = nil
        connected = false
    }

    func loadScenario(_ name: String) {
        if client == nil { connect() }
        guard client != nil else { return }
        scenarioName = name
        errHistory = []
        rateHistory = []
        pointID = 0
        running = true
        try? client?.sendCommand(.loadScenario(name))
    }

    func reset() {
        loadScenario(scenarioName)
    }

    func killWheel(_ i: Int) {
        try? client?.sendCommand(.killWheel(i))
    }

    // MARK: - Private

    private func connect() {
        let c = TelemetryClient(host: host, port: port)
        do {
            try c.connect()
        } catch {
            self.error = "Couldn't reach the sim at \(host):\(port). Is `spacecraft-cli serve` running?"
            return
        }
        client = c
        error = nil
        connected = true
        Thread {
            c.run(onHello: { [weak self] h in
                      DispatchQueue.main.async { self?.ingestHello(h) }
                  },
                  onFrame: { [weak self] f in
                      DispatchQueue.main.async { self?.ingestFrame(f) }
                  },
                  onEnd: { [weak self] reason in
                      DispatchQueue.main.async { self?.ingestEnd(reason) }
                  })
        }.start()
    }

    private func ingestHello(_ h: TelemetryHello) {
        // Assignment here doesn't loop back: the view's onChange only fires
        // on user-driven changes, and loadScenario sets the same value.
        scenarioName = h.scenario
        speed = 1.0
    }

    private func ingestFrame(_ f: TelemetryFrame) {
        guard running else { return }  // Hold freezes the display
        engine.update(from: f)
        pointID += 1
        errHistory.append(PlotPoint(id: pointID, t: f.t, v: f.pointErrDeg))
        let w = bodyRatesDegS
        rateHistory.append(RatePoint(id: pointID, t: f.t, x: w.x, y: w.y, z: w.z))
        if errHistory.count > historyCap {
            errHistory.removeFirst(historyCap / 2)
            rateHistory.removeFirst(historyCap / 2)
        }
    }

    private func ingestEnd(_ reason: String) {
        connected = false
        client?.disconnect()
        client = nil
    }
}
