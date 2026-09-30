import Foundation

/// Closed-loop simulation: truth dynamics + noisy sensors + MEKF + controller.
///
/// This is the "planet-side" test bench: everything a flight computer would
/// do, running against a simulated spacecraft instead of real hardware.

// MARK: - Scenario

public struct ScenarioConfig {
    public var name: String
    public var seed: UInt64 = 42
    public var duration: Double = 300        // s
    public var dt: Double = 0.01            // 100 Hz control loop
    public var initialAttitude: Quat = .identity
    public var initialOmega: Vec3 = .zero    // rad/s
    public var targetAttitude: Quat = .identity
    public var trueGyroBias: Vec3 = .zero
    public var initialAttitudeErrorDeg: Double = 5  // estimator starts this far off
    public var failWheelAt: (time: Double, wheel: Int)?  // fault injection
    public var actuatorMode: ActuatorMode = .wheels

    public init(name: String) { self.name = name }
}

public struct TelemetrySample {
    public var t: Double
    public var pointErrDeg: Double     // true pointing error
    public var estErrDeg: Double       // estimator's attitude error vs truth
    public var omegaDegS: Double       // true body rate magnitude
    public var biasErrDegS: Double     // gyro bias estimation error
    public var wheelSaturation: Double // max |h|/h_max
    public var torqueNorm: Double      // commanded torque magnitude (N*m)
    public var thrusterFirings: Int = 0    // cumulative RCS pulses
    public var thrusterBurnTime: Double = 0  // cumulative RCS burn (s)
}

public struct SimResult {
    public var config: ScenarioConfig
    public var samples: [TelemetrySample]
    public var finalPointErrDeg: Double
    public var settleTime: Double?     // first t after which err stays < 0.5 deg
    public var maxOmegaDegS: Double
    public var finalBiasErrDegS: Double
    public var thrusterFirings: Int
    public var thrusterBurnTime: Double
}

// MARK: - Runner

public func runScenario(_ config: ScenarioConfig) -> SimResult {
    let engine = SimEngine(config: config)
    while engine.t <= config.duration {
        engine.advance()
    }
    let samples = engine.samples

    // Post-process: settle time = first sample after which error stays < 0.5 deg.
    var settle: Double?
    outer: for i in 0..<samples.count {
        for j in i..<samples.count where samples[j].pointErrDeg >= 0.5 {
            continue outer
        }
        settle = samples[i].t
        break
    }

    return SimResult(
        config: config,
        samples: samples,
        finalPointErrDeg: samples.last?.pointErrDeg ?? .nan,
        settleTime: settle,
        maxOmegaDegS: samples.map { $0.omegaDegS }.max() ?? 0,
        finalBiasErrDegS: samples.last?.biasErrDegS ?? .nan,
        thrusterFirings: engine.thrusterFirings,
        thrusterBurnTime: engine.thrusterBurnTime)
}

// MARK: - Built-in scenarios

public func builtinScenario(_ name: String) -> ScenarioConfig? {
    let deg = Double.pi / 180
    switch name {
    case "nominal":
        var c = ScenarioConfig(name: "nominal")
        c.initialAttitude = Quat(angle: 30 * deg, axis: Vec3(0.3, 1, 0.2))
        c.initialOmega = Vec3(0.5 * deg, -0.3 * deg, 0.4 * deg)
        c.trueGyroBias = Vec3(0.01 * deg, -0.008 * deg, 0.012 * deg)
        return c
    case "wheel-failure":
        var c = ScenarioConfig(name: "wheel-failure")
        c.initialAttitude = Quat(angle: 30 * deg, axis: Vec3(0.3, 1, 0.2))
        c.initialOmega = Vec3(0.5 * deg, -0.3 * deg, 0.4 * deg)
        c.trueGyroBias = Vec3(0.01 * deg, -0.008 * deg, 0.012 * deg)
        c.failWheelAt = (time: 45, wheel: 0)
        c.duration = 400
        return c
    case "tumble":
        var c = ScenarioConfig(name: "tumble")
        c.initialAttitude = Quat(angle: 120 * deg, axis: Vec3(1, 0.5, -0.3))
        c.initialOmega = Vec3(8 * deg, -6 * deg, 10 * deg)  // tumbling fast
        c.trueGyroBias = Vec3(0.01 * deg, -0.008 * deg, 0.012 * deg)
        c.duration = 600
        return c
    case "thruster-slew":
        // Same controller, but the actuator is the 12-thruster RCS block.
        // On/off valves + 20 ms minimum impulse bit: expect a coarse slew
        // and a small limit cycle around the target, not smooth convergence.
        var c = ScenarioConfig(name: "thruster-slew")
        c.initialAttitude = Quat(angle: 120 * deg, axis: Vec3(1, 0.5, -0.3))
        c.initialOmega = Vec3(1.0 * deg, -0.5 * deg, 0.8 * deg)
        c.trueGyroBias = Vec3(0.01 * deg, -0.008 * deg, 0.012 * deg)
        c.actuatorMode = .thrusters
        c.duration = 600
        return c
    default:
        return nil
    }
}

// MARK: - CSV export

public func writeCSV(_ result: SimResult, to path: String) throws {
    var lines = ["t,point_err_deg,est_err_deg,omega_deg_s,bias_err_deg_s,wheel_saturation,torque_Nm,thruster_firings,thruster_burn_s"]
    for s in result.samples {
        lines.append(String(format: "%.2f,%.4f,%.4f,%.4f,%.5f,%.4f,%.6f,%d,%.2f",
                            s.t, s.pointErrDeg, s.estErrDeg, s.omegaDegS,
                            s.biasErrDegS, s.wheelSaturation, s.torqueNorm,
                            s.thrusterFirings, s.thrusterBurnTime))
    }
    try lines.joined(separator: "\n").appending("\n")
        .write(toFile: path, atomically: true, encoding: .utf8)
}
