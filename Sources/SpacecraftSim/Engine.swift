import Foundation

// MARK: - Interactive simulation engine
//
// `runScenario` (batch, tested) is built on top of this class. The Xcode
// mission-ops app drives it interactively: pause, speed control, manual
// reaction-wheel kills, and live telemetry reads.
//
// The step order below mirrors `runScenario`'s original loop exactly
// (fault injection -> sense -> estimate -> control -> actuate -> sample),
// so batch results are unchanged.

/// A step-able closed-loop ADCS simulation: dynamics, sensors, MEKF, controller.
public final class SimEngine {
    public let config: ScenarioConfig
    public private(set) var t = 0.0
    public private(set) var samples = [TelemetrySample]()
    public private(set) var lastTorque = Vec3.zero
    public private(set) var failedWheels = Set<Int>()

    private var rng: RNG
    private var sc: Spacecraft
    private var gyro: Gyro
    private let tracker = StarTracker()
    private let sun = SunSensor()
    private let sunInertial = Vec3(1, 0, 0)  // fixed; no orbit model, so no eclipse
    private var ekf: MEKF
    private var ctrl: AttitudeController
    private var nextLog = 0.0
    private var nextStarUpdate = 0.0
    private var nextSunUpdate = 0.0
    private var autoFailed = false
    private let deg = Double.pi / 180
    private let starPeriod: Double
    private let sunPeriod: Double
    public private(set) var sunValid = false  // last sun update had the sun in FOV

    public init(config: ScenarioConfig) {
        self.config = config
        starPeriod = 1.0 / tracker.rateHz
        sunPeriod = 1.0 / sun.rateHz
        rng = RNG(seed: config.seed)

        let inertiaVec = Vec3(12, 14, 10)  // kg*m^2, small-sat class
        sc = Spacecraft(attitude: config.initialAttitude,
                        omega: config.initialOmega,
                        inertia: Mat3(diagonal: inertiaVec))
        sc.actuatorMode = config.actuatorMode
        gyro = Gyro(trueBias: config.trueGyroBias)

        // Estimator starts with a plausible (wrong) attitude: a few degrees off.
        let errAxis = rng.unitVector()
        let errQ = Quat(angle: config.initialAttitudeErrorDeg * deg, axis: errAxis)
        ekf = MEKF(initialAttitude: (errQ * config.initialAttitude).normalized(),
                   attitudeSigma: 10 * deg,
                   biasSigma: 0.02 * deg)

        // Controller: gentle 0.06 rad/s loop, well damped, slow integrator.
        ctrl = AttitudeController.tuned(inertia: inertiaVec,
                                        naturalFreq: 0.06,
                                        damping: 0.9,
                                        integralFreq: 0.006)
    }

    /// Kill a reaction wheel immediately (manual failure injection).
    public func killWheel(_ i: Int) {
        guard (0..<4).contains(i), !failedWheels.contains(i) else { return }
        sc.wheels.fail(wheel: i)
        failedWheels.insert(i)
    }

    /// Advance the simulation by one control step (`config.dt`).
    public func advance() {
        // Fault injection.
        if let f = config.failWheelAt, !autoFailed, t >= f.time {
            killWheel(f.wheel)
            autoFailed = true
        }

        // Sense.
        let gyroMeas = gyro.measure(trueOmega: sc.omega, dt: config.dt, rng: &rng)
        ekf.predict(gyro: gyroMeas, dt: config.dt, sigmaV: gyro.arw, sigmaU: gyro.biasRW)
        if t >= nextStarUpdate {
            let stMeas = tracker.measure(trueAttitude: sc.attitude, rng: &rng)
            ekf.update(starMeasurement: stMeas, sigmaStar: tracker.noiseRad)
            nextStarUpdate += starPeriod
        }
        if t >= nextSunUpdate {
            if let sMeas = sun.measure(trueAttitude: sc.attitude,
                                       sunInertial: sunInertial, rng: &rng) {
                ekf.updateVector(measuredBody: sMeas,
                                 referenceInertial: sunInertial,
                                 sigma: sun.noiseRad)
                sunValid = true
            } else {
                sunValid = false  // sun outside FOV: coast on gyro + star tracker
            }
            nextSunUpdate += sunPeriod
        }

        // Control on the *estimated* state, like flight software would.
        let wEst = ekf.correctedRate(gyroMeasurement: gyroMeas)
        let tauCmd = ctrl.torque(target: config.targetAttitude,
                                 estimatedAttitude: ekf.attitude,
                                 estimatedOmega: wEst,
                                 dt: config.dt)
        lastTorque = tauCmd

        // Actuate truth.
        sc.step(dt: config.dt, torqueCommand: tauCmd, disturbance: disturbanceTorque(t))

        // Telemetry.
        if t >= nextLog {
            samples.append(currentSample())
            nextLog += 0.5
        }

        t += config.dt
    }

    // MARK: - Live state for dashboards

    public var trueAttitude: Quat { sc.attitude }
    public var estimatedAttitude: Quat { ekf.attitude }
    public var targetAttitude: Quat { config.targetAttitude }
    public var omega: Vec3 { sc.omega }
    public var duration: Double { config.duration }

    public var pointingErrorDeg: Double {
        sc.attitude.angle(to: config.targetAttitude) / deg
    }

    public var estimatorErrorDeg: Double {
        ekf.attitude.angle(to: sc.attitude) / deg
    }

    public var biasErrorDegS: Double {
        (ekf.bias - gyro.trueBias).norm / deg
    }

    public var wheelMomenta: [Double] { sc.wheels.momentum }
    public var wheelMaxMomentum: Double { sc.wheels.maxMomentum }
    public var wheelSaturation: Double { sc.wheels.saturation }
    public var thrusterFirings: Int { sc.thrusters.totalFirings }
    public var thrusterBurnTime: Double { sc.thrusters.totalOnTime }

    // MARK: - Private

    private func currentSample() -> TelemetrySample {
        let pe = sc.attitude.angle(to: config.targetAttitude) / deg
        let ee = ekf.attitude.angle(to: sc.attitude) / deg
        let be = (ekf.bias - gyro.trueBias).norm / deg
        return TelemetrySample(t: t,
                               pointErrDeg: pe,
                               estErrDeg: ee,
                               omegaDegS: sc.omega.norm / deg,
                               biasErrDegS: be,
                               wheelSaturation: sc.wheels.saturation,
                               torqueNorm: lastTorque.norm,
                               thrusterFirings: sc.thrusters.totalFirings,
                               thrusterBurnTime: sc.thrusters.totalOnTime)
    }
}

/// Small cyclic disturbance torques (gravity-gradient / aero style).
func disturbanceTorque(_ t: Double) -> Vec3 {
    2e-5 * Vec3(sin(0.05 * t), sin(0.03 * t + 1.0), cos(0.04 * t + 0.5))
}
