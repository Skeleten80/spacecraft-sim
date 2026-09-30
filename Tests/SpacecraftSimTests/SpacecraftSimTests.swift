import XCTest
@testable import SpacecraftSim

let deg = Double.pi / 180

// MARK: - Math

final class MathTests: XCTestCase {
    func testQuaternionRotatesVectors() {
        // 90 deg about z: x-axis -> y-axis.
        let q = Quat(angle: .pi / 2, axis: .unitZ)
        let v = q.rotate(.unitX)
        XCTAssertEqual(v.x, 0, accuracy: 1e-12)
        XCTAssertEqual(v.y, 1, accuracy: 1e-12)
        XCTAssertEqual(v.z, 0, accuracy: 1e-12)
    }

    func testQuaternionConjugateIsInverse() {
        var rng = RNG(seed: 7)
        for _ in 0..<20 {
            let q = Quat(angle: rng.uniform(low: 0, high: .pi),
                         axis: rng.unitVector()).normalized()
            let id = (q * q.conjugated()).normalized()
            XCTAssertEqual(id.x, 0, accuracy: 1e-12)
            XCTAssertEqual(id.y, 0, accuracy: 1e-12)
            XCTAssertEqual(id.z, 0, accuracy: 1e-12)
            XCTAssertEqual(abs(id.w), 1, accuracy: 1e-12)
        }
    }

    func testQuaternionCompositionOrder() {
        // q1 then q2 == (q2 * q1) applied to a vector.
        let q1 = Quat(angle: .pi / 2, axis: .unitX)
        let q2 = Quat(angle: .pi / 2, axis: .unitY)
        let v = Vec3(0.3, -0.7, 0.5).normalized()
        let sequential = q2.rotate(q1.rotate(v))
        let composed = (q2 * q1).rotate(v)
        XCTAssertEqual(sequential.x, composed.x, accuracy: 1e-12)
        XCTAssertEqual(sequential.y, composed.y, accuracy: 1e-12)
        XCTAssertEqual(sequential.z, composed.z, accuracy: 1e-12)
    }

    func testSkewMatchesCross() {
        let a = Vec3(1, 2, 3), b = Vec3(-0.5, 4, 2)
        let viaSkew = skew(a) * b
        let viaCross = a.cross(b)
        XCTAssertEqual(viaSkew.x, viaCross.x, accuracy: 1e-12)
        XCTAssertEqual(viaSkew.y, viaCross.y, accuracy: 1e-12)
        XCTAssertEqual(viaSkew.z, viaCross.z, accuracy: 1e-12)
    }

    func testMatrixInverse() {
        var rng = RNG(seed: 99)
        for _ in 0..<10 {
            var d = [Double]()
            for _ in 0..<36 { d.append(rng.uniform(low: -2, high: 2)) }
            let a = Matrix(rows: 6, cols: 6, data: d)
            guard let inv = a.inverted() else { continue }  // skip near-singular draws
            let id = a * inv
            for r in 0..<6 {
                for c in 0..<6 {
                    XCTAssertEqual(id[r, c], r == c ? 1 : 0, accuracy: 1e-9)
                }
            }
        }
    }
}

// MARK: - Wheel geometry

final class WheelGeometryTests: XCTestCase {
    /// Any 3 of the 4 wheels must span R^3 (single-fault tolerance).
    func testAnyThreeWheelsSpanR3() {
        let asm = ReactionWheelAssembly()
        for dead in 0..<4 {
            var failed = Set<Int>()
            failed.insert(dead)
            let pinv = ReactionWheelAssembly.pseudoInverse(axes: asm.axes, failed: failed)
            // D * D+ must be identity on R^3.
            let d = ReactionWheelAssembly.distribution(axes: asm.axes, failed: failed)
            let id = d * pinv
            for r in 0..<3 {
                for c in 0..<3 {
                    XCTAssertEqual(id[r, c], r == c ? 1 : 0, accuracy: 1e-9,
                                   "dead wheel \(dead)")
                }
            }
        }
    }

    func testFailedWheelProducesNoTorque() {
        var asm = ReactionWheelAssembly()
        asm.fail(wheel: 2)
        let tau = asm.apply(desiredTorque: Vec3(0.05, -0.03, 0.02), dt: 0.01)
        // With a valid allocation the achieved torque matches the command.
        XCTAssertEqual(tau.x, 0.05, accuracy: 1e-9)
        XCTAssertEqual(tau.y, -0.03, accuracy: 1e-9)
        XCTAssertEqual(tau.z, 0.02, accuracy: 1e-9)
        XCTAssertEqual(asm.momentum[2], 0, accuracy: 1e-15)
    }
}

// MARK: - Thrusters

final class ThrusterTests: XCTestCase {
    func testTwelveThrusterBlockGeometry() {
        let asm = ThrusterAssembly()  // 1 N, 0.5 m arms
        XCTAssertEqual(asm.thrusters.count, 12)
        // Thruster 0: +x torque pair. (0, h, 0) x (0, 0, F) = (h*F, 0, 0).
        let a0 = asm.thrusters[0].torqueAuthority
        XCTAssertEqual(a0.x, 0.5, accuracy: 1e-12)
        XCTAssertEqual(a0.y, 0, accuracy: 1e-12)
        XCTAssertEqual(a0.z, 0, accuracy: 1e-12)
        // Six direction groups (+/- about each axis), two thrusters each.
        let groups = ThrusterAssembly.group(thrusters: asm.thrusters, failed: [])
        XCTAssertEqual(groups.count, 6)
        for g in groups { XCTAssertEqual(g.members.count, 2) }
    }

    func testAllocationTracksCommand() {
        // No minimum-impulse gating: achieved torque matches the command.
        var asm = ThrusterAssembly(minOnTime: 0)
        let cmd = Vec3(0.3, -0.2, 0.15)
        let tau = asm.apply(desiredTorque: cmd, dt: 0.01)
        XCTAssertEqual(tau.x, cmd.x, accuracy: 1e-9)
        XCTAssertEqual(tau.y, cmd.y, accuracy: 1e-9)
        XCTAssertEqual(tau.z, cmd.z, accuracy: 1e-9)
    }

    func testMinimumImpulseBitDropsSmallPulses() {
        var asm = ThrusterAssembly()  // minOnTime = 0.02 s
        // 1e-4 N*m -> duty 1e-4 -> 1e-6 s demanded per step: far below the bit.
        for _ in 0..<10 {
            let tau = asm.apply(desiredTorque: Vec3(1e-4, 0, 0), dt: 0.01)
            XCTAssertEqual(tau.norm, 0, accuracy: 1e-15)
        }
        XCTAssertEqual(asm.totalFirings, 0)
        XCTAssertEqual(asm.totalOnTime, 0, accuracy: 1e-15)
    }

    func testPulsesAccumulateAndFire() {
        var asm = ThrusterAssembly()  // minOnTime = 0.02 s
        // 0.4 N*m about x -> duty 0.4 per +x thruster -> 4 ms demanded per
        // step -> the 20 ms bit is reached after 5 steps, then it fires.
        for _ in 0..<10 {
            _ = asm.apply(desiredTorque: Vec3(0.4, 0, 0), dt: 0.01)
        }
        XCTAssertGreaterThan(asm.totalFirings, 0)
        XCTAssertGreaterThan(asm.totalOnTime, 0)
    }

    func testFailedThrusterRedundantPairStillDelivers() {
        var asm = ThrusterAssembly(minOnTime: 0)
        asm.fail(thruster: 0)  // one of the +x pair
        let tau = asm.apply(desiredTorque: Vec3(0.2, 0, 0), dt: 0.01)
        // Surviving +x thruster works at double duty: still exact.
        XCTAssertEqual(tau.x, 0.2, accuracy: 1e-9)
        XCTAssertEqual(tau.y, 0, accuracy: 1e-12)
        XCTAssertEqual(tau.z, 0, accuracy: 1e-12)
    }

    func testSaturationClamps() {
        var asm = ThrusterAssembly(minOnTime: 0)
        // 5 N*m about x: far beyond the 1.0 N*m per-direction authority.
        let tau = asm.apply(desiredTorque: Vec3(5, 0, 0), dt: 0.01)
        XCTAssertEqual(tau.x, 1.0, accuracy: 1e-9)
    }
}

// MARK: - Sun sensor

final class SunSensorTests: XCTestCase {
    func testFOVGating() {
        var rng = RNG(seed: 11)
        let sun = SunSensor()  // boresight +x, 70 deg half-FOV
        // Sun along the boresight: visible.
        XCTAssertNotNil(sun.measure(trueAttitude: .identity,
                                    sunInertial: Vec3(1, 0, 0), rng: &rng))
        // Sun behind the boresight: blind.
        XCTAssertNil(sun.measure(trueAttitude: .identity,
                                 sunInertial: Vec3(-1, 0, 0), rng: &rng))
        // 90 deg off boresight: outside the 70 deg FOV.
        XCTAssertNil(sun.measure(trueAttitude: .identity,
                                 sunInertial: Vec3(0, 1, 0), rng: &rng))
    }

    func testSunNoiseMagnitude() {
        var rng = RNG(seed: 22)
        let sun = SunSensor(noiseDeg: 0.25)
        // 200 samples at 6-sigma must stay within 1.5 deg of truth.
        let limit = cos(1.5 * deg)
        for _ in 0..<200 {
            let m = sun.measure(trueAttitude: .identity,
                                sunInertial: Vec3(1, 0, 0), rng: &rng)!
            XCTAssertGreaterThan(m.dot(Vec3(1, 0, 0)), limit)
            XCTAssertEqual(m.norm, 1, accuracy: 1e-12)
        }
    }
}

// MARK: - Estimator

final class EstimatorTests: XCTestCase {
    func testMEKFConverges() {
        var rng = RNG(seed: 1234)
        var ekf = MEKF(initialAttitude: Quat(angle: 5 * deg, axis: .unitX),
                       attitudeSigma: 10 * deg, biasSigma: 0.02 * deg)
        var gyro = Gyro(trueBias: Vec3(0.01 * deg, -0.008 * deg, 0.012 * deg))
        let tracker = StarTracker()
        let truth = Quat(angle: 20 * deg, axis: Vec3(0.3, 1, 0.2).normalized())
        let dt = 0.01
        var t = 0.0
        while t < 120 {
            let m = gyro.measure(trueOmega: .zero, dt: dt, rng: &rng)
            ekf.predict(gyro: m, dt: dt, sigmaV: gyro.arw, sigmaU: gyro.biasRW)
            if Int(t / dt) % 100 == 0 {
                ekf.update(starMeasurement: tracker.measure(trueAttitude: truth, rng: &rng),
                           sigmaStar: tracker.noiseRad)
            }
            t += dt
        }
        XCTAssertLessThan(ekf.attitude.angle(to: truth) / deg, 0.2)
        // Bias floor is set by gyro ARW vs. 1 Hz star updates (~0.006 deg/s);
        // assert we beat the 0.02 deg/s initial uncertainty by 2x.
        XCTAssertLessThan((ekf.bias - gyro.trueBias).norm / deg, 0.01)
    }

    func testVectorUpdateConverges() {
        // Two independent reference vectors (sun + a cross axis) make the
        // full attitude observable through updateVector alone.
        var rng = RNG(seed: 555)
        var ekf = MEKF(initialAttitude: Quat(angle: 5 * deg, axis: .unitZ),
                       attitudeSigma: 10 * deg, biasSigma: 0.02 * deg)
        let truth = Quat.identity
        let sunRef = Vec3(1, 0, 0)
        let auxRef = Vec3(0, 1, 0)
        let sigma = 0.25 * deg
        var gyro = Gyro()
        let dt = 0.01
        var t = 0.0
        while t < 60 {
            let m = gyro.measure(trueOmega: .zero, dt: dt, rng: &rng)
            ekf.predict(gyro: m, dt: dt, sigmaV: gyro.arw, sigmaU: gyro.biasRW)
            if Int(t / dt) % 50 == 0 {
                ekf.updateVector(measuredBody: noisyBodyVec(sunRef, sigma, &rng),
                                 referenceInertial: sunRef, sigma: sigma)
                ekf.updateVector(measuredBody: noisyBodyVec(auxRef, sigma, &rng),
                                 referenceInertial: auxRef, sigma: sigma)
            }
            t += dt
        }
        XCTAssertLessThan(ekf.attitude.angle(to: truth) / deg, 0.5)
    }

    /// Body-frame measurement of an inertial reference at identity attitude,
    /// corrupted by a small random rotation (1-sigma `sigma` per axis).
    private func noisyBodyVec(_ ref: Vec3, _ sigma: Double, _ rng: inout RNG) -> Vec3 {
        let err = rng.gaussianVec3() * sigma
        let angle = err.norm
        let dq = angle > 1e-15
            ? Quat(angle: angle, axis: err / angle)
            : Quat.identity
        return dq.rotate(ref).normalized()
    }
}

// MARK: - Closed loop

final class ClosedLoopTests: XCTestCase {
    func testNominalConverges() {
        var config = builtinScenario("nominal")!
        config.duration = 300
        let r = runScenario(config)
        XCTAssertLessThan(r.finalPointErrDeg, 0.5)
        XCTAssertNotNil(r.settleTime)
        XCTAssertLessThan(r.settleTime ?? 1e9, 240)
    }

    func testWheelFailureStillConverges() {
        var config = builtinScenario("wheel-failure")!
        let r = runScenario(config)
        // One dead wheel: must still point, possibly a little less precisely.
        XCTAssertLessThan(r.finalPointErrDeg, 2.0)
        XCTAssertNotNil(r.settleTime)
    }

    func testTumbleRecovery() {
        var config = builtinScenario("tumble")!
        let r = runScenario(config)
        XCTAssertLessThan(r.finalPointErrDeg, 1.0)
        XCTAssertLessThan(r.samples.last?.omegaDegS ?? 1e9, 0.1)
    }

    func testThrusterSlewConverges() {
        var config = builtinScenario("thruster-slew")!
        let r = runScenario(config)
        // On/off RCS with a 20 ms impulse bit: coarse slew, then a small
        // limit cycle around the target instead of smooth convergence.
        XCTAssertLessThan(r.finalPointErrDeg, 1.0)
        XCTAssertNotNil(r.settleTime)
        XCTAssertGreaterThan(r.thrusterFirings, 100)
        XCTAssertGreaterThan(r.thrusterBurnTime, 1.0)
    }

    func testDeterminism() {
        let a = runScenario(builtinScenario("nominal")!)
        let b = runScenario(builtinScenario("nominal")!)
        XCTAssertEqual(a.samples.count, b.samples.count)
        XCTAssertEqual(a.finalPointErrDeg, b.finalPointErrDeg)
        XCTAssertEqual(a.settleTime, b.settleTime)
    }
}
