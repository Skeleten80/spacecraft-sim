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

    func testDeterminism() {
        let a = runScenario(builtinScenario("nominal")!)
        let b = runScenario(builtinScenario("nominal")!)
        XCTAssertEqual(a.samples.count, b.samples.count)
        XCTAssertEqual(a.finalPointErrDeg, b.finalPointErrDeg)
        XCTAssertEqual(a.settleTime, b.settleTime)
    }
}
