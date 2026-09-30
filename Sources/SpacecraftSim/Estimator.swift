import Foundation
/// Multiplicative Extended Kalman Filter (MEKF) for attitude estimation.
///
/// The workhorse of real spacecraft attitude determination (Markley &
/// Crassidis). State:
///   - nominal attitude quaternion q_hat (body -> inertial)
///   - gyro bias estimate b_hat (3)
///
/// The covariance tracks the 6x6 error state [delta-theta (3); delta-bias (3)],
/// where delta-theta is the small-angle attitude error vector.
///
/// Predict: integrate q_hat with bias-corrected gyro, propagate covariance.
/// Update: fold in star-tracker quaternion measurements.
public struct MEKF {
    public private(set) var attitude: Quat
    public private(set) var bias: Vec3
    public private(set) var covariance: Matrix  // 6x6

    /// - Parameters:
    ///   - initialAttitude: starting attitude estimate.
    ///   - attitudeSigma: 1-sigma initial attitude uncertainty (rad).
    ///   - biasSigma: 1-sigma initial gyro-bias uncertainty (rad/s).
    public init(initialAttitude: Quat = .identity,
                attitudeSigma: Double,
                biasSigma: Double) {
        self.attitude = initialAttitude.normalized()
        self.bias = .zero
        let d = [attitudeSigma * attitudeSigma,
                 attitudeSigma * attitudeSigma,
                 attitudeSigma * attitudeSigma,
                 biasSigma * biasSigma,
                 biasSigma * biasSigma,
                 biasSigma * biasSigma]
        self.covariance = Matrix.diagonal(d)
    }

    /// Bias-corrected rate estimate (what the controller should use).
    public func correctedRate(gyroMeasurement: Vec3) -> Vec3 {
        gyroMeasurement - bias
    }

    // MARK: Predict

    /// Propagate through one gyro sample.
    /// - Parameters:
    ///   - gyro: raw gyro measurement (rad/s).
    ///   - dt: sample period (s).
    ///   - sigmaV: gyro angle-random-walk (rad/s per sqrt(Hz)).
    ///   - sigmaU: gyro bias random-walk diffusion (rad/s per sqrt(s)).
    public mutating func predict(gyro: Vec3, dt: Double,
                                 sigmaV: Double, sigmaU: Double) {
        let wHat = gyro - bias

        // Quaternion kinematics, exact single-rotation integration.
        let rate = wHat.norm
        if rate > 1e-12 {
            attitude = (attitude * Quat(angle: rate * dt, axis: wHat / rate)).normalized()
        }

        // Error-state dynamics. Our error convention is the *inertial-frame*
        // (left-multiplied) error: q_true = dq(alpha) (x) q_hat, corrected by
        // q_hat <- dq (x) q_hat. Differentiating gives (to first order):
        //   d/dt alpha = -R(q_hat) * (b - b_hat) - R(q_hat) * eta_v
        //   d/dt (b - b_hat) = eta_u
        // so F = [ 0  -R(q_hat) ;  0  0 ]. (The familiar -[w x] term belongs
        // to the body-frame error convention; using it here silently
        // corrupts the bias estimate at high body rates.)
        let r = attitude.toMatrix()  // R(q_hat): body -> inertial
        var f = Matrix.zeros(6, 6)
        for row in 0..<3 {
            for col in 0..<3 {
                f[row, col + 3] = -r[row, col]
            }
        }
        let phi = Matrix.identity(6) + f * dt

        // Discrete process noise (first-order). The gyro noise enters the
        // attitude error through R(q_hat), but it is isotropic, so the
        // covariance contribution is unchanged: R (s^2 I) R' = s^2 I.
        var qd = Matrix.zeros(6, 6)
        let qv = sigmaV * sigmaV * dt
        let qu = sigmaU * sigmaU * dt
        for i in 0..<3 { qd[i, i] = qv; qd[i + 3, i + 3] = qu }

        covariance = phi * covariance * phi.transposed() + qd
        symmetrize()
    }

    // MARK: Update

    /// Fold in one star-tracker quaternion measurement.
    /// - Parameter sigmaStar: 1-sigma per-axis measurement noise (rad).
    public mutating func update(starMeasurement: Quat, sigmaStar: Double) {
        // Innovation: small rotation from estimate to measurement.
        var qErr = starMeasurement * attitude.conjugated()
        if qErr.w < 0 { qErr = -qErr }  // shortest-path sign
        let z = [2 * qErr.x, 2 * qErr.y, 2 * qErr.z]  // small-angle approx

        // H = [I3  0]; S = H P H' + R  (3x3).
        let p11 = covariance.block(row: 0, col: 0, rows: 3, cols: 3)
        var s = p11
        let r = sigmaStar * sigmaStar
        for i in 0..<3 { s[i, i] += r }
        guard let sInv = s.inverted() else { return }

        // K = P H' S^-1  (6x3). P H' is the first three columns of P.
        let pht = covariance.block(row: 0, col: 0, rows: 6, cols: 3)
        let k = pht * sInv

        // State correction.
        var dx = [Double](repeating: 0, count: 6)
        for i in 0..<6 {
            dx[i] = k[i, 0] * z[0] + k[i, 1] * z[1] + k[i, 2] * z[2]
        }
        let dtheta = Vec3(dx[0], dx[1], dx[2])
        let db = Vec3(dx[3], dx[4], dx[5])

        // Apply multiplicative attitude correction on the left:
        // q_hat <- dq(dx_theta) (x) q_hat.
        let dq = Quat(x: 0.5 * dtheta.x, y: 0.5 * dtheta.y, z: 0.5 * dtheta.z, w: 1).normalized()
        attitude = (dq * attitude).normalized()
        bias = bias + db

        // Covariance update: P <- (I - K H) P.
        var kh = Matrix.zeros(6, 6)
        kh.setBlock(row: 0, col: 0, k.block(row: 0, col: 0, rows: 3, cols: 3))
        covariance = (Matrix.identity(6) - kh) * covariance
        symmetrize()
    }

    private mutating func symmetrize() {
        // P <- (P + P') / 2 keeps roundoff from breaking symmetry.
        for r in 0..<6 {
            for c in (r + 1)..<6 {
                let v = 0.5 * (covariance[r, c] + covariance[c, r])
                covariance[r, c] = v
                covariance[c, r] = v
            }
        }
    }
}
