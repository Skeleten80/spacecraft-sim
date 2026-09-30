import Foundation
/// Rigid-body attitude dynamics with a reaction-wheel actuator assembly.
///
/// Conventions:
/// - `attitude` maps body-frame vectors to the inertial frame.
/// - `omega` is the body angular velocity in rad/s, expressed in the body frame.
/// - Euler's equation: I*dw/dt = -w x (I*w) + tau_control + tau_disturbance.

// MARK: - Reaction wheels

/// Four reaction wheels in a canted pyramid configuration (each tilted 45°
/// from the xy-plane, spaced 90° apart in azimuth). Four wheels for three
/// axes = one level of redundancy: any single wheel can fail and the
/// remaining three still span all of R^3. This is the standard reason real
/// spacecraft fly four wheels, and it makes the failure-injection scenario
/// meaningful instead of instantly fatal.
public struct ReactionWheelAssembly {
    public let axes: [Vec3]          // unit spin axes, body frame
    public private(set) var momentum: [Double]  // N*m*s per wheel, signed
    public let maxTorque: Double     // N*m per wheel
    public let maxMomentum: Double   // N*m*s per wheel
    public private(set) var failed: Set<Int> = []

    /// Torque allocation matrix (4x3): wheel momentum rates = -D+ * tau_body.
    private var allocation: Matrix

    public init(maxTorque: Double = 0.1, maxMomentum: Double = 2.0) {
        self.maxTorque = maxTorque
        self.maxMomentum = maxMomentum
        let cant = Double.pi / 4
        var axes = [Vec3]()
        for k in 0..<4 {
            let phi = Double.pi / 4 + Double(k) * Double.pi / 2
            axes.append(Vec3(cos(cant) * cos(phi),
                            cos(cant) * sin(phi),
                            sin(cant)).normalized())
        }
        self.axes = axes
        self.momentum = [Double](repeating: 0, count: 4)
        self.allocation = ReactionWheelAssembly.pseudoInverse(axes: axes, failed: [])
    }

    /// Distribution matrix D (3x4): body torque = -D * hdot.
    static func distribution(axes: [Vec3], failed: Set<Int>) -> Matrix {
        var d = Matrix.zeros(3, 4)
        for (i, a) in axes.enumerated() where !failed.contains(i) {
            d[0, i] = a.x; d[1, i] = a.y; d[2, i] = a.z
        }
        return d
    }

    /// Moore-Penrose pseudo-inverse of D via normal equations: D+ = D^T (D D^T)^-1.
    /// Valid when the surviving wheels span R^3 (true for any 3 of our 4 axes).
    static func pseudoInverse(axes: [Vec3], failed: Set<Int>) -> Matrix {
        let d = distribution(axes: axes, failed: failed)
        let ddt = d * d.transposed()
        guard let inv = ddt.inverted() else {
            // Rank-deficient (shouldn't happen for <=1 failure); fall back to zeros.
            return Matrix.zeros(4, 3)
        }
        return d.transposed() * inv
    }

    public mutating func fail(wheel index: Int) {
        failed.insert(index)
        allocation = ReactionWheelAssembly.pseudoInverse(axes: axes, failed: failed)
    }

    /// Largest |momentum| / maxMomentum across wheels, 0...1 (can exceed 1 only
    /// transiently if limits are misconfigured).
    public var saturation: Double {
        (momentum.map { abs($0) }.max() ?? 0) / maxMomentum
    }

    /// Apply a desired body torque. Returns the torque actually produced after
    /// per-wheel torque limits, momentum saturation, and failures.
    public mutating func apply(desiredTorque: Vec3, dt: Double) -> Vec3 {
        // Desired wheel momentum rates from D * hdot = -tau  ->  hdot = -D+ * tau.
        // `allocation` is the 4x3 pseudo-inverse D+, applied row by row.
        let t = desiredTorque
        var hdot = [Double](repeating: 0, count: 4)
        for i in 0..<4 {
            hdot[i] = -(allocation[i, 0] * t.x + allocation[i, 1] * t.y + allocation[i, 2] * t.z)
        }

        for i in 0..<4 {
            if failed.contains(i) { hdot[i] = 0; continue }
            // Torque limit.
            hdot[i] = min(maxTorque, max(-maxTorque, hdot[i]))
            // Momentum saturation: cannot push further past the limit.
            if abs(momentum[i]) >= maxMomentum && momentum[i] * hdot[i] > 0 {
                hdot[i] = 0
            }
            momentum[i] += hdot[i] * dt
        }

        // Actual body torque = -D * hdot_actual.
        var tau = Vec3.zero
        for i in 0..<4 where !failed.contains(i) {
            tau = tau + axes[i] * (-hdot[i])
        }
        return tau
    }
}

// MARK: - Spacecraft

/// Rigid body with reaction-wheel actuation.
public struct Spacecraft {
    public var attitude: Quat
    public var omega: Vec3
    public let inertia: Mat3
    public let invInertia: Mat3
    public var wheels: ReactionWheelAssembly

    public init(attitude: Quat = .identity,
                omega: Vec3 = .zero,
                inertia: Mat3,
                wheels: ReactionWheelAssembly = ReactionWheelAssembly()) {
        self.attitude = attitude.normalized()
        self.omega = omega
        self.inertia = inertia
        self.invInertia = inertia.inverted() ?? .identity
        self.wheels = wheels
    }

    /// Advance the state by dt seconds.
    /// - Parameters:
    ///   - torqueCommand: desired control torque in the body frame (N*m).
    ///   - disturbance: external torque in the body frame (N*m).
    public mutating func step(dt: Double, torqueCommand: Vec3, disturbance: Vec3) {
        let tauControl = wheels.apply(desiredTorque: torqueCommand, dt: dt)
        let tau = tauControl + disturbance

        // Euler's equation: I*dw = -(w x I*w) + tau
        let iw = inertia * omega
        let alpha = invInertia * (tau - omega.cross(iw))
        omega = omega + alpha * dt

        // Quaternion kinematics: q_dot = 1/2 q (x) [w; 0].
        // Integrated exactly over dt as a single rotation (exact for constant w).
        let rate = omega.norm
        if rate > 1e-12 {
            let dq = Quat(angle: rate * dt, axis: omega / rate)
            attitude = (attitude * dq).normalized()
        }
    }
}
