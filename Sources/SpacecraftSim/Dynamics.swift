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

// MARK: - Thrusters

/// A single on/off reaction-control thruster.
///
/// Position and thrust direction are fixed in the body frame. The valve is
/// binary: the thruster produces `maxThrust` along `direction`, or nothing.
/// There is no throttling — fine control comes from short pulses, and the
/// shortest pulse the valve can produce (the minimum impulse bit) sets the
/// pointing floor: near the target, commanded pulses shrink below one
/// impulse bit, get dropped, and the craft limit-cycles instead of
/// converging smoothly. That limit cycle is the honest RCS behavior.
public struct Thruster {
    public let position: Vec3    // m, body frame
    public let direction: Vec3   // unit, body frame
    public let maxThrust: Double // N

    /// Torque about the body origin at full thrust (N*m).
    public var torqueAuthority: Vec3 { position.cross(direction * maxThrust) }

    public init(position: Vec3, direction: Vec3, maxThrust: Double) {
        self.position = position
        self.direction = direction.normalized()
        self.maxThrust = maxThrust
    }
}

/// On/off RCS thruster assembly with direction-group torque allocation and
/// minimum-impulse-bit quantization via sigma-delta modulation.
///
/// Allocation: thrusters are grouped by torque-authority direction (opposing
/// directions form separate groups). Each group fires its members at a
/// common duty = max(0, tau . groupDir) / groupAuthority, so the achieved
/// torque matches the command exactly (within saturation) — no negative
/// "firing fractions", which a clipped pseudo-inverse would produce.
/// Demanded on-time accumulates per thruster (sigma-delta); once it reaches
/// `minOnTime` the valve opens for that pulse. Small demands therefore
/// produce sparse, full-strength pulses — the average torque tracks the
/// command while the instantaneous torque is bang-bang.
///
/// Net thrust force is not tracked: the sim has no translation state, so
/// only torque enters the dynamics.
public struct ThrusterAssembly {
    public let thrusters: [Thruster]
    public let minOnTime: Double              // s, minimum impulse bit
    public private(set) var failed: Set<Int> = []
    public private(set) var totalFirings = 0   // pulses started, all thrusters
    public private(set) var totalOnTime = 0.0  // s of burn, all thrusters

    private var remaining: [Double]    // s of burn left per thruster
    private var accumulator: [Double]  // demanded on-time not yet fired, per thruster
    private var groups: [(direction: Vec3, members: [Int])]

    public init(thrusters: [Thruster], minOnTime: Double = 0.02) {
        self.thrusters = thrusters
        self.minOnTime = minOnTime
        self.remaining = [Double](repeating: 0, count: thrusters.count)
        self.accumulator = [Double](repeating: 0, count: thrusters.count)
        self.groups = ThrusterAssembly.group(thrusters: thrusters, failed: [])
    }

    /// Default 12-thruster block: a redundant pair for each torque direction
    /// about each body axis (4 per axis), 1 N each on 0.5 m moment arms:
    /// 0.5 N*m per thruster, 1.0 N*m per axis-direction at full duty.
    public init(maxThrust: Double = 1.0, minOnTime: Double = 0.02, halfWidth: Double = 0.5) {
        let h = halfWidth
        let f = maxThrust
        var ts = [Thruster]()
        // +/- x torque.
        ts.append(Thruster(position: Vec3(0, h, 0), direction: Vec3(0, 0, 1), maxThrust: f))
        ts.append(Thruster(position: Vec3(0, -h, 0), direction: Vec3(0, 0, -1), maxThrust: f))
        ts.append(Thruster(position: Vec3(0, h, 0), direction: Vec3(0, 0, -1), maxThrust: f))
        ts.append(Thruster(position: Vec3(0, -h, 0), direction: Vec3(0, 0, 1), maxThrust: f))
        // +/- y torque.
        ts.append(Thruster(position: Vec3(0, 0, h), direction: Vec3(1, 0, 0), maxThrust: f))
        ts.append(Thruster(position: Vec3(0, 0, -h), direction: Vec3(-1, 0, 0), maxThrust: f))
        ts.append(Thruster(position: Vec3(0, 0, h), direction: Vec3(-1, 0, 0), maxThrust: f))
        ts.append(Thruster(position: Vec3(0, 0, -h), direction: Vec3(1, 0, 0), maxThrust: f))
        // +/- z torque.
        ts.append(Thruster(position: Vec3(h, 0, 0), direction: Vec3(0, 1, 0), maxThrust: f))
        ts.append(Thruster(position: Vec3(-h, 0, 0), direction: Vec3(0, -1, 0), maxThrust: f))
        ts.append(Thruster(position: Vec3(h, 0, 0), direction: Vec3(0, -1, 0), maxThrust: f))
        ts.append(Thruster(position: Vec3(-h, 0, 0), direction: Vec3(0, 1, 0), maxThrust: f))
        self.init(thrusters: ts, minOnTime: minOnTime)
    }

    /// Group thrusters by torque-authority direction; opposing directions
    /// form separate groups. Thrusters with no authority are skipped.
    static func group(thrusters: [Thruster], failed: Set<Int>)
        -> [(direction: Vec3, members: [Int])]
    {
        var groups = [(direction: Vec3, members: [Int])]()
        for (i, th) in thrusters.enumerated() where !failed.contains(i) {
            let a = th.torqueAuthority
            let n = a.norm
            guard n > 1e-12 else { continue }
            let u = a / n
            if let g = groups.firstIndex(where: { $0.direction.dot(u) > 1 - 1e-9 }) {
                groups[g].members.append(i)
            } else {
                groups.append((direction: u, members: [i]))
            }
        }
        return groups
    }

    public mutating func fail(thruster index: Int) {
        failed.insert(index)
        groups = ThrusterAssembly.group(thrusters: thrusters, failed: failed)
    }

    /// Apply a desired body torque. Returns the time-averaged torque actually
    /// produced this step after allocation, duty clamping, failures, and
    /// minimum-impulse-bit quantization.
    public mutating func apply(desiredTorque: Vec3, dt: Double) -> Vec3 {
        let n = thrusters.count
        var tau = Vec3.zero

        // Finish pulses already in progress (the last step may be fractional).
        for i in 0..<n {
            if failed.contains(i) || remaining[i] <= 0 { continue }
            let burn = min(dt, remaining[i])
            tau = tau + thrusters[i].torqueAuthority * (burn / dt)
            remaining[i] -= burn
            totalOnTime += burn
        }

        // Sigma-delta: accumulate demanded on-time (even mid-pulse, so no
        // demand is lost), then open the valve of any idle thruster whose
        // accumulated demand reached one minimum impulse bit.
        let t = desiredTorque
        for i in 0..<n where !failed.contains(i) {
            accumulator[i] += groupDuty(for: i, torque: t) * dt
        }
        for i in 0..<n {
            if failed.contains(i) || remaining[i] > 0 { continue }
            if accumulator[i] > 0 && accumulator[i] >= minOnTime {
                let burn = min(dt, accumulator[i])
                tau = tau + thrusters[i].torqueAuthority * (burn / dt)
                remaining[i] = accumulator[i] - burn
                accumulator[i] = 0
                totalFirings += 1
                totalOnTime += burn
            }
        }
        return tau
    }

    /// Firing duty (0...1) for one thruster from its direction group:
    /// the group's common duty, shared by all members.
    private func groupDuty(for i: Int, torque t: Vec3) -> Double {
        for g in groups where g.members.contains(i) {
            var authority = 0.0
            for m in g.members { authority += thrusters[m].torqueAuthority.norm }
            guard authority > 1e-12 else { return 0 }
            return min(1, max(0, t.dot(g.direction) / authority))
        }
        return 0
    }
}

// MARK: - Spacecraft

/// Which actuator the flight computer drives.
public enum ActuatorMode {
    case wheels
    case thrusters
}

/// Rigid body with reaction-wheel or thruster actuation.
public struct Spacecraft {
    public var attitude: Quat
    public var omega: Vec3
    public let inertia: Mat3
    public let invInertia: Mat3
    public var wheels: ReactionWheelAssembly
    public var thrusters: ThrusterAssembly
    public var actuatorMode: ActuatorMode

    public init(attitude: Quat = .identity,
                omega: Vec3 = .zero,
                inertia: Mat3,
                wheels: ReactionWheelAssembly = ReactionWheelAssembly(),
                thrusters: ThrusterAssembly = ThrusterAssembly(),
                actuatorMode: ActuatorMode = .wheels) {
        self.attitude = attitude.normalized()
        self.omega = omega
        self.inertia = inertia
        self.invInertia = inertia.inverted() ?? .identity
        self.wheels = wheels
        self.thrusters = thrusters
        self.actuatorMode = actuatorMode
    }

    /// Advance the state by dt seconds.
    /// - Parameters:
    ///   - torqueCommand: desired control torque in the body frame (N*m).
    ///   - disturbance: external torque in the body frame (N*m).
    public mutating func step(dt: Double, torqueCommand: Vec3, disturbance: Vec3) {
        let tauControl: Vec3
        switch actuatorMode {
        case .wheels:
            tauControl = wheels.apply(desiredTorque: torqueCommand, dt: dt)
        case .thrusters:
            tauControl = thrusters.apply(desiredTorque: torqueCommand, dt: dt)
        }
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
