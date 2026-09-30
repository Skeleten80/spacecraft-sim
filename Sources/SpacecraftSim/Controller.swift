import Foundation
/// Quaternion-feedback attitude controller (Wie & Barba regulator).
///
/// Control law:  tau = -Kp * vec(q_err) - Kd * w - Ki * integral(vec(q_err))
/// where q_err = q_target^-1 (x) q_estimate is the error quaternion.
///
/// The vector part of the error quaternion is (for small angles) half the
/// eigenaxis rotation taking the estimate to the target, so this is a
/// PD controller on SO(3) with an integral term for constant disturbances.
/// Shortest-path sign handling (flip when scalar part < 0) avoids the
/// "unwinding" problem of taking the long way around.
public struct AttitudeController {
    public var kp: Vec3
    public var kd: Vec3
    public var ki: Vec3
    public private(set) var integral: Vec3 = .zero
    public let integralLimit: Double  // anti-windup (rad*s)

    public init(kp: Vec3, kd: Vec3, ki: Vec3 = .zero, integralLimit: Double = 0.5) {
        self.kp = kp; self.kd = kd; self.ki = ki
        self.integralLimit = integralLimit
    }

    /// Tune per-axis gains from inertia for a target closed-loop response.
    /// - Parameters:
    ///   - inertia: diagonal inertia (kg*m^2).
    ///   - naturalFreq: desired closed-loop natural frequency (rad/s).
    ///   - damping: desired damping ratio (0.9 = gentle, no overshoot).
    ///   - integralFreq: integrator corner frequency (rad/s); 0 disables.
    public static func tuned(inertia: Vec3,
                             naturalFreq: Double,
                             damping: Double,
                             integralFreq: Double = 0) -> AttitudeController {
        let wn2 = naturalFreq * naturalFreq
        let kp = Vec3(inertia.x * wn2, inertia.y * wn2, inertia.z * wn2)
        let kd = Vec3(2 * damping * naturalFreq * inertia.x,
                      2 * damping * naturalFreq * inertia.y,
                      2 * damping * naturalFreq * inertia.z)
        var ki = Vec3.zero
        if integralFreq > 0 {
            let wi3 = integralFreq * integralFreq * integralFreq
            ki = Vec3(inertia.x * wi3, inertia.y * wi3, inertia.z * wi3)
        }
        return AttitudeController(kp: kp, kd: kd, ki: ki)
    }

    /// Compute the control torque in the body frame.
    public mutating func torque(target: Quat,
                                estimatedAttitude: Quat,
                                estimatedOmega: Vec3,
                                dt: Double) -> Vec3 {
        var qErr = target.conjugated() * estimatedAttitude
        if qErr.w < 0 { qErr = -qErr }
        let ev = Vec3(qErr.x, qErr.y, qErr.z)

        integral = integral + ev * dt
        let inorm = integral.norm
        if inorm > integralLimit { integral = integral * (integralLimit / inorm) }

        return Vec3(-kp.x * ev.x - kd.x * estimatedOmega.x - ki.x * integral.x,
                    -kp.y * ev.y - kd.y * estimatedOmega.y - ki.y * integral.y,
                    -kp.z * ev.z - kd.z * estimatedOmega.z - ki.z * integral.z)
    }

    public mutating func reset() { integral = .zero }
}

// MARK: - Helpers

public extension Vec3 {
    static func * (l: Vec3, r: Vec3) -> Vec3 {
        Vec3(l.x * r.x, l.y * r.y, l.z * r.z)
    }
}
