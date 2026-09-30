import Foundation
/// Sensor models: a MEMS-grade gyro and a star tracker.
///
/// Both corrupt the true state the way real hardware does — the whole point
/// of the estimator is to recover the truth from these imperfect measurements.

// MARK: - Gyro

/// Rate gyro with bias instability and angle random walk.
///
/// - `trueBias` drifts as a random walk with diffusion `biasRW`
///   (rad/s per sqrt(s)).
/// - Each sample adds white noise with standard deviation
///   `arw / sqrt(dt)` (arw in rad/s per sqrt(Hz)).
public struct Gyro {
    public var trueBias: Vec3
    public let arw: Double
    public let biasRW: Double

    public init(trueBias: Vec3 = .zero,
                arw: Double = 1e-4,
                biasRW: Double = 1e-6) {
        self.trueBias = trueBias
        self.arw = arw
        self.biasRW = biasRW
    }

    public mutating func measure(trueOmega: Vec3, dt: Double, rng: inout RNG) -> Vec3 {
        // Bias random walk: b += sigma_u * sqrt(dt) * N(0,1).
        trueBias = trueBias + rng.gaussianVec3() * (biasRW * sqrt(dt))
        // White measurement noise: sigma_v / sqrt(dt) * N(0,1).
        let noise = rng.gaussianVec3() * (arw / sqrt(dt))
        return trueOmega + trueBias + noise
    }
}

// MARK: - Star tracker

/// Absolute attitude sensor. Returns the true attitude corrupted by a small
/// random rotation (1-sigma `noiseRad` radians per axis), at `rateHz`.
public struct StarTracker {
    public let noiseRad: Double
    public let rateHz: Double

    public init(noiseRad: Double = 1e-4, rateHz: Double = 1.0) {
        self.noiseRad = noiseRad
        self.rateHz = rateHz
    }

    public func measure(trueAttitude: Quat, rng: inout RNG) -> Quat {
        let err = rng.gaussianVec3() * noiseRad
        let angle = err.norm
        let dq: Quat
        if angle > 1e-15 {
            dq = Quat(angle: angle, axis: err / angle)
        } else {
            dq = .identity
        }
        // Noise applied in the inertial frame: q_meas = dq (x) q_true.
        return (dq * trueAttitude).normalized()
    }
}
