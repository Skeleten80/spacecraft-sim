import Foundation
/// Deterministic pseudo-random number generator (XorShift64*).
///
/// Spacecraft sims must be reproducible: same seed -> same sensor noise ->
/// same trajectory. That makes tests meaningful and lets you re-run a
/// scenario bit-for-bit when tuning gains.
public struct RNG {
    private var state: UInt64

    public init(seed: UInt64) {
        // XorShift degenerates on a zero state; use a fixed nonzero fallback.
        self.state = seed == 0 ? 0x9E3779B97F4A7C15 : seed
    }

    public mutating func next() -> UInt64 {
        var x = state
        x ^= x >> 12
        x ^= x << 25
        x ^= x >> 27
        state = x
        return x &* 0x2545F4914F6CDD1D
    }

    /// Uniform in [0, 1).
    public mutating func uniform() -> Double {
        Double(next() >> 11) / Double(1 << 53)
    }

    /// Uniform in [low, high).
    public mutating func uniform(low: Double, high: Double) -> Double {
        low + (high - low) * uniform()
    }

    /// Standard normal via Box-Muller.
    public mutating func gaussian() -> Double {
        var u1 = uniform()
        let u2 = uniform()
        if u1 < 1e-12 { u1 = 1e-12 }
        return sqrt(-2.0 * log(u1)) * cos(2.0 * .pi * u2)
    }

    /// Vector of three independent standard normals.
    public mutating func gaussianVec3() -> Vec3 {
        Vec3(gaussian(), gaussian(), gaussian())
    }

    /// Uniform random unit vector (Marsaglia's method).
    public mutating func unitVector() -> Vec3 {
        while true {
            let x = uniform(low: -1, high: 1)
            let y = uniform(low: -1, high: 1)
            let s = x * x + y * y
            if s < 1.0 && s > 1e-9 {
                let t = sqrt(1.0 - s)
                return Vec3(2 * x * t, 2 * y * t, 1 - 2 * s)
            }
        }
    }
}
