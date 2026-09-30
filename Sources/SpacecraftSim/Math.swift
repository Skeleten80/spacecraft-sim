import Foundation

// MARK: - Vec3

/// 3-element vector. Used for positions, rates, torques, and momenta.
public struct Vec3: Equatable {
    public var x, y, z: Double

    public init(_ x: Double, _ y: Double, _ z: Double) {
        self.x = x; self.y = y; self.z = z
    }

    public static let zero = Vec3(0, 0, 0)
    public static let unitX = Vec3(1, 0, 0)
    public static let unitY = Vec3(0, 1, 0)
    public static let unitZ = Vec3(0, 0, 1)

    public var norm: Double { sqrt(x * x + y * y + z * z) }

    public func normalized() -> Vec3 {
        let n = norm
        guard n > 1e-15 else { return .zero }
        return self / n
    }

    public func dot(_ o: Vec3) -> Double { x * o.x + y * o.y + z * o.z }

    public func cross(_ o: Vec3) -> Vec3 {
        Vec3(y * o.z - z * o.y,
             z * o.x - x * o.z,
             x * o.y - y * o.x)
    }

    public static func + (l: Vec3, r: Vec3) -> Vec3 { Vec3(l.x + r.x, l.y + r.y, l.z + r.z) }
    public static func - (l: Vec3, r: Vec3) -> Vec3 { Vec3(l.x - r.x, l.y - r.y, l.z - r.z) }
    public static prefix func - (v: Vec3) -> Vec3 { Vec3(-v.x, -v.y, -v.z) }
    public static func * (l: Vec3, s: Double) -> Vec3 { Vec3(l.x * s, l.y * s, l.z * s) }
    public static func * (s: Double, l: Vec3) -> Vec3 { l * s }
    public static func / (l: Vec3, s: Double) -> Vec3 { Vec3(l.x / s, l.y / s, l.z / s) }
}

// MARK: - Mat3

/// Fixed 3x3 matrix, row-major. Used for inertia tensors and rotations.
public struct Mat3: Equatable {
    /// Row-major storage, 9 elements.
    public var m: [Double]

    public init(_ m: [Double]) {
        precondition(m.count == 9)
        self.m = m
    }

    public init(diagonal d: Vec3) {
        self.m = [d.x, 0, 0,
                  0, d.y, 0,
                  0, 0, d.z]
    }

    public static let identity = Mat3([1, 0, 0, 0, 1, 0, 0, 0, 1])

    public subscript(r: Int, c: Int) -> Double {
        get { m[r * 3 + c] }
        set { m[r * 3 + c] = newValue }
    }

    public func transposed() -> Mat3 {
        Mat3([m[0], m[3], m[6],
              m[1], m[4], m[7],
              m[2], m[5], m[8]])
    }

    /// Closed-form inverse. Returns nil for singular matrices.
    public func inverted() -> Mat3? {
        let a = m
        let det = a[0] * (a[4] * a[8] - a[5] * a[7])
                  - a[1] * (a[3] * a[8] - a[5] * a[6])
                  + a[2] * (a[3] * a[7] - a[4] * a[6])
        guard abs(det) > 1e-18 else { return nil }
        let inv = 1.0 / det
        return Mat3([
            (a[4] * a[8] - a[5] * a[7]) * inv,
            (a[2] * a[7] - a[1] * a[8]) * inv,
            (a[1] * a[5] - a[2] * a[4]) * inv,
            (a[5] * a[6] - a[3] * a[8]) * inv,
            (a[0] * a[8] - a[2] * a[6]) * inv,
            (a[2] * a[3] - a[0] * a[5]) * inv,
            (a[3] * a[7] - a[4] * a[6]) * inv,
            (a[1] * a[6] - a[0] * a[7]) * inv,
            (a[0] * a[4] - a[1] * a[3]) * inv,
        ])
    }

    public static func * (l: Mat3, r: Mat3) -> Mat3 {
        var o = [Double](repeating: 0, count: 9)
        for i in 0..<3 {
            for j in 0..<3 {
                o[i * 3 + j] = l[i, 0] * r[0, j] + l[i, 1] * r[1, j] + l[i, 2] * r[2, j]
            }
        }
        return Mat3(o)
    }

    public static func * (l: Mat3, v: Vec3) -> Vec3 {
        Vec3(l[0, 0] * v.x + l[0, 1] * v.y + l[0, 2] * v.z,
             l[1, 0] * v.x + l[1, 1] * v.y + l[1, 2] * v.z,
             l[2, 0] * v.x + l[2, 1] * v.y + l[2, 2] * v.z)
    }

    public static func * (l: Mat3, s: Double) -> Mat3 {
        Mat3(l.m.map { $0 * s })
    }
}

/// Skew-symmetric matrix of v, so that skew(v) * u == v.cross(u).
public func skew(_ v: Vec3) -> Mat3 {
    Mat3([0, -v.z, v.y,
          v.z, 0, -v.x,
          -v.y, v.x, 0])
}

// MARK: - Quat

/// Unit quaternion, scalar-last (x, y, z, w), Hamilton convention.
///
/// `attitude` quaternions in this package map body-frame vectors to the
/// inertial frame: v_inertial = q * v_body * q_conjugate.
public struct Quat: Equatable {
    public var x, y, z, w: Double

    public init(x: Double, y: Double, z: Double, w: Double) {
        self.x = x; self.y = y; self.z = z; self.w = w
    }

    public static let identity = Quat(x: 0, y: 0, z: 0, w: 1)

    /// Quaternion for a rotation of `angle` radians about `axis` (need not be unit).
    public init(angle: Double, axis: Vec3) {
        let a = axis.normalized()
        let s = sin(angle / 2)
        self.init(x: a.x * s, y: a.y * s, z: a.z * s, w: cos(angle / 2))
    }

    public func normalized() -> Quat {
        let n = sqrt(x * x + y * y + z * z + w * w)
        guard n > 1e-15 else { return .identity }
        return Quat(x: x / n, y: y / n, z: z / n, w: w / n)
    }

    public func conjugated() -> Quat { Quat(x: -x, y: -y, z: -z, w: w) }

    /// Hamilton product. Applies `rhs` first, then `lhs`.
    public static func * (l: Quat, r: Quat) -> Quat {
        Quat(
            x: l.w * r.x + l.x * r.w + l.y * r.z - l.z * r.y,
            y: l.w * r.y - l.x * r.z + l.y * r.w + l.z * r.x,
            z: l.w * r.z + l.x * r.y - l.y * r.x + l.z * r.w,
            w: l.w * r.w - l.x * r.x - l.y * r.y - l.z * r.z
        )
    }

    public static prefix func - (q: Quat) -> Quat {
        Quat(x: -q.x, y: -q.y, z: -q.z, w: -q.w)
    }

    /// Rotation matrix (body -> inertial).
    public func toMatrix() -> Mat3 {
        let xx = x * x, yy = y * y, zz = z * z
        let xy = x * y, xz = x * z, yz = y * z
        let wx = w * x, wy = w * y, wz = w * z
        return Mat3([
            1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
            2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
            2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy),
        ])
    }

    /// Rotate a body-frame vector into the inertial frame.
    public func rotate(_ v: Vec3) -> Vec3 { toMatrix() * v }

    /// Smallest rotation angle between this and another attitude, in radians.
    public func angle(to other: Quat) -> Double {
        let dot = min(1.0, abs(x * other.x + y * other.y + z * other.z + w * other.w))
        return 2 * acos(dot)
    }
}

// MARK: - Matrix (general, for the Kalman filter)

/// General dense matrix, row-major. Only used where fixed-size types don't
/// fit (the 6x6 EKF covariance). Not performance-critical at these sizes.
public struct Matrix: Equatable {
    public let rows, cols: Int
    public var data: [Double]

    public init(rows: Int, cols: Int, data: [Double]) {
        precondition(data.count == rows * cols)
        self.rows = rows; self.cols = cols; self.data = data
    }

    public static func zeros(_ r: Int, _ c: Int) -> Matrix {
        Matrix(rows: r, cols: c, data: [Double](repeating: 0, count: r * c))
    }

    public static func identity(_ n: Int) -> Matrix {
        var m = zeros(n, n)
        for i in 0..<n { m[i, i] = 1 }
        return m
    }

    public static func diagonal(_ d: [Double]) -> Matrix {
        var m = zeros(d.count, d.count)
        for (i, v) in d.enumerated() { m[i, i] = v }
        return m
    }

    public subscript(r: Int, c: Int) -> Double {
        get { data[r * cols + c] }
        set { data[r * cols + c] = newValue }
    }

    public func block(row: Int, col: Int, rows: Int, cols: Int) -> Matrix {
        var d = [Double]()
        d.reserveCapacity(rows * cols)
        for r in 0..<rows {
            for c in 0..<cols { d.append(self[row + r, col + c]) }
        }
        return Matrix(rows: rows, cols: cols, data: d)
    }

    public mutating func setBlock(row: Int, col: Int, _ m: Matrix) {
        precondition(row + m.rows <= rows && col + m.cols <= cols)
        for r in 0..<m.rows {
            for c in 0..<m.cols { self[row + r, col + c] = m[r, c] }
        }
    }

    public func transposed() -> Matrix {
        var m = Matrix.zeros(cols, rows)
        for r in 0..<rows {
            for c in 0..<cols { m[c, r] = self[r, c] }
        }
        return m
    }

    /// Gauss-Jordan inversion with partial pivoting. Nil if singular.
    public func inverted() -> Matrix? {
        precondition(rows == cols)
        let n = rows
        var a = data
        var inv = Matrix.identity(n).data
        for col in 0..<n {
            var pivot = col
            var best = abs(a[col * n + col])
            for r in (col + 1)..<n {
                let v = abs(a[r * n + col])
                if v > best { best = v; pivot = r }
            }
            guard best > 1e-18 else { return nil }
            if pivot != col {
                for c in 0..<n {
                    a.swapAt(col * n + c, pivot * n + c)
                    inv.swapAt(col * n + c, pivot * n + c)
                }
            }
            let d = a[col * n + col]
            for c in 0..<n { a[col * n + c] /= d; inv[col * n + c] /= d }
            for r in 0..<n where r != col {
                let f = a[r * n + col]
                if f != 0 {
                    for c in 0..<n {
                        a[r * n + c] -= f * a[col * n + c]
                        inv[r * n + c] -= f * inv[col * n + c]
                    }
                }
            }
        }
        return Matrix(rows: n, cols: n, data: inv)
    }

    public static func + (l: Matrix, r: Matrix) -> Matrix {
        precondition(l.rows == r.rows && l.cols == r.cols)
        return Matrix(rows: l.rows, cols: l.cols,
                      data: zip(l.data, r.data).map(+))
    }

    public static func - (l: Matrix, r: Matrix) -> Matrix {
        precondition(l.rows == r.rows && l.cols == r.cols)
        return Matrix(rows: l.rows, cols: l.cols,
                      data: zip(l.data, r.data).map(-))
    }

    public static func * (l: Matrix, r: Matrix) -> Matrix {
        precondition(l.cols == r.rows)
        var d = [Double](repeating: 0, count: l.rows * r.cols)
        for i in 0..<l.rows {
            for k in 0..<l.cols {
                let aik = l[i, k]
                if aik == 0 { continue }
                for j in 0..<r.cols {
                    d[i * r.cols + j] += aik * r[k, j]
                }
            }
        }
        return Matrix(rows: l.rows, cols: r.cols, data: d)
    }

    public static func * (l: Matrix, s: Double) -> Matrix {
        Matrix(rows: l.rows, cols: l.cols, data: l.data.map { $0 * s })
    }

    /// Multiply a 3x1 matrix by interpretation as Vec3.
    public static func * (l: Matrix, v: Vec3) -> Vec3 {
        precondition(l.rows == 3 && l.cols == 3)
        return Vec3(l[0, 0] * v.x + l[0, 1] * v.y + l[0, 2] * v.z,
                    l[1, 0] * v.x + l[1, 1] * v.y + l[1, 2] * v.z,
                    l[2, 0] * v.x + l[2, 1] * v.y + l[2, 2] * v.z)
    }

    /// Interpret an n x 1 matrix as a vector.
    public func columnVector() -> [Double] {
        precondition(cols == 1)
        return data
    }
}
