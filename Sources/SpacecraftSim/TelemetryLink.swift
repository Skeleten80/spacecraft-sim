// TelemetryLink.swift
//
// Network telemetry for the multi-machine mission setup.
//
// Mini 1 ("the spacecraft") runs `spacecraft-cli serve` -> TelemetryServer:
// it steps a SimEngine and publishes every sample as newline-delimited JSON
// (NDJSON) over TCP. Mini 2 ("mission control") runs a TelemetryClient — the
// MissionOps dashboard, a Python analysis script, anything that speaks the
// protocol — and subscribes. A client->server command channel carries control
// back (kill a wheel, load a scenario), so the dashboard keeps its fault
// injection buttons against a remote sim.
//
// Wire protocol (UTF-8, one JSON object per line):
//   server -> client: {"type":"hello","protocolVersion":1,"scenario":...,"dt":...,"duration":...}
//   server -> client: {"type":"frame", ...telemetry...}
//   server -> client: {"type":"end","reason":"complete"}            (run finished)
//   client -> server: {"type":"command","cmd":"killWheel","index":0}
//   client -> server: {"type":"command","cmd":"loadScenario","scenario":"tumble"}
//   client -> server: {"type":"command","cmd":"setSpeed","value":10}
//
// The server accepts subscribers in a loop: each new connection replays the
// scenario from t = 0, so a dashboard can Reset / reconnect at any time.
// Call stop() to break out of serve().
//
// Sockets are raw POSIX (Darwin/Glibc) so the core library keeps its zero
// dependencies and stays Linux-friendly. One subscriber per server; restart
// `serve` to accept another run.

#if canImport(Darwin)
import Darwin
#elseif canImport(Glibc)
import Glibc
#endif
import Foundation

// MARK: - Errors

public enum LinkError: Error, Equatable {
    case socketFailed
    case bindFailed(errno: Int32)
    case listenFailed
    case acceptFailed
    case connectFailed(errno: Int32)
    case sendFailed
    case unknownScenario(String)
}

// MARK: - Messages

/// First line the server sends (and re-sends after a loadScenario command).
public struct TelemetryHello: Codable {
    public var type = "hello"
    public var protocolVersion: Int
    public var scenario: String
    public var dt: Double
    public var duration: Double

    public init(scenario: String, dt: Double, duration: Double) {
        self.protocolVersion = 1
        self.scenario = scenario
        self.dt = dt
        self.duration = duration
    }
}

/// One simulation step, rich enough to drive the full MissionOps dashboard:
/// 3D attitude view, strip charts, and every telemetry card.
public struct TelemetryFrame: Codable {
    public var type = "frame"
    public var t: Double
    public var estimatedAttitude: [Double]  // quat [x, y, z, w]
    public var targetAttitude: [Double]    // quat [x, y, z, w]
    public var omega: [Double]             // rad/s body rates [x, y, z]
    public var pointErrDeg: Double
    public var estErrDeg: Double
    public var omegaDegS: Double
    public var biasErrDegS: Double
    public var wheelSaturation: Double
    public var torqueNorm: Double
    public var wheelMomenta: [Double]
    public var wheelMaxMomentum: Double
    public var failedWheels: [Int]
    public var thrusterFirings: Int
    public var thrusterBurnTime: Double
    public var sunValid: Bool

    public init(engine: SimEngine) {
        let q = engine.estimatedAttitude
        let tq = engine.targetAttitude
        let w = engine.omega
        t = engine.t
        estimatedAttitude = [q.x, q.y, q.z, q.w]
        targetAttitude = [tq.x, tq.y, tq.z, tq.w]
        omega = [w.x, w.y, w.z]
        pointErrDeg = engine.pointingErrorDeg
        estErrDeg = engine.estimatorErrorDeg
        omegaDegS = engine.omega.norm / (Double.pi / 180)
        biasErrDegS = engine.biasErrorDegS
        wheelSaturation = engine.wheelSaturation
        torqueNorm = engine.lastTorque.norm
        wheelMomenta = engine.wheelMomenta
        wheelMaxMomentum = engine.wheelMaxMomentum
        failedWheels = Array(engine.failedWheels).sorted()
        thrusterFirings = engine.thrusterFirings
        thrusterBurnTime = engine.thrusterBurnTime
        sunValid = engine.sunValid
    }
}

/// Last line the server sends when the run completes.
public struct TelemetryEnd: Codable {
    public var type = "end"
    public var reason: String

    public init(reason: String) { self.reason = reason }
}

/// Client -> server control message.
public struct TelemetryCommand: Codable {
    public var type = "command"
    public var cmd: String
    public var index: Int?
    public var scenario: String?
    public var value: Double?

    public static func killWheel(_ i: Int) -> TelemetryCommand {
        TelemetryCommand(cmd: "killWheel", index: i, scenario: nil, value: nil)
    }

    public static func loadScenario(_ name: String) -> TelemetryCommand {
        TelemetryCommand(cmd: "loadScenario", index: nil, scenario: name, value: nil)
    }

    public static func setSpeed(_ simSecondsPerRealSecond: Double) -> TelemetryCommand {
        TelemetryCommand(cmd: "setSpeed", index: nil, scenario: nil,
                         value: simSecondsPerRealSecond)
    }

    private init(cmd: String, index: Int?, scenario: String?, value: Double?) {
        self.cmd = cmd
        self.index = index
        self.scenario = scenario
        self.value = value
    }
}

/// Minimal envelope for dispatching inbound lines by type.
private struct Envelope: Codable {
    var type: String
}

// MARK: - POSIX helpers

private func streamSocket() -> Int32 {
    #if canImport(Darwin)
    return socket(AF_INET, SOCK_STREAM, 0)
    #else
    return socket(AF_INET, Int32(SOCK_STREAM.rawValue), 0)
    #endif
}

private func anyAddress(port: UInt16) -> sockaddr_in {
    var addr = sockaddr_in()
    addr.sin_family = sa_family_t(AF_INET)
    addr.sin_port = port.bigEndian          // htons
    addr.sin_addr.s_addr = INADDR_ANY
    return addr
}

private func loopbackAddress(port: UInt16, host: String) -> sockaddr_in? {
    var addr = sockaddr_in()
    addr.sin_family = sa_family_t(AF_INET)
    addr.sin_port = port.bigEndian
    let ip = host.withCString { inet_addr($0) }
    guard ip != INADDR_NONE else { return nil }
    addr.sin_addr.s_addr = ip
    return addr
}

private func withSockAddr<T>(_ addr: inout sockaddr_in,
                             _ body: (UnsafePointer<sockaddr>, socklen_t) -> T) -> T {
    withUnsafePointer(to: &addr) {
        $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
            body($0, socklen_t(MemoryLayout<sockaddr_in>.size))
        }
    }
}

private func sendAll(_ fd: Int32, _ bytes: [UInt8]) -> Bool {
    var sent = 0
    while sent < bytes.count {
        let n = bytes.withUnsafeBytes { ptr in
            send(fd, ptr.baseAddress!.advanced(by: sent), bytes.count - sent, 0)
        }
        if n <= 0 { return false }
        sent += n
    }
    return true
}

private func sendLine(_ fd: Int32, _ value: some Encodable) -> Bool {
    guard let data = try? JSONEncoder().encode(value) else { return false }
    var bytes = [UInt8](data)
    bytes.append(10)  // newline
    return sendAll(fd, bytes)
}

// MARK: - Server

/// Steps a SimEngine and streams hello/frame/end lines to one TCP subscriber.
/// Pacing: `speed` sim-seconds per real second; 0 means unthrottled.
public final class TelemetryServer {
    public let port: UInt16
    public private(set) var localPort: UInt16 = 0  // actual port after bind (for port 0)

    /// Called on the serving thread right after bind, with the actual port.
    public var onReady: ((UInt16) -> Void)?

    private var config: ScenarioConfig
    private var speed: Double

    private let lock = NSLock()
    private var pendingCommands = [TelemetryCommand]()
    private var clientGone = false

    private let stateLock = NSLock()
    private var listenFd: Int32 = -1
    private var stopRequested = false

    public init(config: ScenarioConfig, port: UInt16 = 9001, speed: Double = 1) {
        self.config = config
        self.port = port
        self.speed = speed
    }

    public convenience init(scenario name: String, port: UInt16 = 9001,
                            speed: Double = 1) throws {
        guard let config = builtinScenario(name) else {
            throw LinkError.unknownScenario(name)
        }
        self.init(config: config, port: port, speed: speed)
    }

    /// Blocks: binds, then accepts subscribers in a loop, streaming a fresh
    /// run of the scenario to each. Returns after stop() is called.
    public func serve() throws {
        let fd = streamSocket()
        guard fd >= 0 else { throw LinkError.socketFailed }

        var reuse: Int32 = 1
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse,
                   socklen_t(MemoryLayout<Int32>.size))

        var addr = anyAddress(port: port)
        let bound = withSockAddr(&addr) { bind(fd, $0, $1) }
        guard bound == 0 else {
            close(fd)
            throw LinkError.bindFailed(errno: errno)
        }

        // Read back the actual port (matters when port == 0).
        var boundAddr = sockaddr_in()
        var len = socklen_t(MemoryLayout<sockaddr_in>.size)
        withUnsafeMutablePointer(to: &boundAddr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                _ = getsockname(fd, $0, &len)
            }
        }
        localPort = boundAddr.sin_port.bigEndian
        onReady?(localPort)

        guard listen(fd, 5) == 0 else {
            close(fd)
            throw LinkError.listenFailed
        }

        stateLock.lock()
        listenFd = fd
        let stopped = stopRequested
        stateLock.unlock()
        guard !stopped else { close(fd); return }

        while true {
            let clientFd = accept(fd, nil, nil)
            if clientFd < 0 {
                if isStopped() { break }
                close(fd)
                throw LinkError.acceptFailed
            }
            runSession(clientFd)
            close(clientFd)
            if isStopped() { break }
        }
        close(fd)
        stateLock.lock()
        listenFd = -1
        stateLock.unlock()
    }

    /// Ask serve() to return. Safe to call from another thread; unblocks a
    /// pending accept(). The active session (if any) finishes its run first
    /// when the client disconnects, or immediately if none is connected.
    public func stop() {
        stateLock.lock()
        stopRequested = true
        let fd = listenFd
        stateLock.unlock()
        if fd >= 0 {
            _ = shutdown(fd, Int32(SHUT_RDWR))
            close(fd)
        }
    }

    private func isStopped() -> Bool {
        stateLock.lock(); defer { stateLock.unlock() }
        return stopRequested
    }

    // MARK: Session

    private func runSession(_ fd: Int32) {
        lock.lock()
        pendingCommands = []
        clientGone = false
        lock.unlock()

        startCommandReader(fd)

        var engine = SimEngine(config: config)
        _ = sendLine(fd, TelemetryHello(scenario: config.name,
                                        dt: config.dt, duration: config.duration))

        while engine.t <= config.duration {
            if isClientGone() { break }
            drainCommands(into: &engine, fd: fd)
            if isClientGone() { break }

            let stepStart = Date()
            engine.advance()
            if !sendLine(fd, TelemetryFrame(engine: engine)) { break }

            if speed > 0 {
                let target = config.dt / speed
                let elapsed = Date().timeIntervalSince(stepStart)
                if target > elapsed { Thread.sleep(forTimeInterval: target - elapsed) }
            }
        }
        if !isClientGone() {
            _ = sendLine(fd, TelemetryEnd(reason: "complete"))
        }
    }

    /// Blocking-recv command reader on its own thread (safe: the session
    /// thread only sends on this fd, never receives).
    private func startCommandReader(_ fd: Int32) {
        Thread { [weak self] in
            guard let self else { return }
            var buffer = [UInt8]()
            var tmp = [UInt8](repeating: 0, count: 4096)
            while true {
                let n = recv(fd, &tmp, tmp.count, 0)
                if n <= 0 { break }  // client closed or error
                buffer.append(contentsOf: tmp[0..<n])
                while let nl = buffer.firstIndex(of: 10) {
                    let line = Data(buffer[0..<nl])
                    buffer.removeFirst(nl + 1)
                    if let cmd = try? JSONDecoder().decode(TelemetryCommand.self, from: line),
                       cmd.type == "command" {
                        self.lock.lock()
                        self.pendingCommands.append(cmd)
                        self.lock.unlock()
                    }
                }
            }
            self.lock.lock()
            self.clientGone = true
            self.lock.unlock()
        }.start()
    }

    private func drainCommands(into engine: inout SimEngine, fd: Int32) {
        lock.lock()
        let cmds = pendingCommands
        pendingCommands = []
        lock.unlock()
        for cmd in cmds {
            switch cmd.cmd {
            case "killWheel":
                if let i = cmd.index { engine.killWheel(i) }
            case "loadScenario":
                if let name = cmd.scenario, let c = builtinScenario(name) {
                    config = c
                    engine = SimEngine(config: c)
                    _ = sendLine(fd, TelemetryHello(scenario: c.name, dt: c.dt, duration: c.duration))
                }
            case "setSpeed":
                if let s = cmd.value { speed = max(0, s) }
            default:
                break  // unknown commands are ignored
            }
        }
    }

    private func isClientGone() -> Bool {
        lock.lock(); defer { lock.unlock() }
        return clientGone
    }
}

// MARK: - Client

/// Subscribes to a TelemetryServer. `run` blocks on the calling thread;
/// `sendCommand` is safe to call from another thread while `run` is active.
public final class TelemetryClient {
    public let host: String
    public let port: UInt16

    private var fd: Int32 = -1

    public init(host: String = "127.0.0.1", port: UInt16 = 9001) {
        self.host = host
        self.port = port
    }

    public func connect() throws {
        let sock = streamSocket()
        guard sock >= 0 else { throw LinkError.socketFailed }
        guard var addr = loopbackAddress(port: port, host: host) else {
            close(sock)
            throw LinkError.connectFailed(errno: EINVAL)
        }
        let r = withSockAddr(&addr) {
            #if canImport(Darwin)
            Darwin.connect(sock, $0, $1)
            #else
            SwiftGlibc.connect(sock, $0, $1)
            #endif
        }
        guard r == 0 else {
            let e = errno
            close(sock)
            throw LinkError.connectFailed(errno: e)
        }
        fd = sock
    }

    public func run(onHello: @escaping (TelemetryHello) -> Void,
                    onFrame: @escaping (TelemetryFrame) -> Void,
                    onEnd: @escaping (String) -> Void) {
        var buffer = [UInt8]()
        var tmp = [UInt8](repeating: 0, count: 65536)
        let decoder = JSONDecoder()
        while true {
            let n = recv(fd, &tmp, tmp.count, 0)
            if n <= 0 { break }
            buffer.append(contentsOf: tmp[0..<n])
            while let nl = buffer.firstIndex(of: 10) {
                let line = Data(buffer[0..<nl])
                buffer.removeFirst(nl + 1)
                guard let env = try? decoder.decode(Envelope.self, from: line) else { continue }
                switch env.type {
                case "hello":
                    if let h = try? decoder.decode(TelemetryHello.self, from: line) { onHello(h) }
                case "frame":
                    if let f = try? decoder.decode(TelemetryFrame.self, from: line) { onFrame(f) }
                case "end":
                    if let e = try? decoder.decode(TelemetryEnd.self, from: line) { onEnd(e.reason) }
                    return
                default:
                    continue
                }
            }
        }
    }

    public func sendCommand(_ cmd: TelemetryCommand) throws {
        guard fd >= 0, sendLine(fd, cmd) else { throw LinkError.sendFailed }
    }

    public func disconnect() {
        if fd >= 0 { close(fd); fd = -1 }
    }
}
