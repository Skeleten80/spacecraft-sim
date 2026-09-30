import XCTest
@testable import SpacecraftSim
import Dispatch
import Foundation

/// End-to-end tests for the network telemetry link (TelemetryLink.swift):
/// a real TelemetryServer streams to a real TelemetryClient over loopback TCP.
final class TelemetryLinkTests: XCTestCase {

    /// Start an unthrottled server for a short scenario; returns the server,
    /// the port it bound, and an expectation fulfilled when serve() returns.
    /// Callers must call server.stop() and wait on `done` at the end —
    /// serve() now accepts subscribers in a loop.
    private func startServer(config: ScenarioConfig)
        -> (TelemetryServer, UInt16, XCTestExpectation) {
        let server = TelemetryServer(config: config, port: 0, speed: 0)
        let lock = NSLock()
        var boundPort: UInt16 = 0
        let ready = expectation(description: "server bound")
        server.onReady = { port in
            lock.lock(); boundPort = port; lock.unlock()
            ready.fulfill()
        }
        let done = expectation(description: "server stopped")
        DispatchQueue.global().async {
            _ = try? server.serve()
            done.fulfill()
        }
        wait(for: [ready], timeout: 10)
        lock.lock(); defer { lock.unlock() }
        return (server, boundPort, done)
    }

    private func shortNominal() -> ScenarioConfig {
        var c = builtinScenario("nominal")!
        c.duration = 2.0  // 200 frames @ 100 Hz
        return c
    }

    func testRoundTripMatchesLocalRun() throws {
        let config = shortNominal()
        let (server, port, done) = startServer(config: config)
        XCTAssertNotEqual(port, 0)

        let client = TelemetryClient(port: port)
        try client.connect()

        var hellos = [TelemetryHello]()
        var frames = [TelemetryFrame]()
        var endReason = ""
        client.run(onHello: { hellos.append($0) },
                   onFrame: { frames.append($0) },
                   onEnd: { endReason = $0 })
        client.disconnect()
        server.stop()
        wait(for: [done], timeout: 10)

        // Protocol framing.
        XCTAssertEqual(hellos.count, 1)
        XCTAssertEqual(hellos.first?.scenario, "nominal")
        XCTAssertEqual(hellos.first?.protocolVersion, 1)
        XCTAssertEqual(hellos.first?.dt ?? -1, config.dt, accuracy: 1e-12)
        XCTAssertEqual(endReason, "complete")

        // The streamed trajectory must equal a local run step-for-step
        // (same seed -> identical SimEngine trajectory). Note: runScenario
        // decimates its stored samples to 0.5 s, while the link streams every
        // 100 Hz step, so the reference is built frame-by-frame here.
        var refEngine = SimEngine(config: config)
        var refFrames = [TelemetryFrame]()
        while refEngine.t <= config.duration {
            refEngine.advance()
            refFrames.append(TelemetryFrame(engine: refEngine))
        }
        XCTAssertEqual(frames.count, refFrames.count)
        XCTAssertEqual(frames.count, 200)
        for (f, r) in zip(frames, refFrames) {
            XCTAssertEqual(f.t, r.t, accuracy: 1e-12)
            XCTAssertEqual(f.pointErrDeg, r.pointErrDeg, accuracy: 1e-12)
            XCTAssertEqual(f.estimatedAttitude, r.estimatedAttitude)
            XCTAssertEqual(f.omega, r.omega)
            XCTAssertEqual(f.thrusterFirings, r.thrusterFirings)
        }

        // Frame carries everything the dashboard needs.
        let first = try XCTUnwrap(frames.first)
        XCTAssertEqual(first.estimatedAttitude.count, 4)
        XCTAssertEqual(first.targetAttitude.count, 4)
        XCTAssertEqual(first.omega.count, 3)
        XCTAssertEqual(first.wheelMomenta.count, 4)
    }

    func testKillWheelCommand() throws {
        let (server, port, done) = startServer(config: shortNominal())
        let client = TelemetryClient(port: port)
        try client.connect()
        // Inject the fault before reading: it sits in the socket buffer and
        // the server applies it at (or within a step or two of) t = 0.
        try client.sendCommand(.killWheel(0))

        var frames = [TelemetryFrame]()
        client.run(onHello: { _ in }, onFrame: { frames.append($0) }, onEnd: { _ in })
        client.disconnect()
        server.stop()
        wait(for: [done], timeout: 10)

        XCTAssertFalse(frames.isEmpty)
        XCTAssertTrue(frames.contains { $0.failedWheels.contains(0) },
                      "no frame reported wheel 0 as failed")
        XCTAssertTrue(try XCTUnwrap(frames.last).failedWheels.contains(0),
                      "wheel 0 failure did not persist to the end of the run")
    }

    func testLoadScenarioCommand() throws {
        let (server, port, done) = startServer(config: shortNominal())
        let client = TelemetryClient(port: port)
        try client.connect()
        try client.sendCommand(.loadScenario("tumble"))

        var hellos = [TelemetryHello]()
        var frames = [TelemetryFrame]()
        client.run(onHello: { hellos.append($0) },
                   onFrame: { frames.append($0) },
                   onEnd: { _ in })
        client.disconnect()
        server.stop()
        wait(for: [done], timeout: 10)

        // Server re-sends hello for the new scenario, then streams it.
        XCTAssertEqual(hellos.count, 2)
        XCTAssertEqual(hellos[1].scenario, "tumble")
        XCTAssertFalse(frames.isEmpty)
    }

    func testSetSpeedCommand() throws {
        // A second subscriber gets a fresh run from t = 0.
        let (server, port, done) = startServer(config: shortNominal())

        let first = TelemetryClient(port: port)
        try first.connect()
        var firstFrames = 0
        first.run(onHello: { _ in }, onFrame: { _ in firstFrames += 1 }, onEnd: { _ in })
        first.disconnect()
        XCTAssertEqual(firstFrames, 200)

        let second = TelemetryClient(port: port)
        try second.connect()
        try second.sendCommand(.setSpeed(0))  // unthrottled (already the default here)
        var secondT0: Double?
        second.run(onHello: { _ in },
                   onFrame: { f in if secondT0 == nil { secondT0 = f.t } },
                   onEnd: { _ in })
        second.disconnect()
        server.stop()
        wait(for: [done], timeout: 10)

        XCTAssertEqual(secondT0 ?? -1, 0.01, accuracy: 1e-12)
    }

    func testUnknownScenarioThrows() {
        XCTAssertThrowsError(try TelemetryServer(scenario: "nope")) { error in
            XCTAssertEqual(error as? LinkError, LinkError.unknownScenario("nope"))
        }
    }

    func testConnectRefusedThrows() {
        // High, unlikely-to-be-bound port on loopback: nothing listens there.
        let client = TelemetryClient(port: 47893)
        XCTAssertThrowsError(try client.connect())
    }
}
