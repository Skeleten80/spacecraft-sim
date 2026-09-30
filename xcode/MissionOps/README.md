# MissionOps — live ADCS dashboard (Xcode, macOS)

A native macOS mission-ops screen for the SpacecraftSim attitude determination
and control simulator. SwiftUI + SceneKit + Charts, Apple Silicon native
(and Intel — it's just Swift, no architecture-specific code).

## Open it

On a Mac with Xcode 15 or later:

1. Open this folder in Xcode: **File → Open → `xcode/MissionOps`**
   (or run `xed xcode/MissionOps` from the repo root in Terminal).
2. Select the **MissionOps** scheme (My Mac destination).
3. Press **⌘R**.

No extra setup: the package depends on the `SpacecraftSim` library via a
relative path, and there are zero third-party dependencies.

## What you get

- **3D attitude view** — the solid spacecraft chasing its target attitude
  (orange wireframe ghost). Drag to orbit the camera.
- **Live telemetry** — pointing error, body rates, MEKF estimator error,
  gyro bias error, commanded torque, wheel saturation, RCS pulse count /
  burn time, sun-sensor TRACK/BLIND.
- **Strip charts** — pointing error and body-rate history.
- **Failure injection** — the fun part: hit **Kill** on any reaction wheel
  mid-sim and watch the controller reallocate torque to the remaining
  three wheels and keep converging.
- **Scenarios** — nominal, wheel-failure (scripted kill at t=45 s), tumble
  recovery, thruster-slew (120° slew on the RCS block: watch the limit
  cycle). Speed control 1×/10×/60×, Hold/Resume, Reset.

## Remote mode — dashboard against a sim on another machine

`RemoteViewModel` drives the same dashboard from a `TelemetryClient`
instead of a local `SimEngine`, so the dashboard becomes Mini 2
("mission control") to another machine's Mini 1 ("the spacecraft").

1. On the sim machine: `swift run spacecraft-cli serve nominal --port 9001`
2. In `Sources/MissionOps/DashboardView.swift`, change one line:
   ```swift
   @StateObject private var vm: RemoteViewModel = {
       let vm = RemoteViewModel()
       vm.host = "192.168.1.10"  // Mini 1's address on your LAN
       vm.port = 9001
       return vm
   }()
   ```
3. ⌘R. Scenario picker, speed control, Reset, and the wheel **Kill** buttons
   all work — they're sent as commands over the link (`loadScenario`,
   `setSpeed`, `killWheel`).

Note: this target is written against the macOS 14 SDK and hasn't been
compile-checked on Linux (SwiftUI/SceneKit/Charts are macOS-only) — if Xcode
reports a build error, it's a real bug; report it and it'll get fixed.

## Notes

- This target is macOS-only, so it lives in its own package: the core
  `SpacecraftSim` library and its tests stay Linux/CI-friendly.
- The sim engine (`SimEngine`) is the same code the CLI batch runner and
  the XCTest suite exercise — the dashboard just drives it interactively.
- Not flight software: same caveats as the main README (no RTOS, no
  rad-hardened hardware). It's a planet-side test bench with a nice screen.
