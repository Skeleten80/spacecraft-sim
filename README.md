# spacecraft-sim

A planet-side spacecraft attitude determination & control (ADCS) simulator in
Swift. Real control theory, real estimation math — running against a simulated
spacecraft instead of flight hardware. Built for testing and learning.

## What it is

A closed-loop simulation of everything a spacecraft flight computer does to
point itself:

| Module | What it does |
|---|---|
| `Math.swift` | Vec3, Mat3, quaternions, small dense-matrix type |
| `Dynamics.swift` | Rigid-body attitude dynamics (Euler's equation) + 4-wheel reaction-wheel assembly (torque/momentum saturation, fault injection) + 12-thruster on/off RCS block with direction-group allocation and minimum-impulse-bit quantization |
| `Sensors.swift` | Rate gyro (bias random walk + angle random walk), star tracker (noisy absolute attitude), sun sensor (FOV-gated noisy sun line) |
| `Estimator.swift` | Multiplicative Extended Kalman Filter — the standard spacecraft attitude filter; fuses gyro + star tracker + sun-vector measurements |
| `Controller.swift` | Quaternion-feedback PD + integral controller with anti-windup |
| `Simulation.swift` | Fixed-step closed-loop runner, telemetry logging, CSV export |

The controller only ever sees the **estimated** state (like real flight
software) — never the truth. The estimator only sees noisy sensors.

## Honest caveats

- This is **not flight software**. Real missions fly on radiation-hardened
  processors under a real-time OS with years of verification. Nothing here
  is certified for anything.
- macOS is not an RTOS and this sim makes no real-time claims; `dt` is
  simulated time.
- Sensor noise values are representative of MEMS-grade hardware, not any
  specific part. Tune them in `Simulation.swift` / `Sensors.swift`.
- The thruster model is on/off valves with a 20 ms minimum impulse bit and
  sigma-delta modulation — no PWPF modulator, no throttling. Net thrust force
  is ignored (the sim has no translation state); only torque enters the
  dynamics.
- The sun sits at a fixed inertial direction; there is no orbit model, so no
  eclipse. The single sun-sensor head goes blind outside its 70° FOV instead.
- The math (quaternion kinematics, MEKF, quaternion-feedback control) is the
  real formulation used in practice — that's the part worth learning.

## Run it

```bash
cd spacecraft-sim
swift run spacecraft-cli nominal --csv nominal.csv
swift run spacecraft-cli wheel-failure --csv failure.csv
swift run spacecraft-cli tumble --csv tumble.csv
swift run spacecraft-cli thruster-slew --csv rcs.csv
```

Scenarios:

- **nominal** — 30° off-point with a slow drift; controller slews and holds.
- **wheel-failure** — wheel 0 dies at t=45 s; the remaining three wheels
  (which still span all of R³) pick up the load.
- **tumble** — starts tumbling at ~10°/s; detumbles, then points.
- **thruster-slew** — 120° slew on the 12-thruster RCS block instead of
  wheels. On/off valves + 20 ms impulse bit give a coarse slew and a small
  limit cycle around the target (~0.2°), not smooth convergence.

Each run prints final pointing error, settle time, peak rate, bias-estimation
error, and wheel momentum loading (or thruster pulse count + burn time in
`thruster-slew`). `--csv` dumps per-sample telemetry
(time, pointing error, estimator error, rates, wheel saturation, torque,
thruster firings, thruster burn time).

## Xcode mission-ops dashboard (macOS)

The same simulator with a live mission-ops screen — 3D attitude view
(spacecraft chasing its target ghost), telemetry cards, strip charts,
scenario/speed controls, and per-wheel **Kill** buttons for fault injection.

```
xcode/MissionOps/      # separate SwiftPM package (macOS-only: SwiftUI + SceneKit + Charts)
```

On a Mac with Xcode 15+: **File → Open → `xcode/MissionOps`**, select the
MissionOps scheme, **⌘R**. Runs natively on Apple Silicon (and Intel — no
architecture-specific code). See `xcode/MissionOps/README.md`.

It lives in its own package so the core library and `swift test` stay
Linux-friendly. The dashboard drives `SimEngine` — the exact code the CLI
and the test suite exercise — interactively.

## Multi-machine mission setup

The sim can run as a network service, so the workload splits the way real
missions do:

```
Mini 1 "the spacecraft"   Mini 2 "mission control"   Mini 3 "the mule"
┌─────────────────────┐   ┌──────────────────────┐   ┌────────────────────┐
│ spacecraft-cli serve │──▶│ MissionOps dashboard │   │ Monte Carlo farm,  │
│ steps SimEngine,     │   │ or any telemetry     │   │ CI, ML on archived │
│ streams NDJSON       │◀──│ client (commands)    │   │ telemetry          │
│ over TCP             │   └──────────────────────┘   └────────────────────┘
└─────────────────────┘
```

**Mini 1** — serve a scenario over TCP (default port 9001):

```bash
swift run spacecraft-cli serve nominal --port 9001 --speed 1
```

`--speed` is sim-seconds per real second (`1` = real-time, `0` = unthrottled);
`--duration` shortens the run. The server accepts subscribers in a loop —
each new connection replays the scenario from t = 0.

**Mini 2** — point the MissionOps dashboard at it (see
`xcode/MissionOps/README.md` for the one-line switch), or subscribe with
anything that reads lines off a socket. The Python client in `tools/` is the
simplest example:

```bash
python3 tools/spacecraft_link.py --host 192.168.1.10 --port 9001 --kill-wheel 1
```

**Wire protocol** (UTF-8, one JSON object per line; see `TelemetryLink.swift`):

```
server → client: {"type":"hello","protocolVersion":1,"scenario":"nominal","dt":0.01,"duration":300}
server → client: {"type":"frame","t":0.01,"estimatedAttitude":[x,y,z,w],"targetAttitude":[…],
                  "omega":[x,y,z],"pointErrDeg":…,"estErrDeg":…,"omegaDegS":…,
                  "biasErrDegS":…,"wheelSaturation":…,"torqueNorm":…,
                  "wheelMomenta":[…],"wheelMaxMomentum":…,"failedWheels":[],
                  "thrusterFirings":0,"thrusterBurnTime":0.0,"sunValid":true}
server → client: {"type":"end","reason":"complete"}
client → server: {"type":"command","cmd":"killWheel","index":0}
client → server: {"type":"command","cmd":"loadScenario","scenario":"tumble"}
client → server: {"type":"command","cmd":"setSpeed","value":10}
```

Frames stream at the full 100 Hz step rate (the batch CLI/CSV decimates
logging to 0.5 s; the link does not). Sockets are raw POSIX with no
dependencies, so the core stays Linux-friendly. One subscriber per server;
restart the client to get a fresh run.

## Tests

```bash
swift test
```

Covers quaternion/matrix math, wheel-geometry fault tolerance (any 3 of 4
wheels span R³), thruster allocation (exact torque tracking, minimum-impulse
quantization, saturation, failed-thruster redundancy), sun-sensor FOV gating
and noise, MEKF convergence on synthetic data (quaternion + vector updates),
closed-loop convergence for all four scenarios, the network telemetry link
(server/client round-trip over loopback TCP, killWheel/loadScenario/setSpeed
commands, multi-session serving), and bit-for-bit determinism
under a fixed seed.

## Try next

- Add a magnetometer and fuse a third vector measurement in the MEKF.
- Add an orbit model so the sun sensor sees real eclipses, and inject
  star-tracker dropouts to see how long gyro-only propagation holds pointing.
- Swap the PD controller for an LQR design and compare settle times.
- Model wheel friction / stiction and watch the integral term earn its keep.
- Model thruster plume impingement / misalignment torques.
