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
| `Dynamics.swift` | Rigid-body attitude dynamics (Euler's equation) + 4-wheel reaction-wheel assembly with torque/momentum saturation and fault injection |
| `Sensors.swift` | Rate gyro (bias random walk + angle random walk) and star tracker (noisy absolute attitude) |
| `Estimator.swift` | Multiplicative Extended Kalman Filter — the standard spacecraft attitude filter |
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
- The math (quaternion kinematics, MEKF, quaternion-feedback control) is the
  real formulation used in practice — that's the part worth learning.

## Run it

```bash
cd spacecraft-sim
swift run spacecraft-cli nominal --csv nominal.csv
swift run spacecraft-cli wheel-failure --csv failure.csv
swift run spacecraft-cli tumble --csv tumble.csv
```

Scenarios:

- **nominal** — 30° off-point with a slow drift; controller slews and holds.
- **wheel-failure** — wheel 0 dies at t=45 s; the remaining three wheels
  (which still span all of R³) pick up the load.
- **tumble** — starts tumbling at ~10°/s; detumbles, then points.

Each run prints final pointing error, settle time, peak rate, bias-estimation
error, and wheel momentum loading. `--csv` dumps per-sample telemetry
(time, pointing error, estimator error, rates, wheel saturation, torque).

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

## Tests

```bash
swift test
```

Covers quaternion/matrix math, wheel-geometry fault tolerance (any 3 of 4
wheels span R³), MEKF convergence on synthetic data, closed-loop convergence
for all three scenarios, and bit-for-bit determinism under a fixed seed.

## Try next

- Add a magnetometer + sun sensor and fuse a third measurement in the MEKF.
- Swap the PD controller for an LQR design and compare settle times.
- Model wheel friction / stiction and watch the integral term earn its keep.
- Inject star-tracker dropouts (eclipse) and see how long the gyro-only
  propagation holds pointing.
