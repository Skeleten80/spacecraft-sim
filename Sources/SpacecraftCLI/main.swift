import Foundation
import SpacecraftSim

func printUsage() {
    print("""
    usage:
      spacecraft-cli <scenario> [--csv PATH] [--duration SECONDS]
      spacecraft-cli serve <scenario> [--port PORT] [--speed SIM_SEC_PER_REAL_SEC]

      scenarios: nominal | wheel-failure | tumble | thruster-slew

    serve: stream live telemetry (NDJSON over TCP) for the multi-machine
    mission setup. --speed 1 is real-time; --speed 0 runs unthrottled.
    """)
}

func fmt(_ v: Double, _ d: Int = 2) -> String {
    String(format: "%.\(d)f", v)
}

var args = CommandLine.arguments.dropFirst()
guard let name = args.first, !name.hasPrefix("-") else {
    printUsage()
    exit(1)
}
args = args.dropFirst()

// Mini 1 mode: stream the sim over TCP for remote dashboards / analysis.
if name == "serve" {
    guard let scenarioName = args.first, !scenarioName.hasPrefix("-") else {
        print("serve needs a scenario")
        printUsage()
        exit(1)
    }
    var port: UInt16 = 9001
    var speed = 1.0
    var duration: Double?
    let rest = Array(args.dropFirst())
    var k = rest.startIndex
    while k < rest.endIndex {
        switch rest[k] {
        case "--port":
            rest.formIndex(after: &k)
            if k < rest.endIndex, let p = UInt16(rest[k]) { port = p }
        case "--speed":
            rest.formIndex(after: &k)
            if k < rest.endIndex, let s = Double(rest[k]) { speed = s }
        case "--duration":
            rest.formIndex(after: &k)
            if k < rest.endIndex, let d = Double(rest[k]) { duration = d }
        default:
            break
        }
        rest.formIndex(after: &k)
    }
    guard var config = builtinScenario(String(scenarioName)) else {
        print("unknown scenario: \(scenarioName)")
        exit(1)
    }
    if let d = duration { config.duration = d }
    do {
        let server = TelemetryServer(config: config, port: port, speed: speed)
        print("serving '\(config.name)' on port \(port) " +
              "(speed: \(speed == 0 ? "unthrottled" : "\(speed)x")) …")
        try server.serve()
    } catch {
        print("serve failed: \(error)")
        exit(1)
    }
    exit(0)
}

var csvPath: String?
var duration: Double?
var i = args.startIndex
while i < args.endIndex {
    switch args[i] {
    case "--csv":
        args.formIndex(after: &i)
        if i < args.endIndex { csvPath = String(args[i]) }
    case "--duration":
        args.formIndex(after: &i)
        if i < args.endIndex { duration = Double(args[i]) }
    default:
        break
    }
    args.formIndex(after: &i)
}

guard var config = builtinScenario(name) else {
    print("unknown scenario: \(name)")
    printUsage()
    exit(1)
}
if let d = duration { config.duration = d }

print("running scenario '\(config.name)' (\(Int(config.duration)) s @ \(1 / config.dt) Hz, seed \(config.seed)) ...")
let result = runScenario(config)

print("")
print("  final pointing error : \(fmt(result.finalPointErrDeg, 3)) deg")
if let s = result.settleTime {
    print("  settled (<0.5 deg) at : \(fmt(s, 1)) s")
} else {
    print("  settled (<0.5 deg) at : never (within \(Int(config.duration)) s)")
}
print("  peak body rate       : \(fmt(result.maxOmegaDegS)) deg/s")
print("  final bias est. error: \(fmt(result.finalBiasErrDegS, 4)) deg/s")
if config.actuatorMode == .wheels,
   let last = result.samples.last {
    print("  final wheel loading  : \(fmt(last.wheelSaturation * 100, 1)) % of momentum capacity")
}
if config.actuatorMode == .thrusters {
    print("  thruster pulses fired: \(result.thrusterFirings)")
    print("  total burn time      : \(fmt(result.thrusterBurnTime, 2)) s")
}

if let path = csvPath {
    do {
        try writeCSV(result, to: path)
        print("  telemetry written to : \(path) (\(result.samples.count) samples)")
    } catch {
        print("  error writing CSV: \(error)")
        exit(1)
    }
}
