import Foundation
import SpacecraftSim

func printUsage() {
    print("""
    usage: spacecraft-cli <scenario> [--csv PATH] [--duration SECONDS]
      scenarios: nominal | wheel-failure | tumble
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
if let last = result.samples.last {
    print("  final wheel loading  : \(fmt(last.wheelSaturation * 100, 1)) % of momentum capacity")
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
