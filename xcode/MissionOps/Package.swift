// swift-tools-version: 5.9
import PackageDescription

// MissionOps: the live mission-ops dashboard for the SpacecraftSim ADCS
// simulator. macOS-only (SwiftUI + SceneKit + Charts) — open this folder
// in Xcode on a Mac and run the MissionOps scheme.
let package = Package(
    name: "MissionOps",
    platforms: [.macOS(.v14)],
    products: [
        .executable(name: "MissionOps", targets: ["MissionOps"]),
    ],
    dependencies: [
        .package(path: "../.."),
    ],
    targets: [
        .executableTarget(
            name: "MissionOps",
            dependencies: [
                .product(name: "SpacecraftSim", package: "SpacecraftSim"),
            ],
            path: "Sources/MissionOps"),
    ]
)
