// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "SpacecraftSim",
    platforms: [.macOS(.v13)],
    products: [
        .library(name: "SpacecraftSim", targets: ["SpacecraftSim"]),
        .executable(name: "spacecraft-cli", targets: ["SpacecraftCLI"]),
    ],
    targets: [
        .target(name: "SpacecraftSim", path: "Sources/SpacecraftSim"),
        .executableTarget(name: "SpacecraftCLI",
                          dependencies: ["SpacecraftSim"],
                          path: "Sources/SpacecraftCLI"),
        .testTarget(name: "SpacecraftSimTests",
                    dependencies: ["SpacecraftSim"],
                    path: "Tests/SpacecraftSimTests"),
    ]
)
