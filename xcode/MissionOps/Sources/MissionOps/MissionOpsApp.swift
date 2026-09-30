import SwiftUI

@main
struct MissionOpsApp: App {
    var body: some Scene {
        WindowGroup {
            DashboardView()
        }
        .windowResizability(.contentMinSize)
    }
}
