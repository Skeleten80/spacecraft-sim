import SwiftUI

/// Small labeled telemetry readout card.
struct StatCard: View {
    var title: String
    var value: String
    var color: Color = .primary

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(title)
                .font(.caption)
                .foregroundStyle(.secondary)
                .tracking(1)
            Text(value)
                .font(.system(.title2, design: .monospaced))
                .foregroundStyle(color)
                .lineLimit(1)
                .minimumScaleFactor(0.7)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(10)
        .background(.quaternary.opacity(0.45), in: RoundedRectangle(cornerRadius: 8))
    }
}

/// One reaction wheel: momentum bar, value, and a Kill button for fault injection.
struct WheelRow: View {
    var index: Int
    var momentum: Double
    var maxMomentum: Double
    var failed: Bool
    var onKill: () -> Void

    var body: some View {
        HStack(spacing: 8) {
            Text("W\(index)")
                .font(.system(.body, design: .monospaced))
                .frame(width: 30, alignment: .leading)
            GeometryReader { geo in
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: 3)
                        .fill(.quaternary)
                    RoundedRectangle(cornerRadius: 3)
                        .fill(failed ? .red : .cyan)
                        .frame(width: geo.size.width * min(1, abs(momentum) / maxMomentum))
                }
            }
            .frame(height: 10)
            Text(String(format: "%+.2f", momentum))
                .font(.system(.caption, design: .monospaced))
                .frame(width: 62, alignment: .trailing)
                .foregroundStyle(.secondary)
            if failed {
                Text("FAILED")
                    .font(.caption)
                    .bold()
                    .foregroundStyle(.red)
                    .frame(width: 64)
            } else {
                Button("Kill", action: onKill)
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                    .frame(width: 64)
            }
        }
    }
}

/// Section header for the telemetry column.
struct PanelSection: View {
    var title: String

    var body: some View {
        Text(title)
            .font(.caption)
            .foregroundStyle(.secondary)
            .tracking(1)
            .padding(.top, 4)
    }
}
