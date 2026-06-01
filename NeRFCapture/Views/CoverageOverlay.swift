//
//  CoverageOverlay.swift  (Otterly Spike 2 — guidance prototype, v1.1)
//
//  On-screen AR guidance HUD for handheld iPhone capture (NOT an immersive mesh — see the track memo).
//  Surfaces the locked quality signal (wider viewing-pose coverage) live:
//    - "quality coverage %" = green-fraction of object surface seen from wide-enough angles,
//    - a top-down ORBIT GRID = elevBins concentric rings (inner=low / mid / outer=high camera height) x
//      orbitBins azimuth sectors; a dot turns green once captured from that (direction x height) — so
//      HEIGHT is a visible dimension (v1.1 fix: capture no longer stops after one flat horizontal orbit),
//    - a NEXT-ANGLE arrow pointing at the widest missing wedge ("go shoot from here"),
//    - the timer Andy asked for (elapsed + auto-captured count) + the auto-capture toggle.
//
import SwiftUI

struct CoverageOverlay: View {
    @ObservedObject var viewModel: ARViewModel

    private var s: AppState { viewModel.appState }
    // computed (NOT stored): a private stored property would make CoverageOverlay's synthesized
    // memberwise init private, breaking CoverageOverlay(viewModel:) from ContentView (another file).
    private var azN: Int { CoverageMeter.orbitBins }
    private var elN: Int { CoverageMeter.elevBins }

    var body: some View {
        VStack {
            header
            Spacer()
            ring
                .frame(width: 210, height: 210)
            Text(heightHint)
                .font(.system(size: 11, weight: .medium, design: .rounded))
                .foregroundStyle(.white.opacity(0.85))
                .padding(.horizontal, 10).padding(.vertical, 4)
                .background(.black.opacity(0.4), in: Capsule())
                .padding(.bottom, 96)   // clear NeRFCapture's bottom End / Save-Frame controls
        }
        .padding(.top, 56)
        .allowsHitTesting(true)
    }

    private var header: some View {
        TimelineView(.periodic(from: .now, by: 1)) { _ in
            HStack(spacing: 14) {
                Label(elapsedString, systemImage: "timer")
                Label("\(s.autoCaptureCount)", systemImage: "camera.fill")
                Toggle("Auto", isOn: Binding(
                    get: { viewModel.appState.autoCapture },
                    set: { viewModel.appState.autoCapture = $0 }))
                    .toggleStyle(.switch)
                    .fixedSize()
            }
            .font(.system(size: 14, weight: .semibold, design: .rounded))
            .foregroundStyle(.white)
            .padding(.horizontal, 14).padding(.vertical, 8)
            .background(.black.opacity(0.45), in: Capsule())
        }
    }

    private var elapsedString: String {
        guard let start = s.sessionStart else { return "0:00" }
        let e = Int(max(0, Date().timeIntervalSince(start)))
        return String(format: "%d:%02d", e / 60, e % 60)
    }

    // top-down orbit grid: elN concentric rings (height bands) x azN sector dots + next-angle arrow + centre %
    private var ring: some View {
        Canvas { ctx, size in
            let c = CGPoint(x: size.width / 2, y: size.height / 2)
            let R = min(size.width, size.height) / 2 - 14
            let cells = s.capturedCells
            for e in 0..<elN {
                let rr = R * (0.45 + 0.235 * CGFloat(e))      // inner ring = low height, outer = high
                for a in 0..<azN {
                    let p = point(c: c, r: rr, azimuthDeg: Double(a) * 360.0 / Double(azN))
                    let idx = a * elN + e
                    let on = idx < cells.count && cells[idx]
                    let dot = Path(ellipseIn: CGRect(x: p.x - 3, y: p.y - 3, width: 6, height: 6))
                    ctx.fill(dot, with: .color(on ? .green : .white.opacity(0.22)))
                }
            }
            if let az = s.nextAngleAz {
                let tip = point(c: c, r: R, azimuthDeg: az)
                var line = Path(); line.move(to: c); line.addLine(to: tip)
                ctx.stroke(line, with: .color(.cyan), style: StrokeStyle(lineWidth: 3, lineCap: .round))
                ctx.fill(arrowHead(at: tip, from: c), with: .color(.cyan))
            }
            let pct = Int((s.coverageGreenFrac * 100).rounded())
            ctx.draw(Text("\(pct)%").font(.system(size: 30, weight: .bold, design: .rounded))
                        .foregroundColor(.white), at: c)
            ctx.draw(Text("quality").font(.system(size: 10, weight: .medium, design: .rounded))
                        .foregroundColor(.white.opacity(0.8)), at: CGPoint(x: c.x, y: c.y + 22))
            ctx.draw(Text("coverage").font(.system(size: 10, weight: .medium, design: .rounded))
                        .foregroundColor(.white.opacity(0.8)), at: CGPoint(x: c.x, y: c.y + 34))
        }
        .background(.black.opacity(0.30), in: Circle())
    }

    // guidance hint. When no under-covered azimuth wedge remains, communicate "broad enough" (the gate
    // naturally stops finding new cells then — so stopping reads as "covered", not as the v1 "stuck at 24").
    // Otherwise nudge toward the least-captured height band so HEIGHT variation is encouraged.
    private var heightHint: String {
        // v1.3 adaptive bands: during warmup, communicate that we're learning the user's actual handheld
        // elev envelope so all three rings span what they can physically reach (no unreachable outer).
        if s.calibratingHeight {
            return String(format: "校准高度范围 %.0f%% · 边拍边轻微变高度",
                          s.calibrationProgress * 100)
        }
        // nil nextAngleAz is overloaded (no wedge / not-computed-yet / diffuse), so require enough observed
        // surface before saying anything, and state only what's known ("no obvious under-covered azimuth").
        if s.coverageVoxels > 300 && s.nextAngleAz == nil {
            return "未见明显欠覆盖方位 · 换个高度补充，或结束采集"
        }
        let cells = s.capturedCells
        var counts = [Int](repeating: 0, count: elN)
        for e in 0..<elN {
            for a in 0..<azN { let i = a * elN + e; if i < cells.count && cells[i] { counts[e] += 1 } }
        }
        guard let minE = counts.indices.min(by: { counts[$0] < counts[$1] }) else { return "环: 内低·中平·外高" }
        let names = ["低", "平", "高"]
        let label = minE < names.count ? names[minE] : "\(minE)"
        return "环: 内低·中平·外高   →  多拍「\(label)」角度"
    }

    private func point(c: CGPoint, r: CGFloat, azimuthDeg: Double) -> CGPoint {
        let a = azimuthDeg * .pi / 180.0
        return CGPoint(x: c.x + r * CGFloat(cos(a)), y: c.y - r * CGFloat(sin(a)))
    }

    private func arrowHead(at tip: CGPoint, from origin: CGPoint) -> Path {
        let dx = tip.x - origin.x, dy = tip.y - origin.y
        let len = max(0.0001, sqrt(dx * dx + dy * dy))
        let ux = dx / len, uy = dy / len
        let nx = -uy, ny = ux
        let h: CGFloat = 12, w: CGFloat = 7
        let baseX = tip.x - ux * h, baseY = tip.y - uy * h
        var p = Path()
        p.move(to: tip)
        p.addLine(to: CGPoint(x: baseX + nx * w, y: baseY + ny * w))
        p.addLine(to: CGPoint(x: baseX - nx * w, y: baseY - ny * w))
        p.closeSubpath()
        return p
    }
}
