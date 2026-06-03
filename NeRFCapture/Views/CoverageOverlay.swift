//
//  CoverageOverlay.swift  (Otterly — guidance prototype, v2)
//
//  On-screen AR guidance HUD for handheld iPhone capture (NOT an immersive mesh — see the track memo).
//  v2 (2026-06-02): the device-orientation "center-disk" (orbit-sector x elevation-band dots) is gone.
//  The HUD now surfaces the validated REGION quality signal directly:
//    - "quality coverage %" = green-fraction of admitted (stably-seen) surface that has wide-enough
//      viewing-angle coverage (parallax),
//    - a NEXT-ANGLE arrow pointing at the widest under-covered azimuth wedge ("go shoot more angles here"),
//    - a "稳住" prompt when the phone is moving too fast for a clean (sharp) capture,
//    - the timer + auto-captured count + the auto-capture toggle.
//
import SwiftUI

struct CoverageOverlay: View {
    @ObservedObject var viewModel: ARViewModel

    private var s: AppState { viewModel.appState }

    var body: some View {
        VStack {
            header
            Spacer()
            ring
                .frame(width: 210, height: 210)
            Text(hint)
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

    // compass: quality % in the centre + a next-angle arrow at the widest under-covered azimuth wedge.
    private var ring: some View {
        Canvas { ctx, size in
            let c = CGPoint(x: size.width / 2, y: size.height / 2)
            let R = min(size.width, size.height) / 2 - 14
            if let az = s.nextAngleAz {
                let tip = point(c: c, r: R, azimuthDeg: az)
                var line = Path(); line.move(to: c); line.addLine(to: tip)
                ctx.stroke(line, with: .color(.cyan), style: StrokeStyle(lineWidth: 3, lineCap: .round))
                ctx.fill(arrowHead(at: tip, from: c), with: .color(.cyan))
            }
            let pct = Int((s.coverageGreenFrac * 100).rounded())
            ctx.draw(Text("\(pct)%").font(.system(size: 30, weight: .bold, design: .rounded))
                        .foregroundColor(s.holdSteady ? .yellow : .white), at: c)
            ctx.draw(Text("quality").font(.system(size: 10, weight: .medium, design: .rounded))
                        .foregroundColor(.white.opacity(0.8)), at: CGPoint(x: c.x, y: c.y + 22))
            ctx.draw(Text("coverage").font(.system(size: 10, weight: .medium, design: .rounded))
                        .foregroundColor(.white.opacity(0.8)), at: CGPoint(x: c.x, y: c.y + 34))
        }
        .background(.black.opacity(0.30), in: Circle())
    }

    // region-quality guidance line.
    private var hint: String {
        if s.holdSteady { return "稳住 · 别动，等它拍这一下" }
        if s.coverageVoxels > 300 && s.nextAngleAz == nil {
            return "角度已充分 · 可换个区域，或结束采集"
        }
        if s.nextAngleAz != nil {
            return "绕到箭头方向 · 给这块多拍几个角度"
        }
        return "慢慢绕着拍 · 每块从多个角度看一遍"
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
