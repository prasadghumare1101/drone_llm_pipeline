import { useEffect, useRef, useState } from 'react';
import { MoreHorizontal, Plus, Minus, Camera, Layers } from 'lucide-react';
import HudStrip from './HudStrip';
import type { Telemetry } from '../hooks/useROS';
import type { MissionWaypoint } from '../hooks/useLLM';

const C = {
  bg: '#EEF2F6', grid: '#DDE4EC', gridBold: '#CBD5E1',
  done: '#10B981', active: '#F59E0B', pending: '#0CA5E9',
  pinDone: '#10B981', pinActive: '#F59E0B', pinPending: '#0CA5E9', pinEnd: '#EF4444',
  drone: '#2563EB', text: '#1F2937', muted: '#6B7280',
};

/**
 * Offline mission map. Draws the compiled ENU waypoint plan, the flown path and
 * the live vehicle marker onto a canvas - no tile server, no network, no deps.
 * Rendering happens entirely in canvas so telemetry ticks never re-render React.
 */
export default function MissionMap({ t, waypoints }: { t: Telemetry; waypoints: MissionWaypoint[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const trailRef = useRef<Array<[number, number]>>([]);
  const [zoom, setZoom] = useState(55);
  const [tilt, setTilt] = useState(50);

  // record the flown trail (bounded, so memory can never grow unbounded)
  useEffect(() => {
    const tr = trailRef.current;
    const last = tr[tr.length - 1];
    if (!last || Math.hypot(last[0] - t.x, last[1] - t.y) > 0.75) {
      tr.push([t.x, t.y]);
      if (tr.length > 1500) tr.shift();
    }
  }, [t.x, t.y]);

  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const draw = () => {
      const dpr = window.devicePixelRatio || 1;
      const w = wrap.clientWidth;
      const h = wrap.clientHeight;
      if (!w || !h) return;
      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr; canvas.height = h * dpr;
        canvas.style.width = `${w}px`; canvas.style.height = `${h}px`;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);

      // ---- world -> screen ------------------------------------------------
      const pts = waypoints.map((p) => [p.x, p.y] as [number, number]);
      pts.push([t.x, t.y], [0, 0]);
      const xs = pts.map((p) => p[0]); const ys = pts.map((p) => p[1]);
      const spanX = Math.max(20, Math.max(...xs) - Math.min(...xs));
      const spanY = Math.max(20, Math.max(...ys) - Math.min(...ys));
      const cx = (Math.max(...xs) + Math.min(...xs)) / 2;
      const cy = (Math.max(...ys) + Math.min(...ys)) / 2;
      const zoomK = 0.45 + (zoom / 100) * 1.3;
      const scale = Math.min(w / (spanX * 1.5), h / (spanY * 1.5)) * zoomK;
      const sx = (mx: number) => w / 2 + (mx - cx) * scale;
      const sy = (my: number) => h / 2 - (my - cy) * scale;   // north = up

      // ---- grid -----------------------------------------------------------
      const stepM = spanX > 400 ? 100 : spanX > 150 ? 50 : 10;
      ctx.lineWidth = 1;
      ctx.font = '10px Inter, sans-serif';
      for (let gx = Math.floor((cx - spanX) / stepM) * stepM; gx <= cx + spanX; gx += stepM) {
        const px = sx(gx);
        if (px < 0 || px > w) continue;
        ctx.strokeStyle = gx === 0 ? C.gridBold : C.grid;
        ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
      }
      for (let gy = Math.floor((cy - spanY) / stepM) * stepM; gy <= cy + spanY; gy += stepM) {
        const py = sy(gy);
        if (py < 0 || py > h) continue;
        ctx.strokeStyle = gy === 0 ? C.gridBold : C.grid;
        ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
      }

      // ---- planned route --------------------------------------------------
      const activeIdx = waypoints.findIndex((p) => p.active);
      ctx.lineWidth = 3.5; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      for (let i = 0; i < waypoints.length - 1; i++) {
        const a = waypoints[i]; const b = waypoints[i + 1];
        ctx.strokeStyle = activeIdx < 0 ? C.pending
          : i < activeIdx - 1 ? C.done : i === activeIdx - 1 ? C.active : C.pending;
        ctx.beginPath(); ctx.moveTo(sx(a.x), sy(a.y)); ctx.lineTo(sx(b.x), sy(b.y)); ctx.stroke();
      }

      // ---- flown trail ----------------------------------------------------
      const trail = trailRef.current;
      if (trail.length > 1) {
        ctx.strokeStyle = 'rgba(37,99,235,0.55)';
        ctx.lineWidth = 2; ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.moveTo(sx(trail[0][0]), sy(trail[0][1]));
        for (const [px, py] of trail) ctx.lineTo(sx(px), sy(py));
        ctx.stroke(); ctx.setLineDash([]);
      }

      // ---- home -----------------------------------------------------------
      ctx.fillStyle = C.muted;
      ctx.beginPath(); ctx.arc(sx(0), sy(0), 4, 0, Math.PI * 2); ctx.fill();
      ctx.fillText('HOME', sx(0) + 7, sy(0) + 3);

      // ---- waypoint pins --------------------------------------------------
      waypoints.forEach((p, i) => {
        const px = sx(p.x); const py = sy(p.y);
        const isLast = i === waypoints.length - 1;
        const col = p.active ? C.pinActive
          : isLast ? C.pinEnd
            : activeIdx >= 0 && i < activeIdx ? C.pinDone : C.pinPending;
        // teardrop
        ctx.fillStyle = col;
        ctx.beginPath();
        ctx.arc(px, py - 9, 7.5, Math.PI, 0);
        ctx.lineTo(px, py + 2);
        ctx.closePath(); ctx.fill();
        ctx.fillStyle = '#fff';
        ctx.beginPath(); ctx.arc(px, py - 9, 3.1, 0, Math.PI * 2); ctx.fill();
        if (p.active) {
          ctx.strokeStyle = col; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(px, py - 9, 13, 0, Math.PI * 2); ctx.stroke();
        }
        ctx.fillStyle = C.text; ctx.font = 'bold 10px Inter, sans-serif';
        ctx.fillText(String(p.idx), px + 10, py - 6);
      });

      // ---- vehicle --------------------------------------------------------
      const dx = sx(t.x); const dy = sy(t.y);
      ctx.save();
      ctx.translate(dx, dy);
      ctx.rotate((t.heading * Math.PI) / 180);
      ctx.fillStyle = C.drone;
      ctx.beginPath(); ctx.moveTo(0, -11); ctx.lineTo(8, 9); ctx.lineTo(0, 4); ctx.lineTo(-8, 9);
      ctx.closePath(); ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.restore();

      // label bubble
      const label = 'AM-4-003';
      ctx.font = 'bold 11px Inter, sans-serif';
      const tw = ctx.measureText(label).width + 14;
      ctx.fillStyle = 'rgba(31,41,55,0.92)';
      ctx.beginPath();
      // rounded rect (kept manual for wide browser support)
      const bx = dx - tw / 2; const by = dy - 42; const bh = 22; const r = 5;
      ctx.moveTo(bx + r, by); ctx.lineTo(bx + tw - r, by);
      ctx.quadraticCurveTo(bx + tw, by, bx + tw, by + r); ctx.lineTo(bx + tw, by + bh - r);
      ctx.quadraticCurveTo(bx + tw, by + bh, bx + tw - r, by + bh); ctx.lineTo(bx + r, by + bh);
      ctx.quadraticCurveTo(bx, by + bh, bx, by + bh - r); ctx.lineTo(bx, by + r);
      ctx.quadraticCurveTo(bx, by, bx + r, by); ctx.closePath(); ctx.fill();
      ctx.fillStyle = '#fff'; ctx.textAlign = 'center';
      ctx.fillText(label, dx, by + 15);
      ctx.textAlign = 'left';

      // scale bar
      ctx.fillStyle = C.muted; ctx.font = '10px Inter, sans-serif';
      ctx.fillText(`${stepM} m grid`, 10, h - 10);
    };

    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [t, waypoints, zoom]);

  return (
    <div className="bg-panelWhite rounded-lg shadow-sm border border-borderGray flex flex-col h-full overflow-hidden">
      <div className="flex items-center justify-between px-4 py-2.5 border-b border-borderGray shrink-0">
        <span className="text-sm font-semibold tracking-wide">MISSION MAP</span>
        <MoreHorizontal size={16} className="text-textMuted" />
      </div>

      <div className="border-b border-borderGray shrink-0"><HudStrip t={t} /></div>

      <div ref={wrapRef} className="flex-1 relative min-h-0">
        <canvas ref={canvasRef} className="absolute inset-0 w-full h-full" />

        {waypoints.length === 0 && (
          <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
            <span className="text-textMuted text-xs bg-panelWhite/80 px-3 py-1.5 rounded">
              No mission loaded — send a command below
            </span>
          </div>
        )}

        <button className="absolute right-3 top-3 bg-panelWhite rounded-md p-2 shadow border border-borderGray">
          <Layers size={18} className="text-textMain" />
        </button>

        {/* left zoom rail */}
        <div className="absolute left-3 top-1/2 -translate-y-1/2 flex flex-col items-center gap-2
                        bg-panelWhite rounded-lg px-1.5 py-2.5 shadow border border-borderGray">
          <Plus size={16} className="text-textMain" />
          <input type="range" min={0} max={100} value={zoom}
                 onChange={(e) => setZoom(+e.target.value)}
                 className="aero-vslider h-24" />
          <Minus size={16} className="text-textMain" />
        </div>

        {/* right camera/tilt rail */}
        <div className="absolute right-3 top-1/2 -translate-y-1/2 flex flex-col items-center gap-2
                        bg-panelWhite rounded-lg px-1.5 py-2.5 shadow border border-borderGray">
          <Camera size={16} className="text-textMain" />
          <input type="range" min={0} max={100} value={tilt}
                 onChange={(e) => setTilt(+e.target.value)}
                 className="aero-vslider h-24" />
        </div>

        <span className="absolute right-3 bottom-2 text-xxs text-textMuted font-medium tracking-wide">
          OPTIMIZED NO-LAG FEED
        </span>
      </div>

      <div className="border-t border-borderGray shrink-0"><HudStrip t={t} compact /></div>
    </div>
  );
}
