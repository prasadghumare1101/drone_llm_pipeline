import { useEffect, useRef, useState } from 'react';
import {
  MoreHorizontal, Plus, Minus, Camera, Layers, PenLine, Undo2, Trash2, Repeat, ListStart, ListEnd, Play,
} from 'lucide-react';
import HudStrip from './HudStrip';
import type { Telemetry } from '../hooks/useROS';
import { API_BASE, type MissionWaypoint } from '../hooks/useLLM';

const C = {
  bg: '#EEF2F6', grid: '#DDE4EC', gridBold: '#CBD5E1',
  done: '#10B981', active: '#F59E0B', pending: '#0CA5E9',
  pinDone: '#10B981', pinActive: '#F59E0B', pinPending: '#0CA5E9', pinEnd: '#EF4444',
  drone: '#2563EB', text: '#1F2937', muted: '#6B7280',
};

/** Aerial image of the simulated world, placed in the local ENU frame (metres). */
interface Basemap {
  world: string;
  url: string;
  bounds: { west: number; east: number; north: number; south: number };
}

/** Map drawing hooks into the interactive session (absent = drawing disabled). */
export interface MapPlanner {
  executing: boolean;             // a step is flying: drawn points are added to it
  flyPath: (points: [number, number][], circuit: boolean, alt: number | null) => void;
  addWaypoints: (points: [number, number][], where: 'next' | 'end') => void;
}

/**
 * Offline mission map. Draws the compiled ENU waypoint plan, the flown path and
 * the live vehicle marker onto a canvas - no tile server, no network, no deps.
 * Rendering happens entirely in canvas so telemetry ticks never re-render React.
 * When the selected Gazebo world has a calibrated aerial texture (see
 * backend_services/basemaps.json) it is drawn underneath; the Layers button
 * toggles it. Only the visible part of the image is drawn each frame.
 */
export default function MissionMap({ t, waypoints, planner }: {
  t: Telemetry; waypoints: MissionWaypoint[]; planner?: MapPlanner;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const trailRef = useRef<Array<[number, number]>>([]);
  const [zoom, setZoom] = useState(55);
  // drawing: click the map to place waypoints (x = East, y = North, metres)
  const [drawing, setDrawing] = useState(false);
  const [draft, setDraft] = useState<[number, number][]>([]);
  const [altText, setAltText] = useState('');
  const viewRef = useRef({ cx: 0, cy: 0, scale: 1, w: 0, h: 0 });
  const [tilt, setTilt] = useState(50);
  const [basemap, setBasemap] = useState<Basemap | null>(null);
  const [aerial, setAerial] = useState(true);
  const [imgReady, setImgReady] = useState(0);      // bumps when a new image has loaded
  const imgRef = useRef<HTMLImageElement | null>(null);

  // which world is loaded can change with each START SIM, so re-check slowly
  useEffect(() => {
    let alive = true;
    const load = async () => {
      try {
        const d = await (await fetch(`${API_BASE}/api/sim/basemap`)).json();
        if (!alive) return;
        setBasemap((cur) => (d.available
          ? (cur?.url === d.url ? cur : { world: d.world, url: d.url, bounds: d.bounds })
          : null));
      } catch { /* control API offline: keep whatever we had */ }
    };
    load();
    const t = setInterval(load, 15000);
    return () => { alive = false; clearInterval(t); };
  }, []);

  useEffect(() => {
    imgRef.current = null;
    if (!basemap) return;
    const img = new Image();
    img.onload = () => { imgRef.current = img; setImgReady((n) => n + 1); };
    img.src = `${API_BASE}${basemap.url}`;
    return () => { img.onload = null; };
  }, [basemap]);

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
      pts.push([t.x, t.y], [0, 0], ...draft);
      const xs = pts.map((p) => p[0]); const ys = pts.map((p) => p[1]);
      // with an aerial map, never zoom in tighter than 120 m by default: the
      // imagery is ~1 m/px, so a 20 m window is just a blur
      const minSpan = aerial && basemap ? 120 : 20;
      const spanX = Math.max(minSpan, Math.max(...xs) - Math.min(...xs));
      const spanY = Math.max(minSpan, Math.max(...ys) - Math.min(...ys));
      const cx = (Math.max(...xs) + Math.min(...xs)) / 2;
      const cy = (Math.max(...ys) + Math.min(...ys)) / 2;
      // exponential: slider 0 = ~8x wider than the fit, 100 = ~5x closer
      const zoomK = 2 ** ((zoom - 55) / 18);
      const scale = Math.min(w / (spanX * 1.5), h / (spanY * 1.5)) * zoomK;
      const sx = (mx: number) => w / 2 + (mx - cx) * scale;
      const sy = (my: number) => h / 2 - (my - cy) * scale;   // north = up
      viewRef.current = { cx, cy, scale, w, h };                // for click -> metres

      // ---- aerial basemap (only the visible window of the image) ---------
      const img = imgRef.current;
      const showAerial = aerial && basemap && img;
      if (showAerial) {
        const b = basemap.bounds;
        const pxPerMx = img.naturalWidth / (b.east - b.west);
        const pxPerMy = img.naturalHeight / (b.north - b.south);
        const x0 = Math.max(b.west, cx - w / 2 / scale);
        const x1 = Math.min(b.east, cx + w / 2 / scale);
        const y0 = Math.max(b.south, cy - h / 2 / scale);
        const y1 = Math.min(b.north, cy + h / 2 / scale);
        if (x1 > x0 && y1 > y0) {
          ctx.imageSmoothingQuality = 'high';
          ctx.drawImage(img,
            (x0 - b.west) * pxPerMx, (b.north - y1) * pxPerMy,
            (x1 - x0) * pxPerMx, (y1 - y0) * pxPerMy,
            sx(x0), sy(y1), (x1 - x0) * scale, (y1 - y0) * scale);
        }
      }

      // ---- grid -----------------------------------------------------------
      const viewW = w / scale;                           // metres across the canvas
      const viewH = h / scale;
      const stepM = viewW > 2000 ? 500 : viewW > 800 ? 100 : viewW > 200 ? 50 : 10;
      ctx.lineWidth = 1;
      ctx.globalAlpha = showAerial ? 0.35 : 1;
      ctx.font = '10px Inter, sans-serif';
      for (let gx = Math.floor((cx - viewW / 2) / stepM) * stepM; gx <= cx + viewW / 2; gx += stepM) {
        const px = sx(gx);
        if (px < 0 || px > w) continue;
        ctx.strokeStyle = gx === 0 ? C.gridBold : C.grid;
        ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
      }
      for (let gy = Math.floor((cy - viewH / 2) / stepM) * stepM; gy <= cy + viewH / 2; gy += stepM) {
        const py = sy(gy);
        if (py < 0 || py > h) continue;
        ctx.strokeStyle = gy === 0 ? C.gridBold : C.grid;
        ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
      }

      ctx.globalAlpha = 1;

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

      // ---- drawn (not yet sent) waypoints ---------------------------------
      if (draft.length) {
        ctx.strokeStyle = '#D946EF'; ctx.lineWidth = 2.5; ctx.setLineDash([7, 5]);
        ctx.beginPath(); ctx.moveTo(sx(t.x), sy(t.y));
        for (const [px, py] of draft) ctx.lineTo(sx(px), sy(py));
        ctx.stroke(); ctx.setLineDash([]);
        draft.forEach(([px, py], i) => {
          ctx.fillStyle = '#D946EF';
          ctx.beginPath(); ctx.arc(sx(px), sy(py), 8, 0, Math.PI * 2); ctx.fill();
          ctx.fillStyle = '#fff'; ctx.font = 'bold 9px Inter, sans-serif'; ctx.textAlign = 'center';
          ctx.fillText(String(i + 1), sx(px), sy(py) + 3); ctx.textAlign = 'left';
        });
      }

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

      // scale bar (+ where the imagery comes from)
      ctx.font = '10px Inter, sans-serif';
      const caption = `${stepM} m grid${showAerial ? ` · aerial: ${basemap.world} (Gazebo terrain)` : ''}`;
      ctx.fillStyle = 'rgba(255,255,255,0.8)';
      ctx.fillRect(6, h - 22, ctx.measureText(caption).width + 8, 16);
      ctx.fillStyle = C.text;
      ctx.fillText(caption, 10, h - 10);
    };

    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [t, waypoints, zoom, aerial, basemap, imgReady, draft]);

  const pick = (e: React.MouseEvent<HTMLDivElement>) => {
    if (!drawing || e.target !== canvasRef.current) return;   // not on the buttons
    const r = canvasRef.current.getBoundingClientRect();
    const { cx, cy, scale, w, h } = viewRef.current;
    const x = cx + (e.clientX - r.left - w / 2) / scale;
    const y = cy - (e.clientY - r.top - h / 2) / scale;
    setDraft((d) => [...d, [Math.round(x * 10) / 10, Math.round(y * 10) / 10]]);
  };

  const send = (fn: () => void) => { fn(); setDraft([]); };
  const altOrNull = altText.trim() === '' ? null : Number(altText);

  return (
    <div className="bg-panelWhite rounded-lg shadow-sm border border-borderGray flex flex-col h-full overflow-hidden">
      <div className="flex items-center justify-between px-4 py-2.5 border-b border-borderGray shrink-0">
        <span className="text-sm font-semibold tracking-wide">MISSION MAP</span>
        <MoreHorizontal size={16} className="text-textMuted" />
      </div>

      <div className="border-b border-borderGray shrink-0"><HudStrip t={t} /></div>

      <div ref={wrapRef} onClick={pick}
           className={`flex-1 relative min-h-0 ${drawing ? 'cursor-crosshair' : ''}`}>
        <canvas ref={canvasRef} className="absolute inset-0 w-full h-full" />

        {/* waypoint drawing - everything sent is validated server-side like any step */}
        {planner && (
          <div className="absolute left-3 top-3 flex flex-col gap-1.5 items-start z-10">
            <button onClick={() => { setDrawing((d) => !d); setDraft([]); }}
                    title="Click the map to place waypoints"
                    className={`flex items-center gap-1.5 px-2.5 py-1.5 rounded-md shadow border text-xxs font-bold
                      ${drawing ? 'bg-fuchsia-500 text-white border-fuchsia-600' : 'bg-panelWhite border-borderGray text-textMain'}`}>
              <PenLine size={13} /> {drawing ? 'DRAWING' : 'DRAW WAYPOINTS'}
            </button>
            {drawing && (
              <div className="bg-panelWhite/95 rounded-md shadow border border-borderGray p-2 flex flex-col gap-1.5 w-56">
                <div className="text-xxs text-textMuted leading-snug">
                  {draft.length === 0
                    ? (planner.executing
                      ? 'Click to add waypoints to the mission that is flying.'
                      : 'Click the map to place waypoints.')
                    : `${draft.length} point(s): ${draft.map(([x, y]) => `(${x.toFixed(0)}, ${y.toFixed(0)})`).join(' ')}`}
                </div>
                {!planner.executing && (
                  <label className="flex items-center gap-1.5 text-xxs font-semibold">
                    ALT
                    <input value={altText} onChange={(e) => setAltText(e.target.value)}
                           placeholder="current" inputMode="decimal"
                           className="w-16 bg-bgMain border border-borderGray rounded px-1 py-0.5 outline-none" />
                    m
                  </label>
                )}
                <div className="flex flex-wrap gap-1">
                  {planner.executing ? (<>
                    <button disabled={!draft.length} onClick={() => send(() => planner.addWaypoints(draft, 'next'))}
                            className="map-btn" title="Fly to these now, then continue the mission">
                      <ListStart size={11} /> INSERT NEXT
                    </button>
                    <button disabled={!draft.length} onClick={() => send(() => planner.addWaypoints(draft, 'end'))}
                            className="map-btn" title="Fly these after the mission's last leg">
                      <ListEnd size={11} /> ADD AT END
                    </button>
                  </>) : (<>
                    <button disabled={!draft.length} onClick={() => send(() => planner.flyPath(draft, false, altOrNull))}
                            className="map-btn" title="Fly through the points once">
                      <Play size={11} /> FLY PATH
                    </button>
                    <button disabled={draft.length < 2} onClick={() => send(() => planner.flyPath(draft, true, altOrNull))}
                            className="map-btn" title="Fly the points and close the loop">
                      <Repeat size={11} /> FLY CIRCUIT
                    </button>
                  </>)}
                  <button disabled={!draft.length} onClick={() => setDraft((d) => d.slice(0, -1))}
                          className="map-btn" title="Remove the last point">
                    <Undo2 size={11} />
                  </button>
                  <button disabled={!draft.length} onClick={() => setDraft([])}
                          className="map-btn" title="Clear the drawn points">
                    <Trash2 size={11} />
                  </button>
                </div>
              </div>
            )}
          </div>
        )}

        {waypoints.length === 0 && !drawing && (
          <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
            <span className="text-textMuted text-xs bg-panelWhite/80 px-3 py-1.5 rounded">
              No mission loaded — send a command below
            </span>
          </div>
        )}

        <button onClick={() => setAerial((a) => !a)} disabled={!basemap}
                title={basemap ? (aerial ? 'Hide aerial map' : 'Show aerial map')
                  : 'No aerial map for this world'}
                className={`absolute right-3 top-3 rounded-md p-2 shadow border
                  ${basemap && aerial ? 'bg-aeroCyan/15 border-aeroCyan' : 'bg-panelWhite border-borderGray'}
                  disabled:opacity-40`}>
          <Layers size={18} className={basemap && aerial ? 'text-aeroCyan' : 'text-textMain'} />
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
