import { Cpu, Loader2, Square } from 'lucide-react';
import type { MissionWaypoint } from '../hooks/useLLM';

/** MISSION AUTOMATION & CONTROL - right column, top. */
export default function MissionRight({
  status, waypoints, running, onAbort,
}: {
  status: string; waypoints: MissionWaypoint[]; running: boolean; onAbort: () => void;
}) {
  return (
    <div className="bg-panelWhite rounded-lg shadow-sm border border-borderGray flex flex-col h-full overflow-hidden">
      <div className="px-4 py-2.5 border-b border-borderGray shrink-0">
        <span className="text-sm font-semibold tracking-wide">MISSION AUTOMATION &amp; CONTROL</span>
      </div>

      {/* LLM plan banner */}
      <div className="flex items-start gap-2.5 px-3 py-2.5 border-b border-borderGray shrink-0">
        <div className="relative shrink-0 mt-0.5">
          <div className="w-8 h-8 rounded-md bg-aeroCyan/10 border border-aeroCyan/40
                          flex items-center justify-center shadow-[0_0_10px_rgba(12,165,233,0.35)]">
            <Cpu size={16} className="text-aeroCyan" />
          </div>
        </div>
        <span className="text-xs font-semibold leading-snug flex-1">{status}</span>
        {running && (
          <button onClick={onAbort} title="Abort mission"
                  className="shrink-0 flex items-center gap-1 px-2 py-1 rounded text-xxs font-bold
                             bg-alertRed/15 text-alertRed hover:bg-alertRed/25">
            <Square size={10} /> ABORT
          </button>
        )}
      </div>

      <div className="flex-1 overflow-y-auto min-h-0">
        <table className="w-full text-xxs text-left border-collapse">
          <thead className="sticky top-0 bg-panelWhite">
            <tr className="text-textMain border-b border-borderGray">
              <th className="py-2 px-2 font-semibold">LAT/LON</th>
              <th className="py-2 px-1 font-semibold">SPEED<br />(km/h)</th>
              <th className="py-2 px-1 font-semibold">ALT (m)<br />GIMBAL</th>
              <th className="py-2 px-1 font-semibold">ACTION</th>
              <th className="py-2 px-1 font-semibold">ENERGY<br />REM.</th>
            </tr>
          </thead>
          <tbody>
            {waypoints.length === 0 && (
              <tr><td colSpan={5} className="py-6 text-center text-textMuted">
                No waypoints — send a mission command
              </td></tr>
            )}
            {waypoints.map((w) => (
              <tr key={w.idx}
                  className={`border-b border-borderGray align-top ${w.active ? 'bg-aeroCyan/10' : ''}`}>
                <td className="py-2 px-2">
                  <div className="font-semibold whitespace-nowrap">
                    WP {w.idx}/{w.total}
                    {w.active && <span className="text-statusGreen font-bold"> (ACTIVE)</span>}
                  </div>
                  <div className="text-textMuted">{w.lat.toFixed(4)}°N,</div>
                  <div className="text-textMuted">{Math.abs(w.lon).toFixed(4)}°W</div>
                </td>
                <td className="py-2 px-1 tabular-nums">{w.speed.toFixed(1)}</td>
                <td className="py-2 px-1 tabular-nums">{w.alt.toFixed(1)}</td>
                <td className="py-2 px-1">{w.action}</td>
                <td className="py-2 px-1 tabular-nums">{w.energy}%</td>
              </tr>
            ))}
          </tbody>
        </table>
        {running && (
          <div className="flex items-center gap-2 px-3 py-2 text-xxs text-aeroCyan font-semibold">
            <Loader2 size={12} className="animate-spin" /> mission executing…
          </div>
        )}
      </div>
    </div>
  );
}
