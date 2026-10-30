import { useCallback, useEffect, useState } from 'react';
import { FileSearch, Upload, X, Loader2, Play, AlertTriangle, Info, OctagonAlert } from 'lucide-react';
import { API_BASE } from '../hooks/useLLM';

interface Finding {
  severity: 'critical' | 'warning' | 'info';
  t: number;
  title: string;
  detail: string;
  likely_cause: string;
  fix: string;
}

interface Report {
  file: string;
  kind: string;
  autopilot: string;
  duration_s: number;
  findings: Finding[];
  timeline: { t: number; kind: string; text: string; severity: string }[];
  mission: { seq: number; name: string; north_m?: number; east_m?: number; alt: number; p: number[] }[];
  replay: { points: [number, number][]; alt: number | null };
}

interface Flight {
  flight: string;
  files: { name: string; bytes: number }[];
  analyzed: boolean;
}

const SEV = {
  critical: { cls: 'border-alertRed bg-alertRed/5', icon: <OctagonAlert size={13} className="text-alertRed shrink-0" /> },
  warning: { cls: 'border-amber-400 bg-amber-50', icon: <AlertTriangle size={13} className="text-amber-600 shrink-0" /> },
  info: { cls: 'border-borderGray bg-bgMain', icon: <Info size={13} className="text-textMuted shrink-0" /> },
};

const mb = (b: number) => `${(b / 1048576).toFixed(b > 1048576 ? 1 : 2)} MB`;

/**
 * Real-flight log analysis: upload a dataflash .BIN / .tlog / mission JSON,
 * get a timeline + findings (stuck episodes with likely cause and fix,
 * failsafes, EKF/GPS, battery, 3DR radio link), and replay the logged mission
 * in the running sim session - through the same validator, terrain, battery
 * and stuck checks as any step.
 */
export default function FlightLogs({ onReplay }: {
  onReplay?: (points: [number, number][], alt: number | null) => void;
}) {
  const [open, setOpen] = useState(false);
  const [flights, setFlights] = useState<Flight[]>([]);
  const [dir, setDir] = useState('');
  const [name, setName] = useState('');
  const [busy, setBusy] = useState('');
  const [note, setNote] = useState('');
  const [sel, setSel] = useState<string | null>(null);
  const [report, setReport] = useState<Report | null>(null);
  const [showTimeline, setShowTimeline] = useState(false);

  const load = useCallback(async () => {
    try {
      const d = await (await fetch(`${API_BASE}/api/flightlogs`)).json();
      setFlights(d.flights ?? []);
      setDir(d.dir ?? '');
    } catch { setNote('control API offline'); }
  }, []);

  useEffect(() => { if (open) load(); }, [open, load]);

  const pick = async (flight: string) => {
    setSel(flight); setReport(null); setShowTimeline(false);
    try {
      const d = await (await fetch(`${API_BASE}/api/flightlogs/${encodeURIComponent(flight)}/report`)).json();
      if (d.ok) setReport(d.report);
    } catch { /* offline */ }
  };

  const upload = async (files: FileList | null) => {
    if (!files?.length) return;
    const flight = name.trim() || `flight_${new Date().toISOString().slice(0, 16).replace(/\D/g, '')}`;
    for (const f of Array.from(files)) {
      setBusy(`uploading ${f.name} (${mb(f.size)})…`);
      try {
        const r = await fetch(`${API_BASE}/api/flightlogs/upload?flight=${encodeURIComponent(flight)}`
          + `&name=${encodeURIComponent(f.name)}`, { method: 'POST', body: f });
        const d = await r.json();
        setNote(d.ok ? `uploaded ${d.file} to ${d.flight}` : `${f.name}: ${d.detail}`);
      } catch { setNote(`${f.name}: upload failed`); }
    }
    setBusy('');
    await load();
    pick(flight.replace(/[^A-Za-z0-9._-]/g, '_'));
  };

  const analyze = async (flight: string) => {
    setBusy('analysing (large .BIN files take a while)…');
    try {
      const d = await (await fetch(`${API_BASE}/api/flightlogs/${encodeURIComponent(flight)}/analyze`,
        { method: 'POST' })).json();
      if (d.ok) { setReport(d.report); setSel(flight); setNote(''); } else setNote(d.detail);
    } catch { setNote('analysis failed (control API offline?)'); }
    setBusy('');
    load();
  };

  const counts = report?.findings.reduce((a, f) => ({ ...a, [f.severity]: (a[f.severity] ?? 0) + 1 }),
    {} as Record<string, number>);

  return (
    <>
      <button onClick={() => setOpen(true)} title="Upload and analyse real flight logs"
              className="flex items-center gap-1 px-2 py-1 rounded text-xxs font-bold text-textMuted hover:bg-bgMain shrink-0">
        <FileSearch size={13} /> <span className="hidden lg:inline">FLIGHT LOGS</span>
      </button>

      {open && (
        <div className="fixed inset-0 z-50 bg-black/40 flex items-center justify-center p-4" onClick={() => setOpen(false)}>
          <div className="bg-panelWhite rounded-lg shadow-xl border border-borderGray w-full max-w-6xl h-[85vh] flex flex-col overflow-hidden"
               onClick={(e) => e.stopPropagation()}>
            <div className="flex items-center justify-between px-4 py-2.5 border-b border-borderGray shrink-0">
              <span className="text-sm font-semibold tracking-wide">REAL FLIGHT LOGS — FAILURE ANALYSIS</span>
              <button onClick={() => setOpen(false)}><X size={16} className="text-textMuted" /></button>
            </div>

            <div className="flex-1 min-h-0 grid grid-cols-1 md:grid-cols-[18rem_1fr]">
              {/* left: upload + flights */}
              <div className="border-r border-borderGray flex flex-col min-h-0">
                <div className="p-3 border-b border-borderGray flex flex-col gap-2 shrink-0">
                  <input value={name} onChange={(e) => setName(e.target.value)}
                         placeholder="flight name, e.g. 2026-10-29_field_A_stuck"
                         className="text-xs bg-bgMain border border-borderGray rounded px-2 py-1.5 outline-none focus:border-aeroCyan" />
                  <label className={`flex items-center justify-center gap-1.5 px-2 py-2 rounded text-xxs font-bold cursor-pointer
                                     bg-aeroCyan/15 text-aeroCyan hover:bg-aeroCyan/25 ${busy ? 'opacity-50 pointer-events-none' : ''}`}>
                    <Upload size={13} /> UPLOAD .BIN / .tlog / mission .json
                    <input type="file" multiple accept=".bin,.BIN,.tlog,.json,.txt,.log,.csv" className="hidden"
                           onChange={(e) => { upload(e.target.files); e.target.value = ''; }} />
                  </label>
                  <div className="text-xxs text-textMuted leading-snug">
                    The dataflash <b>.BIN</b> from the SD card (APM/LOGS) tells the most. Saved in {dir || 'realFlight_logs/'}.
                  </div>
                  {(busy || note) && (
                    <div className="text-xxs flex items-center gap-1.5">
                      {busy && <Loader2 size={11} className="animate-spin text-aeroCyan" />}
                      <span className={busy ? 'text-aeroCyan' : 'text-textMain'}>{busy || note}</span>
                    </div>
                  )}
                </div>
                <div className="flex-1 overflow-y-auto min-h-0">
                  {flights.length === 0 && <div className="p-3 text-xxs text-textMuted">No flights uploaded yet.</div>}
                  {flights.map((f) => (
                    <div key={f.flight} onClick={() => pick(f.flight)}
                         className={`px-3 py-2 border-b border-borderGray cursor-pointer ${sel === f.flight ? 'bg-aeroCyan/10' : 'hover:bg-bgMain'}`}>
                      <div className="flex items-center justify-between gap-2">
                        <span className="text-xs font-semibold truncate">{f.flight}</span>
                        <button onClick={(e) => { e.stopPropagation(); analyze(f.flight); }} disabled={!!busy}
                                className="text-xxs font-bold px-1.5 py-0.5 rounded bg-statusGreen/15 text-statusGreen disabled:opacity-40 shrink-0">
                          {f.analyzed ? 'RE-ANALYSE' : 'ANALYSE'}
                        </button>
                      </div>
                      {f.files.map((x) => (
                        <div key={x.name} className="text-xxs text-textMuted truncate">{x.name} · {mb(x.bytes)}</div>
                      ))}
                    </div>
                  ))}
                </div>
              </div>

              {/* right: report */}
              <div className="flex flex-col min-h-0 overflow-y-auto p-4 gap-3">
                {!report && (
                  <div className="text-xs text-textMuted m-auto text-center max-w-md leading-relaxed">
                    Upload a flight's logs, then press <b>ANALYSE</b>. You get the timeline and the findings:
                    where the vehicle got stuck and the likely reason, failsafes, EKF/GPS trouble, battery and
                    the 3DR radio link. <b>REPLAY IN SIM</b> then flies the logged mission in the sim session.
                  </div>
                )}
                {report && (<>
                  <div className="flex flex-wrap items-center justify-between gap-2">
                    <div className="text-xs">
                      <b>{report.file}</b> · {report.autopilot} {report.kind} · {Math.round(report.duration_s / 60)} min ·{' '}
                      <span className="text-alertRed font-semibold">{counts?.critical ?? 0} critical</span> ·{' '}
                      <span className="text-amber-600 font-semibold">{counts?.warning ?? 0} warnings</span>
                    </div>
                    <button disabled={!onReplay || !report.replay.points.length}
                            onClick={() => { onReplay?.(report.replay.points, report.replay.alt); setOpen(false); }}
                            title={!onReplay ? 'Start an interactive session first'
                              : !report.replay.points.length ? 'No mission waypoints in this log' : ''}
                            className="flex items-center gap-1.5 px-2.5 py-1.5 rounded text-xxs font-bold bg-fuchsia-500 text-white disabled:opacity-35">
                      <Play size={12} /> REPLAY IN SIM ({report.replay.points.length} waypoints
                      {report.replay.alt != null ? ` @ ${report.replay.alt} m` : ''})
                    </button>
                  </div>
                  <div className="text-xxs text-textMuted leading-snug">
                    Replay flies the logged waypoints through the sim's validator, terrain, battery and stuck checks. It
                    tests the mission's geometry and safety, not ArduPilot's exact behaviour (the sim runs PX4).
                  </div>

                  <div className="flex flex-col gap-1.5">
                    {report.findings.length === 0 && <div className="text-xs text-statusGreen">No problems found.</div>}
                    {report.findings.map((f, i) => (
                      <div key={i} className={`border-l-4 rounded px-3 py-2 ${SEV[f.severity].cls}`}>
                        <div className="flex items-start gap-1.5 text-xs font-semibold">
                          {SEV[f.severity].icon}<span className="flex-1">{f.title}</span>
                          <span className="text-xxs text-textMuted font-normal shrink-0">t = {f.t.toFixed(0)} s</span>
                        </div>
                        {f.detail && <div className="text-xxs mt-0.5">{f.detail}</div>}
                        {f.likely_cause && <div className="text-xxs mt-0.5"><b>Likely cause:</b> {f.likely_cause}</div>}
                        {f.fix && <div className="text-xxs mt-0.5"><b>Fix:</b> {f.fix}</div>}
                      </div>
                    ))}
                  </div>

                  {report.mission.length > 0 && (
                    <table className="w-full text-xxs border-collapse">
                      <thead><tr className="border-b border-borderGray text-left">
                        <th className="py-1">#</th><th>item</th><th>north m</th><th>east m</th><th>alt m</th><th>params</th>
                      </tr></thead>
                      <tbody>{report.mission.map((m) => (
                        <tr key={m.seq} className="border-b border-borderGray">
                          <td className="py-0.5">{m.seq}</td><td>{m.name}</td>
                          <td>{m.north_m ?? '—'}</td><td>{m.east_m ?? '—'}</td><td>{m.alt}</td>
                          <td className="text-textMuted">{m.p.join(', ')}</td>
                        </tr>
                      ))}</tbody>
                    </table>
                  )}

                  <button onClick={() => setShowTimeline((s) => !s)} className="text-xxs font-bold text-aeroCyan self-start">
                    {showTimeline ? 'HIDE' : 'SHOW'} TIMELINE ({report.timeline.length} events)
                  </button>
                  {showTimeline && (
                    <div className="font-mono text-xxs leading-relaxed bg-bgMain rounded p-2 max-h-72 overflow-y-auto">
                      {report.timeline.map((e, i) => (
                        <div key={i} className={e.severity === 'warning' ? 'text-alertRed' : ''}>
                          {e.t.toFixed(1).padStart(7)}s  {e.text}
                        </div>
                      ))}
                    </div>
                  )}
                </>)}
              </div>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
