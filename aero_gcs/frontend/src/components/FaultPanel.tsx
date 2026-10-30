import { useCallback, useEffect, useState } from 'react';
import { Zap, ChevronDown, RotateCcw, Loader2 } from 'lucide-react';
import { API_BASE } from '../hooks/useLLM';

const LABEL: Record<string, string> = {
  gps: 'GPS', mag: 'Compass', baro: 'Barometer', accel: 'Accelerometer', gyro: 'Gyro', motor: 'Motor 1',
};

/**
 * Sensor / motor fault injection into the PX4 simulation, for testing how the
 * autopilot and the AI session react. Each injection is also written into the
 * chat transcript so a test run shows what was broken and when.
 */
export default function FaultPanel() {
  const [open, setOpen] = useState(false);
  const [catalog, setCatalog] = useState<Record<string, Record<string, string>>>({});
  const [active, setActive] = useState<Record<string, string>>({});
  const [busy, setBusy] = useState(false);
  const [note, setNote] = useState('');

  const load = useCallback(async () => {
    try {
      const d = await (await fetch(`${API_BASE}/api/sim/faults`)).json();
      setCatalog(d.catalog ?? {}); setActive(d.active ?? {});
    } catch { setNote('control API offline'); }
  }, []);

  useEffect(() => { if (open) load(); }, [open, load]);

  const call = async (path: string) => {
    setBusy(true);
    try {
      const d = await (await fetch(`${API_BASE}${path}`, { method: 'POST' })).json();
      setActive(d.active ?? {});
      setNote(d.ok === false ? (d.detail ?? d.output ?? 'failed') : (d.output || ''));
    } catch { setNote('control API offline'); }
    setBusy(false);
  };

  const count = Object.keys(active).length;

  return (
    <div className="relative shrink-0">
      <button onClick={() => setOpen((o) => !o)} title="Inject sensor / motor faults into the simulation"
              className={`flex items-center gap-1 px-2 py-1 rounded text-xxs font-bold
                ${count ? 'bg-alertRed/15 text-alertRed' : 'text-textMuted hover:bg-bgMain'}`}>
        <Zap size={13} /> <span className="hidden lg:inline">FAULTS{count ? ` (${count})` : ''}</span>
        <ChevronDown size={11} className={open ? 'rotate-180 transition-transform' : 'transition-transform'} />
      </button>

      {open && (
        <div className="absolute right-0 top-9 w-80 bg-panelWhite border border-borderGray rounded-lg shadow-lg z-50 p-2">
          <div className="text-xxs text-textMuted leading-snug px-1 pb-2">
            Inject faults into the PX4 sim to test the autopilot's failsafes and the AI session.
            PX4 SITL cannot do gradual GPS drift or wind: "jump" is a ~111 km spoof.
          </div>
          {Object.entries(catalog).map(([comp, types]) => (
            <div key={comp} className="flex items-center gap-1.5 px-1 py-1 border-t border-borderGray">
              <span className={`text-xxs font-semibold w-24 shrink-0 ${active[comp] ? 'text-alertRed' : ''}`}>
                {LABEL[comp] ?? comp}{active[comp] ? ` · ${active[comp]}` : ''}
              </span>
              <div className="flex flex-wrap gap-1 flex-1">
                {Object.entries(types).map(([t, desc]) => (
                  <button key={t} disabled={busy} title={desc}
                          onClick={() => call(`/api/sim/fault?component=${comp}&type=${t}`)}
                          className={`text-xxs font-bold px-1.5 py-0.5 rounded disabled:opacity-40
                            ${active[comp] === t ? 'bg-alertRed text-white' : 'bg-alertRed/10 text-alertRed hover:bg-alertRed/20'}`}>
                    {t === 'wrong' ? 'jump' : t}
                  </button>
                ))}
                {active[comp] && (
                  <button disabled={busy} onClick={() => call(`/api/sim/fault?component=${comp}&type=ok`)}
                          title="Restore this sensor"
                          className="text-xxs font-bold px-1.5 py-0.5 rounded bg-statusGreen/15 text-statusGreen">
                    <RotateCcw size={10} />
                  </button>
                )}
              </div>
            </div>
          ))}
          <div className="flex items-center justify-between border-t border-borderGray pt-1.5 mt-1 px-1">
            <span className="text-xxs text-textMuted truncate flex items-center gap-1">
              {busy && <Loader2 size={10} className="animate-spin" />}{note}
            </span>
            <button disabled={busy || !count} onClick={() => call('/api/sim/fault/clear')}
                    className="text-xxs font-bold px-2 py-0.5 rounded bg-statusGreen/15 text-statusGreen disabled:opacity-40 shrink-0">
              RESTORE ALL
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
