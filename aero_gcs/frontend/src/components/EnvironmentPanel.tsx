import { useCallback, useEffect, useState } from 'react';
import { CloudSun, ChevronDown, Loader2, Download, Mountain } from 'lucide-react';
import { API_BASE } from '../hooks/useLLM';

interface Env {
  enabled: boolean;
  name: string;
  lat: number;
  lon: number;
  alt_msl: number;
  wind_mps: number;
  wind_from_deg: number;
  gust_mps: number;
  temp_c?: number | null;
  weather_time?: string | null;
}

const PRESETS: { label: string; name: string; lat: number; lon: number; alt: number }[] = [
  { label: 'McMillan airfield (the Gazebo world)', name: 'McMillan airfield', lat: 35.7222542, lon: -120.7665102, alt: 292 },
  { label: 'Field from the vloc mission example', name: 'field A', lat: 21.1081, lon: 79.0286, alt: 319 },
];

const BLANK: Env = { enabled: true, name: 'McMillan airfield', lat: 35.7222542, lon: -120.7665102,
                     alt_msl: 292, wind_mps: 0, wind_from_deg: 0, gust_mps: 0 };

/**
 * Real-location environment for the sim: GPS / magnetic field / air pressure
 * of the place plus its (live-fetched or typed) wind, gusts and turbulence as
 * Gazebo wind. Applies on the next START SIM. The 3D terrain stays the
 * selected Gazebo world.
 */
export default function EnvironmentPanel() {
  const [open, setOpen] = useState(false);
  const [env, setEnv] = useState<Env>(BLANK);
  const [active, setActive] = useState('');
  const [busy, setBusy] = useState(false);
  const [note, setNote] = useState('');
  const [sizeM, setSizeM] = useState(2000);
  const [build, setBuild] = useState<{ state: string; log: string[]; error?: string | null }>({ state: 'idle', log: [] });

  const load = useCallback(async () => {
    try {
      const d = await (await fetch(`${API_BASE}/api/sim/environment`)).json();
      if (d.config?.lat !== undefined) setEnv({ ...BLANK, ...d.config });
      setActive(d.active_summary ?? '');
    } catch { setNote('control API offline'); }
  }, []);

  useEffect(() => { load(); }, [load]);

  const set = (k: keyof Env, v: string | number | boolean) => setEnv((e) => ({ ...e, [k]: v }));
  const num = (k: keyof Env) => (ev: React.ChangeEvent<HTMLInputElement>) => set(k, Number(ev.target.value));

  const fetchWeather = async () => {
    setBusy(true); setNote('');
    try {
      const d = await (await fetch(`${API_BASE}/api/sim/environment/weather?lat=${env.lat}&lon=${env.lon}`,
        { method: 'POST' })).json();
      if (!d.ok) { setNote(d.detail); return; }
      const w = d.weather;
      setEnv((e) => ({ ...e, wind_mps: w.wind_mps, wind_from_deg: w.wind_from_deg, gust_mps: w.gust_mps,
                       alt_msl: w.elevation_m ?? e.alt_msl, temp_c: w.temp_c, weather_time: w.time }));
      setNote(`live weather ${w.time} UTC: ${w.temp_c} °C, ${w.pressure_hpa} hPa`);
    } catch { setNote('control API offline'); } finally { setBusy(false); }
  };

  const save = async () => {
    setBusy(true);
    try {
      const d = await (await fetch(`${API_BASE}/api/sim/environment`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(env),
      })).json();
      setNote(d.ok ? 'Saved - press STOP SIM, then START SIM to apply.' : d.detail);
    } catch { setNote('control API offline'); } finally { setBusy(false); }
  };

  const buildWorld = async () => {
    const name = env.name.toLowerCase().replace(/[^a-z0-9_]+/g, '_').replace(/^_+|_+$/g, '') || 'field';
    try {
      const d = await (await fetch(`${API_BASE}/api/sim/worlds/build`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name, lat: env.lat, lon: env.lon, size_m: sizeM }),
      })).json();
      if (!d.ok) { setNote(d.detail); return; }
    } catch { setNote('control API offline'); return; }
    setBuild({ state: 'running', log: [] });
    const poll = setInterval(async () => {
      try {
        const s = await (await fetch(`${API_BASE}/api/sim/worlds/build`)).json();
        setBuild(s);
        if (s.state === 'done') {
          clearInterval(poll);
          await fetch(`${API_BASE}/api/sim/config?world=${encodeURIComponent(s.result.name)}`, { method: 'POST' });
          window.dispatchEvent(new Event('aero:worlds-changed'));
          setNote(`World '${s.result.name}' built (${s.result.size_m} m, ground ${s.result.ground_range_rel_home_m[0]}`
            + ` to +${s.result.ground_range_rel_home_m[1]} m vs home) and selected - press STOP SIM, then START SIM.`);
        } else if (s.state === 'error') {
          clearInterval(poll);
          setNote(`build failed: ${s.error}`);
        }
      } catch { /* keep polling */ }
    }, 1000);
  };

  const inp = 'w-full bg-bgMain border border-borderGray rounded px-1.5 py-1 text-xxs outline-none focus:border-aeroCyan';
  const lbl = 'text-xxs text-textMuted';

  return (
    <div className="relative shrink-0">
      <button onClick={() => setOpen((o) => !o)} title={`Sim environment: ${active}`}
              className="flex items-center gap-1 px-2 py-1 rounded text-xxs font-bold text-textMuted hover:bg-bgMain">
        <CloudSun size={13} /> <span className="hidden lg:inline">ENVIRONMENT</span>
        <ChevronDown size={11} className={open ? 'rotate-180 transition-transform' : 'transition-transform'} />
      </button>

      {open && (
        <div className="absolute right-0 top-9 w-80 bg-panelWhite border border-borderGray rounded-lg shadow-lg z-50 p-3 flex flex-col gap-2">
          <div className="text-xxs leading-snug"><b>Running now:</b> {active || '—'}</div>
          <select className={inp} defaultValue=""
                  onChange={(e) => { const p = PRESETS[Number(e.target.value)]; if (p) setEnv((x) => ({ ...x, name: p.name, lat: p.lat, lon: p.lon, alt_msl: p.alt })); }}>
            <option value="" disabled>Preset location…</option>
            {PRESETS.map((p, i) => <option key={p.label} value={i}>{p.label}</option>)}
          </select>
          <div className="grid grid-cols-3 gap-1.5">
            <label className="col-span-3"><span className={lbl}>name</span>
              <input className={inp} value={env.name} onChange={(e) => set('name', e.target.value)} /></label>
            <label><span className={lbl}>latitude</span><input className={inp} type="number" step="0.0001" value={env.lat} onChange={num('lat')} /></label>
            <label><span className={lbl}>longitude</span><input className={inp} type="number" step="0.0001" value={env.lon} onChange={num('lon')} /></label>
            <label><span className={lbl}>ground m MSL</span><input className={inp} type="number" value={env.alt_msl} onChange={num('alt_msl')} /></label>
          </div>
          <button onClick={fetchWeather} disabled={busy}
                  className="flex items-center justify-center gap-1.5 px-2 py-1.5 rounded text-xxs font-bold bg-aeroCyan/15 text-aeroCyan hover:bg-aeroCyan/25 disabled:opacity-40">
            {busy ? <Loader2 size={12} className="animate-spin" /> : <Download size={12} />} FETCH LIVE WEATHER HERE
          </button>
          <div className="grid grid-cols-3 gap-1.5">
            <label><span className={lbl}>wind m/s (10 m)</span><input className={inp} type="number" step="0.1" value={env.wind_mps} onChange={num('wind_mps')} /></label>
            <label><span className={lbl}>from °</span><input className={inp} type="number" value={env.wind_from_deg} onChange={num('wind_from_deg')} /></label>
            <label><span className={lbl}>gusts m/s</span><input className={inp} type="number" step="0.1" value={env.gust_mps} onChange={num('gust_mps')} /></label>
          </div>
          <label className="flex items-center gap-1.5 text-xxs font-semibold">
            <input type="checkbox" checked={env.enabled} onChange={(e) => set('enabled', e.target.checked)} />
            use this location and weather in the sim
          </label>
          <button onClick={save} disabled={busy}
                  className="px-2 py-1.5 rounded text-xxs font-bold bg-statusGreen/15 text-statusGreen hover:bg-statusGreen/25 disabled:opacity-40">
            SAVE (applies on next START SIM)
          </button>
          <div className="border-t border-borderGray pt-2 flex flex-col gap-1.5">
            <div className="text-xxs font-semibold">Real terrain + aerial image of this place</div>
            <div className="flex gap-1.5">
              <select className={inp} value={sizeM} onChange={(e) => setSizeM(Number(e.target.value))}>
                <option value={1000}>1 km x 1 km</option>
                <option value={2000}>2 km x 2 km</option>
                <option value={3000}>3 km x 3 km</option>
              </select>
              <button onClick={buildWorld} disabled={build.state === 'running'}
                      className="flex items-center gap-1 px-2 py-1 rounded text-xxs font-bold bg-amber-100 text-amber-700 hover:bg-amber-200 disabled:opacity-40 shrink-0">
                {build.state === 'running' ? <Loader2 size={12} className="animate-spin" /> : <Mountain size={12} />}
                BUILD GAZEBO WORLD HERE
              </button>
            </div>
            {build.state === 'running' && (
              <div className="text-xxs text-aeroCyan">{build.log[build.log.length - 1] ?? 'starting…'}</div>
            )}
            <div className="text-xxs text-textMuted leading-snug">
              SRTM elevation (no buildings or trees - raise the clearance over built-up areas) and Esri World
              Imagery (attribution: Esri, Maxar, Earthstar Geographics). Takes ~30 s; tiles are cached.
            </div>
          </div>
          {note && <div className="text-xxs leading-snug">{note}</div>}
          <div className="text-xxs text-textMuted leading-snug">
            Simulated: GPS position, compass field and air pressure of this place; its wind with turbulence and a
            gust window from 60 s (scaled from 10 m to ~30 m flight height). The 3D terrain stays the selected
            Gazebo world. Weather is fetched from Open-Meteo (only lat/lon are sent).
          </div>
        </div>
      )}
    </div>
  );
}
