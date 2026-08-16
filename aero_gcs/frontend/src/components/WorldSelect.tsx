import { useEffect, useState, useCallback } from 'react';
import { Globe, Loader2 } from 'lucide-react';
import { API_BASE } from '../hooks/useLLM';

/**
 * Airframe + world picker. Applies on the next START SIM, because Gazebo can
 * only load a world at launch. The backend validates both names against what
 * actually exists on disk, so a typo can never produce a hanging make target.
 */
export default function WorldSelect() {
  const [models, setModels] = useState<string[]>([]);
  const [worlds, setWorlds] = useState<string[]>([]);
  const [model, setModel] = useState('iris');
  const [world, setWorld] = useState('empty');
  const [busy, setBusy] = useState(false);
  const [up, setUp] = useState(false);

  const load = useCallback(async () => {
    try {
      const d = await (await fetch(`${API_BASE}/api/sim/config`)).json();
      setModels(d.models ?? []);
      setWorlds(d.worlds ?? []);
      setModel(d.model ?? 'iris');
      setWorld(d.world ?? 'empty');
      setUp(true);
    } catch {
      setUp(false);
    }
  }, []);

  useEffect(() => { load(); }, [load]);

  const apply = async (next: { model?: string; world?: string }) => {
    setBusy(true);
    const q = new URLSearchParams(next as Record<string, string>).toString();
    try {
      await fetch(`${API_BASE}/api/sim/config?${q}`, { method: 'POST' });
      await load();
    } catch { /* offline */ }
    setBusy(false);
  };

  const cls = `bg-bgMain border border-borderGray rounded px-1.5 py-1 text-xxs font-semibold
               outline-none focus:border-aeroCyan disabled:opacity-40 max-w-[7.5rem] truncate`;

  return (
    <div className="hidden md:flex items-center gap-1.5 shrink-0" title="Applies on next START SIM">
      {busy ? <Loader2 size={13} className="text-aeroCyan animate-spin shrink-0" />
            : <Globe size={13} className="text-aeroCyan shrink-0" />}
      <select className={cls} value={model} disabled={!up || busy}
              onChange={(e) => { setModel(e.target.value); apply({ model: e.target.value }); }}>
        {models.length === 0 && <option>{model}</option>}
        {models.map((m) => <option key={m} value={m}>{m}</option>)}
      </select>
      <select className={cls} value={world} disabled={!up || busy}
              onChange={(e) => { setWorld(e.target.value); apply({ world: e.target.value }); }}>
        {worlds.length === 0 && <option>{world}</option>}
        {worlds.map((w) => <option key={w} value={w}>{w}</option>)}
      </select>
    </div>
  );
}
