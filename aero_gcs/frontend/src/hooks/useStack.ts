import { useState, useEffect, useCallback, useRef } from 'react';
import { API_BASE } from './useLLM';

/** The five processes that used to be five terminals. */
export type ServiceName = 'agent' | 'sitl' | 'rosbridge' | 'video' | 'telemetry';

export interface ServiceState {
  name: ServiceName;
  label: string;
  running: boolean;
  pid: number | null;
  detail: string;
}

const LABELS: Record<ServiceName, string> = {
  agent: 'uXRCE-DDS Agent',
  sitl: 'PX4 SITL + Gazebo',
  rosbridge: 'ROS Bridge (:9090)',
  video: 'Video Streamer (:8080)',
  telemetry: 'Telemetry Node',
};

const ORDER: ServiceName[] = ['agent', 'sitl', 'rosbridge', 'telemetry', 'video'];

/** Start/stop the whole simulation stack from the dashboard. */
export const useStack = () => {
  const [services, setServices] = useState<ServiceState[]>(
    ORDER.map((n) => ({ name: n, label: LABELS[n], running: false, pid: null, detail: '' })),
  );
  const [apiUp, setApiUp] = useState(false);
  const [busy, setBusy] = useState(false);
  const inFlight = useRef(false);

  const poll = useCallback(async () => {
    if (inFlight.current) return;
    inFlight.current = true;
    try {
      const r = await fetch(`${API_BASE}/api/stack/status`);
      if (!r.ok) throw new Error();
      const d = await r.json();
      setServices(ORDER.map((n) => ({
        name: n,
        label: LABELS[n],
        running: !!d[n]?.running,
        pid: d[n]?.pid ?? null,
        detail: d[n]?.detail ?? '',
      })));
      setApiUp(true);
    } catch {
      setApiUp(false);
      setServices(ORDER.map((n) => ({
        name: n, label: LABELS[n], running: false, pid: null, detail: '',
      })));
    } finally {
      inFlight.current = false;
    }
  }, []);

  useEffect(() => {
    poll();
    const t = setInterval(poll, 2000);
    return () => clearInterval(t);
  }, [poll]);

  const call = useCallback(async (path: string) => {
    setBusy(true);
    try { await fetch(`${API_BASE}${path}`, { method: 'POST' }); } catch { /* offline */ }
    await poll();
    setBusy(false);
  }, [poll]);

  /** Reap orphaned gzserver/px4 + clear Gazebo scratch. Never touches models
   *  or the serial hardware agent. Runs automatically on start too. */
  const cleanup = useCallback(async () => {
    setBusy(true);
    let detail = '';
    try {
      const r = await fetch(`${API_BASE}/api/stack/cleanup`, { method: 'POST' });
      const d = await r.json();
      detail = `reaped ${d.killed?.length ?? 0}, cleared ${d.removed?.length ?? 0}`;
    } catch {
      detail = 'control API offline';
    }
    await poll();
    setBusy(false);
    return detail;
  }, [poll]);

  /** Destructive: deletes ROS log history. Caller must confirm first. */
  const purgeLogs = useCallback(async () => {
    try {
      const info = await (await fetch(`${API_BASE}/api/logs/size`)).json();
      if (!window.confirm(
        `Delete ROS logs?\n\n${info.human} across ${info.files} files in ${info.path}\n\n` +
        'Models and simulator settings are NOT affected. This cannot be undone.')) {
        return 'cancelled';
      }
      const d = await (await fetch(
        `${API_BASE}/api/logs/purge?confirm=true`, { method: 'POST' })).json();
      return d.ok ? `purged ${d.removed} entries` : (d.detail ?? 'failed');
    } catch {
      return 'control API offline';
    }
  }, []);

  return {
    services,
    apiUp,
    busy,
    startAll: () => call('/api/stack/start'),
    stopAll: () => call('/api/stack/stop'),
    startOne: (n: ServiceName) => call(`/api/stack/start/${n}`),
    stopOne: (n: ServiceName) => call(`/api/stack/stop/${n}`),
    cleanup,
    purgeLogs,
  };
};
