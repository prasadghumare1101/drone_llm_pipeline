import { useState, useEffect, useCallback, useRef } from 'react';

export const API_BASE = 'http://localhost:8000';

/** One row of the MISSION AUTOMATION & CONTROL table / one pin on the map. */
export interface MissionWaypoint {
  idx: number;
  total: number;
  x: number;          // ENU east, metres from home
  y: number;          // ENU north, metres from home
  alt: number;
  lat: number;
  lon: number;
  speed: number;
  action: string;
  energy: number;
  active: boolean;
}

export interface MissionState {
  llmStatus: string;
  waypoints: MissionWaypoint[];
  running: boolean;
  log: string[];
}

const EMPTY: MissionState = {
  llmStatus: 'NO MISSION PLAN LOADED',
  waypoints: [],
  running: false,
  log: [],
};

/**
 * Owns the LLM side of the dashboard: the compiled mission plan, the run log,
 * and dispatching a natural-language prompt to run_pipeline.py on the backend.
 * Polls at 1 Hz - the plan changes rarely, so this stays cheap.
 */
export const useLLM = () => {
  const [state, setState] = useState<MissionState>(EMPTY);
  const [error, setError] = useState<string | null>(null);
  const inFlight = useRef(false);

  const poll = useCallback(async () => {
    if (inFlight.current) return;
    inFlight.current = true;
    try {
      const r = await fetch(`${API_BASE}/api/mission/status`);
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      const d = await r.json();
      setState({
        llmStatus: d.llm_status ?? EMPTY.llmStatus,
        waypoints: d.waypoints ?? [],
        running: !!d.running,
        log: d.log ?? [],
      });
      setError(null);
    } catch {
      setError('control API offline (start backend_services/control_api.py)');
    } finally {
      inFlight.current = false;
    }
  }, []);

  useEffect(() => {
    poll();
    const t = setInterval(poll, 1000);
    return () => clearInterval(t);
  }, [poll]);

  /** Send a natural-language mission request -> validator -> executor -> PX4. */
  const sendCommand = useCallback(async (prompt: string) => {
    const text = prompt.trim();
    if (!text) return;
    setState((s) => ({ ...s, running: true, llmStatus: 'LLM PLANNING MISSION…' }));
    try {
      await fetch(`${API_BASE}/api/mission/run`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ prompt: text }),
      });
    } catch {
      setError('failed to reach control API');
    }
    poll();
  }, [poll]);

  const abortMission = useCallback(async () => {
    try { await fetch(`${API_BASE}/api/mission/abort`, { method: 'POST' }); } catch { /* offline */ }
    poll();
  }, [poll]);

  return { ...state, error, sendCommand, abortMission };
};
