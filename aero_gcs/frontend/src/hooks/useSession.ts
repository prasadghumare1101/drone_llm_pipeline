import { useState, useEffect, useCallback, useRef } from 'react';
import { API_BASE, type MissionWaypoint } from './useLLM';

/** One line of the interactive conversation. */
export interface ChatMessage {
  id: number;
  role: 'operator' | 'drone' | 'system';
  text: string;
  t: number;
}

export interface SessionVehicle {
  x: number; y: number; alt: number; heading_deg: number;
  armed: boolean; in_air: boolean; battery_pct: number | null;
  mode: string; rc_override: boolean; connected: boolean;
}

export type SessionPhase =
  | 'ON_GROUND' | 'PLANNING' | 'AWAITING_ARM' | 'EXECUTING'
  | 'HOVERING' | 'RETURNING' | 'LANDING' | 'RC_OVERRIDE' | 'FAULT';

export type QuickCommand = 'hold' | 'land' | 'rtl' | 'resume' | 'status';

export interface SessionState {
  active: boolean;
  link: string;
  planner: string;
  phase: SessionPhase | null;
  vehicle: SessionVehicle | null;
  watchdogS: number | null;
  armWaitS: number | null;
  canResume: boolean;
  stepText: string;
  waypoints: MissionWaypoint[];
  messages: ChatMessage[];
}

const IDLE: SessionState = {
  active: false, link: '', planner: '', phase: null, vehicle: null,
  watchdogS: null, armWaitS: null, canResume: false, stepText: '',
  waypoints: [], messages: [],
};

const MAX_MESSAGES = 200;

/**
 * Interactive flight session: one chat message = one step from the live
 * position. Polls at 2 Hz and only fetches transcript lines it has not seen,
 * so the payload stays small however long the conversation runs.
 */
export const useSession = () => {
  const [state, setState] = useState<SessionState>(IDLE);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const lastId = useRef(0);
  const inFlight = useRef(false);

  const poll = useCallback(async () => {
    if (inFlight.current) return;
    inFlight.current = true;
    try {
      const r = await fetch(`${API_BASE}/api/session/status?since=${lastId.current}`);
      const d = await r.json();
      if (!d.active) {
        lastId.current = 0;
        setState((s) => (s.active ? { ...IDLE, messages: s.messages } : s));
        return;
      }
      const fresh: ChatMessage[] = d.transcript ?? [];
      if (fresh.length) lastId.current = fresh[fresh.length - 1].id;
      setState((s) => ({
        active: true,
        link: d.link,
        planner: d.planner,
        phase: d.phase,
        vehicle: d.vehicle,
        watchdogS: d.watchdog_s,
        armWaitS: d.arm_wait_s,
        canResume: !!d.can_resume,
        stepText: d.step?.text ?? '',
        waypoints: (d.step?.waypoints ?? []).map((w: MissionWaypoint) => ({
          ...w, energy: w.energy ?? 0,
        })),
        // a restarted session numbers its messages from 1 again
        messages: (s.active ? [...s.messages, ...fresh] : fresh).slice(-MAX_MESSAGES),
      }));
      setError(null);
    } catch {
      setError('control API offline');
    } finally {
      inFlight.current = false;
    }
  }, []);

  useEffect(() => {
    poll();
    const t = setInterval(poll, 500);
    return () => clearInterval(t);
  }, [poll]);

  const post = useCallback(async (path: string, body?: unknown) => {
    try {
      const r = await fetch(`${API_BASE}${path}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: body === undefined ? undefined : JSON.stringify(body),
      });
      const d = await r.json();
      if (d.ok === false && (d.detail || d.reply)) setError(d.detail ?? d.reply);
      else setError(null);
      return d;
    } catch {
      setError('failed to reach control API');
      return null;
    } finally {
      poll();
    }
  }, [poll]);

  const start = useCallback(async (backend: 'px4' | 'sim') => {
    setBusy(true);
    lastId.current = 0;
    setState(IDLE);
    await post(`/api/session/start?backend=${backend}`);
    setBusy(false);
  }, [post]);

  const stop = useCallback(async () => {
    setBusy(true);
    await post('/api/session/stop');
    setBusy(false);
  }, [post]);

  /** Free text. stop / land / return home are recognised server-side and act instantly. */
  const send = useCallback((text: string) => {
    const t = text.trim();
    if (t) post('/api/session/message', { prompt: t });
  }, [post]);

  const quick = useCallback((cmd: QuickCommand) => post(`/api/session/command/${cmd}`), [post]);

  /** The pilot's arm. The AI never arms the vehicle itself. */
  const arm = useCallback(() => post('/api/session/arm'), [post]);

  const simRc = useCallback((on: boolean) => post(`/api/session/sim/rc?on=${on}`), [post]);

  /** Fly a path drawn on the map (validated server-side like any step). */
  const flyPath = useCallback((points: [number, number][], circuit: boolean, alt: number | null) =>
    post('/api/session/path', { points, circuit, alt }), [post]);

  /** Add drawn waypoints to the step that is flying: next, or after its last leg. */
  const addWaypoints = useCallback((points: [number, number][], where: 'next' | 'end') =>
    post('/api/session/insert', { points, where }), [post]);

  return { ...state, busy, error, start, stop, send, quick, arm, simRc, flyPath, addWaypoints };
};
