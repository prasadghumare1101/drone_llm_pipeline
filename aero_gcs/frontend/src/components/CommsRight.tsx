import { useEffect, useRef, useState } from 'react';
import {
  MoreHorizontal, Send, Loader2, Pause, PlaneLanding, Home, Play, Power, Radio, X,
} from 'lucide-react';
import type { useSession, SessionPhase } from '../hooks/useSession';

type Session = ReturnType<typeof useSession>;

const PHASE_LABEL: Record<SessionPhase, string> = {
  ON_GROUND: 'ON GROUND',
  PLANNING: 'PLANNING…',
  AWAITING_ARM: 'WAITING FOR PILOT ARM',
  EXECUTING: 'EXECUTING STEP',
  HOVERING: 'HOVERING — AWAITING ORDERS',
  RETURNING: 'RETURNING HOME',
  LANDING: 'LANDING',
  RC_OVERRIDE: 'RC OVERRIDE — PILOT HAS CONTROL',
  FAULT: 'VEHICLE FAULT',
};

const PHASE_TONE: Record<SessionPhase, string> = {
  ON_GROUND: 'bg-borderGray text-textMain',
  PLANNING: 'bg-aeroCyan/15 text-aeroCyan',
  AWAITING_ARM: 'bg-amber-100 text-amber-700',
  EXECUTING: 'bg-aeroCyan/15 text-aeroCyan',
  HOVERING: 'bg-statusGreen/15 text-statusGreen',
  RETURNING: 'bg-amber-100 text-amber-700',
  LANDING: 'bg-amber-100 text-amber-700',
  RC_OVERRIDE: 'bg-alertRed/15 text-alertRed',
  FAULT: 'bg-alertRed text-white',
};

/**
 * COMMUNICATIONS panel = the interactive flight chat. Each message is one step
 * from wherever the drone is now; it hovers and asks "What next?" afterwards.
 * The input is never disabled during flight, so "stop" / "land" / "return home"
 * can always be typed - the backend acts on those instantly, without the LLM.
 */
export default function CommsRight({ session, log }: { session: Session; log: string[] }) {
  const [text, setText] = useState('');
  const [showLog, setShowLog] = useState(false);
  const scroller = useRef<HTMLDivElement>(null);
  const s = session;

  useEffect(() => {
    const el = scroller.current;
    if (el) el.scrollTop = el.scrollHeight;
  }, [s.messages.length, showLog]);

  const submit = () => {
    if (!text.trim() || !s.active) return;
    s.send(text);
    setText('');
  };

  const v = s.vehicle;
  const onGround = !v?.in_air;
  const override = s.phase === 'RC_OVERRIDE';
  const btn = 'flex items-center gap-1 px-2 py-1 rounded text-xxs font-bold disabled:opacity-35 transition-colors';

  return (
    <div className="bg-panelWhite rounded-lg shadow-sm border border-borderGray flex flex-col h-full overflow-hidden">
      <div className="flex items-center justify-between gap-2 px-4 py-2.5 border-b border-borderGray shrink-0">
        <span className="text-sm font-semibold tracking-wide">COMMUNICATIONS</span>
        <div className="flex items-center gap-2 min-w-0">
          {s.active && s.phase && (
            <span className={`text-xxs font-bold px-2 py-0.5 rounded truncate ${PHASE_TONE[s.phase]}`}>
              {PHASE_LABEL[s.phase]}
            </span>
          )}
          <button onClick={() => setShowLog((x) => !x)} title="Toggle mission log">
            <MoreHorizontal size={16} className="text-textMuted" />
          </button>
        </div>
      </div>

      {/* conversation / log */}
      <div ref={scroller} className="flex-1 overflow-y-auto px-3 py-2 min-h-0">
        {showLog ? (
          <div className="font-mono text-xxs leading-relaxed text-textMuted whitespace-pre-wrap">
            {log.length ? log.join('\n') : 'no one-shot mission output yet'}
          </div>
        ) : !s.active ? (
          <div className="h-full flex flex-col items-center justify-center gap-3 text-center px-2">
            <div className="text-xs font-semibold">Interactive flight</div>
            <p className="text-xxs text-textMuted leading-snug max-w-[18rem]">
              Each message is one step from where the drone is now. It hovers and asks
              “What next?” after every step. The pilot always arms; no reply for 2 minutes
              sends it home.
            </p>
            <div className="flex flex-wrap justify-center gap-2">
              <button onClick={() => s.start('px4')} disabled={s.busy}
                      className={`${btn} bg-aeroCyan/15 text-aeroCyan hover:bg-aeroCyan/25`}>
                {s.busy ? <Loader2 size={12} className="animate-spin" /> : <Power size={12} />}
                START SESSION · PX4 SITL
              </button>
              <button onClick={() => s.start('sim')} disabled={s.busy}
                      className={`${btn} bg-bgMain text-textMain hover:bg-borderGray`}>
                <Power size={12} /> KINEMATIC SIM
              </button>
            </div>
            {s.error && <div className="text-xxs text-alertRed whitespace-pre-wrap">{s.error}</div>}
          </div>
        ) : (
          <div className="flex flex-col gap-1.5">
            {s.messages.map((m) => (
              <div key={m.id}
                   className={m.role === 'operator' ? 'flex justify-end'
                     : m.role === 'system' ? 'flex justify-center' : 'flex justify-start'}>
                <div className={`max-w-[88%] rounded-lg px-2.5 py-1.5 text-xs leading-snug whitespace-pre-wrap
                  ${m.role === 'operator' ? 'bg-aeroCyan text-white'
                    : m.role === 'system' ? 'bg-transparent text-textMuted text-xxs italic'
                      : 'bg-bgMain text-textMain border border-borderGray'}`}>
                  {m.text}
                </div>
              </div>
            ))}
            {s.phase === 'PLANNING' && (
              <div className="flex items-center gap-1.5 text-xxs text-aeroCyan">
                <Loader2 size={11} className="animate-spin" /> planning…
              </div>
            )}
          </div>
        )}
      </div>

      {s.active && (
        <>
          {/* live status line: watchdog countdown, arm wait, vehicle */}
          <div className="px-3 py-1 border-t border-borderGray shrink-0 flex items-center justify-between gap-2 text-xxs text-textMuted">
            <span className="truncate">
              {s.link.toUpperCase()} · {s.planner} planner
              {v && ` · ${v.alt.toFixed(1)} m · ${v.battery_pct === null ? '–' : v.battery_pct.toFixed(0)}% · ${v.mode}`}
            </span>
            {s.watchdogS !== null && (
              <span className={`shrink-0 font-semibold ${s.watchdogS < 30 ? 'text-alertRed' : ''}`}
                    title="No reply before this reaches 0 = return home">
                auto-home {Math.floor(s.watchdogS / 60)}:{String(Math.floor(s.watchdogS % 60)).padStart(2, '0')}
              </span>
            )}
            {s.armWaitS !== null && (
              <span className="shrink-0 font-semibold text-amber-700">arm within {Math.ceil(s.armWaitS)} s</span>
            )}
          </div>

          {/* instant commands: never go through the LLM */}
          <div className="px-2 pt-1.5 flex flex-wrap gap-1.5 shrink-0">
            <button onClick={() => s.quick('hold')} disabled={override || onGround}
                    className={`${btn} bg-amber-100 text-amber-700 hover:bg-amber-200`} title="Hover in place now">
              <Pause size={11} /> HOLD
            </button>
            <button onClick={() => s.quick('land')} disabled={override || onGround}
                    className={`${btn} bg-alertRed/15 text-alertRed hover:bg-alertRed/25`} title="Land where it is">
              <PlaneLanding size={11} /> LAND
            </button>
            <button onClick={() => s.quick('rtl')} disabled={override || onGround}
                    className={`${btn} bg-alertRed/15 text-alertRed hover:bg-alertRed/25`} title="Return home and land">
              <Home size={11} /> HOME
            </button>
            {(s.canResume || override) && (
              <button onClick={() => s.quick('resume')}
                      className={`${btn} bg-statusGreen/15 text-statusGreen hover:bg-statusGreen/25`}
                      title={override ? 'Hand control back to the AI' : 'Continue the interrupted step'}>
                <Play size={11} /> RESUME
              </button>
            )}
            <button onClick={() => s.arm()} disabled={!onGround || v?.armed || override}
                    className={`${btn} ${s.phase === 'AWAITING_ARM'
                      ? 'bg-amber-500 text-white animate-pulse' : 'bg-bgMain text-textMain hover:bg-borderGray'}`}
                    title="Pilot action: arm the vehicle (the AI never arms)">
              <Power size={11} /> PILOT ARM
            </button>
            {s.link === 'sim' && (
              <button onClick={() => s.simRc(!override)}
                      className={`${btn} bg-bgMain text-textMuted hover:bg-borderGray`}
                      title="Simulate the pilot flipping the RC mode switch">
                <Radio size={11} /> {override ? 'RC HAND BACK' : 'SIM RC TAKEOVER'}
              </button>
            )}
            <button onClick={() => s.stop()} disabled={s.busy}
                    className={`${btn} ml-auto text-textMuted hover:bg-bgMain`} title="End the session">
              <X size={11} /> END
            </button>
          </div>
        </>
      )}

      {/* message box: stays enabled while flying so "stop" can always be typed */}
      <div className="p-2 shrink-0">
        {s.active && s.error && <div className="text-xxs text-alertRed px-1 pb-1 truncate">{s.error}</div>}
        <div className="flex items-center gap-1.5 bg-bgMain rounded-lg px-3 py-1.5 border border-borderGray">
          <input
            value={text}
            onChange={(e) => setText(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && submit()}
            disabled={!s.active}
            placeholder={!s.active ? 'Start a session to fly interactively'
              : s.phase === 'EXECUTING' ? 'Flying… type "stop", "land" or "return home"'
                : 'e.g. take off to 10 m · move 5 m left · survey 40x40 m · land'}
            className="flex-1 bg-transparent outline-none text-xs placeholder:text-textMuted disabled:opacity-50"
          />
          <button onClick={submit} disabled={!s.active || !text.trim()}
                  className="text-aeroCyan disabled:text-textMuted/40" title="Send">
            <Send size={17} />
          </button>
        </div>
      </div>
    </div>
  );
}
