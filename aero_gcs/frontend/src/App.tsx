import TopBar from './components/TopBar';
import FpvFeed from './components/FpvFeed';
import TelemetryLeft from './components/TelemetryLeft';
import MissionMap from './components/MissionMap';
import MissionRight from './components/MissionRight';
import CommsRight from './components/CommsRight';
import { useROS, type Telemetry } from './hooks/useROS';
import { useLLM } from './hooks/useLLM';
import { useSession } from './hooks/useSession';

const PHASE_STATUS: Record<string, string> = {
  ON_GROUND: 'INTERACTIVE SESSION - ON GROUND',
  PLANNING: 'LLM PLANNING NEXT STEP…',
  AWAITING_ARM: 'STEP VALIDATED - WAITING FOR PILOT TO ARM',
  EXECUTING: 'EXECUTING VALIDATED STEP',
  HOVERING: 'STEP DONE - HOVERING, AWAITING NEXT ORDER',
  RETURNING: 'RETURNING HOME',
  LANDING: 'LANDING',
  RC_OVERRIDE: 'RC OVERRIDE - PILOT HAS CONTROL',
  FAULT: 'VEHICLE FAULT - AI STOPPED',
};

export default function App() {
  const ros = useROS();
  const { llmStatus, waypoints, running, log, abortMission } = useLLM();
  const session = useSession();

  // An active interactive session owns the mission panel and the map.
  const live = session.active;
  const shownWaypoints = live ? session.waypoints : waypoints;
  const status = live && session.phase
    ? `${PHASE_STATUS[session.phase]}${session.stepText ? `: "${session.stepText}"` : ''}`
    : llmStatus;
  const stepRunning = live ? session.phase === 'EXECUTING' : running;

  // The kinematic sim has no ROS telemetry; show the session's vehicle instead.
  const v = session.vehicle;
  const connected = ros.connected;
  const telemetry: Telemetry = !connected && live && v ? {
    ...ros.telemetry,
    altitude: v.alt, heading: v.heading_deg, x: v.x, y: v.y,
    battery: v.battery_pct ?? 0, armed: v.armed,
    flight_mode: `${v.mode}${v.armed ? '' : ' (DISARMED)'} [SIM]`,
  } : ros.telemetry;

  return (
    <div className="h-screen w-screen bg-bgMain text-textMain font-sans flex flex-col overflow-hidden">
      <TopBar connected={connected} telemetry={telemetry}
              onReplay={live ? (points, alt) => session.flyPath(points, false, alt) : undefined} />

      <div className="px-3 pt-1 shrink-0 text-right">
        <span className="text-xxs font-semibold tracking-widest text-textMuted">
          ULTRA-LIGHTWEIGHT / LOW LATENCY LINK
        </span>
      </div>

      {/*
        Responsive layout.
          >=1280px : the reference 3-column GCS layout (3 / 6 / 3 of 12)
          >=768px  : two columns, map spans the full width on its own row
          <768px   : single column, panels stack and the page scrolls
        Fractional grid ROWS (not % heights) keep every panel proportional at
        any window size, and min-h-0/min-w-0 stop flex children overflowing.
      */}
      <div className="flex-1 min-h-0 p-3 pt-2 overflow-auto xl:overflow-hidden">
        <div className="grid gap-3 h-full min-h-0
                        grid-cols-1
                        md:grid-cols-2 md:grid-rows-[minmax(0,1fr)_minmax(0,1fr)]
                        xl:grid-cols-12 xl:grid-rows-1">

          {/* LEFT: FPV above telemetry */}
          <div className="grid gap-3 min-h-0 min-w-0 h-full
                          grid-rows-[minmax(0,1fr)_minmax(0,2fr)]
                          md:row-span-1 xl:col-span-3">
            <div className="min-h-0 min-w-0"><FpvFeed t={telemetry} /></div>
            <div className="min-h-0 min-w-0"><TelemetryLeft t={telemetry} /></div>
          </div>

          {/* CENTER: mission map */}
          <div className="min-h-0 min-w-0 h-full
                          md:col-span-2 md:row-start-2 xl:col-span-6 xl:row-start-1">
            <MissionMap t={telemetry} waypoints={shownWaypoints}
                        planner={live ? {
                          executing: session.phase === 'EXECUTING',
                          flyPath: session.flyPath,
                          addWaypoints: session.addWaypoints,
                        } : undefined} />
          </div>

          {/* RIGHT: mission automation above communications */}
          <div className="grid gap-3 min-h-0 min-w-0 h-full
                          grid-rows-[minmax(0,1fr)_minmax(0,1fr)]
                          md:row-start-1 md:col-start-2 xl:col-span-3 xl:row-start-1">
            <div className="min-h-0 min-w-0">
              <MissionRight status={status} waypoints={shownWaypoints} running={stepRunning}
                            onAbort={live ? () => session.quick('hold') : abortMission} />
            </div>
            <div className="min-h-0 min-w-0">
              <CommsRight session={session} log={log} />
            </div>
          </div>

        </div>
      </div>
    </div>
  );
}
