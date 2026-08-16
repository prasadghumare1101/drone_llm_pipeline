import TopBar from './components/TopBar';
import FpvFeed from './components/FpvFeed';
import TelemetryLeft from './components/TelemetryLeft';
import MissionMap from './components/MissionMap';
import MissionRight from './components/MissionRight';
import CommsRight from './components/CommsRight';
import { useROS } from './hooks/useROS';
import { useLLM } from './hooks/useLLM';

export default function App() {
  const { connected, telemetry } = useROS();
  const { llmStatus, waypoints, running, log, sendCommand, abortMission } = useLLM();

  return (
    <div className="h-screen w-screen bg-bgMain text-textMain font-sans flex flex-col overflow-hidden">
      <TopBar connected={connected} telemetry={telemetry} />

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
            <MissionMap t={telemetry} waypoints={waypoints} />
          </div>

          {/* RIGHT: mission automation above communications */}
          <div className="grid gap-3 min-h-0 min-w-0 h-full
                          grid-rows-[minmax(0,1fr)_minmax(0,1fr)]
                          md:row-start-1 md:col-start-2 xl:col-span-3 xl:row-start-1">
            <div className="min-h-0 min-w-0">
              <MissionRight status={llmStatus} waypoints={waypoints}
                            running={running} onAbort={abortMission} />
            </div>
            <div className="min-h-0 min-w-0">
              <CommsRight onSend={sendCommand} running={running} log={log} />
            </div>
          </div>

        </div>
      </div>
    </div>
  );
}
