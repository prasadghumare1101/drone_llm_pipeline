import { useState } from 'react';
import { Play, Square, Server, Loader2, ChevronDown, Eraser, Trash2 } from 'lucide-react';
import { useStack } from '../hooks/useStack';

/**
 * Replaces the five terminals. Starts/stops the uXRCE-DDS agent, PX4 SITL,
 * rosbridge, the telemetry node and the video streamer as managed subprocesses.
 */
export default function StackControl() {
  const { services, apiUp, busy, startAll, stopAll, startOne, stopOne,
          cleanup, purgeLogs } = useStack();
  const [open, setOpen] = useState(false);
  const [note, setNote] = useState('');
  const upCount = services.filter((s) => s.running).length;
  const allUp = upCount === services.length;

  return (
    <div className="relative">
      <div className="flex items-center gap-1.5">
        <button
          onClick={allUp ? stopAll : startAll}
          disabled={!apiUp || busy}
          title={allUp ? 'Stop the simulation stack' : 'Start the simulation stack'}
          className={`flex items-center gap-1.5 px-2.5 py-1 rounded text-xxs font-bold transition-colors
            ${!apiUp ? 'bg-borderGray text-textMuted cursor-not-allowed'
              : allUp ? 'bg-alertRed/15 text-alertRed hover:bg-alertRed/25'
                : 'bg-statusGreen/15 text-statusGreen hover:bg-statusGreen/25'}`}
        >
          {busy ? <Loader2 size={12} className="animate-spin" />
            : allUp ? <Square size={12} /> : <Play size={12} />}
          {busy ? 'WORKING' : allUp ? 'STOP SIM' : 'START SIM'}
        </button>

        <button
          onClick={() => setOpen((o) => !o)}
          className="flex items-center gap-1 px-2 py-1 rounded text-xxs font-bold text-textMuted hover:bg-bgMain"
          title="Show service status"
        >
          <Server size={12} />
          {apiUp ? `${upCount}/${services.length}` : 'API'}
          <ChevronDown size={11} className={open ? 'rotate-180 transition-transform' : 'transition-transform'} />
        </button>
      </div>

      {open && (
        <div className="absolute right-0 top-9 w-72 bg-panelWhite border border-borderGray rounded-lg shadow-lg z-50 p-2">
          {!apiUp && (
            <div className="text-xxs text-alertRed font-semibold px-2 py-1.5 leading-snug">
              Control API offline — run:<br />
              <span className="font-mono">python3 backend_services/control_api.py</span>
            </div>
          )}
          {services.map((s) => (
            <div key={s.name} className="flex items-center gap-2 px-2 py-1.5 hover:bg-bgMain rounded">
              <span className={`w-1.5 h-1.5 rounded-full shrink-0 ${s.running ? 'bg-statusGreen' : 'bg-borderGray'}`} />
              <div className="flex-1 min-w-0">
                <div className="text-xxs font-semibold truncate">{s.label}</div>
                {s.detail && <div className="text-xxs text-textMuted truncate">{s.detail}</div>}
              </div>
              <button
                onClick={() => (s.running ? stopOne(s.name) : startOne(s.name))}
                disabled={!apiUp || busy}
                className={`text-xxs font-bold px-1.5 py-0.5 rounded shrink-0
                  ${s.running ? 'text-alertRed hover:bg-alertRed/10' : 'text-statusGreen hover:bg-statusGreen/10'}`}
              >
                {s.running ? 'STOP' : 'START'}
              </button>
            </div>
          ))}

          <div className="border-t border-borderGray mt-1.5 pt-1.5">
            <button
              onClick={async () => setNote(await cleanup())}
              disabled={!apiUp || busy}
              className="w-full flex items-center gap-2 px-2 py-1.5 rounded text-xxs
                         font-semibold text-textMain hover:bg-bgMain disabled:opacity-40"
              title="Kill orphaned gzserver/px4 and clear Gazebo scratch"
            >
              <Eraser size={12} className="text-aeroCyan" />
              Clean stale sim state
            </button>
            <button
              onClick={async () => setNote(await purgeLogs())}
              disabled={!apiUp || busy}
              className="w-full flex items-center gap-2 px-2 py-1.5 rounded text-xxs
                         font-semibold text-textMain hover:bg-bgMain disabled:opacity-40"
              title="Delete ROS log history (asks for confirmation)"
            >
              <Trash2 size={12} className="text-alertRed" />
              Purge ROS logs…
            </button>
            <div className="px-2 pt-1 text-xxs text-textMuted leading-snug">
              {note || 'Models and the serial hardware link are never touched.'}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
