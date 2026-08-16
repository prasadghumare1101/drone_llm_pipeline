import { useState, useEffect } from 'react';
import { MoreHorizontal, Camera, Plus, Minus } from 'lucide-react';
import HudStrip from './HudStrip';
import type { Telemetry } from '../hooks/useROS';

const VIDEO_URL = 'http://localhost:8080/video_feed';

/**
 * Live FPV panel. The MJPEG stream is bound straight to an <img> src so frames
 * never pass through React state - that is what keeps memory flat.
 */
export default function FpvFeed({ t }: { t: Telemetry }) {
  const [zoom, setZoom] = useState(50);
  const [recording, setRecording] = useState(false);
  const [alive, setAlive] = useState(true);
  // Bumped on each retry to bust the cached failed request. Without this the
  // panel would stay dead until a page refresh if the streamer starts late.
  const [attempt, setAttempt] = useState(0);

  useEffect(() => {
    if (alive) return;
    const t = setTimeout(() => { setAttempt((a) => a + 1); setAlive(true); }, 4000);
    return () => clearTimeout(t);
  }, [alive]);

  return (
    <div className="bg-panelWhite rounded-lg shadow-sm border border-borderGray flex flex-col h-full overflow-hidden">
      <div className="flex items-center justify-between px-3 py-2 border-b border-borderGray shrink-0">
        <span className="text-sm font-semibold tracking-wide">FPV FEED</span>
        <MoreHorizontal size={16} className="text-textMuted" />
      </div>

      <HudStrip t={t} compact />

      <div className="flex-1 relative bg-black min-h-0 overflow-hidden">
        {alive ? (
          <img
            key={attempt}
            src={attempt ? `${VIDEO_URL}?r=${attempt}` : VIDEO_URL}
            alt="FPV feed"
            className="w-full h-full object-cover"
            style={{ transform: `scale(${1 + zoom / 100})` }}
            onError={() => setAlive(false)}
          />
        ) : (
          <div className="absolute inset-0 flex flex-col items-center justify-center gap-1 text-white/40 text-xxs text-center px-3">
            <Camera size={20} />
            WAITING FOR VIDEO STREAM
            <span className="text-white/25">start the Video Streamer service</span>
          </div>
        )}

        {/* zoom rail */}
        <div className="absolute left-2 top-2 bottom-2 flex flex-col items-center gap-1
                        bg-panelWhite/85 backdrop-blur-sm rounded-md px-1 py-1.5 shadow">
          <Plus size={11} className="text-textMain" />
          <input type="range" min={0} max={100} value={zoom}
                 onChange={(e) => setZoom(+e.target.value)}
                 className="aero-vslider flex-1" />
          <Minus size={11} className="text-textMain" />
        </div>

        {/* camera + record */}
        <button className="absolute right-2 top-2 bg-panelWhite/85 backdrop-blur-sm rounded-md p-1.5 shadow">
          <Camera size={13} className="text-textMain" />
        </button>
        <button
          onClick={() => setRecording((r) => !r)}
          title={recording ? 'Stop recording' : 'Start recording'}
          className="absolute right-2 bottom-2 bg-panelWhite/85 backdrop-blur-sm rounded-full p-1.5 shadow"
        >
          <span className={`block w-3.5 h-3.5 bg-alertRed ${recording ? 'rounded-sm animate-pulse' : 'rounded-full'}`} />
        </button>
      </div>
    </div>
  );
}
