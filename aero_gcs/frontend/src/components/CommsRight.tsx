import { useState } from 'react';
import { MoreHorizontal, Send, Paperclip, Mic, Loader2 } from 'lucide-react';

const TOGGLES = [
  'Dynamic Wind Adaptation: [Max Gust: 18 kts, Active, Compensation: Active]',
  'Terrain Following: [Active, AGL Set: 80m]',
  'Dynamic Corridor Width: [10m]',
  'Automated Fail-safe Plan: [RTH via WP-B, Alt 50m, Link Loss Strategy]',
  'Dynamic Video Compression: [Enabled, Priority: FPV]',
  'Dynamic Link Priority: [FPV: High, Telemetry: High]',
  'Dynamic Data Stream Optimization: [Active]',
];

/**
 * COMMUNICATIONS panel. The input at the bottom is the natural-language command
 * box: it posts the prompt to the control API, which runs run_pipeline.py.
 */
export default function CommsRight({
  onSend, running, log,
}: {
  onSend: (prompt: string) => void; running: boolean; log: string[];
}) {
  const [on, setOn] = useState<boolean[]>(TOGGLES.map(() => true));
  const [text, setText] = useState('');
  const [showLog, setShowLog] = useState(false);

  const submit = () => {
    if (!text.trim() || running) return;
    onSend(text);
    setText('');
  };

  return (
    <div className="bg-panelWhite rounded-lg shadow-sm border border-borderGray flex flex-col h-full overflow-hidden">
      <div className="flex items-center justify-between px-4 py-2.5 border-b border-borderGray shrink-0">
        <span className="text-sm font-semibold tracking-wide">COMMUNICATIONS</span>
        <button onClick={() => setShowLog((s) => !s)} title="Toggle mission log">
          <MoreHorizontal size={16} className="text-textMuted" />
        </button>
      </div>

      <div className="flex-1 overflow-y-auto px-3 py-2 min-h-0">
        {showLog ? (
          <div className="font-mono text-xxs leading-relaxed text-textMuted whitespace-pre-wrap">
            {log.length ? log.join('\n') : 'no mission output yet'}
          </div>
        ) : (
          on.map((v, i) => (
            <div key={i} className="flex items-start gap-2.5 py-1.5">
              <button
                onClick={() => setOn((p) => p.map((x, j) => (j === i ? !x : x)))}
                className={`w-9 h-5 rounded-full relative shrink-0 mt-0.5 transition-colors
                            ${v ? 'bg-statusGreen' : 'bg-borderGray'}`}
              >
                <span className={`w-4 h-4 bg-white rounded-full absolute top-0.5 shadow transition-all
                                  ${v ? 'right-0.5' : 'left-0.5'}`} />
              </button>
              <span className="text-xxs leading-snug">{TOGGLES[i]}</span>
            </div>
          ))
        )}
      </div>

      {/* natural-language command box */}
      <div className="p-2 border-t border-borderGray shrink-0">
        <div className="flex items-center gap-1.5 bg-bgMain rounded-lg px-3 py-1.5 border border-borderGray">
          <input
            value={text}
            onChange={(e) => setText(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && submit()}
            disabled={running}
            placeholder={running ? 'mission running…' : 'Command the drone…'}
            className="flex-1 bg-transparent outline-none text-xs placeholder:text-textMuted disabled:opacity-50"
          />
          <button onClick={submit} disabled={running || !text.trim()}
                  className="text-aeroCyan disabled:text-textMuted/40" title="Send command">
            {running ? <Loader2 size={17} className="animate-spin" /> : <Send size={17} />}
          </button>
          <button className="text-textMuted" title="Attach mission file"><Paperclip size={17} /></button>
          <button className="text-textMuted" title="Voice command"><Mic size={17} /></button>
        </div>
      </div>
    </div>
  );
}
