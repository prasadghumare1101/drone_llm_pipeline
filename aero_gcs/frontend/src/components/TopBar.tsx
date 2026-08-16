import { useEffect, useState } from 'react';
import { ChevronDown, Wifi, BatteryFull, SignalHigh } from 'lucide-react';
import StackControl from './StackControl';
import WorldSelect from './WorldSelect';
import type { Telemetry } from '../hooks/useROS';

function DroneGlyph() {
  return (
    <svg className="w-5 h-5 shrink-0" viewBox="0 0 24 24" fill="none"
         stroke="#0CA5E9" strokeWidth="1.8" strokeLinecap="round">
      <circle cx="5" cy="5" r="2.4" /><circle cx="19" cy="5" r="2.4" />
      <circle cx="5" cy="19" r="2.4" /><circle cx="19" cy="19" r="2.4" />
      <path d="M6.7 6.7 L9.5 9.5 M17.3 6.7 L14.5 9.5 M6.7 17.3 L9.5 14.5 M17.3 17.3 L14.5 14.5" />
      <rect x="9" y="9" width="6" height="6" rx="1.4" />
    </svg>
  );
}

export default function TopBar({ connected, telemetry }: { connected: boolean; telemetry: Telemetry }) {
  const [clock, setClock] = useState('--:--');

  useEffect(() => {
    const tick = () => setClock(
      new Date().toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit' }),
    );
    tick();
    const t = setInterval(tick, 10000);
    return () => clearInterval(t);
  }, []);

  return (
    // NOTE: no `overflow-hidden` here. The StackControl / WorldSelect popovers are
    // absolutely positioned children, and clipping this bar would hide them.
    // Children truncate via min-w-0 instead.
    <div className="h-14 w-full bg-panelWhite border-b border-borderGray flex items-center
                    gap-2 px-3 shrink-0 shadow-sm relative z-30">
      {/* left: vehicle id + link state + sim controls. Shrinks before the title. */}
      <div className="flex items-center gap-2 min-w-0 shrink">
        <DroneGlyph />
        <span className="font-bold text-[clamp(0.9rem,1.4vw,1.125rem)] tracking-tight">AM-4-003</span>
        <ChevronDown size={15} className="text-textMuted -ml-1 shrink-0 hidden sm:block" />
        <div className={`flex items-center gap-1.5 px-2 py-1 rounded-full text-xxs font-bold border shrink-0
          ${connected
            ? 'bg-panelWhite text-textMain border-borderGray'
            : 'bg-alertRed/10 text-alertRed border-alertRed/30'}`}>
          <span className={`w-2 h-2 rounded-full ${connected ? 'bg-statusGreen' : 'bg-alertRed'}`} />
          <span className="hidden sm:inline">{connected ? 'CONNECTED' : 'DISCONNECTED'}</span>
        </div>
        <WorldSelect />
        <StackControl />
      </div>

      {/* centre: title. Truncates rather than pushing the bar out of shape. */}
      <div className="flex-1 text-center min-w-0 px-2 hidden lg:block">
        <span className="text-[clamp(0.8rem,1.5vw,1.25rem)] tracking-wide truncate inline-block max-w-full align-bottom">
          <span className="text-aeroCyan font-bold">AEROMAST</span>
          <span className="text-textMain font-medium"> FLIGHT CONTROL CENTER</span>
        </span>
      </div>

      {/* right: link + power + clock. Non-essential items drop out first. */}
      <div className="flex items-center justify-end gap-2 text-textMuted ml-auto shrink-0">
        <Wifi size={17} className={connected ? 'text-textMain' : 'text-borderGray'} />
        <BatteryFull size={19}
                     className={telemetry.battery > 20 ? 'text-statusGreen' : 'text-alertRed'} />
        <span className="text-borderGray hidden sm:inline">|</span>
        <span className="text-[clamp(0.85rem,1.2vw,1.1rem)] font-medium text-textMain tabular-nums">
          {clock}
        </span>
        <div className="hidden xl:flex items-end gap-1">
          <span className="text-xxs align-super">{telemetry.signal_rc || 0}</span>
          <SignalHigh size={15} className="text-textMain" />
          <span className="text-xxs align-super">5G</span>
          <SignalHigh size={15} className="text-textMain" />
          <span className="text-sm font-medium text-textMain">5G</span>
        </div>
      </div>
    </div>
  );
}
