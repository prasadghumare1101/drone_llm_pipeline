import { MoreHorizontal, Plane, Navigation, Gauge, BatteryFull, SignalHigh, MapPin, Waypoints } from 'lucide-react';
import type { Telemetry } from '../hooks/useROS';

/**
 * TELEMETRY panel - left column, below the FPV feed.
 * Every row is a self-contained grid cell: no negative margins, no absolute
 * offsets, so the panel reflows cleanly at any window size.
 */
export default function TelemetryLeft({ t }: { t: Telemetry }) {
  /** Stacked row: big value under the label (Altitude / Heading / Speed). */
  const Stat = ({ icon, label, value }: {
    icon: React.ReactNode; label: string; value: string;
  }) => (
    <div className="flex items-center gap-3 py-[max(0.5rem,1.2cqh)] border-b border-borderGray min-w-0">
      <span className="text-aeroCyan shrink-0">{icon}</span>
      <div className="min-w-0">
        <div className="text-[clamp(0.7rem,1.5cqw,0.95rem)] text-textMain truncate">{label}</div>
        <div className="text-[clamp(0.95rem,2.4cqw,1.4rem)] font-semibold tabular-nums truncate">
          {value}
        </div>
      </div>
    </div>
  );

  /** Paired row: two label/value lines (Battery, Signal, GPS). */
  const Pair = ({ icon, l1, v1, l2, v2 }: {
    icon: React.ReactNode; l1: string; v1: string; l2: string; v2: string;
  }) => (
    <div className="flex items-start gap-3 py-[max(0.5rem,1.2cqh)] border-b border-borderGray last:border-0 min-w-0">
      <span className="text-statusGreen shrink-0 mt-0.5">{icon}</span>
      <div className="flex-1 min-w-0 text-[clamp(0.7rem,1.5cqw,0.95rem)]">
        <div className="flex items-baseline justify-between gap-2">
          <span className="text-textMain truncate">{l1}</span>
          <span className="font-semibold tabular-nums shrink-0">{v1}</span>
        </div>
        <div className="flex items-baseline justify-between gap-2">
          <span className="text-textMain truncate">{l2}</span>
          <span className="font-semibold tabular-nums shrink-0">{v2}</span>
        </div>
      </div>
    </div>
  );

  const heading = `${String(Math.round(t.heading)).padStart(3, '0')}° N`;
  const ico = 'clamp(16px, 2vw, 22px)';

  return (
    <div className="bg-panelWhite rounded-lg shadow-sm border border-borderGray
                    flex flex-col h-full overflow-hidden [container-type:size]">
      <div className="flex items-center justify-between px-3 py-2 border-b border-borderGray shrink-0">
        <span className="text-[clamp(0.7rem,1.6cqw,0.9rem)] font-semibold tracking-wide">
          TELEMETRY
        </span>
        <MoreHorizontal size={16} className="text-textMuted shrink-0" />
      </div>

      <div className="flex-1 overflow-y-auto px-3 min-h-0">
        <Stat icon={<Plane size={ico} />} label="Altitude" value={`${t.altitude.toFixed(1)} m`} />
        <Stat icon={<Navigation size={ico} />} label="Heading" value={heading} />
        <Stat icon={<Gauge size={ico} />} label="Speed" value={`${t.speed.toFixed(1)} km/h`} />
        <Pair icon={<BatteryFull size={ico} />}
              l1="Battery Status" v1={`${t.battery.toFixed(0)}%`}
              l2="Temperature" v2={`${t.battery_temp.toFixed(0)}°C`} />
        <Pair icon={<SignalHigh size={ico} />}
              l1="Signal Strength" v1={`RC: ${t.signal_rc.toFixed(0)}%`}
              l2="" v2={`FPV: ${t.signal_fpv.toFixed(0)}%`} />
        <Pair icon={<MapPin size={ico} className="text-aeroCyan" />}
              l1="GPS Lat/Lon" v1={`${t.lat.toFixed(4)}° ${t.lat >= 0 ? 'N' : 'S'}`}
              l2="" v2={`${Math.abs(t.lon).toFixed(4)}° ${t.lon >= 0 ? 'E' : 'W'}`} />
        <Stat icon={<Waypoints size={ico} />} label="Flight Mode" value={t.flight_mode} />
      </div>
    </div>
  );
}
