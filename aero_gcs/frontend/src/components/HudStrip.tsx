import { Plane, Gauge, TrendingUp, BatteryCharging, MapPin } from 'lucide-react';
import type { Telemetry } from '../hooks/useROS';

/** The metric strip shown above the FPV feed and the mission map. */
export default function HudStrip({ t, compact = false }: { t: Telemetry; compact?: boolean }) {
  const iconSize = compact ? 12 : 22;
  const Cell = ({ icon, label, value, sub, green }: any) => (
    <div className={`flex items-center gap-2 flex-1 min-w-0 ${compact ? 'px-1.5' : 'px-3'}`}>
      <span className={green ? 'text-statusGreen shrink-0' : 'text-aeroCyan shrink-0'}>{icon}</span>
      <div className="min-w-0 leading-tight">
        <div className={`${compact ? 'text-xxs' : 'text-sm'} text-textMuted truncate`}>{label}</div>
        <div className={`${compact ? 'text-xxs' : 'text-lg'} font-semibold truncate
                        ${green ? 'text-statusGreen' : 'text-textMain'}`}>{value}</div>
        {sub && <div className="text-xxs text-textMuted truncate">{sub}</div>}
      </div>
    </div>
  );

  return (
    <div className={`flex items-stretch divide-x divide-borderGray bg-panelWhite shrink-0
                     ${compact ? 'py-1' : 'py-2'}`}>
      <Cell icon={<Plane size={iconSize} />} label="Altitude" value={`${t.altitude.toFixed(0)}m`} />
      <Cell icon={<Gauge size={iconSize} />} label="Speed" value={`${t.speed.toFixed(0)} km/h`} />
      <Cell icon={<TrendingUp size={iconSize} />} label="Climb rate" value={`${t.climb_rate.toFixed(1)} m/s`} />
      <Cell icon={<BatteryCharging size={iconSize} />} green
            label={`${t.battery.toFixed(0)}%, ${t.battery_voltage.toFixed(1)}V`}
            value={compact ? `${t.battery_minutes} min left` : ''}
            sub={compact ? '' : `${t.battery_minutes} min left`} />
      <Cell icon={<MapPin size={iconSize} />} green={t.gps_ok}
            label={t.gps_ok ? 'Connected' : 'No GPS'}
            value={compact ? `${t.satellites} satellites` : ''}
            sub={compact ? '' : `${t.satellites} satellites`} />
    </div>
  );
}
