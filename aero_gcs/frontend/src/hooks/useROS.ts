import { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';

/**
 * Live vehicle state. Mirrors the JSON published by
 * backend_services/telemetry_node.py on /gcs/consolidated_telemetry.
 * One consolidated 10 Hz topic -> one setState per tick (no per-field churn).
 */
export interface Telemetry {
  altitude: number;
  heading: number;
  speed: number;
  climb_rate: number;
  battery: number;
  battery_voltage: number;
  battery_temp: number;
  battery_minutes: number;
  signal_rc: number;
  signal_fpv: number;
  lat: number;
  lon: number;
  satellites: number;
  gps_ok: boolean;
  armed: boolean;
  flight_mode: string;
  /** Local ENU position of the vehicle, metres from home. Drives the map marker. */
  x: number;
  y: number;
}

export const DEFAULT_TELEMETRY: Telemetry = {
  altitude: 0, heading: 0, speed: 0, climb_rate: 0,
  battery: 0, battery_voltage: 0, battery_temp: 0, battery_minutes: 0,
  signal_rc: 0, signal_fpv: 0,
  lat: 0, lon: 0, satellites: 0, gps_ok: false,
  armed: false, flight_mode: 'DISCONNECTED',
  x: 0, y: 0,
};

const ROSBRIDGE_URL = 'ws://localhost:9090';

export const useROS = () => {
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState<Telemetry>(DEFAULT_TELEMETRY);

  // Latest message is stashed in a ref and flushed to state on a fixed 10 Hz
  // cadence. This decouples React re-renders from DDS burst rates, which is
  // what keeps the UI light when PX4 publishes faster than we can paint.
  const latest = useRef<Telemetry | null>(null);

  useEffect(() => {
    let disposed = false;
    let ros: any = null;
    let topic: any = null;
    let retry: ReturnType<typeof setTimeout> | null = null;

    const connect = () => {
      if (disposed) return;
      ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

      ros.on('connection', () => { if (!disposed) setConnected(true); });

      // rosbridge emits 'error' on a failed dial; swallow it so the browser
      // console stays clean, the 'close' handler owns reconnection.
      ros.on('error', () => { /* handled by close */ });

      ros.on('close', () => {
        if (disposed) return;
        setConnected(false);
        setTelemetry(DEFAULT_TELEMETRY);
        retry = setTimeout(connect, 2000);   // auto-reconnect
      });

      topic = new ROSLIB.Topic({
        ros,
        name: '/gcs/consolidated_telemetry',
        messageType: 'std_msgs/String',
        throttle_rate: 100,     // server-side cap: 10 Hz
        queue_length: 1,        // never buffer stale frames
      });

      topic.subscribe((message: any) => {
        try {
          latest.current = { ...DEFAULT_TELEMETRY, ...JSON.parse(message.data) };
        } catch {
          /* ignore malformed frame */
        }
      });
    };

    connect();

    const flush = setInterval(() => {
      if (latest.current) {
        setTelemetry(latest.current);
        latest.current = null;
      }
    }, 100);

    return () => {
      disposed = true;
      clearInterval(flush);
      if (retry) clearTimeout(retry);
      try { topic?.unsubscribe(); } catch { /* already gone */ }
      try { ros?.close(); } catch { /* already gone */ }
    };
  }, []);

  return { connected, telemetry };
};
