/**
 * Where the backend services live. Defaults to the machine that served this
 * page, so the same build works on the sim machine itself (localhost) and from
 * another computer on the network (e.g. a Mac browser -> the Linux sim box).
 * Override with VITE_AERO_HOST at build / dev time.
 */
const HOST: string = import.meta.env.VITE_AERO_HOST || window.location.hostname || 'localhost';

export const API_BASE = `http://${HOST}:8000`;        // control_api.py
export const ROSBRIDGE_URL = `ws://${HOST}:9090`;      // rosbridge (telemetry)
export const VIDEO_URL = `http://${HOST}:8080/video_feed`;   // video_streamer.py
