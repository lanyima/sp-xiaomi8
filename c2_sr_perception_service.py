#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
openpilot_toolbox - Universal Real-Time Perception Telemetry Bridge Service
Compatible with Comma 2, Comma 3, Comma 3X across DP, SP098, SP2025, SP2026, CP.

Zero-Interference Shared Memory (SHM) Architecture:
1. Passive Zero-Impact SHM Readers: Reads /dev/shm memory ring buffers directly in read-only mode (O_RDONLY).
   - NEVER calls msgq_init_subscriber() or cereal SubMaster/sub_sock.
   - NEVER increments num_readers (prevents reaching NUM_READERS=15 limit and evicting openpilot readers).
   - NEVER receives or handles SIGUSR2 inter-processor interrupts (IPI).
   - Publishers (card, selfdrived, modeld) have ABSOLUTELY ZERO awareness of this reader.
2. Strict Core Isolation: Forcefully pinned to CPU Core 0 (background/little core).
3. Priority Isolation: nice(19) ensures lowest scheduling priority in CFS.
4. Microsecond Performance: Incremental pointer tracking (< 0.002ms per read).
5. Full Telemetry Pipeline:
   - Vehicle dynamics from carState
   - Extra speed limits from carStateSP (if present)
   - Control & cruise state from selfdriveState / controlsState
   - 3D Perception from modelV2 (4 lane lines, road edges, path, vision leads)
   - Radar targets from radarState
"""

import os
import sys
import time
import json
import mmap
import struct
import signal
import threading
from typing import Dict, Any, List, Optional
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

OPENPILOT_DIR = os.environ.get('OPENPILOT_DIR', '/data/openpilot')
if OPENPILOT_DIR not in sys.path:
    sys.path.insert(0, OPENPILOT_DIR)

DEFAULT_HOST = '0.0.0.0'
DEFAULT_PORT = 7788


def setup_process_isolation() -> None:
    """Isolate process completely from real-time openpilot cores."""
    try:
        os.nice(19)
    except Exception:
        pass

    try:
        if hasattr(os, 'sched_setaffinity'):
            os.sched_setaffinity(0, {0})
    except Exception:
        pass


def safe_float(value: Any, decimals: int = 2) -> Optional[float]:
    try:
        val = float(value)
        return round(val, decimals)
    except (ValueError, TypeError):
        return None


import re

def detect_system_base() -> int:
    """Detect msgq header base offset (sizeof(msgq_header_t))."""
    for header_path in (
        '/data/openpilot/msgq_repo/msgq/msgq.h',
        '/data/openpilot/cereal/messaging/msgq.h',
        '/data/openpilot/msgq/msgq.h',
    ):
        if os.path.isfile(header_path):
            try:
                with open(header_path, 'r', errors='ignore') as f:
                    content = f.read()
                m = re.search(r'#define\s+NUM_READERS\s+(\d+)', content)
                if m:
                    num_readers = int(m.group(1))
                    # struct msgq_header_t: 3 uint64 (24 bytes) + 3 * NUM_READERS * 8 bytes
                    return 24 + num_readers * 24
            except Exception:
                pass
    return 384


class ZeroImpactShmReader:
    """Passively reads openpilot cereal shared memory without registering as an MSGQ subscriber."""

    _SYSTEM_BASE: Optional[int] = None

    def __init__(self, topic: str):
        self.topic = topic
        self._f = None
        self._mm = None
        self._p = 0
        self._base: Optional[int] = None
        self._path: Optional[str] = None

    @classmethod
    def get_system_base(cls) -> int:
        if cls._SYSTEM_BASE is None:
            cls._SYSTEM_BASE = detect_system_base()
        return cls._SYSTEM_BASE

    def _get_path(self) -> Optional[str]:
        if self._path and os.path.exists(self._path):
            return self._path
        for candidate in (f'/dev/shm/msgq_{self.topic}', f'/dev/shm/{self.topic}'):
            if os.path.exists(candidate):
                self._path = candidate
                return candidate
        return None

    def _resolve_base(self, wp: int) -> int:
        if self._base is not None:
            return self._base
        default_base = self.get_system_base()
        candidate_bases = [default_base]
        for b in (456, 576, 384):
            if b not in candidate_bases:
                candidate_bases.append(b)
        for cand in candidate_bases:
            try:
                sz, = struct.unpack('<q', self._mm[cand : cand + 8])
                if 0 < sz <= 5000000:
                    self._base = cand
                    return cand
            except Exception:
                pass
        self._base = default_base
        return default_base

    def read_latest(self) -> Optional[bytes]:
        try:
            if self._f is None or self._mm is None:
                path = self._get_path()
                if not path:
                    return None
                self._f = open(path, 'rb')
                self._mm = mmap.mmap(self._f.fileno(), 0, prot=mmap.PROT_READ)
                self._p = 0
                self._base = None

            wp_raw, = struct.unpack('<Q', self._mm[8:16])
            wp = wp_raw & 0xFFFFFFFF
            base = self._resolve_base(wp)

            # Handle wraparound
            if self._p > wp:
                self._p = 0

            last_pos = None
            last_sz = 0
            while self._p + 8 < wp:
                size, = struct.unpack('<q', self._mm[base + self._p : base + self._p + 8])
                if size <= 0 or size > 5000000:
                    if last_pos is None and self._p == 0:
                        for alt in (456, 384, 576):
                            if alt == base:
                                continue
                            try:
                                alt_sz, = struct.unpack('<q', self._mm[alt : alt + 8])
                                if 0 < alt_sz <= 5000000:
                                    self._base = alt
                                    base = alt
                                    break
                            except Exception:
                                pass
                        if self._base != base:
                            continue
                    break
                last_pos = self._p
                last_sz = size
                self._p += 8 + size
                self._p = (self._p + 7) & ~7

            if last_pos is not None:
                return self._mm[base + last_pos + 8 : base + last_pos + 8 + last_sz]
        except Exception:
            if self._f:
                try: self._f.close()
                except Exception: pass
                self._f = None
                self._mm = None
                self._base = None
        return None


class PerceptionBridge:
    """Zero-overhead perception telemetry bridge."""

    def __init__(self):
        self._lock = threading.Lock()
        self._latest_payload: str = '{}'
        self._sequence = 0
        self._running = True
        self._active_clients = 0
        self._has_had_client = False
        self._start_time = time.monotonic()
        self._last_client_leave_time: Optional[float] = None

        # Cached state
        self._cached_cs: Dict[str, Any] = {}
        self._cached_ctrl: Dict[str, Any] = {}
        self._cached_leads: List[Dict[str, Any]] = []
        self._cached_lanes: List[Any] = []
        self._cached_lane_probs: List[Any] = []
        self._cached_edges: List[Any] = []
        self._cached_path: List[Any] = []
        self._cached_sys: Dict[str, Any] = {}

    def register_client(self) -> None:
        with self._lock:
            self._active_clients += 1
            self._has_had_client = True
            self._last_client_leave_time = None

    def unregister_client(self) -> None:
        with self._lock:
            self._active_clients = max(0, self._active_clients - 1)
            if self._active_clients == 0:
                self._last_client_leave_time = time.monotonic()

    def get_latest_payload(self) -> tuple:
        with self._lock:
            return self._sequence, self._latest_payload

    def _extract_poly(self, obj: Any, step: int = 2) -> List[List[Optional[float]]]:
        try:
            x_arr = obj.x
            y_arr = obj.y
            z_arr = obj.z
            length = min(len(x_arr), len(y_arr), len(z_arr))
            return [
                [round(float(x_arr[i]), 2), round(float(y_arr[i]), 2), round(float(z_arr[i]), 2)]
                for i in range(0, length, step)
            ]
        except Exception:
            return []

    def run(self) -> None:
        setup_process_isolation()

        # Import log_from_bytes for parsing raw capnp buffers
        log_from_bytes = None
        for _ in range(30):
            if not self._running:
                return
            try:
                from cereal.messaging import log_from_bytes as _lfb
                log_from_bytes = _lfb
                break
            except Exception:
                try:
                    from cereal import log as _c_log
                    log_from_bytes = _c_log.Event.from_bytes
                    break
                except Exception:
                    pass
                time.sleep(1.0)

        if log_from_bytes is None:
            print("[perception] cereal log_from_bytes unavailable, exiting.", flush=True)
            return

        # Initialize passive readers (zero IPC registration, 100% read-only)
        cs_reader = ZeroImpactShmReader('carState')
        cs_sp_reader = ZeroImpactShmReader('carStateSP')
        ctrl_reader = ZeroImpactShmReader('selfdriveState')
        ctrl_fallback_reader = ZeroImpactShmReader('controlsState')
        mv2_reader = ZeroImpactShmReader('modelV2')
        radar_reader = ZeroImpactShmReader('radarState')
        dev_reader = ZeroImpactShmReader('deviceState')

        gear_map = {
            'park': 'P', 'p': 'P',
            'reverse': 'R', 'r': 'R',
            'neutral': 'N', 'n': 'N',
            'drive': 'D', 'd': 'D',
            'sport': 'D', 's': 'D',
            'eco': 'D', 'e': 'D',
            'low': 'D', 'l': 'D',
            'brake': 'D', 'b': 'D',
            'manumatic': 'D', 'm': 'D'
        }

        print("[perception] Passive Zero-Impact SHM Bridge started on Core 0.", flush=True)

        while self._running:
            loop_start = time.monotonic()

            # Auto-shutdown watchdog: exit if client leaves or never arrives
            with self._lock:
                clients = self._active_clients
                now = time.monotonic()
                has_had = self._has_had_client
                leave_time = self._last_client_leave_time

            if clients == 0:
                if not has_had and (now - self._start_time > 60.0):
                    print("[perception] No client connected within 60s, auto-exiting.", flush=True)
                    self._running = False
                    os._exit(0)
                elif has_had and leave_time is not None and (now - leave_time > 5.0):
                    print("[perception] All clients disconnected (> 5s), auto-exiting cleanly.", flush=True)
                    self._running = False
                    os._exit(0)

            # 1. Car State
            raw_cs = cs_reader.read_latest()
            if raw_cs:
                try:
                    evt = log_from_bytes(raw_cs)
                    cs = evt.carState
                    raw_gear = str(getattr(cs, 'gearShifter', '')).lower()
                    norm_gear = gear_map.get(raw_gear)
                    if not norm_gear:
                        norm_gear = 'D' if getattr(cs, 'vEgo', 0.0) > 0.5 else 'P'

                    v_cruise = None
                    if hasattr(cs, 'vCruise'):
                        v_cruise = safe_float(cs.vCruise, 1)
                    elif hasattr(cs, 'cruiseState'):
                        v_cruise = safe_float(getattr(cs.cruiseState, 'speed', None), 1)

                    self._cached_cs = {
                        'vEgo': safe_float(cs.vEgo, 3),
                        'aEgo': safe_float(cs.aEgo, 3),
                        'steeringAngleDeg': safe_float(cs.steeringAngleDeg, 2),
                        'steeringTorque': safe_float(cs.steeringTorque, 2),
                        'gasPressed': bool(cs.gasPressed),
                        'brakePressed': bool(cs.brakePressed),
                        'leftBlinker': bool(cs.leftBlinker),
                        'rightBlinker': bool(cs.rightBlinker),
                        'leftBlindspot': bool(getattr(cs, 'leftBlindspot', False)),
                        'rightBlindspot': bool(getattr(cs, 'rightBlindspot', False)),
                        'gearShifter': norm_gear,
                        'vCruiseCarState': v_cruise,
                    }
                except Exception:
                    pass

            # Optional speedLimit from carStateSP
            raw_cs_sp = cs_sp_reader.read_latest()
            if raw_cs_sp:
                try:
                    evt = log_from_bytes(raw_cs_sp)
                    cs_sp = evt.carStateSP
                    if hasattr(cs_sp, 'speedLimit'):
                        self._cached_cs['speedLimit'] = safe_float(cs_sp.speedLimit, 1)
                except Exception:
                    pass

            # 2. Controls / Selfdrive State
            raw_ctrl = ctrl_reader.read_latest() or ctrl_fallback_reader.read_latest()
            if raw_ctrl:
                try:
                    evt = log_from_bytes(raw_ctrl)
                    which_ctrl = evt.which()
                    cts = getattr(evt, which_ctrl)

                    raw_alert = str(getattr(cts, 'alertStatus', 'normal')).lower()
                    status_code = 1 if ('userprompt' in raw_alert or '1' in raw_alert) else (2 if ('critical' in raw_alert or '2' in raw_alert) else 0)

                    v_cruise = self._cached_cs.get('vCruiseCarState')
                    if v_cruise is None and hasattr(cts, 'vCruise'):
                        v_cruise = safe_float(cts.vCruise, 1)

                    self._cached_ctrl = {
                        'enabled': bool(getattr(cts, 'enabled', False)),
                        'active': bool(getattr(cts, 'active', False)),
                        'state': str(getattr(cts, 'state', '')).lower(),
                        'aTarget': safe_float(getattr(cts, 'aTarget', None)),
                        'vCruise': v_cruise,
                        'alertStatus': status_code,
                        'alertText1': str(getattr(cts, 'alertText1', '') or ''),
                        'alertText2': str(getattr(cts, 'alertText2', '') or ''),
                    }
                except Exception:
                    pass

            # 3. Radar State (Leads)
            raw_radar = radar_reader.read_latest()
            if raw_radar:
                try:
                    evt = log_from_bytes(raw_radar)
                    rs = evt.radarState
                    leads = []
                    for name in ('leadOne', 'leadTwo'):
                        lead = getattr(rs, name, None)
                        if lead and bool(getattr(lead, 'status', False)):
                            leads.append({
                                'dRel': safe_float(lead.dRel),
                                'yRel': safe_float(lead.yRel),
                                'vRel': safe_float(lead.vRel),
                                'vLead': safe_float(lead.vLead)
                            })
                    self._cached_leads = leads
                except Exception:
                    pass

            # 4. Perception Model (Lanes, Edges, Path)
            raw_mv2 = mv2_reader.read_latest()
            if raw_mv2:
                try:
                    evt = log_from_bytes(raw_mv2)
                    mv2 = evt.modelV2

                    if not self._cached_leads:
                        vleads = []
                        leads_v3 = getattr(mv2, 'leadsV3', None)
                        if leads_v3:
                            for vlead in leads_v3:
                                if getattr(vlead, 'prob', 0.0) > 0.4:
                                    x_list = getattr(vlead, 'x', [])
                                    y_list = getattr(vlead, 'y', [])
                                    v_list = getattr(vlead, 'v', [])
                                    if len(x_list) > 0 and len(y_list) > 0:
                                        v_ego = self._cached_cs.get('vEgo', 0.0) or 0.0
                                        vleads.append({
                                            'dRel': safe_float(x_list[0]),
                                            'yRel': safe_float(y_list[0]),
                                            'vRel': safe_float(v_list[0] - v_ego) if len(v_list) > 0 else 0.0,
                                            'vLead': safe_float(v_list[0]) if len(v_list) > 0 else 0.0
                                        })
                        if vleads:
                            self._cached_leads = vleads

                    self._cached_lanes = [self._extract_poly(ll, step=2) for ll in getattr(mv2, 'laneLines', [])]
                    self._cached_lane_probs = [safe_float(p, 3) for p in getattr(mv2, 'laneLineProbs', [])]
                    self._cached_edges = [self._extract_poly(re_, step=2) for re_ in getattr(mv2, 'roadEdges', [])]
                    if hasattr(mv2, 'position'):
                        self._cached_path = self._extract_poly(mv2.position, step=2)
                except Exception:
                    pass

            # 5. Device State
            raw_dev = dev_reader.read_latest()
            if raw_dev:
                try:
                    evt = log_from_bytes(raw_dev)
                    ds = evt.deviceState
                    self._cached_sys = {
                        'cpuTempC': [safe_float(t, 1) for t in getattr(ds, 'cpuTempC', [])],
                        'gpuTempC': [safe_float(t, 1) for t in getattr(ds, 'gpuTempC', [])],
                        'memUsage': int(ds.memoryUsagePercent) if getattr(ds, 'memoryUsagePercent', None) is not None else None,
                    }
                except Exception:
                    pass

            # Assemble telemetry frame
            frame: Dict[str, Any] = {
                'ts': int(time.time() * 1000),
                'leads': self._cached_leads,
                'has_lead': len(self._cached_leads) > 0,
                'laneLines': self._cached_lanes,
                'laneLineProbs': self._cached_lane_probs,
                'roadEdges': self._cached_edges,
                'path': self._cached_path,
                'ctrl': self._cached_ctrl,
                'sys': self._cached_sys,
            }
            frame.update(self._cached_cs)

            with self._lock:
                self._sequence += 1
                frame['seq'] = self._sequence
                self._latest_payload = json.dumps(frame, separators=(',', ':'))

            # Strictly 15Hz loop (66ms interval)
            elapsed = time.monotonic() - loop_start
            sleep_time = max(0.01, 0.066 - elapsed)
            time.sleep(sleep_time)


class SsePerceptionServer:
    """Lightweight HTTP server serving real-time SSE stream on port 7788."""

    def __init__(self, bridge: PerceptionBridge, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        self.bridge = bridge
        self.host = host
        self.port = port
        self.httpd: Optional[ThreadingHTTPServer] = None

    def start(self) -> None:
        bridge = self.bridge

        class SseRequestHandler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == '/health':
                    resp = b'{"status":"ok","service":"perception"}\n'
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(resp)
                    return

                if self.path == '/snapshot':
                    seq, payload = bridge.get_latest_payload()
                    resp = payload.encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(resp)
                    return

                if self.path in ('/stop', '/exit', '/shutdown'):
                    resp = b'{"status":"stopped"}\n'
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(resp)))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(resp)
                    print("[perception] HTTP stop requested, exiting now.", flush=True)
                    threading.Thread(target=lambda: (time.sleep(0.05), os._exit(0)), daemon=True).start()
                    return

                # SSE Stream Endpoint (/stream)
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'keep-alive')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()

                bridge.register_client()
                last_seq = -1
                idle_ticks = 0
                try:
                    while bridge._running:
                        seq, payload = bridge.get_latest_payload()
                        if seq != last_seq and payload != '{}':
                            self.wfile.write(('data: ' + payload + '\n\n').encode('utf-8'))
                            self.wfile.flush()
                            last_seq = seq
                            idle_ticks = 0
                        else:
                            idle_ticks += 1
                            if idle_ticks >= 20:  # ~2 seconds keepalive
                                self.wfile.write(b': ping\n\n')
                                self.wfile.flush()
                                idle_ticks = 0

                        time.sleep(0.066)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except Exception:
                    pass
                finally:
                    bridge.unregister_client()

            def do_HEAD(self):
                self.do_GET()

        class ReusableThreadingHTTPServer(ThreadingHTTPServer):
            allow_reuse_address = True
            daemon_threads = True

            def server_bind(self):
                import socket
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                if hasattr(socket, 'SO_REUSEPORT'):
                    try:
                        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                    except Exception:
                        pass
                super().server_bind()

        for attempt in range(5):
            try:
                self.httpd = ReusableThreadingHTTPServer((self.host, self.port), SseRequestHandler)
                break
            except OSError as e:
                if attempt < 4 and ('Address already in use' in str(e) or getattr(e, 'errno', 0) == 98):
                    os.system(f'fuser -k {self.port}/tcp 2>/dev/null || true')
                    time.sleep(0.4)
                else:
                    raise

    def serve_forever(self) -> None:
        if self.httpd:
            self.httpd.serve_forever()

    def shutdown(self) -> None:
        if self.httpd:
            self.httpd.shutdown()


def main():
    setup_process_isolation()
    bridge = PerceptionBridge()

    worker_thread = threading.Thread(target=bridge.run, name="perception-passive-worker", daemon=True)
    worker_thread.start()

    server = SsePerceptionServer(bridge, host=DEFAULT_HOST, port=DEFAULT_PORT)
    server.start()

    def handle_exit(signum, frame):
        bridge._running = False
        server.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)

    print(f"[openpilot_perception] Zero-Impact SHM service listening on {DEFAULT_HOST}:{DEFAULT_PORT}", flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
