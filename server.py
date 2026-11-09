#!/usr/bin/env python3
"""Jog + grid-sweep console for the modified Anycubic i3 Mega S gantry.

Run:  python3 server.py            (then open http://localhost:8780)
      python3 server.py --mock     (simulated printer, no hardware)

Only dependency is pyserial. Speaks plain Marlin G-code over USB, one command in
flight at a time (wait for "ok"), so the host always knows what the machine is doing.
"""
import argparse
import json
import math
import queue
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import daq

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "config.json"
POS_RE = re.compile(r"X:(-?\d+\.?\d*)\s*Y:(-?\d+\.?\d*)\s*Z:(-?\d+\.?\d*)")

DEFAULT_CONFIG = {
    "port": "/dev/cu.usbserial-0001",
    "baud": 250000,
    # 200-step motors at 16x: X/Y 1 mm lead screws, Z 8 mm lead screws.
    "steps_per_mm": {"X": 3200.0, "Y": 3200.0, "Z": 400.0},
    "send_steps_on_connect": False,
    "limits": {"X": [0, 200], "Y": [0, 200], "Z": [0, 150]},
    "enforce_limits": True,
    "feed": {"XY": 300, "Z": 300},
    "saved_points": [],
    "home": None,  # {"x","y","z"} in work coordinates
    "sweep": {
        "x0": 0, "y0": 0, "x1": 50, "y1": 50, "nx": 6, "ny": 6,
        "pattern": "serpentine_x", "z": 10, "lift": False, "safe_z": 20,
        "dwell_ms": 500, "per_point_gcode": "", "passes": 1, "return_home": True,
        "start_index": 0,
    },
    "daq": dict(daq.DEFAULT_DAQ_CONFIG),
}


def load_config():
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if CONFIG_PATH.exists():
        try:
            saved = json.loads(CONFIG_PATH.read_text())
            for k, v in saved.items():
                if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                    cfg[k].update(v)
                else:
                    cfg[k] = v
        except Exception as e:  # a broken config should not stop the console starting
            print("config.json unreadable, using defaults:", e)
    return cfg


def save_config(cfg):
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2))


class MockSerial:
    """Pretends to be Marlin: answers ok, tracks position, takes time to move."""

    def __init__(self):
        self.pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.relative = False
        self.feed = 1500.0
        self.out = ["start", "echo:MOCK Marlin"]
        self.is_open = True
        self.killed = False

    def write(self, data):
        for raw in data.decode().splitlines():
            line = raw.split(";")[0].strip().upper()
            if not line:
                continue
            words = dict((w[0], w[1:]) for w in line.split()[1:] if len(w) > 1)
            cmd = line.split()[0]
            if self.killed:
                continue  # like real Marlin after kill(): silent, no ok, until reset
            if cmd == "M112":
                self.killed = True
                self.out += ["echo:M112 Shutdown", "Error:Printer halted. kill() called!"]
                continue
            if cmd == "G91":
                self.relative = True
            elif cmd == "G90":
                self.relative = False
            elif cmd in ("G0", "G1"):
                if "F" in words:
                    self.feed = float(words["F"])
                dist = 0.0
                for a in "XYZ":
                    if a in words:
                        v = float(words[a])
                        tgt = self.pos[a] + v if self.relative else v
                        dist = max(dist, abs(tgt - self.pos[a]))
                        self.pos[a] = tgt
                time.sleep(min(dist / (self.feed / 60.0), 3.0))
            elif cmd == "G92":
                for a in "XYZ":
                    if a in words:
                        self.pos[a] = float(words[a])
                if not words:
                    self.pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
            elif cmd == "G28":
                axes = [a for a in "XYZ" if a in line] or list("XYZ")
                for a in axes:
                    self.pos[a] = 0.0
                time.sleep(0.5)
            elif cmd == "G4":
                time.sleep(float(words.get("P", 0)) / 1000.0 + float(words.get("S", 0)))
            elif cmd == "M114":
                p = self.pos
                self.out.append(f"X:{p['X']:.2f} Y:{p['Y']:.2f} Z:{p['Z']:.2f} E:0.00 Count X: 0 Y:0 Z:0")
            elif cmd == "M119":
                self.out += ["Reporting endstop status", "x_min: open", "y_min: open", "z_min: open", "z2_min: open"]
            self.out.append("ok")

    def readline(self):
        if self.out:
            return (self.out.pop(0) + "\n").encode()
        time.sleep(0.05)
        return b""

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


class Command:
    def __init__(self, line, timeout=120.0):
        self.line = line
        self.timeout = timeout
        self.done = threading.Event()
        self.ok = False
        self.reply = []


class Machine:
    def __init__(self, cfg, mock=False):
        self.cfg = cfg
        self.mock = mock
        self.ser = None
        self.q = queue.Queue()
        self.log = []  # (seq, direction, text)
        self.seq = 0
        self.lock = threading.Lock()
        self.pos = {"X": None, "Y": None, "Z": None}
        self.referenced = False  # true once the user zeroes or homes; limits are meaningless before
        self.last_pos = None  # work position from before the last reconnect, offered via Restore
        self.busy_line = None
        self.sweep = {"state": "idle", "index": 0, "total": 0, "points": [], "started": None, "msg": ""}
        self._sweep_stop = threading.Event()
        self._sweep_pause = threading.Event()
        # One reader at a time: connect() (boot banner), the worker (replies) and the idle reader
        # all read the same port, and interleaving them splits lines like "ok" into "o" / "k".
        self.io_lock = threading.Lock()
        # After M112 / kill() Marlin ignores everything and never sends "ok" until it is reset.
        self.halted = False
        threading.Thread(target=self._worker, daemon=True).start()

    # ---------- logging ----------
    def _log(self, direction, text):
        with self.lock:
            self.seq += 1
            self.log.append((self.seq, direction, text))
            if len(self.log) > 2000:
                self.log = self.log[-1500:]

    # ---------- connection ----------
    def connect(self, port=None, baud=None):
        # Remember where we were (in work coordinates) before the reset wipes it, for Restore.
        if self.referenced and self.pos.get("X") is not None:
            self.last_pos = dict(self.pos)
        self.disconnect()
        port = port or self.cfg["port"]
        baud = int(baud or self.cfg["baud"])
        with self.io_lock:
            if self.mock:
                ser = MockSerial()
            else:
                import serial
                ser = serial.Serial(port, baud, timeout=0.2)
            self.cfg["port"], self.cfg["baud"] = port, baud
            self._log("sys", f"opened {'MOCK' if self.mock else port} @ {baud}")
            # Opening the port resets the Arduino; Marlin prints its boot banner for a couple of seconds.
            t0 = time.time()
            while time.time() - t0 < 4.0:
                ln = ser.readline().decode(errors="replace").strip()
                if ln:
                    self._log("rx", ln)
            self._drain_queue()  # anything queued before the reset is stale
            self.halted = False  # the reset cleared any kill() state
            self.ser = ser
        self.referenced = False
        init = []  # gantry firmware is built without soft endstops; the console owns limits
        if self.cfg.get("send_steps_on_connect"):
            s = self.cfg["steps_per_mm"]
            init.append(f"M92 X{s['X']} Y{s['Y']} Z{s['Z']}")
        for ln in init + ["G90", "M114"]:
            self.send(ln)
        return True

    def disconnect(self):
        self.stop_sweep()
        ser, self.ser = self.ser, None  # detach first so the worker treats the close as intentional
        if ser:
            try:
                ser.close()
            except Exception:
                pass
            self._log("sys", "disconnected")
        self._drain_queue()

    @property
    def connected(self):
        return self.ser is not None

    # ---------- command pipeline ----------
    def send(self, line, timeout=120.0, wait=False):
        if not self.connected:
            raise RuntimeError("not connected")
        if self.halted:
            raise RuntimeError("firmware halted (E-stop): Disconnect then Connect to reset it")
        c = Command(line.strip(), timeout)
        self.q.put(c)
        if wait:
            c.done.wait()
        return c

    def _drain_queue(self):
        while True:
            try:
                c = self.q.get_nowait()
                c.done.set()
            except queue.Empty:
                return

    def _worker(self):
        while True:
            try:
                c = self.q.get(timeout=0.2)
            except queue.Empty:
                self._read_unsolicited()
                continue
            with self.io_lock:
                ser = self.ser
                if ser is None or self.halted:
                    c.done.set()
                    continue
                try:
                    self.busy_line = c.line
                    self._log("tx", c.line)
                    ser.write((c.line + "\n").encode())
                    deadline = time.time() + c.timeout
                    while time.time() < deadline:
                        if self.halted:
                            break  # no ok is ever coming
                        ln = ser.readline().decode(errors="replace").strip()
                        if not ln:
                            continue
                        self._handle_rx(ln)
                        c.reply.append(ln)
                        if "busy" in ln:  # Marlin keepalive during long moves / homing
                            deadline = time.time() + c.timeout
                        if ln.startswith("ok"):
                            c.ok = True
                            break
                        if ln.startswith("Error") and "Last Line" not in ln and not self.halted:
                            self._log("sys", f"firmware error on: {c.line}")
                    else:
                        self._log("sys", f"timeout waiting for ok: {c.line}")
                except Exception as e:
                    self._serial_failed(ser, e)
                finally:
                    self.busy_line = None
                    c.done.set()

    def _read_unsolicited(self):
        with self.io_lock:
            ser = self.ser
            if ser is None:
                return
            try:
                ln = ser.readline().decode(errors="replace").strip()
                if ln:
                    self._handle_rx(ln)
            except Exception as e:
                self._serial_failed(ser, e)

    def _serial_failed(self, ser, e):
        # A port closed by disconnect() raises here too; only report failures on the live port.
        if ser is self.ser:
            self._log("sys", f"serial error: {e}")
            self.ser = None

    def _handle_rx(self, ln):
        m = POS_RE.search(ln)
        if m and ln.startswith("X:"):
            self.pos = {"X": float(m.group(1)), "Y": float(m.group(2)), "Z": float(m.group(3))}
        if ln == "ok":
            return  # don't flood the log with bare oks
        self._log("rx", ln)
        if "kill() called" in ln or "Printer halted" in ln:
            self._set_halted()

    def _set_halted(self):
        if not self.halted:
            self.halted = True
            self.referenced = False
            self._sweep_stop.set()
            self._drain_queue()
            self._log("sys", "firmware HALTED: Disconnect then Connect to reset the board")

    def emergency_stop(self):
        """M112 written straight to the port, ahead of the queue (no io_lock: the worker may be
        holding it while waiting on a move). Marlin only parses it after the current command
        finishes, so the power switch is the true e-stop."""
        self.stop_sweep()
        self._drain_queue()
        ser = self.ser
        if ser:
            ser.write(b"M112\n")
            self._log("tx", "M112 (emergency)")
        self._set_halted()

    # ---------- motion helpers ----------
    def check_limits(self, target):
        if not self.cfg["enforce_limits"]:
            return None
        if not self.referenced:
            return "position not referenced: jog to a corner and press Zero first (or disable limits)"
        for a, v in target.items():
            lo, hi = self.cfg["limits"][a]
            if v is None:
                continue
            if v < lo - 1e-6 or v > hi + 1e-6:
                return f"{a}={v:.3f} outside limits [{lo}, {hi}]"
        return None

    def jog(self, axis, delta, feed=None):
        if self.q.qsize() > 2:
            return {"ok": False, "error": "busy, jog dropped"}  # don't let held keys pile up moves
        axis = axis.upper()
        cur = self.pos.get(axis)
        # Before zeroing, jogs are unrestricted: with no endstops, jogging is how you reach the
        # reference corner in the first place. Limits only mean something once a zero exists.
        if self.cfg["enforce_limits"] and self.referenced and cur is not None:
            err = self.check_limits({axis: cur + delta})
            if err:
                return {"ok": False, "error": err}
        feed = feed or (self.cfg["feed"]["Z"] if axis == "Z" else self.cfg["feed"]["XY"])
        for ln in ("G91", f"G1 {axis}{delta:.4f} F{feed}", "G90", "M114"):
            self.send(ln)
        # Track locally too, so a second fast jog checks limits against where we're going.
        if cur is not None:
            self.pos[axis] = cur + delta
        return {"ok": True}

    def goto(self, x=None, y=None, z=None, feed=None):
        tgt = {"X": x, "Y": y, "Z": z}
        err = self.check_limits(tgt)
        if err:
            return {"ok": False, "error": err}
        words = " ".join(f"{a}{v:.4f}" for a, v in tgt.items() if v is not None)
        if not words:
            return {"ok": False, "error": "no axes given"}
        feed = feed or self.cfg["feed"]["XY"]
        for ln in ("G90", f"G1 {words} F{feed}", "M114"):
            self.send(ln)
        return {"ok": True}

    def goto_safe(self, x, y, z):
        """Rise to the higher of current/target Z, travel XY, then descend: never drags the tool
        across the work on the way to a saved position."""
        cur_z = self.pos.get("Z")
        if cur_z is None or not self.referenced:
            return {"ok": False, "error": "position not referenced: press Zero (or Restore) first"}
        err = self.check_limits({"X": x, "Y": y, "Z": z})
        if err:
            return {"ok": False, "error": err}
        fxy, fz = self.cfg["feed"]["XY"], self.cfg["feed"]["Z"]
        top = max(cur_z, z)
        lines = ["G90"]
        if top > cur_z + 1e-6:
            lines.append(f"G1 Z{top:.4f} F{fz}")
        lines.append(f"G1 X{x:.4f} Y{y:.4f} F{fxy}")
        if top > z + 1e-6:
            lines.append(f"G1 Z{z:.4f} F{fz}")
        for ln in lines + ["M114"]:
            self.send(ln)
        return {"ok": True}

    def set_home(self):
        p = self.pos
        if not self.referenced or p["X"] is None:
            return {"ok": False, "error": "position not referenced: press Zero (or Restore) first"}
        self.cfg["home"] = {"x": p["X"], "y": p["Y"], "z": p["Z"]}
        save_config(self.cfg)
        self._log("sys", f"home set to X{p['X']:.3f} Y{p['Y']:.3f} Z{p['Z']:.3f}")
        return {"ok": True}

    def go_home(self):
        h = self.cfg.get("home")
        if not h:
            return {"ok": False, "error": "no home point saved yet: use Set home here"}
        return self.goto_safe(h["x"], h["y"], h["z"])

    def restore_position(self):
        """Reconnecting resets the board, which zeroes Marlin's position wherever the gantry is.
        The lead screws don't back-drive, so the last known position is still physically true;
        G92 puts it back so saved points and home keep meaning the same place."""
        lp = self.last_pos
        if not lp:
            return {"ok": False, "error": "no previous position to restore"}
        self.send(f"G92 X{lp['X']:.4f} Y{lp['Y']:.4f} Z{lp['Z']:.4f}")
        self.send("M114")
        self.referenced = True
        self.last_pos = None
        self._log("sys", f"restored position X{lp['X']:.3f} Y{lp['Y']:.3f} Z{lp['Z']:.3f}")
        return {"ok": True}

    def set_zero(self, axes):
        axes = [a for a in axes.upper() if a in "XYZ"]
        self.send("G92 " + " ".join(f"{a}0" for a in axes))
        self.send("M114")
        self.referenced = True  # even a partial zero counts: the user is explicitly defining the frame
        self.last_pos = None  # a fresh zero supersedes the pre-reconnect position
        return {"ok": True}

    # ---------- sweep ----------
    def sweep_points(self, p):
        nx, ny = max(1, int(p["nx"])), max(1, int(p["ny"]))
        xs = [p["x0"] + (p["x1"] - p["x0"]) * (i / (nx - 1) if nx > 1 else 0) for i in range(nx)]
        ys = [p["y0"] + (p["y1"] - p["y0"]) * (j / (ny - 1) if ny > 1 else 0) for j in range(ny)]
        pts = []
        pat = p["pattern"]
        if pat in ("serpentine_x", "raster_x"):
            for r, y in enumerate(ys):
                row = list(enumerate(xs))
                if pat == "serpentine_x" and r % 2:
                    row.reverse()
                pts += [(x, y, r, c) for c, x in row]
        else:  # column-first variants
            for c, x in enumerate(xs):
                col = list(enumerate(ys))
                if pat == "serpentine_y" and c % 2:
                    col.reverse()
                pts += [(x, y, r, c) for r, y in col]
        return [{"x": round(x, 4), "y": round(y, 4), "row": r, "col": c} for x, y, r, c in pts]

    def sweep_gcode(self, p):
        """Full program as text, for export / review. Mirrors what run_sweep sends."""
        out = [f"; grid sweep {p['nx']}x{p['ny']} {p['pattern']}", "G90"]
        fxy, fz = self.cfg["feed"]["XY"], self.cfg["feed"]["Z"]
        pts = self.sweep_points(p)
        for k in range(int(p["passes"])):
            out.append(f"; pass {k + 1}")
            for i, pt in enumerate(pts):
                out += self._point_lines(p, pt, i, fxy, fz)
        if p["return_home"]:
            out += self._return_lines(p, pts, fxy, fz)
        return "\n".join(out)

    def _point_lines(self, p, pt, i, fxy, fz):
        lines = [f"; point {i} r{pt['row']} c{pt['col']}"]
        if p["lift"]:
            lines += [f"G1 Z{p['safe_z']:.4f} F{fz}", f"G1 X{pt['x']:.4f} Y{pt['y']:.4f} F{fxy}", f"G1 Z{p['z']:.4f} F{fz}"]
        else:
            lines += [f"G1 X{pt['x']:.4f} Y{pt['y']:.4f} F{fxy}"]
        lines.append("M400")  # wait for arrival so dwell / actions happen at the point, not en route
        if p["dwell_ms"]:
            lines.append(f"G4 P{int(p['dwell_ms'])}")
        snippet = (p.get("per_point_gcode") or "").strip()
        if snippet:
            fmt = dict(i=i, row=pt["row"], col=pt["col"], x=pt["x"], y=pt["y"], z=p["z"])
            for ln in snippet.splitlines():
                try:
                    ln = ln.format(**fmt)
                except (KeyError, IndexError, ValueError):
                    pass
                if ln.strip():
                    lines.append(ln.strip())
        return lines

    def _return_lines(self, p, pts, fxy, fz):
        x0, y0 = pts[0]["x"], pts[0]["y"]
        if p["lift"]:
            return [f"G1 Z{p['safe_z']:.4f} F{fz}", f"G1 X{x0:.4f} Y{y0:.4f} F{fxy}"]
        return [f"G1 X{x0:.4f} Y{y0:.4f} F{fxy}"]

    def validate_sweep(self, p):
        pts = self.sweep_points(p)
        zs = [p["z"]] + ([p["safe_z"]] if p["lift"] else [])
        for pt in pts:
            for z in zs:
                err = self.check_limits({"X": pt["x"], "Y": pt["y"], "Z": z})
                if err:
                    return pts, f"point ({pt['x']}, {pt['y']}, z={z}): {err}"
        return pts, None

    def start_sweep(self, p):
        if self.sweep["state"] in ("running", "paused"):
            return {"ok": False, "error": "sweep already running"}
        pts, err = self.validate_sweep(p)
        if err:
            return {"ok": False, "error": err}
        self._sweep_stop.clear()
        self._sweep_pause.clear()
        total = len(pts) * int(p["passes"])
        self.sweep = {"state": "running", "index": int(p.get("start_index", 0)), "total": total,
                      "points": pts, "started": time.time(), "msg": ""}
        threading.Thread(target=self._run_sweep, args=(p, pts), daemon=True).start()
        return {"ok": True, "total": total}

    def _run_sweep(self, p, pts):
        fxy, fz = self.cfg["feed"]["XY"], self.cfg["feed"]["Z"]
        try:
            self.send("G90", wait=True)
            # Approach the first point from above if lifting, so we never drag across the work.
            if p["lift"]:
                self.send(f"G1 Z{p['safe_z']:.4f} F{fz}", wait=True)
            else:
                self.send(f"G1 Z{p['z']:.4f} F{fz}", wait=True)
            n = len(pts)
            for k in range(int(self.sweep["index"]), self.sweep["total"]):
                while self._sweep_pause.is_set() and not self._sweep_stop.is_set():
                    self.sweep["state"] = "paused"
                    time.sleep(0.1)
                if self._sweep_stop.is_set():
                    self.sweep["state"], self.sweep["msg"] = "stopped", f"stopped before point {k}"
                    return
                self.sweep["state"] = "running"
                self.sweep["index"] = k
                for ln in self._point_lines(p, pts[k % n], k % n, fxy, fz)[1:]:
                    c = self.send(ln, wait=True)
                    if not c.ok:
                        self.sweep["state"], self.sweep["msg"] = "error", f"no ok for '{ln}' at point {k}"
                        return
                self.send("M114")
            if p["return_home"]:
                for ln in self._return_lines(p, pts, fxy, fz):
                    self.send(ln, wait=True)
            self.send("M114")
            self.sweep["index"] = self.sweep["total"]
            self.sweep["state"], self.sweep["msg"] = "done", f"completed in {time.time() - self.sweep['started']:.1f}s"
        except Exception as e:
            self.sweep["state"], self.sweep["msg"] = "error", str(e)

    def pause_sweep(self, pause=True):
        (self._sweep_pause.set if pause else self._sweep_pause.clear)()
        return {"ok": True}

    def stop_sweep(self):
        self._sweep_stop.set()
        self._sweep_pause.clear()
        return {"ok": True}

    # ---------- state ----------
    def state(self, since=0):
        with self.lock:
            log = [entry for entry in self.log if entry[0] > since][-400:]
        sw = dict(self.sweep)
        sw.pop("points", None)
        return {"connected": self.connected, "mock": self.mock, "pos": self.pos, "referenced": self.referenced,
                "busy": self.busy_line, "queue": self.q.qsize(), "halted": self.halted, "last_pos": self.last_pos, "sweep": sw, "log": log, "cfg": self.cfg}


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [p.device for p in lp.comports()]
    except Exception:
        return []


def make_handler(m: Machine, lk: daq.Lockin):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path)
            q = parse_qs(path.query)
            if path.path in ("/", "/index.html"):
                body = (HERE / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path.path == "/api/state":
                since = int(q.get("since", [0])[0])
                state = m.state(since)
                # Separate cursor: the two logs number their entries independently.
                state["daq"] = lk.state(int(q.get("dsince", [0])[0]))
                self._json(state)
            elif path.path == "/api/ports":
                self._json({"ports": list_ports()})
            elif path.path == "/api/daq/resources":
                self._json({"resources": daq.Lockin.list_resources()})
            elif path.path == "/api/daq/history":
                data_dir = q.get("dir", [lk.cfg["data_dir"]])[0]
                try:
                    self._json({"ok": True, "runs": daq.list_runs(data_dir)})
                except Exception as e:
                    self._json({"ok": False, "error": str(e)}, 400)
            elif path.path == "/api/daq/history/file":
                data_dir = q.get("dir", [lk.cfg["data_dir"]])[0]
                name, ext = q.get("name", [""])[0], q.get("ext", ["csv"])[0]
                try:
                    if ext == "json":
                        self._json({"ok": True, "data": daq.read_run_json(data_dir, name)})
                    else:
                        self._json({"ok": True, "rows": daq.read_run_csv(data_dir, name)})
                except Exception as e:
                    self._json({"ok": False, "error": str(e)}, 400)
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(n) or b"{}")
            route = self.path.replace("/api/", "")
            try:
                if route == "connect":
                    m.connect(data.get("port"), data.get("baud"))
                    r = {"ok": True}
                elif route == "disconnect":
                    m.disconnect(); r = {"ok": True}
                elif route == "send":
                    for ln in data["gcode"].splitlines():
                        if ln.strip():
                            m.send(ln)
                    r = {"ok": True}
                elif route == "jog":
                    r = m.jog(data["axis"], float(data["delta"]), data.get("feed"))
                elif route == "goto":
                    r = m.goto(data.get("x"), data.get("y"), data.get("z"), data.get("feed"))
                elif route == "zero":
                    r = m.set_zero(data.get("axes", "XYZ"))
                elif route == "goto_safe":
                    r = m.goto_safe(float(data["x"]), float(data["y"]), float(data["z"]))
                elif route == "home/set":
                    r = m.set_home()
                elif route == "home/go":
                    r = m.go_home()
                elif route == "restore":
                    r = m.restore_position()
                elif route == "estop":
                    m.emergency_stop(); r = {"ok": True}
                elif route == "config":
                    for k, v in data.items():
                        if isinstance(v, dict) and isinstance(m.cfg.get(k), dict):
                            m.cfg[k].update(v)
                        else:
                            m.cfg[k] = v
                    save_config(m.cfg)
                    r = {"ok": True}
                elif route == "sweep/preview":
                    pts, err = m.validate_sweep(data)
                    r = {"ok": True, "points": pts, "warning": err}
                elif route == "sweep/gcode":
                    r = {"ok": True, "gcode": m.sweep_gcode(data)}
                elif route == "sweep/start":
                    m.cfg["sweep"].update(data); save_config(m.cfg)
                    r = m.start_sweep(data)
                elif route == "sweep/pause":
                    r = m.pause_sweep(bool(data.get("pause", True)))
                elif route == "sweep/stop":
                    r = m.stop_sweep()
                elif route == "daq/connect":
                    lk.connect(data.get("resource")); r = {"ok": True}
                elif route == "daq/disconnect":
                    lk.disconnect(); r = {"ok": True}
                elif route == "daq/run/start":
                    rate = data.get("rate_hz") or lk.cfg["rate_hz"]
                    data_dir = data.get("data_dir") or lk.cfg["data_dir"]
                    settle = data.get("settle_s")
                    r = lk.start_run(data.get("duration_s"), rate, data_dir,
                                     data.get("condition", ""), data.get("notes", ""),
                                     data.get("run_id"), settle)
                    if r.get("ok"):  # start_run validated these; remember them for next time
                        lk.cfg["rate_hz"] = float(rate)
                        lk.cfg["data_dir"] = data_dir
                        if settle not in (None, ""):
                            lk.cfg["settle_s"] = float(settle)
                        save_config(m.cfg)
                elif route == "daq/run/stop":
                    r = lk.stop_run()
                elif route == "daq/run/abort":
                    r = lk.abort_run()
                elif route == "daq/config":
                    for k, v in data.items():
                        if k in daq.DEFAULT_DAQ_CONFIG:  # known keys only; config.json is shared with the gantry
                            lk.cfg[k] = v
                    save_config(m.cfg)
                    r = {"ok": True}
                else:
                    r = {"ok": False, "error": f"unknown route {route}"}
            except Exception as e:
                r = {"ok": False, "error": str(e)}
            if r.get("ok") is False:
                # visible in the respective terminal/log pane, not just a fading toast
                (lk if route.startswith("daq/") else m)._log("sys", f"refused {route}: {r.get('error')}")
            self._json(r)

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="simulated printer, no serial port")
    ap.add_argument("--port", type=int, default=8780)
    ap.add_argument("--host", default="127.0.0.1")
    a = ap.parse_args()
    if not a.mock:
        try:
            import serial  # noqa: F401
        except ImportError:
            # Without this the UI loads but shows no ports and Connect fails, which looks like a hang.
            raise SystemExit(f"pyserial is not installed for {sys.executable}\n"
                             f"  fix: {sys.executable} -m pip install pyserial")
    cfg = load_config()
    m = Machine(cfg, mock=a.mock)
    lk = daq.Lockin(cfg["daq"], position_fn=lambda: m.pos, mock=a.mock)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(m, lk))
    print(f"gantry console on http://localhost:{a.port}  ({'MOCK' if a.mock else 'serial'})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        m.disconnect()
        lk.disconnect()


if __name__ == "__main__":
    main()
