#!/usr/bin/env python3
"""Three-point area sweep with Z locked.

The operator gives three XY corners of the rectangle to sweep:
    P1  the start corner, shared by both edges
    P2  the corner at the end of the length   (P1 -> P2 = length, and its direction)
    P3  the corner at the end of the breadth  (P1 -> P3 = breadth)
The rectangle may be tilted relative to the machine axes. Corners found by jogging are
never exactly square, so the rectangle is squared on P1 -> P2: the breadth is P3's
perpendicular distance from the P1-P2 line, and how far that moved P3 is reported. A
corner more than ANGLE_TOL_DEG from a right angle is refused as a likely wrong point.

Grid: a step (mm) along each edge, starting at P1. When an edge isn't a whole number of
steps the last row/column still lands exactly on the far edge (the last gap is shorter).
Rows run parallel to P1 -> P2 and advance toward P3: serpentine reverses every other row
(shortest path); raster runs every row in the P1 -> P2 direction, so each point is
approached from the same side (consistent lead-screw backlash) at the cost of a return
move per row.

At each point: G1 X Y Z (Z = the locked value, re-sent every point) -> M400 (its ok only
arrives once the stage has stopped) -> M114 readback must match the target -> hold still
for the stop time -> one CSV row. Another instrument measures during the hold; the
per-point arrive/depart timestamps let its records be matched to positions afterwards.
Rows are flushed as they are written, so a stopped or failed sweep keeps every finished
point, and the JSON sidecar records the inputs and how the sweep ended.
"""
import csv
import json
import math
import re
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_AREA_CONFIG = {
    "p1x": 0.0, "p1y": 0.0,      # start corner
    "p2x": 20.0, "p2y": 0.0,     # end of the length
    "p3x": 0.0, "p3y": 10.0,     # end of the breadth
    "step_length": 1.0, "step_breadth": 1.0,
    "z": 10.0, "dwell_s": 1.0, "feed": 300.0,
    "pattern": "serpentine", "return_to_start": True,
    "label": "", "notes": "", "data_dir": "runs",
}
PATTERNS = ("serpentine", "raster")

ANGLE_TOL_DEG = 5.0       # P1's corner further than this from 90 deg is refused
EDGE_EPS_MM = 1e-3        # leftovers smaller than this are "on the edge" (machine step is 1/3200 mm)
MIN_STEP_MM = 0.001
MAX_POINTS = 50_000
POS_TOL_MM = 0.01         # Marlin's M114 prints 2 decimals
MOVE_OVERHEAD_S = 0.1     # per point, M400/M114 round trips + acceleration; for the time estimate only
# firmware/configure_marlin.py sets DEFAULT_MAX_FEEDRATE X/Y = 10 mm/s; Marlin silently clamps a
# faster G1 F to it, so the estimate uses the clamped value and the plan says so.
FIRMWARE_MAX_XY_MM_MIN = 600.0

AREA_COLUMNS = ["index", "row", "col", "along_length_mm", "along_breadth_mm",
                "x_mm", "y_mm", "z_mm", "x_read_mm", "y_read_mm", "z_read_mm",
                "arrive_utc", "depart_utc", "arrive_s", "depart_s", "dwell_s", "status"]


# ---------- inputs and geometry ----------
def parse_params(p):
    """Typed, validated copy of the sweep inputs. Raises ValueError with a message the
    operator can act on."""
    def num(key, label, lo=None):
        try:
            v = float(p.get(key))
        except (TypeError, ValueError):
            raise ValueError(f"{label} must be a number")
        if not math.isfinite(v):
            raise ValueError(f"{label} must be a finite number")
        if lo is not None and v < lo:
            raise ValueError(f"{label} must be at least {lo:g}")
        return v

    q = {}
    for n in (1, 2, 3):
        q[f"p{n}x"] = num(f"p{n}x", f"P{n} X")
        q[f"p{n}y"] = num(f"p{n}y", f"P{n} Y")
    q["step_length"] = num("step_length", "length step", MIN_STEP_MM)
    q["step_breadth"] = num("step_breadth", "breadth step", MIN_STEP_MM)
    q["z"] = num("z", "locked Z")
    q["dwell_s"] = num("dwell_s", "stop time", 0.0)
    q["feed"] = num("feed", "travel feed", 1.0)
    q["pattern"] = p.get("pattern") or "serpentine"
    if q["pattern"] not in PATTERNS:
        raise ValueError(f"pattern must be one of {', '.join(PATTERNS)}")
    q["return_to_start"] = bool(p.get("return_to_start", True))
    q["label"] = str(p.get("label") or "")
    q["notes"] = str(p.get("notes") or "")
    q["data_dir"] = str(p.get("data_dir") or DEFAULT_AREA_CONFIG["data_dir"])
    return q


def rectangle(p1, p2, p3):
    """The rectangle P1 (shared corner), P2 (end of the length), P3 (end of the breadth),
    squared on P1 -> P2. Corners are returned in drawing order P1, P2, P2', P3'."""
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
    ux, uy = x2 - x1, y2 - y1
    length = math.hypot(ux, uy)
    if length < EDGE_EPS_MM:
        raise ValueError("P1 and P2 are the same point: P2 must be the corner at the other end of the length")
    ex, ey = ux / length, uy / length          # unit vector along the length
    nx, ny = -ey, ex                           # perpendicular, flipped below to P3's side
    vx, vy = x3 - x1, y3 - y1
    breadth = vx * nx + vy * ny
    if breadth < 0:
        nx, ny, breadth = -nx, -ny, -breadth
    if breadth < EDGE_EPS_MM:
        raise ValueError("P3 is on the P1-P2 line: P3 must be the corner across the breadth from P1")
    along = vx * ex + vy * ey                  # 0 for a perfect right angle at P1
    angle = math.degrees(math.atan2(breadth, along))
    if abs(angle - 90.0) > ANGLE_TOL_DEG:
        raise ValueError(f"the corner at P1 is {angle:.1f} deg, not 90: P2 and P3 must be the two corners next to P1")
    bx, by = nx * breadth, ny * breadth
    return {
        "origin": [x1, y1], "e_length": [ex, ey], "e_breadth": [nx, ny],
        "length_mm": length, "breadth_mm": breadth,
        "angle_at_p1_deg": angle, "p3_squared_by_mm": abs(along),
        "rotation_deg": math.degrees(math.atan2(ey, ex)),   # length edge vs machine +X
        "corners": [[x1, y1], [x2, y2], [x2 + bx, y2 + by], [x1 + bx, y1 + by]],
    }


def edge_count(length, step):
    """Number of grid positions edge_positions() will return, without building them."""
    n = int(math.floor(length / step + 1e-9))
    return n + 1 + (1 if length - n * step > EDGE_EPS_MM else 0)


def edge_positions(length, step):
    """0, step, 2*step, ... and always the far edge itself."""
    n = int(math.floor(length / step + 1e-9))
    pos = [min(k * step, length) for k in range(n + 1)]
    if length - pos[-1] > EDGE_EPS_MM:
        pos.append(length)
    else:
        pos[-1] = length
    return pos


def grid(rect, step_l, step_b, pattern):
    """Ordered sweep points. row/col index the breadth/length positions; a/b are the
    distances along the length/breadth from P1, in mm."""
    la = edge_positions(rect["length_mm"], step_l)
    lb = edge_positions(rect["breadth_mm"], step_b)
    (ox, oy), (ex, ey), (nx, ny) = rect["origin"], rect["e_length"], rect["e_breadth"]
    pts = []
    for r, b in enumerate(lb):
        cols = list(enumerate(la))
        if pattern == "serpentine" and r % 2:
            cols.reverse()
        for c, a in cols:
            pts.append({"x": round(ox + ex * a + nx * b, 4) + 0.0, "y": round(oy + ey * a + ny * b, 4) + 0.0,  # +0.0: no "-0.0"
                        "row": r, "col": c, "a": round(a, 4), "b": round(b, 4)})
    return pts


def path_length(pts, return_to_start):
    d = sum(math.hypot(q["x"] - p["x"], q["y"] - p["y"]) for p, q in zip(pts, pts[1:]))
    if return_to_start and len(pts) > 1:
        d += math.hypot(pts[-1]["x"] - pts[0]["x"], pts[-1]["y"] - pts[0]["y"])
    return d


# ---------- the sweep ----------
class AreaSweep:
    def __init__(self, machine):
        self.m = machine
        self.state = _idle_state()
        self._stop = threading.Event()
        self._pause = threading.Event()
        self._thread = None

    @property
    def active(self):
        return self.state["state"] in ("running", "paused")

    def plan(self, p):
        """{params, plan, points, error}. params is None when the inputs don't parse; error
        is the first thing stopping the sweep from running (None when it can run)."""
        try:
            q = parse_params(p)
        except ValueError as e:
            return {"params": None, "plan": None, "points": [], "error": str(e)}
        try:
            rect = rectangle((q["p1x"], q["p1y"]), (q["p2x"], q["p2y"]), (q["p3x"], q["p3y"]))
            n_len = edge_count(rect["length_mm"], q["step_length"])
            n_br = edge_count(rect["breadth_mm"], q["step_breadth"])
            if n_len * n_br > MAX_POINTS:
                raise ValueError(f"{n_len} x {n_br} = {n_len * n_br} points is more than {MAX_POINTS}: use larger steps")
            pts = grid(rect, q["step_length"], q["step_breadth"], q["pattern"])
        except ValueError as e:
            return {"params": q, "plan": None, "points": [], "error": str(e)}
        path = path_length(pts, q["return_to_start"])
        feed_eff = min(q["feed"], FIRMWARE_MAX_XY_MM_MIN)
        la = edge_positions(rect["length_mm"], q["step_length"])
        lb = edge_positions(rect["breadth_mm"], q["step_breadth"])
        plan = {
            **rect,
            "n_length": n_len, "n_breadth": n_br, "n_points": len(pts),
            "last_gap_length_mm": la[-1] - la[-2] if len(la) > 1 else 0.0,
            "last_gap_breadth_mm": lb[-1] - lb[-2] if len(lb) > 1 else 0.0,
            "path_mm": path,
            "feed_effective_mm_min": feed_eff,
            "est_s": path / (feed_eff / 60.0) + len(pts) * (q["dwell_s"] + MOVE_OVERHEAD_S),
        }
        # Limits and the work area are both convex, so the four corners at the locked Z
        # decide it; every point is checked anyway so the message can name the first one.
        err = None
        for i, pt in enumerate(pts):
            e = self.m.check_limits({"X": pt["x"], "Y": pt["y"], "Z": q["z"]})
            if e:
                err = f"point {i} (X{pt['x']:.3f} Y{pt['y']:.3f} Z{q['z']:.3f}): {e}"
                break
        return {"params": q, "plan": plan, "points": pts, "error": err}

    def start(self, p):
        if self.active:
            return {"ok": False, "error": "area sweep already running"}
        if not self.m.connected or self.m.halted:
            return {"ok": False, "error": "gantry not connected (or halted)"}
        if self.m.sweep["state"] in ("running", "paused"):
            return {"ok": False, "error": "a grid sweep is running"}
        r = self.plan(p)
        if r["error"]:
            return {"ok": False, "error": r["error"]}
        q, plan, pts = r["params"], r["plan"], r["points"]
        if self.m.pos.get("Z") is None:
            return {"ok": False, "error": "gantry position unknown: refresh it (M114) first"}

        root = Path(q["data_dir"]).expanduser().resolve()  # absolute, so the UI can say where the file is
        root.mkdir(parents=True, exist_ok=True)
        tag = re.sub(r"[^A-Za-z0-9_-]+", "-", q["label"]).strip("-")
        stem = f"AREA_{time.strftime('%Y%m%d_%H%M%S')}" + (f"_{tag}" if tag else "")
        k, base = 1, stem
        while (root / f"{stem}.csv").exists() or (root / f"{stem}.json").exists():
            k += 1
            stem = f"{base}_{k}"
        csv_path, json_path = root / f"{stem}.csv", root / f"{stem}.json"
        meta = {
            "software": "xyz-stage gantry console / area.py",
            "label": q["label"], "notes": q["notes"],
            "inputs": {"P1": [q["p1x"], q["p1y"]], "P2": [q["p2x"], q["p2y"]], "P3": [q["p3x"], q["p3y"]],
                       "step_length_mm": q["step_length"], "step_breadth_mm": q["step_breadth"],
                       "z_mm": q["z"], "dwell_s": q["dwell_s"], "feed_mm_min": q["feed"],
                       "pattern": q["pattern"], "return_to_start": q["return_to_start"]},
            "rectangle": {k: plan[k] for k in ("corners", "length_mm", "breadth_mm", "angle_at_p1_deg",
                                               "p3_squared_by_mm", "rotation_deg")},
            "grid": {"n_length": plan["n_length"], "n_breadth": plan["n_breadth"], "n_points": plan["n_points"],
                     "rows_along": "P1->P2 (length), advancing toward P3"},
            "position_tolerance_mm": POS_TOL_MM,
            "machine": {"limits": self.m.cfg["limits"], "enforce_limits": self.m.cfg["enforce_limits"],
                        "steps_per_mm": self.m.cfg["steps_per_mm"], "referenced": self.m.referenced,
                        "mock": self.m.mock},
            "start_position": dict(self.m.pos),
            "csv": csv_path.name, "columns": AREA_COLUMNS,
            "start_utc": _utc(), "end_utc": None, "state": "running", "points_done": 0, "msg": "",
        }
        json_path.write_text(json.dumps(meta, indent=2))

        self._stop.clear()
        self._pause.clear()
        self.state = {**_idle_state(), "state": "running", "phase": "starting", "total": len(pts),
                      "started": time.time(), "dwell_s": q["dwell_s"], "est_s": plan["est_s"],
                      "file": str(csv_path), "name": stem}
        self._thread = threading.Thread(target=self._worker, args=(q, pts, csv_path, json_path, meta), daemon=True)
        self._thread.start()
        self.m._log("sys", f"area sweep started: {len(pts)} points at Z{q['z']:.3f}, {q['dwell_s']:g} s each -> {csv_path.name}")
        return {"ok": True, "file": str(csv_path), "total": len(pts)}

    def pause(self, on=True):
        if not self.active:
            return {"ok": False, "error": "no area sweep running"}
        (self._pause.set if on else self._pause.clear)()
        return {"ok": True}

    def stop(self):
        """Ends the sweep after the move in progress; a stop time in progress is cut short
        and that point's row is marked dwell_cut. The stage stays where it is."""
        self._stop.set()
        self._pause.clear()
        return {"ok": True}

    def _worker(self, q, pts, csv_path, json_path, meta):
        final, msg, done = "done", "", 0
        t0 = time.monotonic()
        fxy, fz = q["feed"], self.m.cfg["feed"]["Z"]
        try:
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(AREA_COLUMNS)
                f.flush()
                self.state["phase"] = "approach"
                err = self._approach(pts[0], q["z"], fxy, fz)
                if err:
                    raise RuntimeError(f"approach to P1: {err}")
                for i, pt in enumerate(pts):
                    if not self._wait_if_paused(i):
                        final, msg = "stopped", f"stopped before point {i}"
                        break
                    self.state.update(index=i, phase="moving")
                    err = self._move(pt, q["z"], fxy)
                    if err:
                        final, msg = "error", f"point {i}: {err}"
                        break
                    rb = dict(self.m.pos)
                    arrive_utc, arrive = _utc(), time.monotonic()
                    self.state.update(phase="dwell", phase_started=time.time())
                    cut = self._hold(q["dwell_s"])
                    depart_utc, depart = _utc(), time.monotonic()
                    w.writerow([i, pt["row"], pt["col"], pt["a"], pt["b"], pt["x"], pt["y"], q["z"],
                                rb["X"], rb["Y"], rb["Z"], arrive_utc, depart_utc,
                                round(arrive - t0, 4), round(depart - t0, 4), round(depart - arrive, 4),
                                "ok" if cut is None else "dwell_cut"])
                    f.flush()
                    done += 1
                    self.state["points_done"] = done
                    if cut == "stop":
                        final, msg = "stopped", f"stopped during the stop time at point {i} (row marked dwell_cut)"
                        break
                    if cut:
                        final, msg = "error", f"point {i}: {cut}"
                        break
                else:
                    if q["return_to_start"] and len(pts) > 1:
                        self.state["phase"] = "returning"
                        err = self._move(pts[0], q["z"], fxy)
                        if err:
                            final, msg = "error", f"all {done} points done, but the return to P1 failed: {err}"
                    if final == "done":
                        msg = f"{done} points in {_fmt_dur(time.monotonic() - t0)}"
        except Exception as e:
            final, msg = "error", str(e)
        meta.update(state=final, msg=msg, points_done=done, end_utc=_utc(), end_position=dict(self.m.pos))
        try:
            json_path.write_text(json.dumps(meta, indent=2))
        except Exception:
            pass
        self.state.update(state=final, phase="", msg=msg, points_done=done, ended=time.time())
        self.m._log("sys", f"area sweep {final}: {msg} -> {csv_path.name}")

    def _approach(self, first, z, fxy, fz):
        """From wherever the stage is to the first point at the locked Z: rise first if the
        locked Z is higher, travel XY, lower last if it is lower (never drags the transducer
        down across the work)."""
        cur_z = self.m.pos.get("Z")
        if cur_z is None:
            return "gantry position unknown"
        top = max(cur_z, z)
        lines = ["G90"]
        if top > cur_z + 1e-6:
            lines.append(f"G1 Z{top:.4f} F{fz:g}")
        lines.append(f"G1 X{first['x']:.4f} Y{first['y']:.4f} F{fxy:g}")
        if top > z + 1e-6:
            lines.append(f"G1 Z{z:.4f} F{fz:g}")
        return self._send_all(lines)

    def _move(self, pt, z, fxy):
        """Go to a point at the locked Z, wait until the stage has stopped, check the readback."""
        err = self._send_all([f"G1 X{pt['x']:.4f} Y{pt['y']:.4f} Z{z:.4f} F{fxy:g}", "M400", "M114"])
        if err:
            return err
        pos = self.m.pos
        for axis, target in (("X", pt["x"]), ("Y", pt["y"]), ("Z", z)):
            if pos.get(axis) is None or abs(pos[axis] - target) > POS_TOL_MM:
                return f"position readback {axis}={pos.get(axis)} doesn't match the target {target:.4f}"
        return None

    def _send_all(self, lines):
        try:
            for ln in lines:
                c = self.m.send(ln, wait=True)
                if self.m.halted:
                    return "gantry firmware halted (E-stop): Disconnect then Connect to reset it"
                if not c.ok:
                    return f"no ok from the gantry for '{ln}'"
        except RuntimeError as e:  # not connected / halted
            return str(e)
        return None

    def _hold(self, dwell_s):
        """Stand still for dwell_s. Returns None when the full time was held, "stop" when
        the operator stopped the sweep, or a reason when the gantry went away."""
        end = time.monotonic() + dwell_s
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return None
            if self._stop.is_set():
                return "stop"
            if self.m.halted:
                return "gantry firmware halted (E-stop) during the stop time"
            if not self.m.connected:
                return "gantry disconnected during the stop time"
            time.sleep(min(0.01, left))

    def _wait_if_paused(self, i):
        """Pause takes effect between points. Returns False if the sweep was stopped."""
        while self._pause.is_set() and not self._stop.is_set():
            if self.state["state"] != "paused":
                self.state.update(state="paused", phase="paused")
                self.m._log("sys", f"area sweep paused before point {i}")
            time.sleep(0.05)
        if self.state["state"] == "paused":
            self.state["state"] = "running"
            self.m._log("sys", "area sweep resumed")
        return not self._stop.is_set()


def _idle_state():
    return {"state": "idle", "phase": "", "index": 0, "total": 0, "points_done": 0,
            "started": None, "ended": None, "phase_started": None, "dwell_s": 0.0, "est_s": None,
            "file": None, "name": None, "msg": ""}


def _utc():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _fmt_dur(s):
    return f"{s:.1f} s" if s < 90 else f"{s / 60:.1f} min"
