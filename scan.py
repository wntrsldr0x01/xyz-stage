#!/usr/bin/env python3
"""XYZ spatial scan (spec sec. 10): at each grid point move -> verify -> dwell -> acquire -> save.

Couples the gantry (server.Machine) and the lock-in (daq.Lockin) without either knowing
about the other. The XY grid and its ordering come from Machine.sweep_points(), so a scan
covers exactly the points the gantry console's Grid sweep preview shows; Z is a list of
planes. Each point is an ordinary daq.Lockin timed run (raw CSV + JSON sidecar), so fixed-
position recordings and scans share one acquisition path. A scan-level summary CSV gets a
row per point as soon as that point finishes.

Blocked/unblocked responses are compared afterwards (compare_scans / compare_runs) as
complex differences: dX = X_a - X_b, dY = Y_a - Y_b, |d| = hypot(dX, dY) -- never as a
difference of R magnitudes (spec sec. 10).
"""
import csv
import json
import math
import re
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import daq

POS_TOL_MM = 0.01  # M114 readback must match the target this closely before acquiring

SCAN_COLUMNS = ["index", "plane", "row", "col", "x_mm", "y_mm", "z_mm", "status",
                "n_samples", "n_errors", "n_overload",
                "X_mean_V", "X_sd_V", "Y_mean_V", "Y_sd_V", "R_mean_V", "R_sd_V",
                "mag_V", "phase_deg", "achieved_rate_hz", "stationary", "settings_changed", "run_file"]

COMPARE_COLUMNS = ["x_mm", "y_mm", "z_mm", "X_a_V", "Y_a_V", "X_b_V", "Y_b_V",
                   "dX_V", "dY_V", "dmag_V", "dphase_deg"]


class Scanner:
    def __init__(self, machine, lockin):
        self.m = machine
        self.lk = lockin
        self.state = _idle_state()
        self._stop = threading.Event()
        self._thread = None

    @property
    def active(self):
        return self.state["state"] == "running"

    # ---------- planning ----------
    def plan(self, p):
        """Ordered scan points and the first problem found (None if the plan is valid)."""
        try:
            planes = parse_planes(p.get("z_planes"), p.get("z"))
        except ValueError as e:
            return [], str(e)
        grid = self.m.sweep_points(p)
        pts = []
        for k, z in enumerate(planes):
            # Serpentine: run every other plane backwards so each plane starts where the
            # previous one ended -- no long diagonal travel between planes.
            g = grid[::-1] if p.get("pattern", "").startswith("serpentine") and k % 2 else grid
            pts += [{**pt, "z": z, "plane": k} for pt in g]
        if not self.m.referenced:
            return pts, "gantry position not referenced: press Zero (or Restore) first -- scan coordinates would be meaningless"
        for i, pt in enumerate(pts):
            for z in [pt["z"]] + ([p["safe_z"]] if p.get("lift") else []):
                err = self.m.check_limits({"X": pt["x"], "Y": pt["y"], "Z": z})
                if err:
                    return pts, f"point {i} ({pt['x']}, {pt['y']}, z={z}): {err}"
        return pts, None

    # ---------- run control ----------
    def start(self, p):
        if self.active:
            return {"ok": False, "error": "scan already running"}
        if not self.m.connected or self.m.halted:
            return {"ok": False, "error": "gantry not connected (or halted)"}
        if self.m.sweep["state"] in ("running", "paused"):
            return {"ok": False, "error": "a gantry sweep is running"}
        if not self.lk.connected:
            return {"ok": False, "error": "SR860 not connected"}
        if self.lk.run["state"] in ("running", "settling"):
            return {"ok": False, "error": "an SR860 run is in progress"}
        try:
            duration_s = float(p.get("duration_s"))
            rate_hz = float(p.get("rate_hz"))
            settle_s = float(p.get("settle_s") or 0.0)
            start_index = int(p.get("start_index") or 0)
        except (TypeError, ValueError):
            return {"ok": False, "error": "per-point duration, rate, settle and start index must be numbers"}
        # A scan needs a finite per-point duration; the rate check mirrors Lockin.start_run
        # so a bad rate fails here, before the stage moves, not at point 0.
        if duration_s <= 0:
            return {"ok": False, "error": "per-point duration must be positive"}
        if not 0 < rate_hz <= daq.MAX_RATE_HZ:
            return {"ok": False, "error": f"rate must be in (0, {daq.MAX_RATE_HZ:g}] Hz"}
        if settle_s < 0:
            return {"ok": False, "error": "settle time can't be negative"}
        pts, err = self.plan(p)
        if err:
            return {"ok": False, "error": err}
        if not pts:
            return {"ok": False, "error": "scan has no points"}
        if not 0 <= start_index < len(pts):
            return {"ok": False, "error": f"start index must be 0..{len(pts) - 1}"}

        condition = p.get("condition") or ""
        tag = re.sub(r"[^A-Za-z0-9_-]+", "-", condition).strip("-") or "scan"
        root = Path(p.get("data_dir") or self.lk.cfg["data_dir"])
        scan_dir = root / f"SCAN_{time.strftime('%Y%m%d_%H%M%S')}_{tag}"
        scan_dir.mkdir(parents=True, exist_ok=False)
        meta = {
            "software": "xyz-stage gantry console / scan.py",
            "condition": condition,
            "notes": p.get("notes") or "",
            "params": {k: p.get(k) for k in ("x0", "x1", "nx", "y0", "y1", "ny", "pattern", "lift",
                                             "safe_z", "z", "z_planes")},
            "planes": parse_planes(p.get("z_planes"), p.get("z")),
            "per_point": {"duration_s": duration_s, "rate_hz": rate_hz, "settle_s": settle_s},
            "n_points": len(pts),
            "start_index": start_index,
            "feed_mm_min": dict(self.m.cfg["feed"]),
            "instrument": self.lk.identity,
            "start_utc": datetime.now(timezone.utc).isoformat(),
            "end_utc": None,
            "state": "running",
            "points_done": 0,
        }
        (scan_dir / "scan.json").write_text(json.dumps(meta, indent=2))

        self._stop.clear()
        self.state = {"state": "running", "phase": "starting", "index": start_index, "total": len(pts),
                      "dir": str(scan_dir), "name": scan_dir.name, "started": time.time(), "msg": "",
                      "points_done": 0}
        self._thread = threading.Thread(target=self._worker,
                                        args=(p, pts, scan_dir, meta, duration_s, rate_hz, settle_s, start_index),
                                        daemon=True)
        self._thread.start()
        return {"ok": True, "dir": str(scan_dir), "total": len(pts)}

    def stop(self):
        """Stop now: the point being acquired is cut short (its run is marked incomplete),
        the stage stays where it is."""
        self._stop.set()
        if self.lk.run["state"] in ("running", "settling"):
            self.lk.stop_run()
        return {"ok": True}

    def _worker(self, p, pts, scan_dir, meta, duration_s, rate_hz, settle_s, start_index):
        fxy, fz = self.m.cfg["feed"]["XY"], self.m.cfg["feed"]["Z"]
        final, msg, done = "done", "", 0
        try:
            with open(scan_dir / "scan_summary.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(SCAN_COLUMNS)
                f.flush()
                for i in range(start_index, len(pts)):
                    if self._stop.is_set():
                        final, msg = "stopped", f"stopped before point {i}"
                        break
                    pt = pts[i]
                    self.state.update(index=i, phase="moving")
                    err = self._move_to(pt, p, fxy, fz)
                    if err:
                        final, msg = "error", f"point {i}: {err}"
                        break
                    if self._stop.is_set():
                        final, msg = "stopped", f"stopped at point {i} before acquiring"
                        break
                    self.state["phase"] = "acquiring"
                    r = self.lk.start_run(duration_s, rate_hz, str(scan_dir), meta["condition"],
                                          meta["notes"], run_id=f"P{i:04d}", settle_s=settle_s)
                    if not r.get("ok"):
                        final, msg = "error", f"point {i}: SR860 refused run: {r.get('error')}"
                        break
                    while self.lk.run["state"] not in daq.FINAL_STATES:
                        time.sleep(0.05)
                    run = self.lk.run
                    writer.writerow(_summary_row(i, pt, run))
                    f.flush()
                    done += 1
                    self.state["points_done"] = done
                    if run["state"] == "error":
                        # Lost USB link or a failed run: stop the scan rather than carry on
                        # recording garbage at the next points (spec sec. 11).
                        final, msg = "error", f"point {i}: SR860 run failed ({run.get('msg')})"
                        break
                    if self._stop.is_set():
                        final, msg = "stopped", f"stopped during point {i} (point marked incomplete)"
                        break
                else:
                    msg = f"{done} points in {time.time() - self.state['started']:.0f} s"
        except Exception as e:
            final, msg = "error", str(e)
        meta.update(state=final, end_utc=datetime.now(timezone.utc).isoformat(), points_done=done, msg=msg)
        try:
            (scan_dir / "scan.json").write_text(json.dumps(meta, indent=2))
        except Exception:
            pass
        self.state.update(state=final, phase="", msg=msg, points_done=done)
        self.lk._log("sys", f"scan {final}: {msg} -> {scan_dir.name}")

    def _move_to(self, pt, p, fxy, fz):
        """Rise, travel, lower (same shape as Machine.goto_safe), wait for the motion to
        finish, then check the M114 readback. Returns an error string or None."""
        x, y, z = pt["x"], pt["y"], pt["z"]
        cur_z = self.m.pos.get("Z")
        if cur_z is None:
            return "gantry position unknown"
        top = max(cur_z, z, p["safe_z"] if p.get("lift") else z)
        lines = ["G90"]
        if top > cur_z + 1e-6:
            lines.append(f"G1 Z{top:.4f} F{fz}")
        lines.append(f"G1 X{x:.4f} Y{y:.4f} F{fxy}")
        if top > z + 1e-6:
            lines.append(f"G1 Z{z:.4f} F{fz}")
        lines += ["M400", "M114"]  # M400: the ok only arrives once the stage has actually stopped
        try:
            for ln in lines:
                c = self.m.send(ln, wait=True)
                if not c.ok:
                    return f"no ok from gantry for '{ln}'"
        except RuntimeError as e:  # not connected / halted
            return str(e)
        pos = self.m.pos
        for axis, target in (("X", x), ("Y", y), ("Z", z)):
            if pos.get(axis) is None or abs(pos[axis] - target) > POS_TOL_MM:
                return f"position readback {axis}={pos.get(axis)} doesn't match target {target}"
        return None


def _idle_state():
    return {"state": "idle", "phase": "", "index": 0, "total": 0, "dir": None, "name": None,
            "started": None, "msg": "", "points_done": 0}


def parse_planes(z_planes, default_z):
    """'10' / '10, 12.5, 15' / [10, 12.5] -> [10.0, 12.5, 15.0]."""
    if z_planes in (None, "", []):
        if default_z in (None, ""):
            raise ValueError("no Z plane given")
        return [float(default_z)]
    if isinstance(z_planes, str):
        parts = [s for s in re.split(r"[,\s]+", z_planes.strip()) if s]
    else:
        parts = list(z_planes)
    try:
        return [float(v) for v in parts]
    except (TypeError, ValueError):
        raise ValueError(f"bad Z plane list: {z_planes!r}")


def _summary_row(i, pt, run):
    s = run.get("summary") or {}
    st = s.get("stats") or {}
    X, Y, R, mc = st.get("X_V", {}), st.get("Y_V", {}), st.get("R_V", {}), st.get("mean_complex", {})
    return [i, pt["plane"], pt["row"], pt["col"], pt["x"], pt["y"], pt["z"],
            s.get("end_state", run["state"]),
            s.get("n_samples", ""), s.get("n_errors", ""), s.get("n_overload", ""),
            X.get("mean", ""), X.get("sd", ""), Y.get("mean", ""), Y.get("sd", ""),
            R.get("mean", ""), R.get("sd", ""), mc.get("magnitude_v", ""), mc.get("phase_deg", ""),
            s.get("achieved_rate_hz", ""), s.get("stationary", ""),
            ";".join(s.get("settings_changed") or []), Path(run.get("path") or "").name]


def complex_delta(xa, ya, xb, yb):
    """Spec sec. 10: compare complex components, then take the magnitude. Not R_a - R_b."""
    dx, dy = xa - xb, ya - yb
    return {"dX_V": dx, "dY_V": dy, "dmag_V": math.hypot(dx, dy), "dphase_deg": math.degrees(math.atan2(dy, dx))}


# ---------- reading scans back ----------
def _resolve_child(data_dir, name, must_be_dir=True):
    if not name or "/" in name or "\\" in name or name in (".", ".."):
        raise ValueError("invalid name")
    base = Path(data_dir).resolve()
    path = (base / name).resolve()
    if path.parent != base or (must_be_dir and not path.is_dir()):
        raise ValueError("invalid name")
    return path


def list_scans(data_dir):
    base = Path(data_dir)
    if not base.is_dir():
        return []
    out = []
    for d in sorted(base.glob("SCAN_*"), key=lambda q: q.stat().st_mtime, reverse=True):
        meta_path = d / "scan.json"
        if not d.is_dir() or not meta_path.exists():
            continue
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            continue
        out.append({"name": d.name, "condition": meta.get("condition"), "state": meta.get("state"),
                    "n_points": meta.get("n_points"), "points_done": meta.get("points_done"),
                    "start_utc": meta.get("start_utc")})
    return out


def _num(v):
    if v in ("", None):
        return None
    for conv in (int, float):
        try:
            return conv(v)
        except ValueError:
            pass
    return v


def read_scan(data_dir, name):
    d = _resolve_child(data_dir, name)
    meta = json.loads((d / "scan.json").read_text())
    rows = []
    summary = d / "scan_summary.csv"
    if summary.exists():
        with open(summary, newline="") as f:
            for r in csv.DictReader(f):
                rows.append({k: (v if k in ("status", "run_file", "settings_changed", "stationary") else _num(v))
                             for k, v in r.items()})
    return {"meta": meta, "rows": rows}


def _key(x, y, z):
    return (round(float(x), 3), round(float(y), 3), round(float(z), 3))


def compare_scans(data_dir, name_a, name_b):
    """Point-by-point complex difference A - B (e.g. A = unblocked, B = blocked), matched on
    (x, y, z). Points missing or without clean statistics in either scan are listed, not
    guessed. Also writes the result as COMPARE_<a>__minus__<b>.csv in data_dir."""
    a, b = read_scan(data_dir, name_a), read_scan(data_dir, name_b)

    def index(scan):
        return {_key(r["x_mm"], r["y_mm"], r["z_mm"]): r for r in scan["rows"]
                if isinstance(r.get("X_mean_V"), float) and isinstance(r.get("Y_mean_V"), float)}

    ia, ib = index(a), index(b)
    rows = []
    for k in sorted(set(ia) & set(ib), key=lambda t: (t[2], t[1], t[0])):
        ra, rb = ia[k], ib[k]
        rows.append({"x_mm": k[0], "y_mm": k[1], "z_mm": k[2],
                     "X_a_V": ra["X_mean_V"], "Y_a_V": ra["Y_mean_V"],
                     "X_b_V": rb["X_mean_V"], "Y_b_V": rb["Y_mean_V"],
                     **complex_delta(ra["X_mean_V"], ra["Y_mean_V"], rb["X_mean_V"], rb["Y_mean_V"])})
    out_path = Path(data_dir) / f"COMPARE_{name_a}__minus__{name_b}.csv"
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COMPARE_COLUMNS)
        w.writeheader()
        w.writerows(rows)
    return {"rows": rows, "file": str(out_path),
            "only_in_a": len(set(ia) - set(ib)), "only_in_b": len(set(ib) - set(ia)),
            "condition_a": a["meta"].get("condition"), "condition_b": b["meta"].get("condition")}


def compare_runs(data_dir, name_a, name_b):
    """Same complex difference for two fixed-position runs, from their JSON stats."""
    ja, jb = daq.read_run_json(data_dir, name_a), daq.read_run_json(data_dir, name_b)
    sa, sb = ja.get("stats") or {}, jb.get("stats") or {}
    if "X_V" not in sa or "X_V" not in sb:
        raise ValueError("one of the runs has no clean samples to compare")
    out = complex_delta(sa["X_V"]["mean"], sa["Y_V"]["mean"], sb["X_V"]["mean"], sb["Y_V"]["mean"])
    out.update(condition_a=ja.get("condition"), condition_b=jb.get("condition"),
               position_a=ja.get("position"), position_b=jb.get("position"),
               same_position=daq._positions_match(ja.get("position") or {}, jb.get("position") or {}, tol=1e-3))
    return out
