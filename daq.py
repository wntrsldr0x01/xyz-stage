#!/usr/bin/env python3
"""SR860 lock-in acquisition: USB-TMC connect, timed CSV logging, run metadata.

Talks to the instrument through the official srsinst.sr860 driver (SNAP?/ref./signal./
status. SCPI wrapped as Python descriptors) rather than hand-rolled PyVISA calls, per the
spec's "prefer reusing the SRS driver" instruction. Position tags come from an injected
position_fn so this module has no dependency on server.Machine -- the two instruments
connect, fail and are driven independently.
"""
import csv
import json
import math
import re
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_DAQ_CONFIG = {
    "resource": "",      # VISA resource string, e.g. "USB0::0xB506::0x2000::SN.../INSTR"
    "rate_hz": 5.0,
    "data_dir": "runs",
    "settle_s": 0.0,      # dwell after Start before the first sample is taken
    "check_overload": True,  # one extra CUROVLDSTAT? query per sample
    "scan_z_planes": "",     # remembered scan form values (scan.py)
    "scan_duration_s": 5.0,
    "monitor": True,         # live view (no recording) while connected and not recording
    "monitor_rate_hz": 5.0,
}

STREAM_MAXLEN = 6000      # live-plot buffer: 10 min at 10 Hz
STREAM_MAX_PER_POLL = 3000
# Measured on the bench (SR860 FW V1.51, USB-TMC, pyvisa-py, macOS): SNAP? 0,1 ~11 ms mean,
# 20 ms max; +CUROVLDSTAT? ~20 ms mean, 40 ms max. 20 Hz is about the ceiling.
MAX_RATE_HZ = 20.0
# pyvisa's default timeout is infinite and the SRS driver never sets one, so a lost
# reply would hang the run forever. Every query on the bench took < 50 ms.
VISA_TIMEOUT_S = 2.0


class LinkError(RuntimeError):
    """The USB-TMC exchange with the instrument failed. On the bench a single failed
    transfer left the SR860's endpoint stalled, and a late reply could be read as the
    answer to the next query, so the session can't be trusted afterwards: the run ends
    (incomplete) and Connect has to be pressed again (see Lockin._open_and_identify)."""

# CUROVLDSTAT? bits, from srsinst.sr860 Status.OverloadStatusBitDict.
OVERLOAD_BITS = {0: "ch1_scale", 1: "ch2_scale", 3: "ref_unlocked", 4: "input_range",
                 8: "data1_scale", 9: "data2_scale", 10: "data3_scale", 11: "data4_scale"}

# Exact column contract from the spec (section 8). Keep this order stable --
# existing CSVs are read back by column name during replay (FR-10).
RUN_COLUMNS = ["timestamp_utc", "elapsed_s", "x_mm", "y_mm", "z_mm", "ref_hz",
               "X_V", "Y_V", "R_V", "theta_deg", "status", "query_latency_ms"]

FINAL_STATES = ("done", "stopped", "aborted", "error")


class Lockin:
    """Mirrors the shape of server.Machine: background worker thread, state() snapshot
    dict, polled by the same /api/state loop as the gantry console."""

    def __init__(self, cfg, position_fn=None, mock=False):
        self.cfg = cfg
        self.mock = mock
        self._position_fn = position_fn or (lambda: {"X": None, "Y": None, "Z": None})
        self.inst = None
        self.identity = None  # {"model", "serial", "firmware", "resource"}
        self.latency_ms = None  # last measured query round trip (ping or sample)
        self.lock = threading.Lock()  # guards log and the sample stream (HTTP threads read, workers write)
        self.log = []
        self.seq = 0
        self.run = _idle_run()
        self._run_stop = threading.Event()
        self._run_abort = threading.Event()
        self._run_thread = None
        # Live-plot stream: samples from the current run or live monitor. "i" counts across
        # streams so a client can ask for only what it hasn't seen; stream_id changes when a
        # new run/monitor session starts so the client knows to reset its plot.
        self._stream = deque(maxlen=STREAM_MAXLEN)
        self._stream_i = 0
        self._stream_id = 0
        self._stream_kind = None   # "run" | "monitor"
        self._stream_t0 = time.monotonic()
        # Live monitor (no recording). Paused by instrument changes (_ctl_lock) so a slow
        # auto-range can't make a monitor query time out and drop the link.
        self._mon_thread = None
        self._mon_stop = threading.Event()
        self._ctl_lock = threading.Lock()

    def position_fn(self):
        # Always a copy: server.Machine.jog() mutates its pos dict in place, so keeping
        # the live reference would make the "start position" silently follow the stage
        # and the stationary check could never fail.
        return dict(self._position_fn() or {})

    # ---------- logging ----------
    def _log(self, direction, text):
        with self.lock:
            self.seq += 1
            self.log.append((self.seq, direction, text))
            if len(self.log) > 2000:
                self.log = self.log[-1500:]

    # ---------- connection ----------
    @staticmethod
    def list_resources():
        """VISA resources, rear-panel USB-TMC and GPIB/TCPIP alike. VisaInterface.find()
        already drops ASRL (serial) entries; the front-panel flash-drive port never
        shows up here at all -- it isn't a VISA interface."""
        try:
            from srsinst.sr860.instruments.visainterface import VisaInterface
            return VisaInterface.find()
        except Exception:
            return []

    def connect(self, resource=None):
        self.disconnect()
        resource = resource or self.cfg["resource"]
        if not resource:
            raise RuntimeError("no VISA resource selected")
        if self.mock:
            self.inst = MockSR860()
            self.inst.comm.set_timeout(VISA_TIMEOUT_S)
            model, serial, firmware = self.inst.check_id()
        else:
            model, serial, firmware = self._open_and_identify(resource)
        self.identity = {"model": model, "serial": serial, "firmware": firmware, "resource": resource}
        self.cfg["resource"] = resource
        self._log("sys", f"connected {model} SN{serial} FW{firmware} @ {resource}")
        self.ping()
        if self.cfg.get("monitor", True):
            self.start_monitor()
        return True

    # ---------- live monitor + sample stream ----------
    def _new_stream(self, kind):
        with self.lock:
            self._stream_id += 1
            self._stream_kind = kind
            self._stream.clear()
            self._stream_t0 = time.monotonic()

    def _push(self, sample):
        with self.lock:
            self._stream_i += 1
            sample["i"] = self._stream_i
            self._stream.append(sample)

    @property
    def monitoring(self):
        return self._mon_thread is not None and self._mon_thread.is_alive()

    def start_monitor(self):
        """Poll X/Y for the live view without writing anything to disk."""
        if not self.connected or self.run["state"] == "running" or self.monitoring:
            return {"ok": True}
        self._mon_stop.clear()
        self._new_stream("monitor")
        self._mon_thread = threading.Thread(target=self._monitor_worker, daemon=True)
        self._mon_thread.start()
        return {"ok": True}

    def stop_monitor(self):
        self._mon_stop.set()
        t = self._mon_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=5.0)
        return {"ok": True}

    def set_monitor(self, on):
        """The Live toggle: remembered in cfg so a run ending (or the next connect)
        resumes the live view only if the operator wants it."""
        self.cfg["monitor"] = bool(on)
        return self.start_monitor() if on else self.stop_monitor()

    def _monitor_worker(self):
        interval = 1.0 / max(0.5, min(float(self.cfg.get("monitor_rate_hz", 5.0)), MAX_RATE_HZ))
        check_ovld = bool(self.cfg.get("check_overload", True))
        t0 = time.monotonic()
        while not self._mon_stop.is_set():
            with self._ctl_lock:  # an instrument change in progress owns the session
                if self._mon_stop.is_set() or self.inst is None:
                    break
                try:
                    x, y, r, theta, latency_ms = self.snap_xyrt()
                    status = "ok"
                    if check_ovld:
                        ovld = self.read_overload()
                        if ovld:
                            status = "overload:" + "+".join(ovld)
                except Exception as e:
                    self._drop_link(f"live view: {type(e).__name__}: {e}")
                    break
            self.latency_ms = latency_ms
            self._push({"elapsed_s": time.monotonic() - t0, "x": x, "y": y, "r": r, "theta": theta,
                        "status": status})
            self._mon_stop.wait(interval)

    def ping(self):
        """One *IDN? round trip: confirms the link and measures query latency (spec sec. 7
        connection panel). Refused during a run -- the run's own per-sample latency is
        shown then, and the run thread owns the session."""
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        if self.run["state"] in ("running", "settling"):
            return {"ok": False, "error": "a run is in progress (its sample latency is shown instead)"}
        with self._ctl_lock:
            if self.inst is None:
                return {"ok": False, "error": "not connected"}
            t0 = time.monotonic()
            try:
                model, _, _ = self.inst.check_id()
                if model is None:
                    raise LinkError("malformed *IDN? reply")
            except Exception as e:
                self._drop_link(f"ping: {type(e).__name__}: {e}")
                return {"ok": False, "error": f"no answer from the SR860 ({e}); press Connect to recover"}
            self.latency_ms = (time.monotonic() - t0) * 1000.0
        return {"ok": True, "latency_ms": round(self.latency_ms, 2)}

    def disconnect(self):
        # Join the worker before closing the instrument handle: closing a VISA
        # session while _run_worker is mid query_text() on the same handle is a
        # real race on hardware, not just a cosmetic state-lag issue.
        self.stop_run()
        if self._run_thread is not None and self._run_thread.is_alive():
            self._run_thread.join(timeout=5.0)
        self.stop_monitor()
        if self.inst is not None:
            try:
                self.inst.disconnect()
            except Exception:
                pass
            self._log("sys", "disconnected")
        self.inst = None
        self.identity = None

    @property
    def connected(self):
        return self.inst is not None

    def _open_and_identify(self, resource, attempts=3):
        """Open the VISA session and confirm an SR860 answers *IDN?. If it doesn't (a
        previous session left the USB endpoint stalled, or a stale reply comes back),
        close, USB-TMC-clear and try again. Learned on the bench (SR860 FW V1.51):
          - it holds ONE pending reply; a new query discards an unread one (IEEE 488.2
            query-interrupted), so a fresh *IDN? after a clear resynchronizes by itself;
          - reading with no reply pending stalls the bulk-IN endpoint (pipe error), and the
            stall survives closing/reopening the session -- only a USB-TMC clear fixes it.
        So: strictly one query -> one read, and clear only when the plain query fails."""
        from srsinst.sr860 import SR860  # lazy: optional dependency (see Machine/pyserial)
        last = None
        for attempt in range(attempts):
            try:
                self.inst = SR860("visa", resource)
                self.inst.comm.set_timeout(VISA_TIMEOUT_S)
                model, serial, firmware = self.inst.check_id()  # raises on a non-SR860 reply
                if model is not None:
                    if attempt:
                        self._log("sys", f"SR860 answering after USB-TMC clear (attempt {attempt + 1})")
                    self.inst.comm.send("*CLS")  # clear error bits a broken earlier session left
                    return model, serial, firmware
                last = "malformed *IDN? reply"
            except Exception as e:
                last = f"{type(e).__name__}: {e}"
            try:
                if self.inst is not None:
                    self.inst.disconnect()
            except Exception:
                pass
            self.inst = None
            if attempt < attempts - 1:
                status = usbtmc_clear(resource)
                self._log("sys", f"SR860 not answering ({last}); USB-TMC clear: {status or 'not applicable'}")
                time.sleep(0.2)
        raise LinkError(f"SR860 not answering on {resource} after {attempts} attempts ({last}). "
                        f"Check the rear-panel USB cable and that the instrument is on; "
                        f"if it persists, unplug/replug the USB cable.")

    def _drop_link(self, reason):
        """Close a session that can no longer be trusted (see LinkError). Called from the
        run worker, so it doesn't join the run thread the way disconnect() does."""
        inst, self.inst, self.identity = self.inst, None, None
        if inst is not None:
            try:
                inst.disconnect()
            except Exception:
                pass
        self._log("sys", f"SR860 link closed after a communication error ({reason}). "
                         f"Press Connect to recover: it re-checks the link and USB-TMC-clears it if needed.")

    # ---------- instrument reads ----------
    def snap_xyrt(self):
        """X and Y from one SNAP? 0,1 (FR-04: synchronous X/Y); R and theta derived.
        The SR860's SNAP? takes 2 or 3 parameters (not 4+ like the SR830) -- an invalid
        query stalls its USB endpoint. Deriving R = hypot(X, Y), theta = atan2(Y, X) also
        keeps them true values: the instrument's own R output has its display offset
        applied when one is enabled (on the bench R read -3.1e-8 V with a 2.21% offset on,
        true |X+iY| = 7.9e-8 V). theta matched the instrument's to 0.001 deg."""
        t0 = time.monotonic()
        reply = self.inst.comm.query_text("SNAP? 0,1")
        latency_ms = (time.monotonic() - t0) * 1000.0
        x, y = (float(v.strip()) for v in reply.strip().split(","))
        return x, y, math.hypot(x, y), math.degrees(math.atan2(y, x)), latency_ms

    def read_overload(self):
        """Current (not latched) overload bits, as names. Empty list = no overload."""
        bits = int(self.inst.status.overload)
        return [name for bit, name in OVERLOAD_BITS.items() if bits & (1 << bit)]

    def read_metadata(self):
        """Settings snapshot for the run's JSON sidecar (FR-06). Read once at run
        start and again at run end (to catch unlogged changes, sec. 11), not per
        sample -- re-querying a dozen settings every tick would wreck the poll rate.
        Stops at the first failed read (LinkError): after a failure the reply queue is
        out of step, so later "values" could belong to different settings."""
        inst = self.inst

        def get(name, fn):
            try:
                return fn()
            except Exception as e:
                raise LinkError(f"reading {name}: {type(e).__name__}: {e}") from e

        mode = get("input_mode", lambda: inst.signal.input_mode)
        meta = {
            "ref_hz": get("ref_hz", lambda: inst.ref.frequency),
            "ref_source": get("ref_source", lambda: inst.ref.reference_source),
            "ref_phase_deg": get("ref_phase", lambda: inst.ref.phase),
            "harmonic": get("harmonic", lambda: inst.ref.harmonic),
            "sine_out_amplitude_v": get("sine_out_amplitude", lambda: inst.ref.sine_out_amplitude),
            "time_constant_s": get("time_constant", lambda: inst.signal.time_constant),
            "filter_slope_db_oct": get("filter_slope", lambda: inst.signal.filter_slope),
            "sync_filter": get("sync_filter", lambda: inst.signal.sync_filter),
            "input_mode": mode,
        }
        if mode == "current":
            meta["sensitivity_a"] = get("sensitivity", lambda: inst.signal.current_sensitivity)
            meta["current_input_gain_ohm"] = get("current_input_gain", lambda: inst.signal.current_input_gain)
        else:
            meta["sensitivity_v"] = get("sensitivity", lambda: inst.signal.voltage_sensitivity)
            meta["voltage_input_config"] = get("voltage_input_mode", lambda: inst.signal.voltage_input_mode)
            meta["voltage_input_coupling"] = get("voltage_input_coupling", lambda: inst.signal.voltage_input_coupling)
            meta["voltage_input_shield"] = get("voltage_input_shield", lambda: inst.signal.voltage_input_shield)
            meta["voltage_input_range_v"] = get("voltage_input_range", lambda: inst.signal.voltage_input_range)
        # Output offset/expand/ratio change the X/Y values the instrument reports (spec
        # sec. 12 H: recorded units and signs must match the front panel).
        out = {}
        for k in ("X", "Y", "R"):
            out[k] = {"offset_on": get(f"{k} offset", lambda k=k: inst.output.offset[k]),
                      "offset_pct": get(f"{k} offset %", lambda k=k: inst.output.offset_percent[k]),
                      "expand": get(f"{k} expand", lambda k=k: inst.output.expand[k]),
                      "ratio": get(f"{k} ratio", lambda k=k: inst.output.ratio_mode[k])}
        meta["output_processing"] = out
        return meta

    # ---------- timed run ----------
    def start_run(self, duration_s, rate_hz, data_dir, condition="", notes="", run_id=None, settle_s=None):
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        if self.run["state"] in ("running", "settling"):
            return {"ok": False, "error": "run already in progress"}
        try:
            rate_hz = float(rate_hz)
            duration_s = None if duration_s in (None, "") else float(duration_s)
            settle_s = float(self.cfg.get("settle_s", 0.0) if settle_s in (None, "") else settle_s)
        except (TypeError, ValueError):
            return {"ok": False, "error": "rate, duration and settle must be numbers"}
        if not 0 < rate_hz <= MAX_RATE_HZ:
            return {"ok": False, "error": f"rate must be in (0, {MAX_RATE_HZ:g}] Hz"}
        if duration_s is not None and duration_s <= 0:
            return {"ok": False, "error": "duration must be positive (leave blank to run until stopped)"}
        if settle_s < 0:
            return {"ok": False, "error": "settle time can't be negative"}

        self.stop_monitor()  # the run owns the session from here
        try:
            meta = self.read_metadata()
        except Exception as e:
            self._drop_link(str(e))
            return {"ok": False, "error": f"couldn't read SR860 settings ({e}); press Connect to recover the link"}
        xy_mods = _xy_modifiers(meta)
        if xy_mods:
            self._log("sys", f"WARNING: SR860 output settings modify the recorded X/Y: {', '.join(xy_mods)}")
        pos = self.position_fn()
        # run_id lands in the filename: keep it to safe characters so it can't add path
        # separators or "..", i.e. can't write outside data_dir.
        run_id = re.sub(r"[^A-Za-z0-9_-]+", "-", str(run_id or "")).strip("-") or time.strftime("Run%H%M%S")
        out_dir = Path(data_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        # Filenames carry position + frequency per spec section 8, so a run is
        # self-describing on disk even before anyone opens the JSON sidecar.
        freq = f"{meta['ref_hz']:.0f}Hz" if meta["ref_hz"] is not None else "naHz"
        base = (f"AE_{time.strftime('%Y%m%d')}_X{_fmt_coord(pos.get('X'))}_Y{_fmt_coord(pos.get('Y'))}"
                f"_Z{_fmt_coord(pos.get('Z'))}_{freq}_{run_id}")
        stem, n = base, 2
        while (out_dir / f"{stem}.csv").exists() or (out_dir / f"{stem}.json").exists():
            stem, n = f"{base}_{n}", n + 1  # never overwrite an earlier run (spec sec. 8)
        csv_path, json_path = out_dir / f"{stem}.csv", out_dir / f"{stem}.json"

        self._run_stop.clear()
        self._run_abort.clear()
        self.run = {**_idle_run(), "state": "settling" if settle_s > 0 else "running",
                    "started": time.time(), "settle_s": settle_s, "duration_s": duration_s,
                    "rate_hz": rate_hz, "run_id": run_id, "path": str(csv_path)}
        self._run_thread = threading.Thread(
            target=self._run_worker,
            args=(duration_s, rate_hz, csv_path, json_path, meta, pos, condition, notes, settle_s),
            daemon=True)
        self._run_thread.start()
        if settle_s > 0 and self.cfg.get("monitor", True):
            self.start_monitor()  # watch the signal settle; the run stops it when recording begins
        self._log("sys", f"run started: {csv_path.name} ({rate_hz:g} Hz, "
                         f"{'until stopped' if duration_s is None else f'{duration_s:g} s'})")
        return {"ok": True, "path": str(csv_path)}

    def _run_worker(self, duration_s, rate_hz, csv_path, json_path, meta, start_pos, condition, notes, settle_s):
        # Settling dwell happens before the file is even opened, so a run stopped
        # while settling leaves no empty CSV behind (spec sec. 9 step 3).
        settle_deadline = time.monotonic() + settle_s
        while time.monotonic() < settle_deadline:
            if self._run_stop.is_set():
                self.run["state"] = "aborted" if self._run_abort.is_set() else "stopped"
                self.run["msg"] = "ended during settling, nothing recorded"
                return
            time.sleep(min(0.1, max(0.0, settle_deadline - time.monotonic())))
        self.stop_monitor()
        if self.inst is None:  # the live view lost the link while settling
            self.run["state"], self.run["msg"] = "error", "SR860 link lost while settling, nothing recorded"
            return
        self.run["state"] = "running"
        self._new_stream("run")

        check_ovld = bool(self.cfg.get("check_overload", True))
        interval = 1.0 / rate_hz
        t_start = time.monotonic()
        start_utc = datetime.now(timezone.utc)
        n_ok = n_err = n_ovld = 0
        ok_values = []    # (x, y, r) of status == "ok" samples, for the summary stats
        tick_times = []   # elapsed time of every attempted sample, for achieved rate + jitter
        latencies = []
        comm_error = None  # set on the first failed query; the run ends there (see LinkError)
        try:
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(RUN_COLUMNS)
                f.flush()
                next_tick = t_start
                while not self._run_stop.is_set():
                    elapsed = time.monotonic() - t_start
                    if duration_s is not None and elapsed >= duration_s:
                        break
                    tick_times.append(elapsed)
                    pos = self.position_fn()
                    try:
                        x, y, r, theta, latency_ms = self.snap_xyrt()
                        status = "ok"
                        if check_ovld:
                            ovld = self.read_overload()
                            if ovld:
                                status = "overload:" + "+".join(ovld)
                    except Exception as e:
                        # Never fill a failed read with a fabricated zero (FR-08): leave the
                        # value columns blank and flag the row. Also never carry on after it:
                        # the next reply could be this query's late answer (stale X/Y).
                        x = y = r = theta = latency_ms = ""
                        status = "error"
                        n_err += 1
                        comm_error = f"{type(e).__name__}: {e}"
                        self._log("sys", f"sample failed: {comm_error}")
                    else:
                        n_ok += 1
                        latencies.append(latency_ms)
                        if status == "ok":
                            ok_values.append((x, y, r))
                        else:
                            n_ovld += 1
                        self._push({"elapsed_s": elapsed, "x": x, "y": y, "r": r, "theta": theta,
                                    "status": status})
                    writer.writerow([
                        datetime.now(timezone.utc).isoformat(), f"{elapsed:.4f}",
                        _csv_num(pos.get("X")), _csv_num(pos.get("Y")), _csv_num(pos.get("Z")),
                        _csv_num(meta["ref_hz"]),
                        x, y, r, theta, status,
                        f"{latency_ms:.2f}" if latency_ms != "" else "",
                    ])
                    f.flush()  # every row on disk immediately: a crash or yanked cable loses nothing written
                    self.run["index"] = n_ok + n_err
                    self.run["n_errors"] = n_err
                    self.run["n_overload"] = n_ovld
                    self.run["last_status"] = status
                    self.run["last_written_utc"] = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                    if latency_ms != "":
                        self.run["last_latency_ms"] = round(latency_ms, 2)
                        self.latency_ms = latency_ms
                    if comm_error:
                        break
                    next_tick += interval
                    sleep_for = next_tick - time.monotonic()
                    if sleep_for > 0:
                        self._run_stop.wait(sleep_for)  # wakes immediately on Stop/Abort
                    else:
                        next_tick = time.monotonic()  # fell behind: resync instead of a busy catch-up burst

            elapsed_total = time.monotonic() - t_start
            end_utc = datetime.now(timezone.utc)
            end_pos = self.position_fn()
            stationary = _positions_match(start_pos, end_pos)
            if not stationary:
                self._log("sys", f"WARNING: stage moved during run (start {start_pos} -> end {end_pos})")
            # Re-read settings to catch knob changes made mid-run (spec sec. 11). Skipped
            # when the link is broken -- it can't be trusted for anything until reconnected.
            settings_changed = None
            if not comm_error:
                try:
                    end_meta = self.read_metadata()
                except Exception as e:
                    comm_error = f"re-reading settings at run end: {e}"
                else:
                    settings_changed = [k for k, v in meta.items() if end_meta.get(k) != v]
                    if settings_changed:
                        self._log("sys", f"WARNING: instrument settings changed during run: {', '.join(settings_changed)}")
            if comm_error:
                self._drop_link(comm_error)

            if comm_error:
                state = "error"
            elif self._run_abort.is_set():
                state = "aborted"
            elif self._run_stop.is_set():
                state = "stopped"
            else:
                state = "done"
            # An open-ended run (no duration) ended by Stop is complete by definition.
            incomplete = (state in ("error", "aborted")
                          or (state == "stopped" and duration_s is not None and elapsed_total < duration_s))
            summary = {
                "file": str(csv_path),
                "end_state": state,
                "end_reason": f"communication error: {comm_error}" if comm_error else state,
                "incomplete": incomplete,
                "n_samples": n_ok,
                "n_errors": n_err,
                "n_overload": n_ovld,
                "duration_s": round(elapsed_total, 3),
                "target_rate_hz": rate_hz,
                **_timing_stats(tick_times, latencies),
                "stationary": stationary,
                "settings_changed": settings_changed,
                "xy_output_processing": _xy_modifiers(meta),
                "stats": _value_stats(ok_values),
            }
            doc = {
                "software": "xyz-stage gantry console / daq.py",
                "connection": {"type": "USB-TMC (VISA)" if not self.mock else "mock",
                               "resource": (self.identity or {}).get("resource")},
                "instrument": self.identity,
                "position": start_pos,
                "end_position": end_pos,
                "sampling": {"target_rate_hz": rate_hz, "duration_s": duration_s, "settle_s": settle_s,
                             "check_overload": check_ovld},
                "instrument_settings": meta,
                "condition": condition,
                "notes": notes,
                "start_utc": start_utc.isoformat(),
                "end_utc": end_utc.isoformat(),
                "xy_source": "SNAP? 0,1 (X and Y from the same instant)",
                "r_theta_source": "derived: R = hypot(X, Y), theta = atan2(Y, X) in degrees "
                                  "(not the instrument's R output, which includes its R offset when enabled)",
                "stats_over": "status == ok samples only (overload and error rows excluded)",
                **summary,
            }
            json_path.write_text(json.dumps(doc, indent=2))
            self.run["state"] = state
            self.run["summary"] = summary
            self.run["msg"] = (f"{n_ok} samples, {n_err} errors, {n_ovld} overloaded"
                               + (" · COMMUNICATION ERROR: reconnect" if comm_error else "")
                               + (" · STAGE MOVED" if not stationary else "")
                               + (" · SETTINGS CHANGED" if settings_changed else "")
                               + (" · INCOMPLETE" if incomplete else ""))
            self._log("sys", f"run {state}: {self.run['msg']} -> {csv_path.name}")
            if self.connected and self.cfg.get("monitor", True):
                self.start_monitor()
        except Exception as e:
            self.run["state"] = "error"
            self.run["msg"] = str(e)
            self._log("sys", f"run failed: {e}")

    def stop_run(self):
        """End the run now: close files cleanly, write the JSON sidecar. A timed run
        ended early is marked incomplete; an open-ended run is not."""
        self._run_stop.set()
        return {"ok": True}

    def abort_run(self):
        """Like Stop, but the run is always marked aborted/incomplete. Data already
        written is kept either way -- nothing is ever deleted."""
        self._run_abort.set()
        self._run_stop.set()
        return {"ok": True}

    # ---------- state ----------
    def state(self, since=0, sample_since=0):
        with self.lock:
            log = [entry for entry in self.log if entry[0] > since][-400:]
            # Only samples the client hasn't seen ("i" > sample_since): keeps each poll small
            # even with a 10-minute plot window.
            samples = [s for s in self._stream if s["i"] > sample_since][-STREAM_MAX_PER_POLL:]
            stream = {"id": self._stream_id, "kind": self._stream_kind}
        return {"connected": self.connected, "mock": self.mock, "identity": self.identity,
                "latency_ms": None if self.latency_ms is None else round(self.latency_ms, 2),
                "monitoring": self.monitoring, "stream": stream,
                "run": dict(self.run), "samples": samples, "log": log, "cfg": self.cfg}

    # ---------- instrument controls ----------
    def settings(self):
        """Current values of every control (CONTROLS), read from the instrument."""
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        if self.run["state"] in ("running", "settling"):
            return {"ok": False, "error": "a run is in progress"}
        with self._ctl_lock:
            try:
                values = {c["name"]: _ctl_get(self.inst, c) for c in CONTROLS}
            except Exception as e:
                self._drop_link(f"reading settings: {type(e).__name__}: {e}")
                return {"ok": False, "error": f"couldn't read settings ({e}); press Connect to recover"}
        return {"ok": True, "values": values}

    def set_control(self, name, value):
        """Change one SR860 setting, then read it back to confirm it took. Refused while
        recording: the run's settings snapshot (and the settings-changed check) would
        otherwise no longer describe the data."""
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        if self.run["state"] in ("running", "settling"):
            return {"ok": False, "error": "can't change SR860 settings while recording"}
        c = CONTROLS_BY_NAME.get(name)
        if c is None:
            return {"ok": False, "error": f"unknown setting {name!r}"}
        try:
            value = _ctl_validate(c, value)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        with self._ctl_lock:
            try:
                before = _ctl_get(self.inst, c)
                _ctl_set(self.inst, c, value)
                after = _ctl_get(self.inst, c)
            except Exception as e:
                self._drop_link(f"setting {name}: {type(e).__name__}: {e}")
                return {"ok": False, "error": f"SR860 didn't answer ({e}); press Connect to recover"}
        self._log("sys", f"{c['label']}: {_fmt_ctl(before)} -> {_fmt_ctl(after)}")
        if not _ctl_equal(c, after, value):
            return {"ok": False, "value": after,
                    "error": f"SR860 kept {c['label']} at {_fmt_ctl(after)} (asked for {_fmt_ctl(value)})"}
        return {"ok": True, "value": after}

    def action(self, name):
        """Auto phase / auto range / auto scale. These change settings, so like set_control
        they're refused while recording; the new values are read back and logged."""
        if not self.connected:
            return {"ok": False, "error": "not connected"}
        if self.run["state"] in ("running", "settling"):
            return {"ok": False, "error": "can't run auto functions while recording"}
        a = ACTIONS.get(name)
        if a is None:
            return {"ok": False, "error": f"unknown action {name!r}"}
        with self._ctl_lock:
            try:
                before = {n: _ctl_get(self.inst, CONTROLS_BY_NAME[n]) for n in a["affects"]}
                getattr(self.inst.auto, a["method"])()
                # Auto range/scale can take a while: wait for the readback with a long timeout
                # instead of letting a 2 s timeout break the link.
                self.inst.comm.set_timeout(20.0)
                try:
                    time.sleep(a["settle_s"])
                    after = {n: _ctl_get(self.inst, CONTROLS_BY_NAME[n]) for n in a["affects"]}
                finally:
                    self.inst.comm.set_timeout(VISA_TIMEOUT_S)
            except Exception as e:
                self._drop_link(f"{a['label']}: {type(e).__name__}: {e}")
                return {"ok": False, "error": f"SR860 didn't answer ({e}); press Connect to recover"}
        changes = ", ".join(f"{CONTROLS_BY_NAME[n]['label']} {_fmt_ctl(before[n])} -> {_fmt_ctl(after[n])}"
                            for n in a["affects"] if before[n] != after[n]) or "no change"
        self._log("sys", f"{a['label']}: {changes}")
        return {"ok": True, "values": after}


def _idle_run():
    return {"state": "idle", "index": 0, "n_errors": 0, "n_overload": 0, "started": None,
            "settle_s": 0, "duration_s": None, "rate_hz": None, "run_id": None, "path": None,
            "msg": "", "summary": None, "last_status": None, "last_written_utc": None,
            "last_latency_ms": None}


def _fmt_coord(v):
    return "na" if v is None else f"{v:g}"


def _csv_num(v):
    return "" if v is None else v


def _xy_modifiers(meta):
    """SR860 output settings that change what X/Y mean in the CSV (offset, expand, ratio).
    The R offset doesn't matter here: R is derived from X/Y, not read from the instrument."""
    mods = []
    for k in ("X", "Y"):
        o = (meta.get("output_processing") or {}).get(k) or {}
        if o.get("offset_on"):
            mods.append(f"{k} offset {o.get('offset_pct')}%")
        if o.get("expand") not in (None, 1):
            mods.append(f"{k} expand x{o.get('expand')}")
        if o.get("ratio"):
            mods.append(f"{k} ratio mode")
    return mods


# ---------- instrument controls ----------
# Option values mirror srsinst.sr860 Signal/Reference dicts, generated the same way the driver
# builds them so floats compare equal to its keys (duplicated here so --mock needs no driver).
_V_SENS = [1.0] + [float(f"{j * 10 ** -(i + 1):.1e}") for i in range(9) for j in (5.0, 2.0, 1.0)]
_A_SENS = [1e-6] + [float(f"{j * 10 ** -(i + 7):.1e}") for i in range(9) for j in (5.0, 2.0, 1.0)]
_TCS = [float(f"{j * 10 ** (i - 6):.1e}") for i in range(11) for j in (1.0, 3.0)]


def _ctl(name, label, path, kind, group, options=None, lo=None, hi=None, unit="", rtol=0.0, atol=0.0, mode=None):
    return {"name": name, "label": label, "path": path, "kind": kind, "group": group, "options": options,
            "min": lo, "max": hi, "unit": unit, "rtol": rtol, "atol": atol, "mode": mode}


CONTROLS = [
    _ctl("sensitivity_v", "Sensitivity", ("signal", "voltage_sensitivity"), "choice", "Gain & filter", _V_SENS, unit="V", mode="voltage"),
    _ctl("sensitivity_a", "Sensitivity", ("signal", "current_sensitivity"), "choice", "Gain & filter", _A_SENS, unit="A", mode="current"),
    _ctl("time_constant", "Time constant", ("signal", "time_constant"), "choice", "Gain & filter", _TCS, unit="s"),
    _ctl("filter_slope", "Filter slope", ("signal", "filter_slope"), "choice", "Gain & filter", [6, 12, 18, 24], unit="dB/oct"),
    _ctl("sync_filter", "Sync filter", ("signal", "sync_filter"), "choice", "Gain & filter", ["off", "on"]),
    _ctl("input_mode", "Input", ("signal", "input_mode"), "choice", "Input", ["voltage", "current"]),
    _ctl("voltage_input_mode", "Config", ("signal", "voltage_input_mode"), "choice", "Input", ["A", "A-B"], mode="voltage"),
    _ctl("voltage_input_coupling", "Coupling", ("signal", "voltage_input_coupling"), "choice", "Input", ["AC", "DC"], mode="voltage"),
    _ctl("voltage_input_shield", "Shield", ("signal", "voltage_input_shield"), "choice", "Input", ["float", "ground"], mode="voltage"),
    _ctl("voltage_input_range", "Range", ("signal", "voltage_input_range"), "choice", "Input", [1.0, 0.3, 0.1, 0.03, 0.01], unit="V", mode="voltage"),
    _ctl("current_input_gain", "Current gain", ("signal", "current_input_gain"), "choice", "Input", [1e6, 1e8], unit="V/A", mode="current"),
    _ctl("reference_source", "Source", ("ref", "reference_source"), "choice", "Reference", ["internal", "external", "dual", "chop"]),
    # SR860 limits from the driver's FloatCommand declarations (FREQ 0.001 Hz-500 kHz, SLVL 0-2 V);
    # readback tolerances from their significant figures (FREQ 6, SLVL 3, PHAS 7).
    _ctl("frequency", "Frequency", ("ref", "frequency"), "float", "Reference", lo=0.001, hi=500000.0, unit="Hz", rtol=1e-5, atol=1e-3),
    _ctl("phase", "Phase", ("ref", "phase"), "float", "Reference", lo=-360000.0, hi=360000.0, unit="°", atol=1e-3),
    _ctl("harmonic", "Harmonic", ("ref", "harmonic"), "int", "Reference", lo=1, hi=99),
    _ctl("sine_out_amplitude", "Sine out", ("ref", "sine_out_amplitude"), "float", "Reference", lo=0.0, hi=2.0, unit="V", rtol=1e-3, atol=1e-9),
    _ctl("offset_X", "X offset on", ("output", "offset", "X"), "bool", "Output offsets"),
    _ctl("offset_Y", "Y offset on", ("output", "offset", "Y"), "bool", "Output offsets"),
    _ctl("offset_R", "R offset on", ("output", "offset", "R"), "bool", "Output offsets"),
]
CONTROLS_BY_NAME = {c["name"]: c for c in CONTROLS}

ACTIONS = {
    "auto_phase": {"label": "Auto phase", "method": "set_phase", "affects": ["phase"], "settle_s": 0.5},
    "auto_range": {"label": "Auto range", "method": "set_range", "affects": ["voltage_input_range"], "settle_s": 1.0},
    "auto_scale": {"label": "Auto scale", "method": "set_scale", "affects": ["sensitivity_v"], "settle_s": 1.0},
}


def _ctl_get(inst, c):
    obj = getattr(inst, c["path"][0])
    if len(c["path"]) == 3:
        return getattr(obj, c["path"][1])[c["path"][2]]
    return getattr(obj, c["path"][1])


def _ctl_set(inst, c, value):
    obj = getattr(inst, c["path"][0])
    if len(c["path"]) == 3:
        getattr(obj, c["path"][1])[c["path"][2]] = value
    else:
        setattr(obj, c["path"][1], value)


def _ctl_validate(c, value):
    """Return the exact value to send (the driver's own key for choices), or raise ValueError.
    Nothing outside the instrument's documented set/range is ever sent."""
    if c["kind"] == "choice":
        for o in c["options"]:
            if o == value:
                return o
            if isinstance(o, float) and isinstance(value, (int, float)) and abs(o - value) <= 1e-9 * abs(o):
                return o
        raise ValueError(f"{c['label']}: {value!r} isn't one of the SR860's values")
    if c["kind"] == "bool":
        if isinstance(value, bool):
            return value
        raise ValueError(f"{c['label']}: expected true/false")
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{c['label']}: {value!r} isn't a number")
    if not math.isfinite(v) or not c["min"] <= v <= c["max"]:
        raise ValueError(f"{c['label']} must be between {c['min']:g} and {c['max']:g} {c['unit']}".strip())
    if c["kind"] == "int":
        if v != int(v):
            raise ValueError(f"{c['label']} must be a whole number")
        return int(v)
    return v


def _ctl_equal(c, actual, wanted):
    if c["kind"] == "float":
        return actual is not None and abs(float(actual) - wanted) <= max(c["atol"], c["rtol"] * abs(wanted))
    if c["kind"] == "bool":
        return bool(actual) == bool(wanted)
    return actual == wanted


def _fmt_ctl(v):
    return f"{v:g}" if isinstance(v, float) else str(v)


def usbtmc_clear(resource):
    """USB-TMC device clear straight over pyusb: INITIATE_CLEAR, CHECK_CLEAR_STATUS, then
    clear the bulk-endpoint halts. Needed because on the bench (SR860 + pyvisa-py on
    macOS) one failed transfer left the endpoint stalled across close/reopen and even a
    USB port reset, and pyvisa-py has no viClear for USB. Returns a short status string,
    or None when it doesn't apply (not a USB resource, pyusb unavailable -- e.g. NI-VISA
    on Windows, which does its own clear). Never raises."""
    m = re.match(r"USB\d*::([^:]+)::([^:]+)::([^:]+)::", resource or "")
    if not m:
        return None
    try:
        import usb.core
        import usb.util
    except ImportError:
        return None

    def num(s):
        return int(s, 16) if s.lower().startswith("0x") else int(s)

    try:
        vid, pid, serial = num(m.group(1)), num(m.group(2)), m.group(3)
        devices = list(usb.core.find(find_all=True, idVendor=vid, idProduct=pid))
    except Exception as e:
        return f"skipped ({type(e).__name__}: {e})"
    for dev in devices:
        try:
            if usb.util.get_string(dev, dev.iSerialNumber) != serial:
                continue
            intf = next(i for i in dev.get_active_configuration()
                        if i.bInterfaceClass == 0xFE and i.bInterfaceSubClass == 0x03)  # USB-TMC
            n = intf.bInterfaceNumber
            usb.util.claim_interface(dev, n)
            try:
                status = dev.ctrl_transfer(0xA1, 5, 0, n, 1, timeout=2000)[0]  # INITIATE_CLEAR
                if status == 0x01:
                    for _ in range(40):
                        st = dev.ctrl_transfer(0xA1, 6, 0, n, 2, timeout=2000)  # CHECK_CLEAR_STATUS
                        if st[0] != 0x02:  # 0x02 = still pending
                            break
                        time.sleep(0.05)
                for ep in intf:
                    dev.clear_halt(ep.bEndpointAddress)
            finally:
                usb.util.release_interface(dev, n)
            return "ok"
        except Exception as e:
            return f"failed ({type(e).__name__}: {e})"
        finally:
            usb.util.dispose_resources(dev)
    return "device not found on USB"


def _positions_match(a, b, tol=1e-6):
    for axis in ("X", "Y", "Z"):
        av, bv = a.get(axis), b.get(axis)
        if av is None or bv is None:
            continue  # unreferenced axis: nothing to compare, not a mismatch
        if abs(av - bv) > tol:
            return False
    return True


def _mean_sd(vals):
    n = len(vals)
    mean = sum(vals) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in vals) / (n - 1)) if n > 1 else 0.0
    return mean, sd


def _timing_stats(tick_times, latencies):
    """Achieved rate and poll jitter (FR-03, sec. 11) from the actual sample times,
    not the requested rate."""
    out = {"achieved_rate_hz": None, "interval_mean_ms": None, "interval_sd_ms": None,
           "interval_max_ms": None, "latency_mean_ms": None, "latency_max_ms": None}
    if len(tick_times) > 1:
        intervals = [b - a for a, b in zip(tick_times, tick_times[1:])]
        mean, sd = _mean_sd(intervals)
        span = tick_times[-1] - tick_times[0]
        out.update(achieved_rate_hz=round((len(tick_times) - 1) / span, 3) if span > 0 else None,
                   interval_mean_ms=round(mean * 1000, 2), interval_sd_ms=round(sd * 1000, 2),
                   interval_max_ms=round(max(intervals) * 1000, 2))
    if latencies:
        out.update(latency_mean_ms=round(sum(latencies) / len(latencies), 2),
                   latency_max_ms=round(max(latencies), 2))
    return out


def _value_stats(ok_values):
    if not ok_values:
        return {}
    out = {}
    for i, key in enumerate(("X_V", "Y_V", "R_V")):
        vals = [v[i] for v in ok_values]
        mean, sd = _mean_sd(vals)
        out[key] = {"mean": mean, "sd": sd, "min": min(vals), "max": max(vals), "n": len(vals)}
    # Phase of the mean complex response, not the mean of per-sample phases: averaging
    # angles breaks across the +/-180 wrap, and the complex mean is what the blocked/
    # unblocked comparison (spec sec. 10) is built on.
    mx, my = out["X_V"]["mean"], out["Y_V"]["mean"]
    out["mean_complex"] = {"magnitude_v": math.hypot(mx, my), "phase_deg": math.degrees(math.atan2(my, mx))}
    return out


# ---------- history (FR-10) ----------
def list_runs(data_dir):
    """Every run in data_dir, newest first, paired by basename with its .json
    sidecar when one exists (a run still in progress may only have the CSV)."""
    out_dir = Path(data_dir)
    if not out_dir.is_dir():
        return []
    runs = []
    # AE_* only: COMPARE_*.csv results live in the same folder and aren't runs.
    for csv_path in sorted(out_dir.glob("AE_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True):
        json_path = csv_path.with_suffix(".json")
        runs.append({"name": csv_path.stem, "has_json": json_path.exists(),
                     "mtime": csv_path.stat().st_mtime, "size": csv_path.stat().st_size})
    return runs


def _resolve_run_file(data_dir, name, ext):
    """name comes from the client -- reject anything that isn't a bare filename
    stem so a crafted name can't escape data_dir (e.g. "../../etc/passwd")."""
    if ext not in ("csv", "json"):
        raise ValueError("ext must be csv or json")
    if not name or "/" in name or "\\" in name or name in (".", ".."):
        raise ValueError("invalid run name")
    base = Path(data_dir).resolve()
    path = (base / f"{name}.{ext}").resolve()
    if path.parent != base:
        raise ValueError("invalid run name")
    return path


def read_run_csv(data_dir, name):
    path = _resolve_run_file(data_dir, name, "csv")
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def read_run_json(data_dir, name):
    path = _resolve_run_file(data_dir, name, "json")
    return json.loads(path.read_text())


SUMMARY_COLUMNS = ["run", "start_utc", "condition", "notes", "x_mm", "y_mm", "z_mm", "ref_hz",
                   "time_constant_s", "end_state", "incomplete", "n_samples", "n_errors", "n_overload",
                   "achieved_rate_hz", "X_mean_V", "X_sd_V", "Y_mean_V", "Y_sd_V", "R_mean_V", "R_sd_V",
                   "mag_V", "phase_deg", "stationary", "settings_changed"]


def summarize_runs(data_dir):
    """One row per run (from its JSON sidecar) for the summary CSV export (FR-10)."""
    rows = []
    for run in list_runs(data_dir):
        if not run["has_json"]:
            continue
        try:
            j = read_run_json(data_dir, run["name"])
        except Exception:
            continue
        st, pos, inst = j.get("stats") or {}, j.get("position") or {}, j.get("instrument_settings") or {}
        X, Y, R, mc = st.get("X_V", {}), st.get("Y_V", {}), st.get("R_V", {}), st.get("mean_complex", {})
        rows.append({
            "run": run["name"], "start_utc": j.get("start_utc"), "condition": j.get("condition"),
            "notes": j.get("notes"), "x_mm": pos.get("X"), "y_mm": pos.get("Y"), "z_mm": pos.get("Z"),
            "ref_hz": inst.get("ref_hz"), "time_constant_s": inst.get("time_constant_s"),
            "end_state": j.get("end_state"), "incomplete": j.get("incomplete"),
            "n_samples": j.get("n_samples"), "n_errors": j.get("n_errors"), "n_overload": j.get("n_overload"),
            "achieved_rate_hz": j.get("achieved_rate_hz"),
            "X_mean_V": X.get("mean"), "X_sd_V": X.get("sd"), "Y_mean_V": Y.get("mean"), "Y_sd_V": Y.get("sd"),
            "R_mean_V": R.get("mean"), "R_sd_V": R.get("sd"),
            "mag_V": mc.get("magnitude_v"), "phase_deg": mc.get("phase_deg"),
            "stationary": j.get("stationary"), "settings_changed": ";".join(j.get("settings_changed") or []),
        })
    return rows


# ---------- mock instrument ----------
class MockSR860:
    """Fakes just enough of the srsinst.sr860 API surface (check_id, comm.query_text,
    ref.*, signal.*, status.overload) to drive the UI away from the bench."""

    def __init__(self):
        self._t0 = time.monotonic()
        self.ref = _MockRef()
        self.signal = _MockSignal()
        self.status = _MockStatus()
        self.output = _MockOutput()
        self.auto = _MockAuto(self)
        self.comm = self  # query_text lives here so snap_xyrt()'s inst.comm.query_text(...) works unchanged

    def check_id(self):
        return "SR860", "MOCK0001", "v1.50"

    def set_timeout(self, seconds):
        pass

    def query_text(self, cmd):
        if cmd.startswith("SNAP?"):
            params = [p.strip() for p in cmd[5:].split(",")]
            if not 2 <= len(params) <= 3:
                # Like the real SR860 (which also stalls its USB endpoint on this).
                raise IOError(f"SR860 SNAP? takes 2 or 3 parameters, got {len(params)}")
            t = time.monotonic() - self._t0
            x = 1e-6 * math.sin(2 * math.pi * 0.2 * t)
            y = 1e-6 * math.cos(2 * math.pi * 0.2 * t)
            r_display = math.hypot(x, y) - 0.0221 * 5e-6  # R output with a 2.21% offset on, as on the bench
            vals = {"0": x, "1": y, "2": r_display, "3": math.degrees(math.atan2(y, x))}
            return ",".join(f"{vals[p]:.10e}" for p in params) + "\n"
        return "0\n"

    def disconnect(self):
        pass


class _MockRef:
    frequency = 1000.0
    reference_source = "internal"
    phase = 0.0
    harmonic = 1
    sine_out_amplitude = 0.0


class _MockSignal:
    input_mode = "voltage"
    time_constant = 0.1
    filter_slope = 24
    sync_filter = "off"
    voltage_sensitivity = 1e-6
    voltage_input_mode = "A"
    voltage_input_coupling = "AC"
    voltage_input_shield = "float"
    voltage_input_range = 1.0
    current_sensitivity = 1e-6
    current_input_gain = 1e6


class _MockStatus:
    overload = 0


class _MockAuto:
    def __init__(self, inst):
        self._inst = inst

    def set_phase(self):
        self._inst.ref.phase = 37.5

    def set_range(self):
        self._inst.signal.voltage_input_range = 0.1

    def set_scale(self):
        self._inst.signal.voltage_sensitivity = 2e-06


class _MockOutput:
    # Matches the bench SR860: only the R offset is on.
    def __init__(self):
        self.offset = {"X": False, "Y": False, "R": True}
        self.offset_percent = {"X": -1.06, "Y": -0.04, "R": 2.21}
        self.expand = {"X": 1, "Y": 1, "R": 1}
        self.ratio_mode = {"X": False, "Y": False, "R": False}
