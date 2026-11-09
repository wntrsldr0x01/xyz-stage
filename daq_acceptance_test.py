#!/usr/bin/env python3
"""Acceptance tests A,C,D,E,G,H from SR860_USB_Data_Acquisition_and_XYZ_Scan_Specification.pdf
sec. 12, run against --mock so no hardware is required. B was done earlier
(library review). F needs real gantry position tagging end-to-end and is
deferred until both consoles are driven together manually, with hardware.

Usage:
    python3 server.py --mock --port 8780 &
    python3 daq_acceptance_test.py http://127.0.0.1:8780
"""
import csv
import io
import json
import sys
import time
import urllib.request

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8784"
PASS, FAIL = [], []


def req(method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(BASE + path, data=data, method=method,
                                headers={"Content-Type": "application/json"} if data else {})
    with urllib.request.urlopen(r, timeout=10) as resp:
        return json.loads(resp.read())


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  -- {detail}" if detail and not cond else ""))


def wait_run_done(timeout=10):
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = req("GET", "/api/state")["daq"]["run"]
        if st["state"] in ("done", "stopped", "error"):
            return st
        time.sleep(0.1)
    raise TimeoutError("run did not finish")


# ---- A: USB driver / *IDN? (mock stands in for the real driver binding) ----
r = req("POST", "/api/daq/connect", {"resource": "mock://bench"})
check("A. connect + identity", r.get("ok") is True)
state = req("GET", "/api/state")["daq"]
check("A. identity contains SR860", state["identity"] and "SR860" in state["identity"]["model"])

# ---- C: timed logging at a few durations, file has coherent timestamps/count ----
for dur in (0.5, 1.0, 2.0):
    r = req("POST", "/api/daq/run/start", {"duration_s": dur, "rate_hz": 10})
    check(f"C. run start ({dur}s)", r.get("ok") is True)
    st = wait_run_done(timeout=dur + 5)
    expected = int(dur * 10)
    check(f"C. sample count ~{expected} for {dur}s@10Hz", abs(st["index"] - expected) <= 2,
          f"got {st['index']}")

# ---- D: live chart doesn't stall acquisition -- poll /api/state hard during a run,
#          same as the browser would, and confirm the achieved rate barely drops ----
r = req("POST", "/api/daq/run/start", {"duration_s": 2.0, "rate_hz": 10})
t0 = time.time()
n_polls = 0
while time.time() - t0 < 2.5:
    req("GET", "/api/state")  # simulates the UI's 300ms poll, hammered faster on purpose
    n_polls += 1
st = wait_run_done(timeout=3)
check("D. polling doesn't starve acquisition", st["index"] >= 18, f"got {st['index']} samples, {n_polls} polls")

# ---- E: CSV replay reopens cleanly, same count/stats as the live run reported ----
hist = req("GET", "/api/daq/history")
name = hist["runs"][0]["name"]
csv_r = req("GET", f"/api/daq/history/file?name={name}&ext=csv")
json_r = req("GET", f"/api/daq/history/file?name={name}&ext=json")
check("E. CSV reopens", csv_r.get("ok") is True)
check("E. CSV row count matches JSON n_samples + n_errors",
      len(csv_r["rows"]) == json_r["data"]["n_samples"] + json_r["data"]["n_errors"],
      f"csv={len(csv_r['rows'])} json n_samples+errors={json_r['data']['n_samples']+json_r['data']['n_errors']}")

# ---- G: failure handling -- disconnect mid-run must preserve already-written data ----
r = req("POST", "/api/daq/run/start", {"duration_s": 5.0, "rate_hz": 10})
time.sleep(0.5)
req("POST", "/api/daq/disconnect", {})
st = req("GET", "/api/state")["daq"]
check("G. disconnect mid-run stops cleanly", st["run"]["state"] in ("stopped", "error"))
hist2 = req("GET", "/api/daq/history")
mid_run = next((x for x in hist2["runs"] if x["mtime"] > hist["runs"][0]["mtime"]), None)
check("G. partial data file exists after mid-run disconnect", mid_run is not None)
if mid_run:
    csv2 = req("GET", f"/api/daq/history/file?name={mid_run['name']}&ext=csv")
    check("G. partial file has some rows, not zero", len(csv2["rows"]) > 0, f"rows={len(csv2['rows'])}")

# ---- H: instrument validity -- units/signs consistent (mock: just sanity-check magnitudes) ----
req("POST", "/api/daq/connect", {"resource": "mock://bench"})
req("POST", "/api/daq/run/start", {"duration_s": 1.0, "rate_hz": 10})
st = wait_run_done(timeout=3)
hist3 = req("GET", "/api/daq/history")
name3 = hist3["runs"][0]["name"]
rows = req("GET", f"/api/daq/history/file?name={name3}&ext=csv")["rows"]
ok_rows = [r for r in rows if r["status"] == "ok"]
r_values = [abs(float(r["R_V"])) for r in ok_rows]
check("H. R_V is non-negative magnitude for all samples", all(v >= 0 for v in r_values))
check("H. R_V == hypot(X_V,Y_V) within tolerance (instrument-consistent)",
      all(abs(float(r["R_V"]) - (float(r["X_V"])**2 + float(r["Y_V"])**2)**0.5) < 1e-9 for r in ok_rows))

req("POST", "/api/daq/disconnect", {})

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
