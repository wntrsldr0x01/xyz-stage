# xyz-stage

Jog + grid-sweep console and custom Marlin firmware for an XYZ stage built on a modified Anycubic i3 Mega S (Trigorilla 1.4 board).

Browser console for jogging and running grid sweeps over USB serial (Marlin, 250000 baud).

```
pip install -r requirements.txt
python3 server.py          # open http://localhost:8780
python3 server.py --mock   # simulated printer
```

## Firmware
The stock Anycubic firmware (Marlin 1.1.0-RC8) drives the E1 socket as a second Z motor and E0
as an extruder, which does not match this wiring. `firmware/` rebuilds Marlin 2.1.2.5 for the
wiring as it is:

| Motor        | Socket | Firmware axis |
|--------------|--------|---------------|
| X stage      | X      | X             |
| other stage  | E1     | Y             |
| right Z      | Z      | Z             |
| left Z       | E0     | Z2 (moves with Z) |

No extruder, heaters, thermistors, endstops, runout sensor or LCD are configured (all unplugged).
EEPROM is on, so calibration saved with `M500` survives power cycles.

```
firmware/build_and_flash.sh
```
This downloads Marlin, applies `configure_marlin.py`, builds, backs up the stock flash + EEPROM to
`~/marlin-gantry/stock_*.hex` (first run only) and flashes. It prints the restore command at the end.

## SR860 lock-in (USB-TMC)
Connect the SR860's **rear-panel** USB computer interface (not the front-panel
flash-drive port) to the laptop. `requirements.txt` installs the official
`srsinst.sr860` driver plus `pyvisa`/`pyvisa-py`/`pyusb` as the VISA backend --
no NI-VISA install needed, but `pyusb` needs the `libusb` system library:
```
brew install libusb
```
With the SR860 powered on and connected, open the console, pick the resource
from the dropdown in the **SR860 Lock-in** panel (it should look like
`USB0::0x...::0x...::<serial>::INSTR`) and press Connect. If the dropdown is
empty, the instrument isn't enumerating as a VISA-TMC device yet -- check the
cable is in the rear USB-TMC port, not the front flash-drive port.

`python3 server.py --mock` fakes the SR860 too (a slowly rotating X/Y signal),
for UI work away from the bench.

Each run writes `runs/AE_<date>_X.._Y.._Z.._<freq>_<id>.csv` (one row per sample, flushed
as it's taken) and a `.json` sidecar (instrument settings, start/end position, achieved
rate and jitter, overload/error counts, summary stats).

## XYZ scan
The **XYZ scan** panel runs the Grid sweep's XY grid over one or more Z planes, using the
SR860 panel's rate/settle/condition. Per point: move, `M400`, `M114` readback check,
settle, timed run. Everything goes in `runs/SCAN_<date>_<time>_<condition>/`: each point's
raw CSV + JSON, `scan_summary.csv` (one row per point) and `scan.json`. Motion commands are
refused while a scan runs; E-stop isn't.

Blocked/unblocked: record one scan per condition, mark them A and B in the scan list and
press Compare. The difference is taken on the complex response (ΔX, ΔY of the per-point
means, |Δ| = √(ΔX²+ΔY²)), not R_A − R_B, and saved as `runs/COMPARE_*.csv`. The run
history has the same A/B compare for two fixed-position runs.

## Before first motion
- **Steps/mm**: X/Y 3200 (1 mm lead screws), Z 400 (8 mm lead screws), assuming 200-step motors at
  16x microstepping. At 3200 steps/mm the board's step rate caps X/Y at ~10 mm/s, so keep XY feed
  around 300 mm/min. Fine-tune with the calibrator in *Machine settings*, Apply, then `M500`.
- **Direction**: if an axis runs backwards, flip it with the motor plug or ask for an `INVERT_*_DIR`
  rebuild. Check both Z motors turn the same way with a small Z jog before any long move.
- **No homing**: there are no endstops. Jog to a reference corner and press **Zero XYZ** (G92);
  console soft limits are measured from there. The zero is lost on reset/power cycle.
- M112 is processed after the current command; the power switch is the real e-stop.
