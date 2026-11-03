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

## Before first motion
- **Steps/mm**: X/Y 3200 (1 mm lead screws), Z 400 (8 mm lead screws), assuming 200-step motors at
  16x microstepping. At 3200 steps/mm the board's step rate caps X/Y at ~10 mm/s, so keep XY feed
  around 300 mm/min. Fine-tune with the calibrator in *Machine settings*, Apply, then `M500`.
- **Direction**: if an axis runs backwards, flip it with the motor plug or ask for an `INVERT_*_DIR`
  rebuild. Check both Z motors turn the same way with a small Z jog before any long move.
- **No homing**: there are no endstops. Jog to a reference corner and press **Zero XYZ** (G92);
  console soft limits are measured from there. The zero is lost on reset/power cycle.
- M112 is processed after the current command; the power switch is the real e-stop.
