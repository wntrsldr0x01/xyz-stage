#!/usr/bin/env python3
"""Patch a stock Marlin 2.1.2.5 tree for the modified i3 Mega S gantry.

Wiring kept as-is on the Trigorilla 1.4 board:
  X stage      -> X socket
  Y stage      -> E1 socket   (pins 36/34/30)
  right Z      -> Z socket
  left Z (Z2)  -> E0 socket   (Marlin assigns Z2 to E0 automatically when EXTRUDERS=0)
No extruder, no heaters, no thermistors, no endstops, no runout sensor, no LCD.

Usage: configure_marlin.py /path/to/Marlin-2.1.2.5
"""
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
cfg = root / "Marlin" / "Configuration.h"
adv = root / "Marlin" / "Configuration_adv.h"
pins = root / "Marlin" / "src" / "pins" / "ramps" / "pins_RAMPS.h"


def setdef(text, name, value=None, enable=True):
    """Set '#define NAME value' (first occurrence, commented or not)."""
    pat = re.compile(rf"^(\s*)(//\s*)?#define\s+{name}\b.*$", re.M)
    m = pat.search(text)
    if not m:
        raise SystemExit(f"{name} not found")
    line = f"{m.group(1)}{'' if enable else '//'}#define {name}" + (f" {value}" if value is not None else "")
    return text[:m.start()] + line + text[m.end():]


c = cfg.read_text()
for name, val in [
    ("MOTHERBOARD", "BOARD_TRIGORILLA_14"),
    ("SERIAL_PORT", "0"),
    ("BAUDRATE", "250000"),
    ("CUSTOM_MACHINE_NAME", '"Gantry i3MegaS"'),
    ("EXTRUDERS", "0"),
    ("TEMP_SENSOR_0", "0"),
    ("TEMP_SENSOR_BED", "0"),
    ("X_DRIVER_TYPE", "A4988"),
    ("Y_DRIVER_TYPE", "A4988"),
    ("Z_DRIVER_TYPE", "A4988"),
    ("Z2_DRIVER_TYPE", "A4988"),  # defining Z2 enables the second Z stepper
    # 200-step motors at 16x microstepping: X/Y 1 mm lead -> 3200, Z 8 mm lead -> 400.
    # 3200 steps/mm makes the AVR step rate (~40 kHz) the speed limit, hence 10 mm/s on X/Y.
    ("DEFAULT_AXIS_STEPS_PER_UNIT", "{ 3200, 3200, 400 }"),
    ("DEFAULT_MAX_FEEDRATE", "{ 10, 10, 8 }"),
    ("DEFAULT_MAX_ACCELERATION", "{ 200, 200, 100 }"),
    ("DEFAULT_ACCELERATION", "200"),
    ("DEFAULT_TRAVEL_ACCELERATION", "200"),
    # Nothing is plugged into the endstop headers: with pullups, an open input reads HIGH,
    # so invert so that "nothing connected" reports as not-triggered.
    ("X_MIN_ENDSTOP_INVERTING", "true"),
    ("Y_MIN_ENDSTOP_INVERTING", "true"),
    ("Z_MIN_ENDSTOP_INVERTING", "true"),
    ("Z_MIN_PROBE_ENDSTOP_INVERTING", "true"),
    ("MAX_FEEDRATE_EDIT_VALUES", "{ 600, 600, 10 }"),
    ("MAX_ACCEL_EDIT_VALUES", "{ 6000, 6000, 200 }"),
    ("MAX_JERK_EDIT_VALUES", "{ 20, 20, 0.6 }"),
    # Right Z (Z socket) runs reversed relative to left Z. INVERT_Z_DIR flips both Z motors;
    # INVERT_Z2_VS_Z_DIR (below) flips left Z (Z2) back, so only the right one changes.
    ("INVERT_Z_DIR", "true"),
    # Both stages ran backwards on the first jog test.
    ("INVERT_X_DIR", "true"),
    ("INVERT_Y_DIR", "true"),
    ("X_BED_SIZE", "300"),
    ("Y_BED_SIZE", "300"),
    ("Z_MAX_POS", "300"),
]:
    c = setdef(c, name, val)
c = setdef(c, "EEPROM_SETTINGS")  # so M92/M203/M500 calibration survives power cycles
for off in ["THERMAL_PROTECTION_HOTENDS", "THERMAL_PROTECTION_BED", "PREVENT_COLD_EXTRUSION",
            "PREVENT_LENGTHY_EXTRUDE", "FILAMENT_RUNOUT_SENSOR", "SDSUPPORT", "MIN_SOFTWARE_ENDSTOPS",
            "MAX_SOFTWARE_ENDSTOPS"]:
    if re.search(rf"^\s*#define\s+{off}\b", c, re.M):
        c = setdef(c, off, enable=False)
cfg.write_text(c)

a = adv.read_text()
# With EXTRUDERS 0 there is no E axis, so per-axis lists must have exactly X, Y, Z entries.
a = setdef(a, "AXIS_RELATIVE_MODES", "{ false, false, false }")
a = setdef(a, "MANUAL_FEEDRATE", "{ 50*60, 50*60, 4*60 }")
a = setdef(a, "INVERT_Z2_VS_Z_DIR")
a = setdef(a, "MINIMUM_STEPPER_PULSE", "2")  # safe for A4988 and DRV8825 alike
a = setdef(a, "M114_REALTIME")  # M114 R reports actual (not planned) position mid-move
for off in ["THERMAL_PROTECTION_CHAMBER", "THERMAL_PROTECTION_COOLER"]:
    if re.search(rf"^\s*#define\s+{off}\b", a, re.M):
        a = setdef(a, off, enable=False)
adv.write_text(a)

p = pins.read_text()
old = "#define Y_STEP_PIN                            60\n#define Y_DIR_PIN                             61\n#define Y_ENABLE_PIN                          56  // (A2)\n"
if old in p:
    p = p.replace(old, "// GANTRY: Y stage is wired to the E1 socket\n#define Y_STEP_PIN                            36\n#define Y_DIR_PIN                             34\n#define Y_ENABLE_PIN                          30\n")
    pins.write_text(p)
elif "GANTRY: Y stage" not in p:
    raise SystemExit("pins_RAMPS.h Y pins not found")
print("configured", root)
