#!/usr/bin/env bash
# Build custom Marlin for the gantry, back up the stock firmware, then flash.
#   firmware/build_and_flash.sh            # build + backup + flash
#   firmware/build_and_flash.sh --build    # build only
# Close the gantry console first: the serial port can only be open in one program.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="${WORK:-$HOME/marlin-gantry}"
PORT="${PORT:-/dev/cu.usbserial-0001}"
VER=2.1.2.5
mkdir -p "$WORK"
cd "$WORK"

if [ ! -x venv/bin/pio ]; then
  python3 -m venv venv
  venv/bin/pip -q install platformio
fi
PIO="$WORK/venv/bin/pio"

if [ ! -d "Marlin-$VER" ]; then
  curl -fsSL "https://github.com/MarlinFirmware/Marlin/archive/refs/tags/$VER.tar.gz" -o marlin.tgz
  tar xzf marlin.tgz
fi
python3 "$HERE/configure_marlin.py" "$WORK/Marlin-$VER"

(cd "Marlin-$VER" && "$PIO" run -e mega2560)
HEX="$WORK/Marlin-$VER/.pio/build/mega2560/firmware.hex"
cp "$HEX" "$WORK/gantry_firmware.hex"
echo "built: $WORK/gantry_firmware.hex"
[ "${1:-}" = "--build" ] && exit 0

find_avrdude() {
  find "$HOME/.platformio/packages" -maxdepth 3 -type f -name avrdude -perm -u+x 2>/dev/null | head -1
}
if command -v avrdude >/dev/null 2>&1; then
  AVR=(avrdude)  # e.g. Homebrew; uses its own default config
else
  AVRDUDE="$(find_avrdude)"
  if [ -z "$AVRDUDE" ]; then
    # PlatformIO only fetches avrdude during its own upload step; fetch it explicitly.
    "$PIO" pkg install -g -t platformio/tool-avrdude
    AVRDUDE="$(find_avrdude)"
  fi
  [ -n "$AVRDUDE" ] || { echo "avrdude not found; try: brew install avrdude"; exit 1; }
  AVR=("$AVRDUDE" -C "$(dirname "$AVRDUDE")/avrdude.conf")
fi
echo "using: ${AVR[*]}"
if [ ! -f "$WORK/stock_flash.hex" ]; then
  echo "backing up stock firmware from $PORT ..."
  "${AVR[@]}" -p m2560 -c wiring -P "$PORT" -b 115200 -D \
    -U flash:r:"$WORK/stock_flash.hex":i -U eeprom:r:"$WORK/stock_eeprom.hex":i
  echo "backup: $WORK/stock_flash.hex  $WORK/stock_eeprom.hex"
fi

echo "flashing $PORT ..."
"${AVR[@]}" -p m2560 -c wiring -P "$PORT" -b 115200 -D -U flash:w:"$WORK/gantry_firmware.hex":i
echo "done. Restore stock with:"
echo "  ${AVR[*]} -p m2560 -c wiring -P $PORT -b 115200 -D -U flash:w:$WORK/stock_flash.hex:i"
