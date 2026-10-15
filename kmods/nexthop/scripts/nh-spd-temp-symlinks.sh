#!/bin/sh
# Publish stable sysfs handles for the DDR5 SPD hub temperature sensors.
#
# The hubs are auto-instantiated by the kernel on the AMD FCH SMBus, so their
# i2c bus number tracks how many CPU I2C controllers the BIOS enables. Address
# 0x50 is DIMM channel A, 0x51 is channel B.
set -eu

DRIVER_DIR=/sys/bus/i2c/drivers/spd5118
LINK_DIR=/run/nexthop_bsp/sensors
WAIT_SECS=${NH_SPD_WAIT_SECS:-10}

[ -d "$DRIVER_DIR" ] || exit 0

bound() {
  for dev in "$DRIVER_DIR"/*-005[01]; do
    if [ -e "$dev" ]; then
      return 0
    fi
  done
  return 1
}

waited=0
while ! bound; do
  if [ "$waited" -ge "$WAIT_SECS" ]; then
    echo "no spd5118 hub bound under $DRIVER_DIR after ${WAIT_SECS}s" >&2
    exit 0
  fi
  waited=$((waited + 1))
  sleep 1
done

mkdir -p "$LINK_DIR"

for dev in "$DRIVER_DIR"/*-005[01]; do
  [ -e "$dev" ] || continue
  case "${dev##*-}" in
  0050) name=DIMM1_TEMP ;;
  0051) name=DIMM2_TEMP ;;
  *) continue ;;
  esac
  target=$(readlink -f "$dev")
  ln -sfn "$target" "$LINK_DIR/$name"
  echo "$LINK_DIR/$name -> $target"
done
