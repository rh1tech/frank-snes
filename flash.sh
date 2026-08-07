#!/bin/bash
#
# flash.sh — flash frank-snes to a connected board.
#
# On M1/M2 there is one chip and the default does what it always did:
# picotool over USB BOOTSEL.
#
# C2 (FRANK Core 2) has two RP2350s, each with its own USB-C port and its
# own BOOTSEL button, so which one picotool finds depends on which is in
# BOOTSEL. Pass --slave to flash the sound slave instead of the emulator.
#
# Usage:
#   ./flash.sh                    # master (or the only chip on M1/M2)
#   ./flash.sh --slave            # C2 sound slave
#   ./flash.sh --both             # C2: master, then slave (prompts)
#   ./flash.sh firmware.uf2       # an explicit image
#
set -uo pipefail

cd "$(dirname "$0")"

TARGET="master"
FIRMWARE=""

while [ $# -gt 0 ]; do
    case "$1" in
        --slave)  TARGET="slave" ;;
        --master) TARGET="master" ;;
        --both)   TARGET="both" ;;
        -h|--help)
            sed -n '3,18p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        -*)
            echo "Error: unknown option '$1'" >&2
            exit 1 ;;
        *)
            FIRMWARE="$1" ;;
    esac
    shift
done

flash_one() {
    local fw="$1" what="$2"

    if [ ! -f "$fw" ]; then
        # Accept either extension, whichever the build left behind.
        if [ -f "${fw%.elf}.uf2" ]; then
            fw="${fw%.elf}.uf2"
        elif [ -f "${fw%.uf2}.elf" ]; then
            fw="${fw%.uf2}.elf"
        else
            echo "Error: $what firmware not found: $fw" >&2
            echo "Run ./build.sh first." >&2
            return 1
        fi
    fi

    echo "Flashing $what: $fw"
    picotool load -f "$fw" && picotool reboot -f
}

if [ -n "$FIRMWARE" ]; then
    flash_one "$FIRMWARE" "firmware"
    exit $?
fi

MASTER_FW="./build/frank-snes.elf"
SLAVE_FW="./slave/build/frank-snes-slave.elf"

case "$TARGET" in
    master) flash_one "$MASTER_FW" "master" ;;
    slave)  flash_one "$SLAVE_FW"  "slave"  ;;
    both)
        flash_one "$MASTER_FW" "master" || exit 1
        echo
        echo "Now put the SLAVE (U6) into BOOTSEL — hold its BOOTSEL button"
        echo "while power-cycling or pressing its RUN button — then press Enter."
        read -r _
        flash_one "$SLAVE_FW" "slave"
        ;;
esac
