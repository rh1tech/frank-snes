#!/bin/bash
#
# build.sh — build frank-snes.
#
# On C2 (FRANK Core 2) this builds both halves: the emulator for the
# RP2350B master into ./build, and the sound slave for the RP2350A into
# ./slave/build. The two must be built together — they share the wire
# protocol in link/, and the master refuses to run against a slave whose
# LINK_PROTO_VER or system clock disagrees.
#
# Usage: ./build.sh <M1|M2|C2> [CPU_SPEED] [PSRAM_SPEED]
#
set -e

# Board variant: M1, M2 or C2 (required, no default - must be explicit)
BOARD_VARIANT="${1:?Usage: ./build.sh <M1|M2|C2> [CPU_SPEED] [PSRAM_SPEED]}"

case "$BOARD_VARIANT" in
	M1|M2|C2) ;;
	*) echo "Error: BOARD_VARIANT must be M1, M2 or C2 (got '$BOARD_VARIANT')" >&2; exit 1 ;;
esac

# Optional build-time overrides
: "${CPU_SPEED:=${2:-504}}"
: "${PSRAM_SPEED:=${3:-166}}"
: "${FRANK_SNES_PROFILE:=OFF}"
: "${FRANK_SNES_FAST_MODE:=ON}"
: "${FRANK_SNES_SUPERFX_DIAG:=OFF}"
: "${FRANK_SNES_AUTOBOOT:=OFF}"
: "${FRANK_SNES_AUTOPAD:=OFF}"
: "${USB_HID_ENABLED:=OFF}"
# C2 only: keep sound on the master instead of offloading it, for A/B
# comparison against the split on identical hardware.
: "${C2_LOCAL_SOUND:=0}"

JOBS=$(sysctl -n hw.ncpu 2>/dev/null || nproc)

# C2 has no NES pad and no PS/2 header — GPIO20..43 are the link — so USB
# HID is the only input path. CMakeLists forces this on regardless; say
# so rather than letting the flag be silently ignored.
if [ "$BOARD_VARIANT" = "C2" ] && [ "$USB_HID_ENABLED" = "OFF" ]; then
	echo "Note: USB_HID_ENABLED=OFF ignored on C2 — HID is the only input path"
	USB_HID_ENABLED=ON
fi

rm -rf ./build
mkdir build
cd build

cmake \
	-DPICO_PLATFORM=rp2350 \
	-DFRANK_SNES_PROFILE=${FRANK_SNES_PROFILE} \
	-DFRANK_SNES_FAST_MODE=${FRANK_SNES_FAST_MODE} \
	-DFRANK_SNES_SUPERFX_DIAG=${FRANK_SNES_SUPERFX_DIAG} \
	-DFRANK_SNES_AUTOBOOT=${FRANK_SNES_AUTOBOOT} \
	-DFRANK_SNES_AUTOPAD=${FRANK_SNES_AUTOPAD} \
	-DBOARD_VARIANT=${BOARD_VARIANT} \
	-DCPU_SPEED=${CPU_SPEED} \
	-DPSRAM_SPEED=${PSRAM_SPEED} \
	-DUSB_HID_ENABLED=${USB_HID_ENABLED} \
	-DC2_LOCAL_SOUND=${C2_LOCAL_SOUND} \
	..
make -j${JOBS}

cd ..

if [ "$BOARD_VARIANT" = "C2" ] && [ "$C2_LOCAL_SOUND" = "0" ]; then
	echo
	echo "=== Building C2 sound slave ==="
	# The slave must run at the master's clock: the receiving PIO program
	# has to complete its loop inside the transmitter's byte period, and
	# each side derives that from its own system clock.
	rm -rf ./slave/build
	mkdir -p slave/build
	(
		cd slave/build
		cmake -DPICO_PLATFORM=rp2350 -DCPU_SPEED=${CPU_SPEED} ..
		make -j${JOBS}
	)
	echo
	echo "master: build/frank-snes.uf2"
	echo "slave:  slave/build/frank-snes-slave.uf2"
	echo "Flash both — see ./flash.sh --help"
fi
