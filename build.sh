#!/bin/bash
rm -rf ./build
mkdir build
cd build

# Board variant: M1 or M2 (required, no default - must be explicit)
BOARD_VARIANT="${1:?Usage: ./build.sh <M1|M2> [CPU_SPEED] [PSRAM_SPEED]}"

# Optional build-time overrides
: "${CPU_SPEED:=${2:-504}}"
: "${PSRAM_SPEED:=${3:-166}}"
: "${FRANK_SNES_PROFILE:=OFF}"
: "${FRANK_SNES_FAST_MODE:=ON}"
: "${FRANK_SNES_SUPERFX_DIAG:=OFF}"
: "${FRANK_SNES_AUTOBOOT:=OFF}"
: "${FRANK_SNES_AUTOPAD:=OFF}"
: "${USB_HID_ENABLED:=OFF}"

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
	..
make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)
