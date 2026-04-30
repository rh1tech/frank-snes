#!/bin/bash
#
# release.sh - Build release firmware for frank-snes
#
# Builds two variants: M1 and M2
# Output: frank-snes_m1_A_BB.uf2, frank-snes_m2_A_BB.uf2
#
# Flags:
#   -t, --test     Build at the current version (no increment, no version.txt
#                  write). Output filenames get a _test postfix:
#                  frank-snes_m2_A_BB_test.uf2.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

VERSION_FILE="version.txt"

TEST_MODE=0
POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--test)
            TEST_MODE=1
            shift
            ;;
        -h|--help)
            sed -n '3,12p' "$0"
            exit 0
            ;;
        *)
            POSITIONAL+=("$1")
            shift
            ;;
    esac
done
set -- "${POSITIONAL[@]}"

if [[ -f "$VERSION_FILE" ]]; then
    read -r LAST_MAJOR LAST_MINOR < "$VERSION_FILE"
else
    LAST_MAJOR=1
    LAST_MINOR=0
fi

# Strip leading zeros so arithmetic doesn't treat "09" as octal.
LAST_MAJOR=$((10#$LAST_MAJOR))
LAST_MINOR=$((10#$LAST_MINOR))

NEXT_MINOR=$((LAST_MINOR + 1))
NEXT_MAJOR=$LAST_MAJOR
if [[ $NEXT_MINOR -ge 100 ]]; then
    NEXT_MAJOR=$((NEXT_MAJOR + 1))
    NEXT_MINOR=0
fi

echo ""
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────────┐${NC}"
if [[ $TEST_MODE -eq 1 ]]; then
    echo -e "${CYAN}│                  FRANK SNES Release Builder (TEST)                 │${NC}"
else
    echo -e "${CYAN}│                    FRANK SNES Release Builder                      │${NC}"
fi
echo -e "${CYAN}└─────────────────────────────────────────────────────────────────┘${NC}"
echo ""
echo -e "Last version: ${YELLOW}${LAST_MAJOR}.$(printf '%02d' $LAST_MINOR)${NC}"
echo ""

if [[ $TEST_MODE -eq 1 ]]; then
    # Test builds reuse the current on-disk version with a _test postfix
    # and never rewrite version.txt. Ignore any positional version arg.
    MAJOR=$LAST_MAJOR
    MINOR=$LAST_MINOR
    if [[ -n "$1" ]]; then
        echo -e "${YELLOW}Note: ignoring version arg in --test mode${NC}"
    fi
else
    DEFAULT_VERSION="${NEXT_MAJOR}.$(printf '%02d' $NEXT_MINOR)"
    if [[ -n "$1" ]]; then
        INPUT_VERSION="$1"
    else
        read -p "Enter version [default: $DEFAULT_VERSION]: " INPUT_VERSION
        INPUT_VERSION=${INPUT_VERSION:-$DEFAULT_VERSION}
    fi

    if [[ "$INPUT_VERSION" == *"."* ]]; then
        MAJOR="${INPUT_VERSION%%.*}"
        MINOR="${INPUT_VERSION##*.}"
    else
        read -r MAJOR MINOR <<< "$INPUT_VERSION"
    fi
fi

MINOR=$((10#$MINOR))
MAJOR=$((10#$MAJOR))

if [[ $MAJOR -lt 0 ]]; then
    echo -e "${RED}Error: Major version must be >= 1${NC}"
    exit 1
fi
if [[ $MINOR -lt 0 || $MINOR -ge 100 ]]; then
    echo -e "${RED}Error: Minor version must be 0-99${NC}"
    exit 1
fi

VERSION="${MAJOR}_$(printf '%02d' $MINOR)"
POSTFIX=""
if [[ $TEST_MODE -eq 1 ]]; then
    POSTFIX="_test"
fi
echo ""
if [[ $TEST_MODE -eq 1 ]]; then
    echo -e "${GREEN}Building TEST version: ${MAJOR}.$(printf '%02d' $MINOR)${POSTFIX}${NC}"
else
    echo -e "${GREEN}Building release version: ${MAJOR}.$(printf '%02d' $MINOR)${NC}"
    printf '%d %02d\n' "$MAJOR" "$MINOR" > "$VERSION_FILE"
fi

RELEASE_DIR="$SCRIPT_DIR/release"
mkdir -p "$RELEASE_DIR"

VARIANTS=("M1" "M2")
FAIL=0

for VARIANT in "${VARIANTS[@]}"; do
    VARIANT_LOWER=$(echo "$VARIANT" | tr '[:upper:]' '[:lower:]')
    OUTPUT_NAME="frank-snes_${VARIANT_LOWER}_${VERSION}${POSTFIX}.uf2"

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "${CYAN}Building: $OUTPUT_NAME${NC}"
    echo ""

    rm -rf build
    mkdir build
    cd build

    cmake .. -DBOARD_VARIANT=${VARIANT} -DUSB_HID_ENABLED=ON > /dev/null 2>&1

    if make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4) > /dev/null 2>&1; then
        if [[ -f "frank-snes.uf2" ]]; then
            cp "frank-snes.uf2" "$RELEASE_DIR/$OUTPUT_NAME"
            echo -e "  ${GREEN}✓ ${VARIANT}${NC} → release/$OUTPUT_NAME"
        else
            echo -e "  ${RED}✗ ${VARIANT} UF2 not found${NC}"
            FAIL=1
        fi
    else
        echo -e "  ${RED}✗ ${VARIANT} build failed${NC}"
        FAIL=1
    fi

    cd "$SCRIPT_DIR"
done

rm -rf build

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [[ $FAIL -eq 0 ]]; then
    echo -e "${GREEN}Release build complete!${NC}"
else
    echo -e "${RED}Some builds failed!${NC}"
fi
echo ""
ls -la "$RELEASE_DIR"/frank-snes_*_${VERSION}${POSTFIX}.uf2 2>/dev/null | awk '{print "  " $9 " (" $5 " bytes)"}'
echo ""
echo -e "Version: ${CYAN}${MAJOR}.$(printf '%02d' $MINOR)${POSTFIX}${NC}"
echo ""
