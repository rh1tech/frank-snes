#!/usr/bin/env python3
"""
Parse the frank-gamepad capture logs in gamepads/ and regenerate the
gamepad_map_t tables inside drivers/usbhid/hid_app.c.

Capture format (one file per VID/PID):

    Source=HID|XINPUT
    VID=0xNNNN
    PID=0xNNNN
    ReportLen=N
    Baseline=HH HH HH ...

    UP=byte[N]:+0xMM
    DOWN=byte[N]:=0xMM
    LEFT=byte[N]:-0xMM
    ...

Operators:
    +0xMM   a bit set on top of the baseline    -> pure button bit
    -0xMM   a bit cleared vs baseline           -> usually stick drift or
                                                   a hat release
    =0xMM   byte changed *to* MM                -> hat switch position

A BTN_ALT=byte[...] entry is a second press recorded for the same button;
we prefer the ALT value when the primary line is obviously stick drift
(XInput axis bytes 4..7 with -0x80 against a 0x80 baseline).

Usage:
    python3 scripts/gen_gamepad_maps.py

Rewrites the BEGIN/END GENERATED GAMEPAD MAPS and BEGIN/END GENERATED
XINPUT MAPS blocks in drivers/usbhid/hid_app.c in place.
"""

from __future__ import annotations

import pathlib
import re
import sys
from dataclasses import dataclass, field

ROOT = pathlib.Path(__file__).resolve().parent.parent
GAMEPAD_DIR = ROOT / "gamepads"
HID_APP = ROOT / "drivers/usbhid/hid_app.c"

BUTTONS = ["a", "b", "x", "y", "l", "r", "start", "select"]
DPAD_NAMES = {"UP", "DOWN", "LEFT", "RIGHT"}

ENTRY_RE = re.compile(
    r"^([A-Z_]+)=byte\[(\d+)\]:([+\-=])0x([0-9A-Fa-f]+)"
    r"(?:\s+byte\[(\d+)\]:([+\-=])0x([0-9A-Fa-f]+))?\s*$"
)


@dataclass
class Entry:
    """One capture line: BTN = byte[i]:op0xNN [byte[j]:op0xMM]"""
    name: str
    byte: int
    op: str       # '+', '-', '='
    value: int
    byte2: int | None = None
    op2: str | None = None
    value2: int | None = None


@dataclass
class Capture:
    path: pathlib.Path
    source: str
    vid: int
    pid: int
    report_len: int
    baseline: list[int]
    entries: list[Entry] = field(default_factory=list)


def parse_capture(path: pathlib.Path) -> Capture | None:
    source = vid = pid = None
    report_len = 0
    baseline: list[int] = []
    entries: list[Entry] = []

    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue

        key, _, val = line.partition("=")
        key = key.strip()
        val = val.strip()

        if key == "Source":
            source = val
        elif key == "VID":
            vid = int(val, 16)
        elif key == "PID":
            pid = int(val, 16)
        elif key == "ReportLen":
            report_len = int(val)
        elif key == "Baseline":
            baseline = [int(tok, 16) for tok in val.split()]
        elif key.split("_")[0] in DPAD_NAMES or key.split("_")[0].upper() in {
            "A", "B", "X", "Y", "L", "R", "START", "SELECT"
        }:
            m = ENTRY_RE.match(line)
            if not m:
                continue
            entries.append(
                Entry(
                    name=m.group(1),
                    byte=int(m.group(2)),
                    op=m.group(3),
                    value=int(m.group(4), 16),
                    byte2=int(m.group(5)) if m.group(5) else None,
                    op2=m.group(6),
                    value2=int(m.group(7), 16) if m.group(7) else None,
                )
            )

    if source is None or vid is None or pid is None:
        print(f"[skip] {path.name}: missing Source/VID/PID", file=sys.stderr)
        return None

    return Capture(
        path=path,
        source=source,
        vid=vid,
        pid=pid,
        report_len=report_len,
        baseline=baseline,
        entries=entries,
    )


def is_stick_drift(entry: Entry, source: str) -> bool:
    """XInput's synthetic frame puts stick axes on bytes 4..7 (baseline 0x80).
    A '-0x80' there is an axis returning to 0; that's drift, not a button."""
    if source != "XINPUT":
        return False
    return 4 <= entry.byte <= 7 and entry.op == "-"


def pick_entry(entries: list[Entry], btn: str, source: str) -> Entry | None:
    """For a logical button name (A, B, ..., SELECT), pick the best capture
    line. Prefer the primary entry, but fall back to _ALT when the primary
    is clearly stick drift."""
    primary = next((e for e in entries if e.name == btn), None)
    alt = next((e for e in entries if e.name == btn + "_ALT"), None)
    if primary and is_stick_drift(primary, source) and alt:
        return alt
    return primary


def detect_dpad(cap: Capture) -> tuple[str, int, int]:
    """Return (mode, dpad_x, dpad_y).

    mode: "DPAD_AXIS" | "DPAD_HAT" | "DPAD_BITS"
    For HAT, dpad_x is the hat byte; dpad_y is unused (0).
    For BITS, both are 0xFF (XInput wButtons low byte).
    """
    up = pick_entry(cap.entries, "UP", cap.source)
    down = pick_entry(cap.entries, "DOWN", cap.source)
    left = pick_entry(cap.entries, "LEFT", cap.source)
    right = pick_entry(cap.entries, "RIGHT", cap.source)

    if cap.source == "XINPUT":
        # XInput synthetic frames are UP=0x01 ... RIGHT=0x08 in byte 0.
        # If *any* dpad entry is a '+' on byte 0 it's BITS.
        if up and up.byte == 0 and up.op == "+":
            return ("DPAD_BITS", 0xFF, 0xFF)

    # HAT: hat byte shows '=' values for each cardinal direction.
    hat_ops = {up.op if up else None, down.op if down else None,
               left.op if left else None, right.op if right else None}
    hat_bytes = {e.byte for e in (up, down, left, right) if e}
    if hat_ops <= {"-", "="} and "=" in hat_ops and len(hat_bytes) == 1:
        return ("DPAD_HAT", next(iter(hat_bytes)), 0)

    # AXIS: left/right change byte X, up/down change byte Y.
    if left and right and up and down:
        if left.byte == right.byte and up.byte == down.byte and left.byte != up.byte:
            return ("DPAD_AXIS", left.byte, up.byte)

    # Fallback: pretend byte 0/1 axes. Won't work well but keeps C compiling.
    print(
        f"[warn] {cap.path.name}: ambiguous d-pad, defaulting to AXIS on 0/1",
        file=sys.stderr,
    )
    return ("DPAD_AXIS", 0, 1)


def render_btn(e: Entry | None) -> str:
    if e is None:
        return "{ .byte = 0xFF, .mask = 0 }"
    # Use the primary byte/mask regardless of op; '=' still implies the
    # bit we care about is represented by `value`.
    # For XInput we strip stick-drift entries in pick_entry() already.
    return f"{{ .byte = {e.byte}, .mask = 0x{e.value:02X} }}"


def render_map(cap: Capture, mode: str, dpad_x: int, dpad_y: int) -> str:
    lines = [
        f"    // Capture: {cap.path.name}  baseline: "
        + " ".join(f"{b:02X}" for b in cap.baseline),
        "    {",
        f"        .vid = 0x{cap.vid:04X}, .pid = 0x{cap.pid:04X},",
        f"        .dpad_mode = {mode}, .dpad_x = "
        + (f"0x{dpad_x:02X}" if mode == "DPAD_BITS" else str(dpad_x))
        + ", .dpad_y = "
        + (f"0x{dpad_y:02X}" if mode == "DPAD_BITS" else str(dpad_y))
        + ",",
    ]
    for btn in BUTTONS:
        e = pick_entry(cap.entries, btn.upper(), cap.source)
        pad = " " * (6 - len(btn))
        lines.append(f"        .{btn}{pad} = {render_btn(e)},")
    lines.append("    },")
    return "\n".join(lines)


def replace_block(text: str, begin: str, end: str, body: str) -> str:
    pat = re.compile(
        rf"(// {re.escape(begin)}.*?\n)(.*?)(// {re.escape(end)}.*?\n)",
        re.DOTALL,
    )
    if not pat.search(text):
        raise RuntimeError(f"Could not find {begin} / {end} block in hid_app.c")
    return pat.sub(lambda m: m.group(1) + body + m.group(3), text)


def main() -> int:
    captures: list[Capture] = []
    # Sort for deterministic output; skip obvious duplicates (" copy.txt").
    paths = sorted(
        p for p in GAMEPAD_DIR.glob("gamepad_*.txt")
        if "copy" not in p.stem.lower()
    )
    # Include XBox360_*.txt too (the frank-gamepad capture tool also writes
    # that prefix for XInput pads).
    paths += sorted(GAMEPAD_DIR.glob("XBox*_gamepad_*.txt"))
    seen: set[tuple[int, int, str]] = set()
    for p in paths:
        cap = parse_capture(p)
        if cap is None:
            continue
        key = (cap.vid, cap.pid, cap.source)
        if key in seen:
            print(f"[skip] {p.name}: duplicate of an earlier capture", file=sys.stderr)
            continue
        seen.add(key)
        captures.append(cap)

    hid_bodies: list[str] = []
    xinput_bodies: list[str] = []
    for cap in captures:
        mode, dx, dy = detect_dpad(cap)
        body = render_map(cap, mode, dx, dy)
        if cap.source == "XINPUT":
            xinput_bodies.append(body)
        else:
            hid_bodies.append(body)

    hid_block = "static const gamepad_map_t known_hid_maps[] = {\n"
    hid_block += "\n".join(hid_bodies) + "\n"
    hid_block += "};\n"

    if xinput_bodies:
        xinput_block = "static const gamepad_map_t known_xinput_maps[] = {\n"
        xinput_block += "\n".join(xinput_bodies) + "\n"
        xinput_block += "};\n"
        xinput_block += (
            "static const size_t known_xinput_map_count ="
            " sizeof(known_xinput_maps) / sizeof(known_xinput_maps[0]);\n"
        )
    else:
        xinput_block = (
            "static const gamepad_map_t known_xinput_maps[] = { {0} };\n"
            "static const size_t known_xinput_map_count = 0;\n"
        )

    text = HID_APP.read_text()
    text = replace_block(
        text, "BEGIN GENERATED GAMEPAD MAPS", "END GENERATED GAMEPAD MAPS", hid_block
    )
    text = replace_block(
        text,
        "BEGIN GENERATED XINPUT MAPS",
        "END GENERATED XINPUT MAPS",
        xinput_block,
    )
    HID_APP.write_text(text)

    print(f"Wrote {len(hid_bodies)} HID and {len(xinput_bodies)} XInput maps")
    for cap in captures:
        print(f"  {cap.source:6s} 0x{cap.vid:04X}/0x{cap.pid:04X}  {cap.path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
