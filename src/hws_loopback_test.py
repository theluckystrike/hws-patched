#!/usr/bin/env python3
"""
hws_loopback_test.py
────────────────────
Automated “GPU → HDMI → capture-card” loop-back test for KDE/Wayland.

For every mode in MODES it will:
  1. set the GPU connector to that mode using libdrm's `modetest`,
  2. stream 1 s from /dev/video0 with ffmpeg so the driver’s DMA fires,
  3. read BAR0 and diff it against the *baseline* snapshot,
  4. print only the offsets that changed **and** are *not* yet named in
     hws_reg.h (so you can spot unknown registers quickly).

Requirements
────────────
  ▸ sudo apt/yum install libdrm-tests ffmpeg python3
  ▸ Run as root:  sudo ./hws_loopback_test.py
  ▸ Adjust the PCI address, DRM connector/CRTC and the list of MODES below.
"""

import mmap
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Tuple
from pathlib import Path

# ───── Site-specific paths ──────────────────────────────────────────────────
PCI_ADDR        = "0000:17:00.0"
BAR0_PATH       = f"/sys/bus/pci/devices/{PCI_ADDR}/resource0"
BAR0_SIZE       = 64 * 1024                     # 64 KiB
HDR_PATH        = Path(__file__).with_name("hws_reg.h")

CAPTURE_DEV     = "/dev/video0"
CAPTURE_SEC     = 1                             # seconds per mode

# ----> Adjust for *your* GPU connector / CRTC <----
CONNECTOR_ID    = 110 # run:  modetest -c
CRTC_ID         = 109 # run:  modetest -c

# Modes we want to walk through                                          
MODES = [
    "1920x1080@60",
    "1280x720@60",
    "720x480@60",
]

# ───── Helpers ──────────────────────────────────────────────────────────────
WORD = 4

def read_bar0() -> bytes:
    with open(BAR0_PATH, "rb") as f:
        mm = mmap.mmap(f.fileno(), BAR0_SIZE, access=mmap.ACCESS_READ)
        try:
            data = mm.read(BAR0_SIZE)
        finally:
            mm.close()
    return data

BAR_STRIDE      = 0x80       # == PCI_BARADDROFSIZE in the driver
HDR_PATH        = Path("hws_reg.h")

# ----------------------------------------------------------------------
def diff_words(base: bytes, cur: bytes) -> List[Tuple[int, int, int]]:
    """
    Return a list of (offset, base_word, cur_word) tuples where the 32-bit
    word value has changed between *base* and *cur*.
    """
    diffs = []
    for off in range(0, min(len(base), len(cur)), WORD):
        b = int.from_bytes(base[off : off + WORD], "little")
        c = int.from_bytes(cur [off : off + WORD], "little")
        if b != c:
            diffs.append((off, b, c))
    return diffs

# ----------------------------------------------------------------------
def parse_known_offsets(hdr: Path = HDR_PATH) -> set[int]:
    """
    Scan *hws_reg.h* and return a **set of byte offsets** that are already
    defined symbolically.

    We recognise two common macro forms:

        CVBS_IN_BASE + <N> * PCI_BARADDROFSIZE
        CVBS_IN_BASE + <CONST> + <N> * PCI_BARADDROFSIZE   (e.g. +0x4000+…)

    Both decimal and hexadecimal literals are accepted.
    """
    txt = hdr.read_text(errors="ignore")

    known: set[int] = set()

    # 1) Simple form:   + <N> * PCI_BARADDROFSIZE
    pat_simple = re.compile(
        r"\+\s*(0x[0-9A-Fa-f]+|\d+)\s*\*\s*PCI_BARADDROFSIZE"
    )
    for m in pat_simple.finditer(txt):
        idx = int(m.group(1), 0)
        known.add(idx * BAR_STRIDE)

    # 2) Form with an extra constant:  + <CONST> + <N> * PCI_BARADDROFSIZE
    pat_with_const = re.compile(
        r"\+\s*(0x[0-9A-Fa-f]+|\d+)\s*\+\s*(0x[0-9A-Fa-f]+|\d+)\s*\*\s*PCI_BARADDROFSIZE"
    )
    for m in pat_with_const.finditer(txt):
        const = int(m.group(1), 0)
        idx   = int(m.group(2), 0)
        known.add(const + idx * BAR_STRIDE)

    return known


def parse_known_offsets(hdr: Path) -> set:
    """
    Return a *byte offset set* of every register macro defined in hws_reg.h.
    The pattern looks for  (CVBS_IN_BASE + <number> * PCI_BARADDROFSIZE)
    and converts it to an offset in bytes   (<number> * 4).
    """
    txt = hdr.read_text(errors="ignore")
    pat = re.compile(r"\+\s*(\d+)\s*\*\s*PCI_BARADDROFSIZE")
    return {int(m.group(1)) * WORD for m in pat.finditer(txt)}

KNOWN_OFFSETS = parse_known_offsets(HDR_PATH)

def mode_to_modetest_arg(mode: str) -> str:
    """
    Convert '1920x1080@60' → '1920x1080@60' (modetest already uses that form),
    but you could hook custom timings here if needed.
    """
    return mode

def set_mode(mode: str):
    arg = mode_to_modetest_arg(mode)
    cmd = [
        "modetest",
        "-M", "drm",                 # auto driver
        "-s", f"{CONNECTOR_ID}@{CRTC_ID}:{arg}"
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)

def capture_one_second():
    subprocess.run(
        ["ffmpeg", "-loglevel", "quiet",
         "-f", "v4l2", "-i", CAPTURE_DEV,
         "-t", str(CAPTURE_SEC),
         "-f", "rawvideo", "-y", "/dev/null"],
        check=True)

def format_hex(val: int) -> str:
    return f"0x{val:08X}"

# ───── Main loop ────────────────────────────────────────────────────────────
def main():
    if os.geteuid() != 0:
        print("This script must be run as root.", file=sys.stderr)
        sys.exit(1)

    if not Path(BAR0_PATH).exists():
        print(f"{BAR0_PATH} not found – check PCI address?", file=sys.stderr)
        sys.exit(1)

    print(f"[+] Reading baseline BAR0 ({BAR0_SIZE//1024} KiB) …")
    baseline = read_bar0()
    print("[+] Baseline captured, starting test loop\n")

    for i, mode in enumerate(MODES, 1):
        print(f"─── [{i}/{len(MODES)}] Mode {mode} ─────────────────────────")

        try:
            set_mode(mode)
        except subprocess.CalledProcessError as e:
            print(f"  !! modetest failed for mode {mode}: {e}")
            continue

        # small settle time
        time.sleep(0.5)

        try:
            capture_one_second()
        except subprocess.CalledProcessError:
            print("  !! ffmpeg capture failed – continuing.")

        current = read_bar0()
        diffs   = diff_words(baseline, current)

        # Show only offsets NOT in KNOWN_OFFSETS
        unknown = [(o, b, c) for (o, b, c) in diffs if o not in KNOWN_OFFSETS]

        if not unknown:
            print("  No *unknown* register changes.\n")
        else:
            for off, old, new in unknown:
                print(f"  Offset {format_hex(off)}: {format_hex(old)} -> {format_hex(new)}")
            print(f"  Changed words not yet in hws_reg.h: {len(unknown)}\n")

    print("== Test loop complete ==")

if __name__ == "__main__":
    main()

