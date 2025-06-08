#!/usr/bin/env python3
"""
it_content_type_loopback.py + BAR0 diff     2025-06-08
────────────────────────────────────────────────────────
For each combination of video MODE × HDMI “content type”:

  1.  Snapshot BAR0 (baseline).
  2.  Atomic mode-set + property write   (modetest -s … -w …).
  3.  Wait a moment, capture 1 s via V4L2 (ffmpeg).
  4.  Snapshot BAR0 again, diff vs baseline.
  5.  Print changed 32-bit words, filtered per CLI flag:

        (no flag)   unknown registers only   ← default
        --known     only documented ones
        --all       everything

Run with sudo:
    sudo ./it_content_type_loopback.py          # unknown-only
    sudo ./it_content_type_loopback.py --known  # show documented
    sudo ./it_content_type_loopback.py --all    # show all
"""

import mmap, os, subprocess, sys, time
from argparse import ArgumentParser
from pathlib import Path
from typing import List, Tuple

# ─── Device / site-specific -------------------------------------------------
CAPTURE_DEV   = "/dev/video0"
CONNECTOR_ID  = 110
CRTC_ID       = 109
MODES         = ["640x480@60", "1280x720@60", "1920x1080@60"]
CONTENT_TYPES = ["Graphics", "Photo", "Cinema", "Game"]

CAPTURE_SEC   = 1
DWELL_SEC     = 1

# BAR0
PCI_BAR0_PATH = "/sys/bus/pci/devices/0000:17:00.0/resource0"
BAR0_SIZE     = 64 * 1024
WORD_BYTES    = 4

# ─── Documented register word indices (offset/4) ---------------------------
KNOWN_WORD_INDICES = {
      0,1,2,3,4,5,8,9,          # globals
     25,                        # VBUF1 base
     32,33,34,35, 40,41,42,43,  # toggles
     88,                        # device info
     90,91,92,93,94,95,96,97,   # IN_RES/BCHS
    110,111,112,113,
    120,121,122,123,
    130,131,132,133,
}

# ─── BAR0 helpers -----------------------------------------------------------
def read_bar0(path=PCI_BAR0_PATH, size=BAR0_SIZE) -> bytes:
    with open(path, "rb") as f:
        mm = mmap.mmap(f.fileno(), size, access=mmap.ACCESS_READ)
        try:
            return mm.read(size)
        finally:
            mm.close()

def diff_bar0(a: bytes, b: bytes, mode: str) -> List[Tuple[int,int,int]]:
    diffs = []
    for off in range(0, min(len(a), len(b)), WORD_BYTES):
        old = int.from_bytes(a[off:off+WORD_BYTES], "little")
        new = int.from_bytes(b[off:off+WORD_BYTES], "little")
        if old == new:
            continue

        idx = off // WORD_BYTES
        if mode == "unknown" and idx in KNOWN_WORD_INDICES:
            continue
        if mode == "known"   and idx not in KNOWN_WORD_INDICES:
            continue
        diffs.append((off, old, new))
    return diffs

def fmt32(v: int) -> str:
    return f"0x{v:08X}"

def print_diffs(diffs: List[Tuple[int,int,int]], indent="    "):
    if not diffs:
        print(f"{indent}(no changes)\n")
        return
    for off, ov, nv in diffs:
        print(f"{indent}Offset {off:#010x}: {fmt32(ov)} → {fmt32(nv)}")
    print(f"{indent}Total changes: {len(diffs)}\n")

# ─── Helpers: mode-set, capture --------------------------------------------
def run_quiet(cmd):
    subprocess.run(cmd, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def set_mode_ct(mode: str, ct: str):
    run_quiet([
        "modetest",
        "-s", f"{CONNECTOR_ID}@{CRTC_ID}:{mode}",
        "-w", f'{CONNECTOR_ID}:"content type":{ct}'
    ])

def capture_one_second():
    run_quiet([
        "ffmpeg", "-loglevel", "quiet",
        "-f", "v4l2", "-i", CAPTURE_DEV,
        "-t", str(CAPTURE_SEC),
        "-f", "rawvideo", "-y", "/dev/null"
    ])

# ─── Main ------------------------------------------------------------------
def main():
    # CLI
    ap = ArgumentParser(description="Mode×ContentType loopback with BAR0 diff")
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--all",   action="store_true",
                     help="show all changing registers")
    grp.add_argument("--known", action="store_true",
                     help="show only documented registers")
    args = ap.parse_args()
    mode_filter = "all" if args.all else "known" if args.known else "unknown"

    if os.geteuid() != 0:
        sys.exit("Run with sudo (needs modetest).")
    if not Path(CAPTURE_DEV).exists():
        sys.exit(f"{CAPTURE_DEV} not found.")
    if not Path(PCI_BAR0_PATH).exists():
        sys.exit(f"{PCI_BAR0_PATH} not found.")

    print(f"[*] Testing {len(MODES)} modes × {len(CONTENT_TYPES)} content types")
    print(f"[*] BAR0 diff filter: {mode_filter}\n")

    step = 1
    for ct in CONTENT_TYPES:
        for m in MODES:
            print(f"=== Step {step}:  {m:<12}  CT={ct} ===")
            step += 1

            baseline = read_bar0()            # snapshot BEFORE change
            try:
                set_mode_ct(m, ct)
            except subprocess.CalledProcessError:
                print("    !! modetest failed (property missing?)\n")
                continue

            time.sleep(DWELL_SEC)

            try:
                capture_one_second()
            except subprocess.CalledProcessError:
                print("    !! ffmpeg capture failed\n")

            time.sleep(0.2)                   # small settle
            after = read_bar0()
            diffs = diff_bar0(baseline, after, mode_filter)
            print_diffs(diffs)

    print("[*] Done.")

if __name__ == "__main__":
    main()

