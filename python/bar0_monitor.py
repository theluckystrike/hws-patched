#!/usr/bin/env python3
"""
bar0_monitor.py — Show *only* the BAR0 locations we have NOT put a name on yet.

  • Known registers are enumerated in KNOWN_WORD_INDICES (word == 32-bit offset).
  • Pass “--all” on the command-line if you ever want to see every change again.
"""

import mmap
import os
import sys
import time
from argparse import ArgumentParser

# ── PCI BAR settings ─────────────────────────────────────────────────────────
PCI_BAR0_PATH = "/sys/bus/pci/devices/0000:17:00.0/resource0"
BAR0_SIZE     = 64 * 1024       # 64 KiB
WORD_BYTES    = 4               # 32-bit registers

# ── Fill in every register index that *is* documented in hws_reg.h ──────────
KNOWN_WORD_INDICES = {
      0,   1,   2,   3,   4,   5,   8,   9,        # globals
     25,                                            # DMA base address
    # Video & Audio DMA toggles
     32,  33,  34,  35,                             # VBUF_TOGGLE ch0-3
     40,  41,  42,  43,                             # ABUF_TOGGLE ch0-3
    # Device info
     88,
    # Input-resolution & BCHS per channel
     90,  91,  92,  93,  94,  95,  96,  97,
    # Frame-rate (in) / output size / frame-rate (out)
    110, 111, 112, 113,
    120, 121, 122, 123,
    130, 131, 132, 133,
}

def read_bar0(path=PCI_BAR0_PATH, size=BAR0_SIZE):
    with open(path, "rb") as f:
        mm = mmap.mmap(f.fileno(), length=size, access=mmap.ACCESS_READ)
        try:
            return mm.read(size)
        finally:
            mm.close()

def diff_buffers(base: bytes, cur: bytes, *, only_unknown: bool):
    """Return [(offset, old, new), …] filtered for unknown registers if asked."""
    diffs = []
    for off in range(0, min(len(base), len(cur)), WORD_BYTES):
        w0 = int.from_bytes(base[off:off+WORD_BYTES], "little")
        w1 = int.from_bytes(cur [off:off+WORD_BYTES], "little")
        if w0 == w1:
            continue

        word_idx = off // WORD_BYTES
        if only_unknown and word_idx in KNOWN_WORD_INDICES:
            continue          # already documented – skip

        diffs.append((off, w0, w1))
    return diffs

def print_diffs(diffs):
    if not diffs:
        print("  (no *unknown* register changes)\n")
        return
    for off, old, new in diffs:
        print(f"  Offset 0x{off:08X}: 0x{old:08X} -> 0x{new:08X}")
    print(f"\n  Total changes: {len(diffs)}\n")

def monitor_loop(only_unknown=True):
    try:
        base = read_bar0()
    except (PermissionError, FileNotFoundError) as e:
        sys.exit(f"[ERROR] {e}")

    print("Captured initial BAR0 snapshot — monitoring every second.\n"
          "Press Ctrl-C to quit.\n")

    n = 1
    while True:
        try:
            cur = read_bar0()
        except Exception as e:
            print(f"[ERROR] {e}")
            time.sleep(1)
            continue

        diffs = diff_buffers(base, cur, only_unknown=only_unknown)
        print(f"--- Iteration {n}  ({time.strftime('%H:%M:%S')}) ---")
        print_diffs(diffs)
        n += 1
        time.sleep(1)

if __name__ == "__main__":
    ap = ArgumentParser(description="BAR0 monitor for HWS device")
    ap.add_argument("--all", action="store_true",
                    help="show *all* changing registers, not just the unknown ones")
    args = ap.parse_args()
    monitor_loop(only_unknown=not args.all)

