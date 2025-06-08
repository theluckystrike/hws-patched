#!/usr/bin/env python3
"""
it_content_type_loopback.py  (v2)
─────────────────────────────────
For each HDMI Content-Type (Graphics, Photo, Cinema, Game) and for each video
mode in MODES it

  • sets the mode *and* the connector’s “content type” property in one atomic
    `modetest` commit ( -s … -w … ),
  • captures 1 s from the capture card (so the DMA fires),
  • prints the V4L2_CID_DV_RX_IT_CONTENT_TYPE reported by the driver.

Run with sudo:
    sudo ./it_content_type_loopback.py
"""

import os, subprocess, sys, time
from pathlib import Path

# ─── Site-specific ----------------------------------------------------------------
CAPTURE_DEV  = "/dev/video0"
CONNECTOR_ID = 110         # from  `modetest -c`
CRTC_ID      = 109
MODES        = ["640x480@60", "1280x720@60", "1920x1080@60"]
CAPTURE_SEC  = 1
DWELL_SEC    = 1           # settle time after mode set

# The four HDMI-CEA content-type enums as the kernel names them
CONTENT_TYPES = ["Graphics", "Photo", "Cinema", "Game"]

# Map numeric driver values → readable
IT_MAP = {0: "Graphics", 1: "Photo", 2: "Cinema", 3: "Game"}

# ─── helpers ----------------------------------------------------------------------
def run_quiet(cmd):
    subprocess.run(cmd, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def set_mode_and_ct(mode: str, ct_name: str):
    """Atomic mode-set plus “content type” write."""
    run_quiet([
        "modetest",
        "-s", f"{CONNECTOR_ID}@{CRTC_ID}:{mode}",
        "-w", f'{CONNECTOR_ID}:"content type":{ct_name}'
    ])

def capture_one_second():
    run_quiet([
        "ffmpeg", "-loglevel", "quiet",
        "-f", "v4l2", "-i", CAPTURE_DEV,
        "-t", str(CAPTURE_SEC),
        "-f", "rawvideo", "-y", "/dev/null"
    ])

def read_it_content_type():
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", CAPTURE_DEV,
             "--get-ctrl", "dv_rx_it_content_type"],
            text=True, stderr=subprocess.DEVNULL)
        val = int(out.split(":")[1])
        return IT_MAP.get(val, f"Unknown({val})")
    except subprocess.CalledProcessError:
        return "N/A (ctrl missing)"
    except (ValueError, IndexError):
        return "N/A (parse error)"

# ─── main loop --------------------------------------------------------------------
def main():
    if os.geteuid() != 0:
        sys.exit("Run with sudo (needs modetest).")
    if not Path(CAPTURE_DEV).exists():
        sys.exit(f"{CAPTURE_DEV} not found.")

    print(f"[*] Testing {len(MODES)} modes × {len(CONTENT_TYPES)} content types\n")

    for ct in CONTENT_TYPES:
        for mode in MODES:
            banner = f"{mode:>12}  |  {ct:<8}"
            print(f"=== {banner} ===")
            try:
                set_mode_and_ct(mode, ct)
            except subprocess.CalledProcessError:
                print("    !! modetest failed (property unsupported?)\n")
                continue

            time.sleep(DWELL_SEC)
            try:
                capture_one_second()
            except subprocess.CalledProcessError:
                print("    !! ffmpeg capture failed\n")
                continue

            it_reported = read_it_content_type()
            print(f"    → driver reports IT Content-Type = {it_reported}\n")

    print("[*] Done.")

if __name__ == "__main__":
    main()

