#!/usr/bin/env python3
"""
check_struct.py – show which members of a C struct are read / write-only / unused.

Quick start (no flags):
    1. Paste your struct into the STRUCT_TEXT constant below.
    2. python3 check_struct.py /path/to/src

Advanced:
    --struct-file hdr.h        # read the struct from a file
    --struct-name NAME         # read from stdin and tell me which struct

Colours:
    🟢 green   = at least one read
    🟡 yellow  = writes only
    🔴 red     = never referenced
"""
import argparse, re, sys
from pathlib import Path
from pycparser import c_parser, c_ast

# ────────────────────────────────────────────────────────────────────────────────
# ✂️  PASTE YOUR STRUCT DEFINITION BETWEEN THE TRIPLE QUOTES  ✂️
STRUCT_TEXT = r"""
struct hws_pcie_dev {
	struct pci_dev			*pdev;
	struct uio_info info;
	struct hws_audio		audio[MAX_VID_CHANNELS];
	struct hws_video		video[MAX_VID_CHANNELS];
	struct work_struct		video_work;
	struct work_struct		audio_work;
	spinlock_t				videoslock[MAX_VID_CHANNELS];
	spinlock_t				audiolock[MAX_VID_CHANNELS];
	u32 *map_bar0_addr;
	struct workqueue_struct *wq;
	struct workqueue_struct *auwq;
	unsigned long video_data[MAX_VID_CHANNELS];
 	struct tasklet_struct dpc_video_tasklet[MAX_VID_CHANNELS];
 	unsigned long audio_data[MAX_VID_CHANNELS];
 	struct tasklet_struct dpc_audio_tasklet[MAX_VID_CHANNELS];
	int irq_count;	/* interrupt counter */	
	int irq_line;		/* flag if irq allocated successfully */	
	int msi_enabled;	/* flag if msi was enabled for the device */	
	int msix_enabled;	/* flag if msi-x was enabled for the device */	
	int irq_user_count;/* user interrupt count */     
	int m_PciDeviceLost;
	struct msix_entry entry[32];	
	u32 dwDeviceID;
	u32 dwVendorID;
	u32 m_Device_Version;
	int  m_DeviceHW_Version;
	u32 m_AddreeSpace;
	u32 n_VideoModle;
	u32  m_Device_SupportYV12;
	u32 m_Device_SubVersion;
	u32 m_Device_PortID;
	int  m_MaxHWVideoBufferSize;
	int m_nMaxChl;
	int m_nCurreMaxVideoChl;
	int m_nCurreMaxLineInChl;
	uint8_t m_bStartRun;	// Use for start run for check i2c
	dma_addr_t   		m_pbyVideo_phys[MAX_VID_CHANNELS] ;
    uint8_t		*m_pbyVideoBuffer[MAX_VID_CHANNELS];
	uint8_t     *m_pbyVideoBuffer_area[MAX_VID_CHANNELS];
	u32		    m_dwVideoBuffer[MAX_VID_CHANNELS];
	u32		    m_dwVideoHighBuffer[MAX_VID_CHANNELS];
	VCAP_STATUS_INFO    m_pVCAPStatus[MAX_VID_CHANNELS][MAX_VIDEO_QUEUE];
	ACAP_VIDEO_INFO m_VideoInfo[MAX_VID_CHANNELS];

	VIDEO_INFO      	m_format[MAX_VID_CHANNELS];
	
	ACAP_AUDIO_INFO     m_AudioInfo[MAX_VID_CHANNELS];
	uint8_t				m_bChangeVideoSize[MAX_VID_CHANNELS];
	
	struct task_struct *mMain_tsk; 
	int m_curr_No_Video[MAX_VID_CHANNELS];
	dma_addr_t   		m_pbyAudio_phys[MAX_VID_CHANNELS] ;
	uint8_t     *m_pbyAudioBuffer[MAX_VID_CHANNELS];
	uint8_t		*m_pbyUserAudioBuffer[MAX_VID_CHANNELS];
	uint8_t     *m_pAudioData[MAX_VID_CHANNELS];
	uint8_t     *m_pbyAudioBuffer_area[MAX_VID_CHANNELS];
	uint8_t     *m_pAudioData_area[MAX_VID_CHANNELS];
	uint8_t		m_bBufferAllocate;
	u32		m_dwAudioBuffer[MAX_VID_CHANNELS];
	u32		m_dwAudioBufferHigh[MAX_VID_CHANNELS];
	uint8_t m_bVCapStarted[MAX_VID_CHANNELS];
	uint8_t	 m_bACapStarted[MAX_VID_CHANNELS];
	uint8_t     m_nVideoBusy[MAX_VID_CHANNELS];
	uint8_t   m_bVideoStop[MAX_VID_CHANNELS];
	int       m_nRDVideoIndex[MAX_VID_CHANNELS];
	int        m_nVideoBufferIndex[MAX_VID_CHANNELS];
	int       m_nVideoHalfDone[MAX_VID_CHANNELS];
	uint8_t	  m_dwAudioField[MAX_VID_CHANNELS];
	uint8_t   m_nAudioBusy[MAX_VID_CHANNELS];
	uint8_t   m_nAudioBufferIndex[MAX_VID_CHANNELS];
	uint8_t	  m_pAudioEvent[MAX_VID_CHANNELS];
	uint8_t		m_pVideoEvent[MAX_VID_CHANNELS];
	uint8_t		m_bVCapIntDone[MAX_VID_CHANNELS];
	uint8_t m_bAudioRun[MAX_VID_CHANNELS];
	uint8_t m_bAudioStop[MAX_VID_CHANNELS];
	int       m_nRDAudioIndex[MAX_VID_CHANNELS];
	u32       m_dwAudioPTKSize;
	int m_dwSWFrameRate[MAX_VID_CHANNELS];

	ULONG m_contrast[MAX_VID_CHANNELS];
	ULONG m_brightness[MAX_VID_CHANNELS];
	ULONG m_saturation[MAX_VID_CHANNELS];
	ULONG m_hue[MAX_VID_CHANNELS];
	
};
"""
# ────────────────────────────────────────────────────────────────────────────────

# ANSI colours
GREEN, YELLOW, RED, RESET = "\033[92m", "\033[93m", "\033[91m", "\033[0m"

# ──────────────────────────  parsing helpers  ──────────────────────────────────
def extract_members(src: str, struct_name: str) -> list[str]:
    parser = c_parser.CParser()
    ast = parser.parse(src)

    class Grabber(c_ast.NodeVisitor):
        def __init__(self):
            self.members = []
        def visit_Struct(self, node):
            if node.name == struct_name and node.decls:
                self.members += [d.name for d in node.decls]

    g = Grabber()
    g.visit(ast)
    return g.members

# ────────────────────────────  code scan  ──────────────────────────────────────
def scan_usage(root: Path, members: list[str]) -> dict[str, dict[str, int]]:
    read_re  = {m: re.compile(rf"(?:->|\.){m}\b(?!\s*=)") for m in members}
    write_re = {m: re.compile(rf"(?:->|\.){m}\b\s*=")     for m in members}
    counts   = {m: {"read": 0, "write": 0} for m in members}

    for path in root.rglob("*"):
        if path.suffix.lower() not in {".c", ".h"} or not path.is_file():
            continue
        try:
            txt = path.read_text(errors="ignore")
        except Exception:
            continue
        for m in members:
            counts[m]["write"] += len(write_re[m].findall(txt))
            counts[m]["read"]  += len(read_re[m].findall(txt))
    return counts

def color_line(name: str, rd: int, wr: int) -> str:
    if rd:
        col = GREEN
    elif wr:
        col = YELLOW
    else:
        col = RED
    label = f"{rd:>4}R/{wr:>4}W" if (rd or wr) else "UNUSED"
    return f"{col}{name:<30} {label}{RESET}"

# ────────────────────────────  main  ───────────────────────────────────────────
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path, help="Root of C source tree to scan")
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--struct-file", type=Path,
                     help="File containing the struct definition")
    grp.add_argument("--struct-name", type=str,
                     help="Name of struct to find in stdin")
    args = ap.parse_args()

    if args.struct_file:
        struct_src = args.struct_file.read_text()
        struct_name = None
    elif args.struct_name:
        struct_src = sys.stdin.read()
        struct_name = args.struct_name
    else:
        struct_src = STRUCT_TEXT
        struct_name = None

    members = extract_members(struct_src, struct_name)
    if not members:
        sys.exit(f"{RED}No struct members found – check your input.{RESET}")

    counts = scan_usage(args.root, members)

    print(f"\nScanning {args.root} for struct member usage\n" + "-"*60)
    unused = 0
    for m in sorted(members):
        r, w = counts[m]["read"], counts[m]["write"]
        if r == 0 and w == 0:
            unused += 1
        print(color_line(m, r, w))
    print(f"\n{len(members)} members; {unused} never referenced.\n")

if __name__ == "__main__":
    main()

