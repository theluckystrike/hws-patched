/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _HWS_PCIE_REG_H
#define _HWS_PCIE_REG_H

#define  PCI_BARADDROFSIZE 4


#define MIN_VAMP_BRIGHTNESS_UNITS   0
#define MAX_VAMP_BRIGHTNESS_UNITS   0xff

#define MIN_VAMP_CONTRAST_UNITS     0
#define MAX_VAMP_CONTRAST_UNITS     0xff

#define MIN_VAMP_SATURATION_UNITS   0
#define MAX_VAMP_SATURATION_UNITS   0xff

#define MIN_VAMP_HUE_UNITS          0
#define MAX_VAMP_HUE_UNITS          0xff


#define BrightnessDefault  0x80
#define ContrastDefault    0x80
#define SaturationDefault  0x80
#define HueDefault         0x00

/* ── Core / global status ─────────────────────────────────────────────────── */
#define HWS_REG_SYS_STATUS            (CVBS_IN_BASE +  0 * PCI_BARADDROFSIZE)
/* bit3: DMA busy, bit2: int, … */

#define HWS_SYS_DMA_BUSY_BIT     BIT(3)   /* 0x08 = DMA busy flag */

#define HWS_REG_DEC_MODE       (CVBS_IN_BASE +  0 * PCI_BARADDROFSIZE)
/*  Write 0x00 to fully reset decoder,
 *  set bit 31=1 to “start run”,
 *  low byte=0x13 selects YUYV/BT.709/etc,
 *  in ReadChipId() we also write 0x00 and 0x10 here for chip-ID sequencing.
 */

/* per-pipe base: 0x4000, stride 0x800 ------------------------------------ */
#define HWS_REG_PIPE_BASE(n)   (CVBS_IN_BASE + ((n) * 0x800))
#define HWS_REG_HPD(n)         (HWS_REG_PIPE_BASE(n) + 0x14)  /* +5 V & HPD */

/* handy bit masks */
#define HWS_HPD_BIT            BIT(0)      /* hot-plug detect */
#define HWS_5V_BIT             BIT(3)      /* cable +5-volt */

#define HWS_REG_INT_ENABLE     (CVBS_IN_BASE +  4 * PCI_BARADDROFSIZE)
/* Write 0x3FFFF here to enable all video/audio interrupts. */

#define HWS_REG_INT_STATUS            (CVBS_IN_BASE +  1 * PCI_BARADDROFSIZE) /* per-channel done flags       */
#define HWS_SYS_BUSY_BIT          BIT(2)      /* matches old 0x04 test   */

/* ── Capture enable switches ──────────────────────────────────────────────── */
#define HWS_REG_VCAP_ENABLE           (CVBS_IN_BASE +  2 * PCI_BARADDROFSIZE) /* bit0-3: CH0-CH3 video enable */
#define HWS_REG_ACAP_ENABLE           (CVBS_IN_BASE +  3 * PCI_BARADDROFSIZE) /* bit0-3: CH0-CH3 audio enable */
#define HWS_REG_ACTIVE_STATUS          (CVBS_IN_BASE +  5  * PCI_BARADDROFSIZE) /* bits0-3: signal present, bits8-11: interlace */
#define HWS_REG_HDCP_STATUS            (CVBS_IN_BASE +  8  * PCI_BARADDROFSIZE) /* bits0-3: HDCP detected                       */
#define HWS_REG_DMA_MAX_SIZE   (CVBS_IN_BASE +  9 * PCI_BARADDROFSIZE)

/* ── Buffer addresses (written once during init / reset) ─────────────────── */
#define HWS_REG_VBUF1_ADDR            (CVBS_IN_BASE + 25 * PCI_BARADDROFSIZE) /* base of host-visible buffer  */

/* ── Per-channel live buffer toggles (read-only) ──────────────────────────── */
#define HWS_REG_VBUF_TOGGLE(ch)       (CVBS_IN_BASE + (32 + (ch)) * PCI_BARADDROFSIZE)
/*      Returns 0 or 1 = which half of the video ring the DMA engine is
 *      currently filling for channel *ch* (0–3).                              */

#define HWS_REG_ABUF_TOGGLE(ch)       (CVBS_IN_BASE + (40 + (ch)) * PCI_BARADDROFSIZE)
/*      Returns 0 or 1 = which half of the audio ring the DMA engine is
 *      currently filling for channel *ch* (0–3).                              */

/* ── per-interrupt bits (video 0-3, audio 0-3) ────────────────────── */
#define HWS_INT_VDONE_BIT(ch)     BIT(ch)         /* 0x01,0x02,0x04,0x08  */
#define HWS_INT_ADONE_BIT(ch)     BIT(8 + (ch))   /* 0x100 .. 0x800 */

#define HWS_REG_INT_ACK           (CVBS_IN_BASE + 0x4000 + 1 * PCI_BARADDROFSIZE)

#define HWS_REG_IN_RES(ch)             (CVBS_IN_BASE + (90  + (ch) * 2) * PCI_BARADDROFSIZE) /* 16-bit W | 16-bit H      */
#define HWS_REG_BCHS(ch)               (CVBS_IN_BASE + (91  + (ch) * 2) * PCI_BARADDROFSIZE) /* B|C|H|S packed bytes     */

#define HWS_REG_FRAME_RATE(ch)         (CVBS_IN_BASE + (110 + (ch))    * PCI_BARADDROFSIZE)  /* input fps                */
#define HWS_REG_OUT_RES(ch)            (CVBS_IN_BASE + (120 + (ch))    * PCI_BARADDROFSIZE)  /* programmed out W|H       */
#define HWS_REG_OUT_FRAME_RATE(ch)     (CVBS_IN_BASE + (130 + (ch))    * PCI_BARADDROFSIZE)  /* programmed out fps       */


/* ── “device version / port ID / subversion” register ───────────────────── */
#define HWS_REG_DEVICE_INFO   (CVBS_IN_BASE +  88 * PCI_BARADDROFSIZE)
/*  Reading this 32-bit word returns:
 *    bits  7:0   = “device version”
 *    bits 15:8   = “device sub-version”
 *    bits 23:24  = “HW key / port ID” etc.
 *    bits 31:28  = “support YV12” flags
 */


/* ── Convenience aliases for individual channels (optional) ──────────────── */
#define HWS_REG_VBUF_TOGGLE_CH0       HWS_REG_VBUF_TOGGLE(0)
#define HWS_REG_VBUF_TOGGLE_CH1       HWS_REG_VBUF_TOGGLE(1)
#define HWS_REG_VBUF_TOGGLE_CH2       HWS_REG_VBUF_TOGGLE(2)
#define HWS_REG_VBUF_TOGGLE_CH3       HWS_REG_VBUF_TOGGLE(3)

#define HWS_REG_ABUF_TOGGLE_CH0       HWS_REG_ABUF_TOGGLE(0)
#define HWS_REG_ABUF_TOGGLE_CH1       HWS_REG_ABUF_TOGGLE(1)
#define HWS_REG_ABUF_TOGGLE_CH2       HWS_REG_ABUF_TOGGLE(2)
#define HWS_REG_ABUF_TOGGLE_CH3       HWS_REG_ABUF_TOGGLE(3)

typedef enum
{
	YUYV = 0,
	UYVY,
	YVYU,
	VYUY,
	RGBP,
	RGBR,
	RGBO,
	RGBQ,
	RGB3,
	BGR3,
	RGB4,
	BGR4,	
}framegrabber_pixfmt_enum_t;

typedef struct  {
	const char *name;
	int   fourcc;          /* v4l2 format id */
	char  depth;
	char  is_yuv;
	framegrabber_pixfmt_enum_t pixfmt_out;
}framegrabber_pixfmt_t;

	
typedef struct
{
	struct frame_size frame_size;
	int refresh_rate;
	bool  is_interlace;
} v4l2_model_timing_t;


#define V4L2_MODEL_TIMING(w,h,f,i) \
	{ \
		.frame_size = { \
			.width = w, \
			.height = h, \
		}, \
		.refresh_rate = f, \
		.is_interlace = i, \
	}
	

#endif
