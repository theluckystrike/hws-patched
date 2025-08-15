/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_video_pipeline.h"

#include "hws.h"
#include "hws_reg.h"
#include "hws_pci.h"
#include "hws_init.h"
#include "hws_dma.h"
#include "hws_audio_pipeline.h"

#define FRAME_DONE_MARK 0x55AAAA55


void hws_set_dma_address(struct hws_pcie_dev *hws)
{
	u32 addr_mask     = PCI_E_BAR_ADD_MASK;
	u32 addr_low_mask = PCI_E_BAR_ADD_LOWMASK;
	u32 table_off     = 0x208;          /* first entry in PCI addr table */
	int i;

	for (i = 0; i < hws->max_channels; i++, table_off += 8) {
		/* ───────────── VIDEO DMA entry ───────────── */
		if (hws->video[i].buf_virt) {
			dma_addr_t paddr  = hws->video[i].buf_phys_addr;
			u32 lo           = lower_32_bits(paddr);
			u32 hi           = upper_32_bits(paddr);
			u32 pci_addr     = lo & addr_low_mask;
			lo              &= addr_mask;

			/* Program the 64-bit BAR remap entry */
			writel(hi, hws->bar0_base + PCI_ADDR_TABLE_BASE + table_off);
			writel(lo, hws->bar0_base + PCI_ADDR_TABLE_BASE + table_off + PCIE_BARADDROFSIZE);

			/* CBVS buffer address + half-frame length */
			writel((i + 1) * PCIEBAR_AXI_BASE + pci_addr, hws->bar0_base + CBVS_IN_BUF_BASE  + i * PCIE_BARADDROFSIZE);

			writel(hws->video[i].fmt_curr.half_size / 16, hws->bar0_base + CBVS_IN_BUF_BASE2 + i * PCIE_BARADDROFSIZE);
		}

		/* ───────────── AUDIO tail entry ───────────── */
		if (hws->audio[i].buf_virt) {
			dma_addr_t paddr  = hws->audio[i].buf_phys_addr;
			u32 pci_addr     = lower_32_bits(paddr) & addr_low_mask;

			writel((i + 1) * PCIEBAR_AXI_BASE + pci_addr, hws->bar0_base + 
				CBVS_IN_BUF_BASE + (8 + i) * PCIE_BARADDROFSIZE
				);
		}
	}

	/* Enable PCIe interrupts for all sources */
	writel(0x003fffff, hws->bar0_base + INT_EN_REG_BASE);
}

// FIXME: use correctly, should be in ipc
static void hws_program_video_from_vb2(struct hws_pcie_dev *hws,
                                       unsigned int ch,
                                       struct vb2_buffer *vb)
{
    const u32 addr_mask     = PCI_E_BAR_ADD_MASK;
    const u32 addr_low_mask = PCI_E_BAR_ADD_LOWMASK;
    const u32 table_off     = 0x208 + ch * 8;   /* one 64-bit slot per ch */

    dma_addr_t paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
    u32 lo = lower_32_bits(paddr);
    u32 hi = upper_32_bits(paddr);
    u32 pci_addr = lo & addr_low_mask;
    lo &= addr_mask;

    /* 64-bit BAR remap entry for this channel */
    writel(hi, hws->bar0_base + PCI_ADDR_TABLE_BASE + table_off);
    writel(lo, hws->bar0_base + PCI_ADDR_TABLE_BASE + table_off + PCIE_BARADDROFSIZE);

    /* Capture engine per-channel registers */
    writel((ch + 1) * PCIEBAR_AXI_BASE + pci_addr,
           hws->bar0_base + CBVS_IN_BUF_BASE  + ch * PCIE_BARADDROFSIZE);

    /* If your HW still uses half-buffer toggling, keep programming half_size */
    writel(hws->video[ch].fmt_curr.half_size / 16,
           hws->bar0_base + CBVS_IN_BUF_BASE2 + ch * PCIE_BARADDROFSIZE);
}

static inline u32 hws_read_port_hpd(struct hws_pcie_dev *pdx, int port)
{
	/* OR the two pipes that belong to this HDMI jack */
	int pipe0 = port * 2;
	int pipe1 = pipe0 + 1;
	// FIXME
	return  READ_REGISTER_ULONG(pdx, HWS_REG_HPD(pipe0)) |
		READ_REGISTER_ULONG(pdx, HWS_REG_HPD(pipe1));
}

int set_video_format_size(struct hws_pcie_dev *pdx, int ch, int w, int h)
{
	int hf_size;
	int down_size;

	if (pdx->support_yv12 == 0) {
		hf_size = (w * h) / (16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = (w * h * 2) - hf_size;
	} else if (pdx->support_yv12 == 1) {
		hf_size = (w * h * 12) / (16 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 12) / 8) - hf_size;
	} else {
		hf_size = (w * h * 5) / (8 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 5) / 4) - hf_size;
	}
	pdx->video[ch].fmt_curr.width = w; // Image Width
	pdx->video[ch].fmt_curr.height = h; // Image Height 

	pdx->video[ch].fmt_curr.half_size = hf_size;
	pdx->video[ch].fmt_curr.down_size = down_size;

	return 1;
}

void change_video_size(struct hws_pcie_dev *pdx, int ch, int w, int h,
			    int interlace)
{
	int j;
	int halfframeLength;
	unsigned long flags;
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
		pdx->video[ch].queue_status[j].width = w;
		pdx->video[ch].queue_status[j].height = h;
		pdx->video[ch].queue_status[j].interlace = interlace;
	}
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);

	set_video_format_size(pdx, ch, w, h);
	halfframeLength = pdx->video[ch].fmt_curr.half_size / 16;

	// FIXME
	WRITE_REGISTER_ULONG(
		pdx, (DWORD)(CBVS_IN_BUF_BASE2 + (ch * PCIE_BARADDROFSIZE)),
		halfframeLength); //Buffer 1 address
}

int check_video_capture(struct hws_pcie_dev *pdx, int index)
{
	u32 status;
	int enable;

	// FIXME
	status = READ_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE);
	enable = (status >> index) & 0x01;
	return enable;
}

void hws_enable_video_capture(struct hws_pcie_dev *hws,
                                     unsigned int chan,
                                     bool on)
{
    u32 status;

    /* bail if the PCI device is lost or chan out of range */
    if (hws->pci_lost || chan >= hws->max_channels)
        return;

    /* read-modify-write the bitmask register */
    status = readl(hws->bar0_base + HWS_REG_VCAP_ENABLE);
    if (on)
        status |= BIT(chan);
    else
        status &= ~BIT(chan);

    /* track state in our per-channel struct */
    hws->video[chan].cap_active = on;

    /* write it back */
    writel(status, hdev->bar0_base + HWS_REG_VCAP_ENABLE);
}

void check_card_status(struct hws_pcie_dev *pdx)
{
	u32 status;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);

	if ((status & BIT(0)) != BIT(0)) {
		InitVideoSys(pdx, 1);
	}
}

int StartVideoCapture(struct hws_pcie_dev *pdx, int index)
{
	int j;

	if (pdx->video[index].cap_active == 1) {
		// FIXME: legacy in func
		check_card_status(pdx);
		if (check_video_capture(pdx, index) == 0) {
			hws_enable_video_capture(pdx, index, true);
		}
		return -1;
	}
	//--------------------
	check_card_status(pdx);
	//--------------------
	// FIXME: check if I need to set `stop_requested` in the video[index] to 0
	//spin_lock_irqsave(&pdx->videoslock[index], flags);
	for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
		pdx->m_pVCAPStatus[index][j].byLock = MEM_UNLOCK;
		pdx->m_pVCAPStatus[index][j].byPath = 2;
		pdx->m_VideoInfo[index].pStatusInfo[j].byLock = MEM_UNLOCK;
	}
	pdx->video[index].rd_idx = 0;
	pdx->m_VideoInfo[index].dwisRuning = 1;
	pdx->m_VideoInfo[index].m_nVideoIndex = 0;
	//spin_unlock_irqrestore(&pdx->videoslock[index], flags);

	pdx->video[index].size_changed_flag = 0;
	pdx->video[index].irq_done_flag = 1;
	pdx->video[index].irq_event = 1;
	atomic_set(&hws->audio[ch].dma_busy, 0);

	pdx->video_data[index] = 0;
	hws_enable_video_capture(pdx, index, true);
	return 0;
}

void StopVideoCapture(struct hws_pcie_dev *pdx, int index)
{
	if (pdx->video[index].cap_active == 0)
		return;

	pdx->m_VideoInfo[index].dwisRuning = 0;
	atomic_set(&pdx->video[index].stop_requested, 1);
	pdx->video[index].irq_event = 0;
	pdx->video[index].size_changed_flag = 0;
	hws_enable_video_capture(pdx, index, false);
	pdx->video[index].irq_done_flag = 0;
}

void check_video_format(struct hws_pcie_dev *pdx)
{
	int i;
	for (i = 0; i < pdx->cur_max_video_ch; i++) {
        // FIXME: figure out if we can check update_hpd_status early and exit fast
		pdx->video[index].signal_loss_cnt = get_video_status(pdx, i);

        /* If we just detected a loss on an active capture channel… */
		if ((pdx->video[index].signal_loss_cnt  == 0x1) &&
		    (pdx->video[index].cap_active == true)) {
              /* …schedule the “no‐video” handler on the vido_wq workqueue */
			queue_work(pdx->vido_wq, &pdx->video[i].videowork);
		}
	}
}

void hws_init_video_sys(struct hws_pcie_dev *hws, bool enable)
{
    int i, j;

    /* If already running and we're not resetting, nothing to do */
    if (hws->start_run && !enable)
        return;

    /* 1) reset the decoder mode register to 0 */
    writel(0x00000000, hdev->bar0_base + HWS_REG_DEC_MODE);

    // FIXME: remove, once the device has been created, on stream_on it should point the DMA address
    // correctly to the vb2 buffer. The legacy behavior was to allocate to the driver owned buffer
    // new behavior should make intelligent decision around when the devices (either alsa or vb2) are created
    hws_set_dma_address(hws);

    /* 3) on a full reset, clear all per-channel status and indices */
    if (!enable) {
        for (i = 0; i < hws->max_channels; i++) {
            struct vcap_status *vs = hws->video[i].queue_status;

            for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
                vs[j].lock      = MEM_UNLOCK;
                vs[j].path      = 2;
                vs[j].field     = 0;
                vs[j].interlace = 0;
            }

            /* reset audio write pointer */
            hws->audio[i].wr_idx = 0;

            /* helpers to arm/disable capture engines */
            hws_enable_video_capture(hws, i, false);
            hws_enable_audio_capture(hws, i, 0);
        }
    }

    /* 4) enable all interrupts */
    writel(0x003FFFFF, hws->bar0_base + INT_EN_REG_BASE);

    /* 5) “Start run”: set bit31, wait a bit, then program low 24 bits */
    writel(0x80000000, hws->bar0_base + HWS_REG_DEC_MODE);
    udelay(500);
    writel(0x80FFFFFF, hws->bar0_base + HWS_REG_DEC_MODE);
    writel(0x00000013, hws->bar0_base + HWS_REG_DEC_MODE);

    /* 6) record that we're now running */
    hws->start_run = true;
}

/* ──────────────────────────────────────────────────────────────── */
/* Utility: compare-and-write to a 32-bit register only if needed  */
/* ──────────────────────────────────────────────────────────────── */
static inline void write_if_diff(struct hws_pcie_dev *pdx,
                                 u32 reg_addr, u32 new_val)
{

    // FIXME
    if (READ_REGISTER_ULONG(pdx, reg_addr) != new_val)
        // FIXME
        WRITE_REGISTER_ULONG(pdx, reg_addr, new_val);
}

/* ──────────────────────────────────────────────────────────────── */
/* 0. HPD / +5 V                                                   */
/* ──────────────────────────────────────────────────────────────── */
static bool update_hpd_status(struct hws_pcie_dev *pdx, unsigned int ch)
{
    u32  hpd    = hws_read_port_hpd(pdx, ch);   /* jack-level status   */
    bool power  = !!(hpd & HWS_5V_BIT);         /* +5 V present        */
    bool hpd_hi = !!(hpd & HWS_HPD_BIT);        /* HPD line asserted   */

    /* Cache the +5 V state in V4L2 ctrl core (only when it changes) */
    if (power != pdx->video[ch].detect_tx_5v_ctrl->cur.val)
        v4l2_ctrl_s_ctrl(pdx->video[ch].detect_tx_5v_ctrl, power);

    /* “Signal present” means *both* rails up. Tweak if your HW differs. */
    return power && hpd_hi;
}

/* ──────────────────────────────────────────────────────────────── */
/* 1. Active / interlace flags                                     */
/* ──────────────────────────────────────────────────────────────── */
static bool update_active_and_interlace_flags(struct hws_pcie_dev *pdx,
                                              unsigned int ch)
{
    // FIXME
    u32  reg        = READ_REGISTER_ULONG(pdx, HWS_REG_ACTIVE_STATUS);
    bool active_vid = !!((reg >> ch) & 0x01);
    bool interlace  = !!(((reg >> 8) >> ch) & 0x01);

    /* Track interlace in driver state so other paths don’t re-decode */
    pdx->video[ch].queue_status[0].interlace = interlace;

    return active_vid;               /* false → “no video” early-out   */
}

/* ──────────────────────────────────────────────────────────────── */
/* 2a. Modern devices (HW version > 0)                              */
/* ──────────────────────────────────────────────────────────────── */
static void handle_hwv2_path(struct hws_pcie_dev *pdx, unsigned int ch)
{
    /* ── frame-rate in ─────────────────────────────────────────── */
    // FIXME
    u32 frame_rate = READ_REGISTER_ULONG(pdx, HWS_REG_FRAME_RATE(ch));
    pdx->video[ch].queue_status[0].frame_rate = frame_rate;

    /* ── output resolution (programmed) ────────────────────────── */
    // FIXME
    u32 reg          = READ_REGISTER_ULONG(pdx, HWS_REG_OUT_RES(ch));
    u16 out_res_w    =  reg        & 0xFFFF;
    u16 out_res_h    = (reg >> 16) & 0xFFFF;

    if (out_res_w != pdx->video[ch].queue_status[0].out_width ||
        out_res_h != pdx->video[ch].queue_status[0].out_height)
    {
        u32 packed =  (pdx->video[ch].queue_status[0].out_height << 16) |
                       pdx->video[ch].queue_status[0].out_width;
        write_if_diff(pdx, HWS_REG_OUT_RES(ch), packed);
    }

    /* ── output frame-rate (programmed) ────────────────────────── */
    // FIXME
    u32 out_fps = READ_REGISTER_ULONG(pdx, HWS_REG_OUT_FRAME_RATE(ch));
    if (out_fps != pdx->video[ch].queue_status[0].out_fps)
        write_if_diff(pdx, HWS_REG_OUT_FRAME_RATE(ch),
                           pdx->video[ch].queue_status[0].out_fps);

    /* ── BCHS (brightness/contrast/hue/saturation) ─────────────── */
    // FIXME
    reg = READ_REGISTER_ULONG(pdx, HWS_REG_BCHS(ch));
    u8 br =  reg        & 0xFF;
    u8 co = (reg >> 8)  & 0xFF;
    u8 hu = (reg >> 16) & 0xFF;
    u8 sa = (reg >> 24) & 0xFF;

    if (br != pdx->video[ch].current_brightness ||
        co != pdx->video[ch].current_contrast   ||
        hu != pdx->video[ch].current_hue        ||
        sa != pdx->video[ch].current_saturation)
    {

        u32 packed =  (pdx->video[ch].current_saturation << 24) |
                      (pdx->video[ch].current_hue        << 16) |
                      (pdx->video[ch].current_contrast   <<  8) |
                       pdx->video[ch].current_brightness;
        write_if_diff(pdx, HWS_REG_BCHS(ch), packed);
    }

    /* ── HDCP detect bit ───────────────────────────────────────── */
    // FIXME
    reg = READ_REGISTER_ULONG(pdx, HWS_REG_HDCP_STATUS);
    pdx->video[ch].queue_status[0].hdcp = !!((reg >> ch) & 0x01);
}

/* ──────────────────────────────────────────────────────────────── */
/* 2b. Legacy devices (SW FPS estimator)                           */
/* ──────────────────────────────────────────────────────────────── */
static void handle_legacy_path(struct hws_pcie_dev *pdx, unsigned int ch)
{
    // FIXME
    u32 sw_rate = pdx->m_dwSWFrameRate[ch];

    if (sw_rate > 10) {
        int fps;
        if      (sw_rate > 55 * 2) fps = 60;
        else if (sw_rate > 45 * 2) fps = 50;
        else if (sw_rate > 25 * 2) fps = 30;
        else if (sw_rate > 20 * 2) fps = 25;
        else                       fps = 60;  /* default fallback */

        // FIXME
        pdx->m_pVCAPStatus[ch][0].dwFrameRate = fps;
    }

    // FIXME
    pdx->m_dwSWFrameRate[ch] = 0;        /* reset estimator window */
}

/* ──────────────────────────────────────────────────────────────── */
/* 3. Live input resolution + change_video_size() trigger            */
/* ──────────────────────────────────────────────────────────────── */
static void update_live_resolution(struct hws_pcie_dev *pdx, unsigned int ch)
{
    // FIXME
    u32 reg   = READ_REGISTER_ULONG(pdx, HWS_REG_IN_RES(ch));
    u16 res_w =  reg        & 0xFFFF;
    u16 res_h = (reg >> 16) & 0xFFFF;

    bool interlace = pdx->video[ch].queue_status[0].interlace;

    bool within_hw =
        (res_w <= MAX_VIDEO_HW_W) &&
        ((!interlace &&  res_h      <= MAX_VIDEO_HW_H) ||
         ( interlace && (res_h * 2) <= MAX_VIDEO_HW_H));

    bool changed =
        (res_w      != pdx->video[ch].queue_status[0].width) ||
        (res_h      != pdx->video[ch].queue_status[0].height)||
        (interlace  != pdx->video[ch].queue_status[0].interlace);

    if (within_hw && changed)
        change_video_size(pdx, ch, res_w, res_h, interlace);
}

int get_video_status(struct hws_pcie_dev *pdx, unsigned int ch)
{
    // FIXME: I don't think this works?
    // if (!update_hpd_status(pdx, ch))
    //    return 1;                         /* no +5 V / HPD */

    if (!update_active_and_interlace_flags(pdx, ch))
        return 1;                         /* no active video */

    if (pdx->hw_ver> 0)
        handle_hwv2_path(pdx, ch);
    else
        // FIXME: legacy struct names in subfunction
        handle_legacy_path(pdx, ch);

    update_live_resolution(pdx, ch);

    return 0;                             /* success */
}
