/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/pci.h>
#include <linux/kernel.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include "hws.h"
#include "hws_reg.h"
#include "hws_dma.h"
#include "hws_video.h"
#include "hws_interrupt.h"
#include "hws_v4l2_ioctl.h"
#include "hws_video_pipeline.h"

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

//struct hws_pcie_dev  *sys_dvrs_hw_pdx=NULL;
//EXPORT_SYMBOL(sys_dvrs_hw_pdx);
//u32 *map_bar0_addr=NULL; //for sys bar0
//EXPORT_SYMBOL(map_bar0_addr);

#define FRAME_DONE_MARK 0x55AAAA55

//-------------------------------------------

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
            /* helpers to arm/disable capture engines */
            hws_enable_video_capture(hws, i, false);
	    // FIXME: use false here
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

static ssize_t hws_read(struct file *file, char *buf, size_t count,
			loff_t *ppos)
{
	//printk( "%s()\n", __func__);
	return -1;
}

static int hws_open(struct file *file)
{
    struct hws_video *vid = video_drvdata(file);

    /* Hard-fail additional opens while a capture is active */
    if (!v4l2_fh_is_singular_file(file) && vb2_is_busy(vid->queue))
        return -EBUSY;

    return 0;          /* nothing else to count */
}

static int hws_release(struct file *file)
{
    return vb2_fop_release(file);
}

//----------------------------
static const struct v4l2_file_operations hws_fops = {
	.owner = THIS_MODULE,
	.open = hws_open,
	.release = hws_release,
	.read = hws_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static const struct v4l2_ioctl_ops hws_ioctl_fops = {
	.vidioc_querycap = hws_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = hws_vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = hws_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = hws_vidioc_try_fmt_vid_cap,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_g_std = hws_vidioc_g_std,
	.vidioc_s_std = hws_vidioc_s_std,
	.vidioc_enum_framesizes = hws_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = hws_vidioc_enum_frameintervals,
	// .vidioc_g_ctrl = hws_vidioc_g_ctrl,
	// .vidioc_s_ctrl = hws_vidioc_s_ctrl,
	// .vidioc_queryctrl = hws_vidioc_queryctrl,
	.vidioc_enum_input = hws_vidioc_enum_input,
	.vidioc_g_input = hws_vidioc_g_input,
	.vidioc_s_input = hws_vidioc_s_input,
	.vidioc_log_status = vidioc_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_g_parm = hws_vidioc_g_parm,
	.vidioc_s_parm = hws_vidioc_s_parm,
};


static int hws_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   const struct device *alloc_devs[])
{
	struct hws_video *videodev = q->drv_priv;
	struct hws_pcie_dev *pdx = videodev->dev;
	unsigned long flags;
    size_t size, tmp;

	if (vb2_is_busy(q))
        return -EBUSY;

	// FIXME: Get smart on lock scope, might be better to not spin lock
	// on this scope to do overflow math
	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);

    if (check_mul_overflow(videodev->current_out_width, videodev->current_out_height, &tmp) ||
        check_mul_overflow(tmp, 2, &size)) {
		spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index],
				       flags);
        return -EOVERFLOW;
    }
    size = PAGE_ALIGN(size);

	if (*num_planes) {
        int ret = sizes[0] < size ? -EINVAL : 0;
        spin_unlock_irqrestore(
            &pdx->videoslock[videodev->channel_index], flags);
        return ret;
	}
	*num_planes = 1;
	sizes[0] = size;
    alloc_devs[0]    = &pdx->pdev->dev;
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
	return 0;
}

static int hws_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct hwsvideo_buffer *buf =
		container_of(vbuf, struct hwsvideo_buffer, vb);
	struct hws_video *videodev = vb->vb2_queue->drv_priv;
	struct hws_pcie_dev *pdx = videodev->dev;
	u32 size;
	unsigned long flags;

	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);
	size = 2 * videodev->current_out_width *
	       videodev->current_out_height; // 16bit
	if (vb2_plane_size(vb, 0) < size) {
		spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index],
				       flags);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);
	buf->mem = vb2_plane_vaddr(vb, 0);
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
	return 0;
}

static void hws_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct hws_video *videodev = vb->vb2_queue->drv_priv;
	struct hwsvideo_buffer *buf =
		container_of(vbuf, struct hwsvideo_buffer, vb);
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;

	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);
	list_add_tail(&buf->queue, &videodev->queue);
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
}

static int hws_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hws_video *videodev = q->drv_priv;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;

	atomic_set(&videodev->sequence_number, 0);
	//---------------
	// FIXME: This function sucks
	StartVideoCapture(videodev->dev, videodev->channel_index);

	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);
	while (!list_empty(&videodev->queue)) {
		struct hwsvideo_buffer *buf = list_entry(
			videodev->queue.next, struct hwsvideo_buffer, queue);
		list_del(&buf->queue);

		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
	//-----------------------
	return 0;
}

static void hws_stop_streaming(struct vb2_queue *q)
{
	struct hws_video *videodev = q->drv_priv;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
	//-----------------------------------
	StopVideoCapture(videodev->dev, videodev->index);
	//------------------
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	while (!list_empty(&videodev->queue)) {
		struct hwsvideo_buffer *buf = list_entry(
			videodev->queue.next, struct hwsvideo_buffer, queue);
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
//-----------------------------------------------------------------
}

static const struct vb2_ops hwspcie_video_qops = {
	.queue_setup = hws_queue_setup,
	.buf_prepare = hws_buffer_prepare,
	// .buf_finish = hws_buffer_finish,
	.buf_queue = hws_buffer_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = hws_start_streaming,
	.stop_streaming = hws_stop_streaming,
};

int hws_video_register(struct hws_pcie_dev *dev)
{
	int i, err;
	struct video_device *vdev;
	struct vb2_queue *q;

	err = devm_v4l2_device_register(&dev->pdev->dev, &dev->v4l2_dev);
	if (err) {
		dev_err(&dev->pdev->dev,
			"v4l2_device_register failed: %d\n", err);
		return err;
	}

	for (i = 0; i < dev->cur_max_video_ch; i++) {
		struct hws_video *hws = &dev->video[i];

		/* init channel state */
		hws->channel_index      = i;
		hws->parent             = dev;
		hws->pixel_format       = V4L2_PIX_FMT_YUYV;
		hws->current_brightness = BrightnessDefault;
		hws->current_contrast   = ContrastDefault;
		hws->current_saturation = SaturationDefault;
		hws->current_hue        = HueDefault;

		/* initialise locks & lists */
		mutex_init(&hws->state_lock);
		mutex_init(&hws->capture_queue_lock);
		spin_lock_init(&hws->irq_lock);
		INIT_LIST_HEAD(&hws->capture_queue);

		/* setup video_device */
		vdev = devm_video_device_alloc(&dev->pdev->dev, 0);
		if (!vdev) {
		    dev_err(&dev->pdev->dev, "video_device_alloc failed\n");
		    err = -ENOMEM;
		    goto err_unreg_nodes;
		}
		hws->vdev = vdev;

		vdev->v4l2_dev     = &dev->v4l2_dev;
		vdev->fops         = &hws_fops;
		vdev->ioctl_ops    = &hws_ioctl_fops;
		vdev->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		vdev->lock         = &hws->state_lock;
		vdev->ctrl_handler = &hws->control_handler;
		vdev->vfl_dir      = VFL_DIR_RX;
		vdev->release      = video_device_release_empty;
		video_set_drvdata(vdev, hws);
		video_device_set_name(vdev, "%s-hdmi%d",
				      KBUILD_MODNAME, i);

		/* Setup vb2 queue with designated initializer */
		q = &hws->buffer_queue;

		// FIXME: Figure out if the hw only supports 4 GB RAM
		*q = (struct vb2_queue) {
			.type            = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.io_modes        = VB2_READ | VB2_MMAP | VB2_USERPTR,
			.gfp_flags       = GFP_DMA32,
			.drv_priv        = hws,
			.buf_struct_size = sizeof(struct hwsvideo_buffer),
			.ops             = &hwspcie_video_qops,
			.mem_ops         = &vb2_vmalloc_memops,
			.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
			.lock            = &hws->capture_queue_lock,
			.dev             = &dev->pdev->dev,
		};

		vdev->queue = q;

		err = vb2_queue_init(q);
		if (err) {
			dev_err(&dev->pdev->dev,
				"vb2_queue_init(ch%d) failed: %d\n", i, err);
			goto err_unreg_nodes;
		}

		INIT_WORK(&hws->video_work, video_data_process);
		err = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
		if (err) {
			dev_err(&dev->pdev->dev,
				"video_register_device(ch%d) failed: %d\n",
				i, err);
			goto err_unreg_nodes;
		}
	}

	return 0;
err_unreg_nodes:
    while (--i >= 0) {
        struct hws_video *hws = &dev->video[i];
        vb2_queue_cleanup(&hws->buffer_queue);
        video_unregister_device(hws->vdev);
    }
    return err;
}

void hws_video_unregister(struct hws_pcie_dev *dev)
{
    int i;

    /* For each channel, in reverse order of registration: */
    for (i = 0; i < dev->cur_max_video_ch; i++) {
        struct hws_video *hws = &dev->video[i];

        /* 1) Stop any pending work */
        cancel_work_sync(&hws->video_work);

        /* 2) Unregister the V4L2 video device;
         *    this also calls ->release and frees the struct video_device
         *    that was allocated via devm_video_device_alloc().
         */
        video_unregister_device(hws->vdev);

        /* 3) Clean up the vb2 queue buffers and any vmalloc’d buffers. */
        vb2_queue_cleanup(&hws->buffer_queue);
    }

    /* Note: devm_v4l2_device_register() is managed by the device,
     * so you don’t need to explicitly unregister it here.
     */
}

