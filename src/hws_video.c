/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/overflow.h>
#include <media/videobuf2-v4l2.h>

#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
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
/* One vb2 buffer wrapper per frame */
struct hwsvideo_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head       queue;   /* linked on hws_video.capture_queue */
};

/* Convenience cast */
static inline struct hwsvideo_buffer *to_hwsbuf(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct hwsvideo_buffer, vb);
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

static int check_video_capture(struct hws_pcie_dev *hws, unsigned int ch)
{
	u32 status;

	if (!hws || !hws->bar0_base)
		return -ENODEV;
	if (ch >= hws->max_channels)
		return -EINVAL;

	status = readl(hws->bar0_base + HWS_REG_VCAP_ENABLE);

	/* Common pattern for a dead/removed PCIe device */
	if (unlikely(status == 0xFFFFFFFF)) {
		hws->pci_lost = true;
		return -ENODEV;
	}

	return !!(status & BIT(ch));
}

void hws_enable_video_capture(struct hws_pcie_dev *hws,
                                     unsigned int chan,
                                     bool on)
{
	u32 status;

	if (!hws || hws->pci_lost || chan >= hws->max_channels)
		return;

	status = readl(hws->bar0_base + HWS_REG_VCAP_ENABLE);
	status = on ? (status | BIT(chan)) : (status & ~BIT(chan));
	writel(status, hws->bar0_base + HWS_REG_VCAP_ENABLE);
	(void)readl(hws->bar0_base + HWS_REG_VCAP_ENABLE);

	hws->video[chan].cap_active = on;

	dev_dbg(&hws->pdev->dev, "vcap %s ch%u (reg=0x%08x)\n",
		on ? "ON" : "OFF", chan, status);
}

static int hws_check_card_status(struct hws_pcie_dev *hws)
{
	u32 status;

	if (!hws || !hws->bar0_base)
		return -ENODEV;

	status = readl(hws->bar0_base + HWS_REG_SYS_STATUS);

	/* Common “device missing” pattern */
	if (unlikely(status == 0xFFFFFFFF)) {
		hws->pci_lost = true;
		dev_err(&hws->pdev->dev, "PCIe device not responding\n");
		return -ENODEV;
	}

	/* If RUN/READY bit (bit0) isn’t set, (re)initialize the video core */
	if (!(status & BIT(0))) {
		dev_dbg(&hws->pdev->dev,
			"SYS_STATUS not ready (0x%08x), reinitializing\n", status);
		hws_init_video_sys(hws, true);
		/* Optional: verify the core cleared its busy bit, if you have one */
		/* int ret = hws_check_busy(hws); */
		/* if (ret) return ret; */
	}

	return 0;
}


void check_video_format(struct hws_pcie_dev *pdx)
{
	int i;
	for (i = 0; i < pdx->cur_max_video_ch; i++) {
		// FIXME: figure out if we can check update_hpd_status early and exit fast
		// get_video_status calls -> update_active_and_interlace_flags -> readl 
		pdx->video[index].signal_loss_cnt = get_video_status(pdx, i);

		/* If we just detected a loss on an active capture channel… */
		if ((pdx->video[index].signal_loss_cnt  == 0x1) &&
		    (pdx->video[index].cap_active == true)) {
		      /* …schedule the “no‐video” handler on the vido_wq workqueue */
			// FIXME: this is where we can catch if the system has blanked on us
			// probably need to rewire this, because we removed the videowork
			// https://chatgpt.com/s/t_689fce8c84308191a15ec967d75165a7
			// queue_work(pdx->vido_wq, &pdx->video[i].videowork);
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
    writel(0x00000000, hws->bar0_base + HWS_REG_DEC_MODE);

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
static bool hws_update_active_interlace(struct hws_pcie_dev *pdx, unsigned int ch)
{
	u32 reg;
	bool active, interlace;

	/* Defensive: ensure channel is in range for this device */
	if (ch >= HWS_MAX_CHANNELS)
		return false;

	reg = readl(pdx->bar0_base + HWS_REG_ACTIVE_STATUS);

	active    = !!(reg & BIT(ch));
	interlace = !!(reg & BIT(8 + ch));

	/* Cache interlace flag so other paths don't need to re-decode the register */
	pdx->video[ch].queue_status[0].interlace = interlace;

	return active;
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


static int hws_open(struct file *file)
{
    struct hws_video *vid = video_drvdata(file);

    /* Hard-fail additional opens while a capture is active */
    if (!v4l2_fh_is_singular_file(file) && vb2_is_busy(&vid->buffer_queue))
        return -EBUSY;

    return 0;
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
	struct hws_video *vid = q->drv_priv;
	struct hws_pcie_dev *hws = vid->parent;
	size_t size, tmp;

	/* One plane: YUYV (2 bytes/pixel). Use current programmed OUT WxH. */
	if (check_mul_overflow(vid->fmt_curr.width, vid->fmt_curr.height, &tmp) ||
	    check_mul_overflow(tmp, 2, &size))
		return -EOVERFLOW;

	size = PAGE_ALIGN(size);

	if (*num_planes) {
		if (sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*num_planes   = 1;
	sizes[0]      = size;
	if (alloc_devs)
		alloc_devs[0] = &hws->pdev->dev; /* dma-contig device */

	/* Optional: ensure a minimum number of buffers */
	if (*num_buffers < 3)
		*num_buffers = 3;

	return 0;
}

static int hws_buffer_prepare(struct vb2_buffer *vb)
{
	struct hws_video *vid = vb->vb2_queue->drv_priv;
	size_t need = (size_t)vid->fmt_curr.width * vid->fmt_curr.height * 2;

	if (vb2_plane_size(vb, 0) < need)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, need);
	return 0;
}

static void hws_buffer_queue(struct vb2_buffer *vb)
{
	struct hws_video *vid = vb->vb2_queue->drv_priv;
	struct hwsvideo_buffer *buf = to_hwsbuf(vb);
	unsigned long flags;

	spin_lock_irqsave(&vid->irq_lock, flags);
	list_add_tail(&buf->queue, &vid->capture_queue);

	/* If streaming and no in-flight buffer, prime HW immediately */
	if (vid->cap_active && !vid->cur) {
		vid->cur = buf;
		hws_program_video_from_vb2(vid->parent, vid->channel_index, &buf->vb.vb2_buf);
		/* Arm engine if not already */
		hws_enable_video_capture(vid->parent, vid->channel_index, true);
	}
	spin_unlock_irqrestore(&vid->irq_lock, flags);
}

static int hws_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hws_video *vid = q->drv_priv;
	struct hws_pcie_dev *hws = vid->parent;
	int ret, en;
	unsigned long flags;

	/* 1) Make sure the card is alive and the core is initialized */
	ret = hws_check_card_status(hws);
	if (ret)
		return ret;

	/* 2) If we think the channel is active, verify HW; re-enable if needed */
	if (READ_ONCE(vid->cap_active)) {
		en = check_video_capture(hws, vid->channel_index);
		if (en < 0)
			return en;
		if (en == 0)
			hws_enable_video_capture(hws, vid->channel_index, true);
	}

	/* 3) Must have at least one queued buffer to start */
	spin_lock_irqsave(&vid->irq_lock, flags);
	if (list_empty(&vid->capture_queue)) {
		spin_unlock_irqrestore(&vid->irq_lock, flags);
		return -ENOSPC;
	}

	vid->stop_requested        = false;
	vid->last_buf_half_toggle  = 0;
	vid->half_seen             = false;

	/* 4) Prime first buffer if nothing in-flight */
	if (!vid->cur) {
		vid->cur = list_first_entry(&vid->capture_queue,
					    struct hwsvideo_buffer, queue);
		list_del(&vid->cur->queue);

		hws_program_video_from_vb2(hws, vid->channel_index,
					   &vid->cur->vb.vb2_buf);
	}

	/* 5) Ensure HW capture is actually enabled for this channel */
	en = check_video_capture(hws, vid->channel_index);
	if (en <= 0) {
		if (en < 0) {
			spin_unlock_irqrestore(&vid->irq_lock, flags);
			return en; /* device error */
		}
		hws_enable_video_capture(hws, vid->channel_index, true);
	}

	vid->cap_active = true;
	spin_unlock_irqrestore(&vid->irq_lock, flags);
	return 0;
}

static void hws_stop_streaming(struct vb2_queue *q)
{
	struct hws_video *vid = q->drv_priv;
	unsigned long flags;

	/* Stop HW first */
	hws_enable_video_capture(vid->parent, vid->channel_index, false);

	spin_lock_irqsave(&vid->irq_lock, flags);
	vid->cap_active = false;
	vid->stop_requested = true;

	/* Return in-flight (if any) + all queued as ERROR */
	if (vid->cur) {
		vb2_buffer_done(&vid->cur->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		vid->cur = NULL;
	}
	while (!list_empty(&vid->capture_queue)) {
		struct hwsvideo_buffer *b = list_first_entry(&vid->capture_queue,
			struct hwsvideo_buffer, queue);
		list_del(&b->queue);
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&vid->irq_lock, flags);
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
	int i, ret;

	ret = devm_v4l2_device_register(&dev->pdev->dev, &dev->v4l2_device);
	if (ret) {
		dev_err(&dev->pdev->dev, "v4l2_device_register failed: %d\n", ret);
		return ret;
	}

	for (i = 0; i < dev->cur_max_video_ch; i++) {
		struct hws_video *ch = &dev->video[i];
		struct video_device *vdev;
		struct vb2_queue *q;

		/* hws_video_init_channel() should have set:
		 * - ch->parent, ch->channel_index
		 * - locks (state_lock, irq_lock)
		 * - capture_queue (INIT_LIST_HEAD)
		 * - control_handler + controls
		 * - fmt_curr (width/height)
		 * Don’t reinitialize any of those here.
		 */

		vdev = devm_video_device_alloc(&dev->pdev->dev, 0);
		if (!vdev) {
			dev_err(&dev->pdev->dev, "video_device_alloc ch%u failed\n", i);
			ret = -ENOMEM;
			goto err_unwind;
		}
		ch->video_device = vdev;

		/* Basic V4L2 node setup */
		snprintf(vdev->name, sizeof(vdev->name), "%s-hdmi%u", KBUILD_MODNAME, i);
		vdev->v4l2_dev     = &dev->v4l2_device;
		vdev->fops         = &hws_fops;          /* your file_ops */
		vdev->ioctl_ops    = &hws_ioctl_fops;    /* your ioctl_ops */
		vdev->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		vdev->lock         = &ch->state_lock;    /* serialize file ops */
		vdev->ctrl_handler = &ch->control_handler;
		vdev->vfl_dir      = VFL_DIR_RX;
		vdev->release      = video_device_release_empty;
		video_set_drvdata(vdev, ch);

		/* vb2 queue init (dma-contig) */
		q = &ch->buffer_queue;
		memset(q, 0, sizeof(*q));
		q->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		q->io_modes        = VB2_MMAP | VB2_DMABUF;
		q->drv_priv        = ch;
		q->buf_struct_size = sizeof(struct hwsvideo_buffer);
		q->ops             = &hwspcie_video_qops;      /* your vb2_ops */
		q->mem_ops         = &vb2_dma_contig_memops;
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		q->lock            = &ch->state_lock;          /* reuse state_lock */
		q->dev             = &dev->pdev->dev;

		ret = vb2_queue_init(q);
		vdev->queue = q;
		if (ret) {
			dev_err(&dev->pdev->dev, "vb2_queue_init ch%u failed: %d\n", i, ret);
			goto err_unwind;
		}

		/* Make controls live (no-op if none or already set up) */
		if (ch->control_handler.error) {
			ret = ch->control_handler.error;
			dev_err(&dev->pdev->dev, "ctrl handler ch%u error: %d\n", i, ret);
			goto err_unwind;
		}
		v4l2_ctrl_handler_setup(&ch->control_handler);

		ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
		if (ret) {
			dev_err(&dev->pdev->dev,
				"video_register_device ch%u failed: %d\n", i, ret);
			goto err_unwind;
		}
	}

	return 0;

err_unwind:
	for (i = i - 1; i >= 0; i--) {
        struct hws_video *ch = &dev->video[i];

        if (video_is_registered(ch->video_device))
            video_unregister_device(ch->video_device);

        if (ch->buffer_queue.ops)
            vb2_queue_release(&ch->buffer_queue);
	v4l2_ctrl_handler_free(&ch->control_handler);
        ch->video_device = NULL;
    }
    return ret;
}

void hws_video_unregister(struct hws_pcie_dev *dev)
{
	int i;

	if (!dev)
		return;

	for (i = 0; i < dev->cur_max_video_ch; i++) {
		struct hws_video *ch = &dev->video[i];
		unsigned long flags;

		/* 1) Stop hardware capture for this channel (if running). */
		if (ch->cap_active)
			hws_enable_video_capture(dev, i, false);

		/* 2) Drain SW queue + complete in-flight buffer as ERROR. */
		spin_lock_irqsave(&ch->irq_lock, flags);

		if (ch->cur) {
			vb2_buffer_done(&ch->cur->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			ch->cur = NULL;
		}
		while (!list_empty(&ch->capture_queue)) {
			struct hwsvideo_buffer *b =
				list_first_entry(&ch->capture_queue,
						 struct hwsvideo_buffer, queue);
			list_del(&b->queue);
			vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}

		spin_unlock_irqrestore(&ch->irq_lock, flags);

		/* 3) Release vb2 queue (safe to call once if it was inited). */
		if (ch->buffer_queue.ops)
			vb2_queue_release(&ch->buffer_queue);

		/* 4) Free V4L2 controls. */
		v4l2_ctrl_handler_free(&ch->control_handler);

		/* 5) Unregister the video node (if it was registered). */
		if (ch->video_device) {
			if (video_is_registered(ch->video_device))
				video_unregister_device(ch->video_device);
			ch->video_device = NULL;
		}

		/* 6) Reset lightweight state (optional). */
		ch->cap_active      = false;
		ch->stop_requested  = false;
		ch->last_buf_half_toggle = 0;
		ch->half_seen       = false;
		ch->signal_loss_cnt = 0;
		INIT_LIST_HEAD(&ch->capture_queue);
	}
}

