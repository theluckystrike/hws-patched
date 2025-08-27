/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/overflow.h>
#include <linux/delay.h>
#include <linux/bits.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>

#include "hws.h"
#include "hws_reg.h"
#include "hws_video.h"
#include "hws_audio.h"
#include "hws_irq.h"
#include "hws_v4l2_ioctl.h"

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>


#define FRAME_DONE_MARK 0x55AAAA55

static void update_live_resolution(struct hws_pcie_dev *pdx, unsigned int ch);
static bool hws_update_active_interlace(struct hws_pcie_dev *pdx, unsigned int ch);
static void handle_hwv2_path(struct hws_pcie_dev *hws, unsigned int ch);
static void handle_legacy_path(struct hws_pcie_dev *hws, unsigned int ch);
static u32 hws_calc_sizeimage(struct hws_video *v, u16 w, u16 h, bool interlaced);

static int hws_ctrls_init(struct hws_video *vid)
{
	struct v4l2_ctrl_handler *hdl = &vid->control_handler;

	/* Create BCHS + one DV status control */
	v4l2_ctrl_handler_init(hdl, 5);

	vid->ctrl_brightness = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
		V4L2_CID_BRIGHTNESS,
		MIN_VAMP_BRIGHTNESS_UNITS, MAX_VAMP_BRIGHTNESS_UNITS, 1, BrightnessDefault);

	vid->ctrl_contrast = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
		V4L2_CID_CONTRAST,
		MIN_VAMP_CONTRAST_UNITS, MAX_VAMP_CONTRAST_UNITS, 1, ContrastDefault);

	vid->ctrl_saturation = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
		V4L2_CID_SATURATION,
		MIN_VAMP_SATURATION_UNITS, MAX_VAMP_SATURATION_UNITS, 1, SaturationDefault);

	vid->ctrl_hue = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
		V4L2_CID_HUE,
		MIN_VAMP_HUE_UNITS, MAX_VAMP_HUE_UNITS, 1, HueDefault);

	vid->detect_tx_5v_control = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
		V4L2_CID_DV_RX_POWER_PRESENT, 0, 1, 1, 0);
	if (vid->detect_tx_5v_control)
		vid->detect_tx_5v_control->flags |=
			V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		int err = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return err;
	}
	return 0;
}

int hws_video_init_channel(struct hws_pcie_dev *pdev, int ch)
{
	struct hws_video *vid;
	struct v4l2_ctrl_handler *hdl;

	/* basic sanity */
	if (!pdev || ch < 0 || ch >= pdev->max_channels)
		return -EINVAL;

	vid = &pdev->video[ch];

	/* hard reset the per-channel struct (safe here since we init everything next) */
	memset(vid, 0, sizeof(*vid));

	/* identity */
	vid->parent        = pdev;
	vid->channel_index = ch;

	/* locks & lists */
	mutex_init(&vid->state_lock);
	spin_lock_init(&vid->irq_lock);
	INIT_LIST_HEAD(&vid->capture_queue);
	vid->sequence_number = 0;
	vid->active = NULL;

	/* typed tasklet: bind handler once */
	tasklet_setup(&vid->bh_tasklet, hws_bh_video);

	/* default format (adjust to your HW) */
	vid->pix.width        = 1920;
	vid->pix.height       = 1080;
	vid->pix.fourcc       = V4L2_PIX_FMT_YUYV;
	vid->pix.bytesperline = ALIGN(vid->pix.width * 2, 64);
	vid->pix.sizeimage    = vid->pix.bytesperline * vid->pix.height;
	vid->pix.field        = V4L2_FIELD_NONE;
	vid->pix.colorspace   = V4L2_COLORSPACE_REC709;
	vid->pix.ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	vid->pix.quantization = V4L2_QUANTIZATION_LIM_RANGE;
	vid->pix.xfer_func    = V4L2_XFER_FUNC_DEFAULT;
	vid->pix.interlaced   = false;
	vid->pix.half_size    = vid->pix.sizeimage / 2; /* if HW uses halves */
	vid->alloc_sizeimage  = vid->pix.sizeimage;

	/* color controls default (mid-scale) */
	vid->current_brightness  = 0x80;
	vid->current_contrast    = 0x80;
	vid->current_saturation  = 0x80;
	vid->current_hue         = 0x80;

	/* capture state */
	vid->cap_active           = false;
	vid->stop_requested       = false;
	vid->last_buf_half_toggle = 0;
	vid->half_seen            = false;
	vid->signal_loss_cnt      = 0;

	/* Create BCHS + DV power-present as modern controls */
	{
		int err = hws_ctrls_init(vid);
		if (err) {
			dev_err(&pdev->pdev->dev,
				"v4l2 ctrl init failed on ch%d: %d\n", ch, err);
			return err;
		}
	}

	return 0;
}

static void hws_video_drain_queue_locked(struct hws_video *vid)
{
	/* Return in-flight first */
	if (vid->active) {
		vb2_buffer_done(&vid->active->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		vid->active = NULL;
	}

	/* Then everything queued */
	while (!list_empty(&vid->capture_queue)) {
		struct hwsvideo_buffer *b =
			list_first_entry(&vid->capture_queue,
					 struct hwsvideo_buffer, list);
		list_del_init(&b->list);
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
}

void hws_video_cleanup_channel(struct hws_pcie_dev *pdev, int ch)
{
	struct hws_video *vid;
	unsigned long flags;

	if (!pdev || ch < 0 || ch >= pdev->max_channels)
		return;

	vid = &pdev->video[ch];

	/* 1) Stop HW best-effort for this channel */
	hws_enable_video_capture(vid->parent, vid->channel_index, false);

	/* 2) Flip software state so IRQ/BH will be no-ops if they run */
	WRITE_ONCE(vid->stop_requested, true);
	WRITE_ONCE(vid->cap_active, false);

	/* 3) Make sure the tasklet can’t run anymore (prevents races with drain) */
	tasklet_kill(&vid->bh_tasklet);

	/* 4) Drain SW capture queue & in-flight under lock */
	spin_lock_irqsave(&vid->irq_lock, flags);
	hws_video_drain_queue_locked(vid);
	spin_unlock_irqrestore(&vid->irq_lock, flags);

	/* 5) Release VB2 queue if initialized */
	if (vid->buffer_queue.ops)
		vb2_queue_release(&vid->buffer_queue);

	/* 6) Free V4L2 controls */
	v4l2_ctrl_handler_free(&vid->control_handler);

	/* 7) Unregister the video_device if we own it */
	if (vid->video_device && video_is_registered(vid->video_device))
		video_unregister_device(vid->video_device);
	/* If you allocated it with video_device_alloc(), release it here:
	 * video_device_release(vid->video_device);
	 */
	vid->video_device = NULL;

	/* 8) Reset simple state (don’t memset the whole struct here) */
	mutex_destroy(&vid->state_lock);
	INIT_LIST_HEAD(&vid->capture_queue);
	vid->active              = NULL;
	vid->stop_requested      = false;
	vid->last_buf_half_toggle = 0;
	vid->half_seen           = false;
	vid->signal_loss_cnt     = 0;
}




/* Convenience cast */
static inline struct hwsvideo_buffer *to_hwsbuf(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct hwsvideo_buffer, vb);
}

void hws_program_video_from_vb2(struct hws_pcie_dev *hws,
                                       unsigned int ch,
                                       struct vb2_buffer *vb)
{
	const u32 addr_mask     = PCI_E_BAR_ADD_MASK;
	const u32 addr_low_mask = PCI_E_BAR_ADD_LOWMASK;
	const u32 table_off     = 0x208 + ch * 8; /* one 64-bit slot per ch */

	dma_addr_t paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
	u32 lo = lower_32_bits(paddr);
	u32 hi = upper_32_bits(paddr);
	u32 pci_addr = lo & addr_low_mask;

	lo &= addr_mask;

	/* Program 64-bit BAR remap entry for this channel */
	writel_relaxed(hi, hws->bar0_base + PCI_ADDR_TABLE_BASE + table_off);
	writel_relaxed(lo, hws->bar0_base + PCI_ADDR_TABLE_BASE +
	                              table_off + PCIE_BARADDROFSIZE);

	/* Program capture engine per-channel base/size */
	writel_relaxed((ch + 1) * PCIEBAR_AXI_BASE + pci_addr,
		       hws->bar0_base + CVBS_IN_BUF_BASE +
		                      ch * PCIE_BARADDROFSIZE);

	/* If HW uses half-buffer toggling, program half_size (device-specific) */
	writel_relaxed(hws->video[ch].pix.half_size / 16,
		       hws->bar0_base + CVBS_IN_BUF_BASE2 +
		                      ch * PCIE_BARADDROFSIZE);

	/* Ensure above posted writes reach the device before capture starts */
	readl(hws->bar0_base + HWS_REG_INT_STATUS);
}


static inline u32 hws_read_port_hpd(struct hws_pcie_dev *hws, unsigned int port)
{
	u32 v0, v1;
	unsigned int pipe0, pipe1;

	if (!hws || !hws->bar0_base)
		return 0;

	/* Each HDMI jack uses two pipes: 2*port and 2*port+1.
	 * Use cur_max_video_ch since it reflects the active HW config.
	 */
	pipe0 = port * 2;
	pipe1 = pipe0 + 1;
	if (pipe1 >= hws->cur_max_video_ch)
		return 0;

	v0 = readl(hws->bar0_base + HWS_REG_HPD(pipe0));
	v1 = readl(hws->bar0_base + HWS_REG_HPD(pipe1));

	/* Treat all-ones as device-gone and avoid propagating garbage. */
	if (unlikely(v0 == 0xFFFFFFFF || v1 == 0xFFFFFFFF)) {
		hws->pci_lost = true;
		return 0;
	}

	return v0 | v1;
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

void hws_init_video_sys(struct hws_pcie_dev *hws, bool enable)
{
    int i;

    if (hws->start_run && !enable)
        return;

    /* 1) reset the decoder mode register to 0 */
    writel(0x00000000, hws->bar0_base + HWS_REG_DEC_MODE);

    /* 3) on a full reset, clear all per-channel status and indices */
    if (!enable) {
        for (i = 0; i < hws->max_channels; i++) {
            /* helpers to arm/disable capture engines */
            hws_enable_video_capture(hws, i, false);
            hws_enable_audio_capture(hws, i, false);
        }
    }

    /* 4) enable all interrupts */
    writel(0x3FFFFF, hws->bar0_base + INT_EN_REG_BASE);

    /* 5) “Start run”: set bit31, wait a bit, then program low 24 bits */
    writel(0x80000000, hws->bar0_base + HWS_REG_DEC_MODE);
    // udelay(500);
    writel(0x80FFFFFF, hws->bar0_base + HWS_REG_DEC_MODE);
    writel(0x00000013, hws->bar0_base + HWS_REG_DEC_MODE);

    /* 6) record that we're now running */
    hws->start_run = true;
}

int hws_check_card_status(struct hws_pcie_dev *hws)
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
	    // FIXME: I don't think this works?
	    // if (!update_hpd_status(pdx, ch))
	    //    return 1;                         /* no +5 V / HPD */

	    if (!hws_update_active_interlace(pdx, i))
		// return 1;                         /* no active video */
		pdx->video[i].signal_loss_cnt = 1;
	    else {

		    if (pdx->hw_ver> 0)
			handle_hwv2_path(pdx, i);
		    else
			// FIXME: legacy struct names in subfunction
			handle_legacy_path(pdx, i);

		    update_live_resolution(pdx, i);
			pdx->video[i].signal_loss_cnt = 0;
	    }


		/* If we just detected a loss on an active capture channel… */
		if ((pdx->video[i].signal_loss_cnt  == 0x1) &&
		    (pdx->video[i].cap_active == true)) {
		      /* …schedule the “no‐video” handler on the vido_wq workqueue */
			// FIXME: this is where we can catch if the system has blanked on us
			// probably need to rewire this, because we removed the videowork
			// https://chatgpt.com/s/t_689fce8c84308191a15ec967d75165a7
			// queue_work(pdx->vido_wq, &pdx->video[i].videowork);
		}
	}
}


static inline void hws_write_if_diff(struct hws_pcie_dev *hws,
				     u32 reg_off, u32 new_val)
{
	void __iomem *addr;
	u32 old;

	if (!hws || !hws->bar0_base)
		return;

	addr = hws->bar0_base + reg_off;

	old = readl(addr);
	/* Treat all-ones as device gone; avoid writing garbage. */
	if (unlikely(old == 0xFFFFFFFF)) {
		hws->pci_lost = true;
		return;
	}

	if (old != new_val) {
		writel(new_val, addr);
		/* Post the write on some bridges / enforce ordering. */
		(void)readl(addr);
	}
}

static bool update_hpd_status(struct hws_pcie_dev *pdx, unsigned int ch)
{
    u32  hpd    = hws_read_port_hpd(pdx, ch);   /* jack-level status   */
    bool power  = !!(hpd & HWS_5V_BIT);         /* +5 V present        */
    bool hpd_hi = !!(hpd & HWS_HPD_BIT);        /* HPD line asserted   */

    /* Cache the +5 V state in V4L2 ctrl core (only when it changes) */
    if (power != pdx->video[ch].detect_tx_5v_control->cur.val)
        v4l2_ctrl_s_ctrl(pdx->video[ch].detect_tx_5v_control, power);

    /* “Signal present” means *both* rails up. Tweak if your HW differs. */
    return power && hpd_hi;
}

static bool hws_update_active_interlace(struct hws_pcie_dev *pdx, unsigned int ch)
{
	u32 reg;
	bool active, interlace;

	if (ch >= pdx->cur_max_video_ch)
		return false;

	reg = readl(pdx->bar0_base + HWS_REG_ACTIVE_STATUS);
	active    = !!(reg & BIT(ch));
	interlace = !!(reg & BIT(8 + ch));

	WRITE_ONCE(pdx->video[ch].pix.interlaced, interlace);
	return active;
}

/* Modern hardware path: keep HW registers in sync with current per-channel
 * software state. Adjust the OUT_* bits below to match your HW contract.
 */
static void handle_hwv2_path(struct hws_pcie_dev *hws, unsigned int ch)
{
	struct hws_video *vid;
	u32 reg, in_fps, cur_out_res, want_out_res;

	if (!hws || !hws->bar0_base || ch >= hws->max_channels)
		return;

	vid = &hws->video[ch];

	/* 1) Input frame rate (read-only; log or export via debugfs if wanted) */
	in_fps = readl(hws->bar0_base + HWS_REG_FRAME_RATE(ch));
	/* dev_dbg(&hws->pdev->dev, "ch%u input fps=%u\n", ch, in_fps); */

	/* 2) Output resolution programming
	 * If your HW expects a separate “scaled” size, add fields to track it.
	 * For now, mirror the current format (fmt_curr) to OUT_RES.
	 */
	want_out_res = (vid->pix.height << 16) | vid->pix.width;
	cur_out_res  = readl(hws->bar0_base + HWS_REG_OUT_RES(ch));
	if (cur_out_res != want_out_res)
		hws_write_if_diff(hws, HWS_REG_OUT_RES(ch), want_out_res);

	/* 3) Output FPS: only program if you actually track a target.
	 * Example heuristic (disabled by default):
	 *
	 *   u32 out_fps = (vid->fmt_curr.height >= 1080) ? 60 : 30;
	 *   hws_write_if_diff(hws, HWS_REG_OUT_FRAME_RATE(ch), out_fps);
	 */

	/* 4) BCHS controls: pack from per-channel current_* fields */
	reg = readl(hws->bar0_base + HWS_REG_BCHS(ch));
	{
		u8 br =  reg        & 0xFF;
		u8 co = (reg >> 8)  & 0xFF;
		u8 hu = (reg >> 16) & 0xFF;
		u8 sa = (reg >> 24) & 0xFF;

		if (br != vid->current_brightness ||
		    co != vid->current_contrast   ||
		    hu != vid->current_hue        ||
		    sa != vid->current_saturation) {
			u32 packed =  (vid->current_saturation << 24) |
				      (vid->current_hue        << 16) |
				      (vid->current_contrast   <<  8) |
				       vid->current_brightness;
			hws_write_if_diff(hws, HWS_REG_BCHS(ch), packed);
		}
	}

	/* 5) HDCP detect: read only (no cache field in your structs today) */
	reg = readl(hws->bar0_base + HWS_REG_HDCP_STATUS);
	/* bool hdcp = !!(reg & BIT(ch)); // use if you later add a field/control */
}

static void handle_legacy_path(struct hws_pcie_dev *hws, unsigned int ch)
{
	/* No-op by default. If you introduce a SW FPS accumulator, map it here.
	 *
	 * Example skeleton:
	 *
	 *   u32 sw_rate = READ_ONCE(hws->sw_fps[ch]); // incremented elsewhere
	 *   if (sw_rate > THRESHOLD) {
	 *       u32 fps = pick_fps_from_rate(sw_rate);
	 *       hws_write_if_diff(hws, HWS_REG_OUT_FRAME_RATE(ch), fps);
	 *       WRITE_ONCE(hws->sw_fps[ch], 0);
	 *   }
	 */
	(void)hws;
	(void)ch;
}

static void hws_video_apply_mode_change(struct hws_pcie_dev *pdx, unsigned int ch,
					u16 w, u16 h, bool interlaced)
{
	struct hws_video *v = &pdx->video[ch];
	unsigned long flags;
	u32 new_size;
	bool reenable = false;

	if (!pdx || !pdx->bar0_base)
		return;
	if (ch >= pdx->max_channels)
		return;
	if (!w || !h || w > MAX_VIDEO_HW_W ||
	    (!interlaced && h > MAX_VIDEO_HW_H) ||
	    ( interlaced && (h * 2) > MAX_VIDEO_HW_H))
		return;

	WRITE_ONCE(v->stop_requested, true);
	WRITE_ONCE(v->cap_active, false);
	smp_wmb();

	hws_enable_video_capture(pdx, ch, false);
	readl(pdx->bar0_base + HWS_REG_INT_STATUS);

	tasklet_kill(&v->bh_tasklet);

	spin_lock_irqsave(&v->irq_lock, flags);
	if (v->active) {
		vb2_buffer_done(&v->active->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		v->active = NULL;
	}
	while (!list_empty(&v->capture_queue)) {
		struct hwsvideo_buffer *b =
			list_first_entry(&v->capture_queue, struct hwsvideo_buffer, list);
		list_del_init(&b->list);
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&v->irq_lock, flags);

	/* Update software pixel state */
	v->pix.width      = w;
	v->pix.height     = h;
	v->pix.interlaced = interlaced;

	new_size = hws_calc_sizeimage(v, w, h, interlaced);

	/* If buffers are smaller than new requirement, signal src-change & error the queue */
	if (vb2_is_busy(&v->buffer_queue) && new_size > v->alloc_sizeimage) {
		struct v4l2_event ev = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
		};
		ev.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION;

		v4l2_event_queue(v->video_device, &ev);
		vb2_queue_error(&v->buffer_queue);
		return;
	}

	/* Program HW with new resolution */
	hws_write_if_diff(pdx, HWS_REG_OUT_RES(ch), (h << 16) | w);

	/* Legacy half-buffer programming */
	writel(v->pix.half_size / 16,
	       pdx->bar0_base + CVBS_IN_BUF_BASE2 + ch * PCIE_BARADDROFSIZE);
	(void)readl(pdx->bar0_base + CVBS_IN_BUF_BASE2 + ch * PCIE_BARADDROFSIZE);

	/* Reset per-channel toggles/counters */
	WRITE_ONCE(v->last_buf_half_toggle, 0);
	v->sequence_number = 0;

	/* Re-prime first VB2 buffer if present */
	spin_lock_irqsave(&v->irq_lock, flags);
	if (!list_empty(&v->capture_queue)) {
		v->active = list_first_entry(&v->capture_queue, struct hwsvideo_buffer, list);
		list_del_init(&v->active->list);
		hws_program_video_from_vb2(pdx, ch, &v->active->vb.vb2_buf);
		reenable = true;
	}
	spin_unlock_irqrestore(&v->irq_lock, flags);

	if (!reenable)
		return;

	WRITE_ONCE(v->stop_requested, false);
	WRITE_ONCE(v->cap_active, true);
	smp_wmb();
	hws_enable_video_capture(pdx, ch, true);
	readl(pdx->bar0_base + HWS_REG_INT_STATUS);
}

static void update_live_resolution(struct hws_pcie_dev *pdx, unsigned int ch)
{
	u32 reg   = readl(pdx->bar0_base + HWS_REG_IN_RES(ch));
	u16 res_w =  reg        & 0xFFFF;
	u16 res_h = (reg >> 16) & 0xFFFF;
	bool interlace = READ_ONCE(pdx->video[ch].pix.interlaced);

	bool within_hw =
		(res_w <= MAX_VIDEO_HW_W) &&
		((!interlace &&  res_h      <= MAX_VIDEO_HW_H) ||
		 ( interlace && (res_h * 2) <= MAX_VIDEO_HW_H));

	if (!within_hw)
		return;

	if (res_w != pdx->video[ch].pix.width ||
	    res_h != pdx->video[ch].pix.height) {
		hws_video_apply_mode_change(pdx, ch, res_w, res_h, interlace);
	}
}

static int hws_open(struct file *file)
{
    int ret;
    struct hws_video *vid;

    /* Create V4L2 file handle so events & priorities work */
    ret = v4l2_fh_open(file);
    if (ret)
        return ret;

    vid = video_drvdata(file);

    /* Hard-fail additional opens while a capture is active */
    if (!v4l2_fh_is_singular_file(file) && vb2_is_busy(&vid->buffer_queue))
    {
        v4l2_fh_release(file);
        return -EBUSY;
    }

    return 0;
}

static int hws_release(struct file *file)
{
    return vb2_fop_release(file);
}

static const struct v4l2_file_operations hws_fops = {
	.owner = THIS_MODULE,
	.open = hws_open,
	.release = hws_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static int hws_subscribe_event(struct v4l2_fh *fh,
			       const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops hws_ioctl_fops = {
	/* Core caps/info */
	.vidioc_querycap = hws_vidioc_querycap,

	/* Pixel format: still needed to report YUYV etc. */
	.vidioc_enum_fmt_vid_cap = hws_vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = hws_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = hws_vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = hws_vidioc_try_fmt_vid_cap,

	/* Buffer queueing / streaming */
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	/* Inputs */
	.vidioc_enum_input        = hws_vidioc_enum_input,
	.vidioc_g_input           = hws_vidioc_g_input,
	.vidioc_s_input           = hws_vidioc_s_input,

	/* DV timings (HDMI/DVI/VESA modes) */
	.vidioc_query_dv_timings  = hws_vidioc_query_dv_timings,
	.vidioc_enum_dv_timings   = hws_vidioc_enum_dv_timings,
	.vidioc_g_dv_timings      = hws_vidioc_g_dv_timings,
	.vidioc_s_dv_timings      = hws_vidioc_s_dv_timings,
	.vidioc_dv_timings_cap    = hws_vidioc_dv_timings_cap,

	.vidioc_log_status = vidioc_log_status,
	.vidioc_subscribe_event = hws_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_g_parm = hws_vidioc_g_parm,
	.vidioc_s_parm = hws_vidioc_s_parm,
};


static u32 hws_calc_sizeimage(struct hws_video *v, u16 w, u16 h, bool interlaced)
{
	/* example for packed 16bpp (YUYV); replace with your real math/align */
	u32 lines        = h; /* full frame lines for sizeimage */
	u32 bytesperline = ALIGN(w * 2, 64);

	/* publish into pix, since we now carry these in-state */
	v->pix.bytesperline = bytesperline;
	v->pix.sizeimage    = bytesperline * lines;
	v->pix.half_size    = v->pix.sizeimage / 2; /* if HW uses halves */
	v->pix.field        = interlaced ? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;

	return v->pix.sizeimage;
}


static int hws_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct hws_video *vid = q->drv_priv;
	struct hws_pcie_dev *hws = vid->parent;
	size_t size, tmp;

	/* One plane: YUYV (2 bytes/pixel). Use current programmed OUT WxH. */
	if (check_mul_overflow(vid->pix.width, vid->pix.height, &tmp) ||
	    check_mul_overflow(tmp, 2, &size))
		return -EOVERFLOW;

	size = PAGE_ALIGN(size);
	/* One plane: use current sizeimage (bytesperline * height) */
	size = vid->pix.sizeimage;
	if (!size) {
		vid->pix.bytesperline = ALIGN(vid->pix.width * 2, 64);
		vid->pix.sizeimage    = vid->pix.bytesperline * vid->pix.height;
		size = vid->pix.sizeimage;
	}

	if (*num_planes) {
		if (sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*num_planes   = 1;
	sizes[0]      = size;
	vid->alloc_sizeimage = size;

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
	size_t need = vid->pix.sizeimage;
	if (vb2_plane_size(vb, 0) < vid->alloc_sizeimage)
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
	list_add_tail(&buf->list, &vid->capture_queue);

	/* If streaming and no in-flight buffer, prime HW immediately */
	if (vid->cap_active && !vid->active) {
		vid->active = buf;
		hws_program_video_from_vb2(vid->parent, vid->channel_index, &buf->vb.vb2_buf);
		/* Arm engine if not already */
		hws_enable_video_capture(vid->parent, vid->channel_index, true);
	}
	spin_unlock_irqrestore(&vid->irq_lock, flags);
}

static int hws_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hws_video *v = q->drv_priv;
	struct hws_pcie_dev *hws = v->parent;
	unsigned long flags;
	int en, ret;

	/* 1) Card/device sanity */
	ret = hws_check_card_status(hws);
	if (ret)
		return ret;

	/* 2) Must have at least one queued buffer to start */
	spin_lock_irqsave(&v->irq_lock, flags);
	if (list_empty(&v->capture_queue)) {
		spin_unlock_irqrestore(&v->irq_lock, flags);
		return -ENOSPC;
	}

	/* (Re)init per-stream state visible to IRQ/BH */
	WRITE_ONCE(v->stop_requested, false);
	WRITE_ONCE(v->last_buf_half_toggle, 0);
	v->half_seen = false;

	/* 3) Prime first buffer if nothing in-flight */
	if (!v->active) {
		struct hwsvideo_buffer *first;
		first = list_first_entry(&v->capture_queue,
					 struct hwsvideo_buffer, list);
		list_del(&first->list);
		v->active = first;

		/* Make sure any CPU writes (if any) are visible before doorbell */
		wmb();
		hws_program_video_from_vb2(hws, v->channel_index,
					   &first->vb.vb2_buf);
	}
	spin_unlock_irqrestore(&v->irq_lock, flags);

	/* 4) Ensure HW capture is enabled for this channel.
	 *    Do this after programming the buffer; flush posted writes first.
	 */
	(void)readl(hws->bar0_base + HWS_REG_INT_STATUS); /* flush PCI writes */

	en = check_video_capture(hws, v->channel_index);
	if (en < 0)
		return en;
	if (en == 0)
		hws_enable_video_capture(hws, v->channel_index, true);

	/* 5) Publish running state */
	mutex_lock(&v->state_lock);
	WRITE_ONCE(v->cap_active, true);
	mutex_unlock(&v->state_lock);

	return 0;
}

static void hws_stop_streaming(struct vb2_queue *q)
{
	struct hws_video *v = q->drv_priv;
	unsigned long flags;

	/* 1) Stop HW first to quiesce further DMA/IRQs for this channel */
	hws_enable_video_capture(v->parent, v->channel_index, false);

	/* 2) Flip software state */
	mutex_lock(&v->state_lock);
	WRITE_ONCE(v->cap_active, false);
	WRITE_ONCE(v->stop_requested, true);
	mutex_unlock(&v->state_lock);

	/* 3) Return in-flight + queued buffers as ERROR */
	spin_lock_irqsave(&v->irq_lock, flags);

	if (v->active) {
		vb2_buffer_done(&v->active->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		v->active = NULL;
	}

	while (!list_empty(&v->capture_queue)) {
		struct hwsvideo_buffer *b =
			list_first_entry(&v->capture_queue,
					 struct hwsvideo_buffer, list);
		list_del(&b->list);
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&v->irq_lock, flags);
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

	ret = v4l2_device_register(&dev->pdev->dev, &dev->v4l2_device);
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

		vdev = video_device_alloc();
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
		if (ch->control_handler.error)
			goto err_unwind;
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
        if (ch->video_device) {
            /* If not registered, we must free the alloc’d vdev ourselves */
            if (!video_is_registered(ch->video_device))
                video_device_release(ch->video_device);
            ch->video_device = NULL;
        }
    }
    v4l2_device_unregister(&dev->v4l2_device);
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

		if (ch->active) {
			vb2_buffer_done(&ch->active->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			ch->active= NULL;
		}
		while (!list_empty(&ch->capture_queue)) {
			struct hwsvideo_buffer *b =
				list_first_entry(&ch->capture_queue,
						 struct hwsvideo_buffer, list);
			list_del(&b->list);
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
		    else
			video_device_release(ch->video_device);
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
	v4l2_device_unregister(&dev->v4l2_device);
}

