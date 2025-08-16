/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/iopoll.h>

#include <media/v4l2-ctrls.h>

#include "hws.h"
#include "hws_audio.h"
#include "hws_reg.h"
#include "hws_video.h"
#include "hws_v4l2_ioctl.h"

#define DRV_NAME "HWS driver"
#define HWS_REG_DEVICE_INFO   0x0000
#define HWS_REG_DEC_MODE      0x0004
#define HWS_BUSY_POLL_DELAY_US   10
#define HWS_BUSY_POLL_TIMEOUT_US 1000000


/* register layout inside HWS_REG_DEVICE_INFO */
#define DEVINFO_VER          GENMASK( 7,  0)
#define DEVINFO_SUBVER       GENMASK(15,  8)
#define DEVINFO_YV12         GENMASK(31, 28)
#define DEVINFO_HWKEY        GENMASK(27, 24)
#define DEVINFO_PORTID       GENMASK(25, 24)   /* low 2 bits of HW-key */


#define MAKE_ENTRY( __vend, __chip, __subven, __subdev, __configptr) {	\
	.vendor		= (__vend),					\
	.device		= (__chip),					\
	.subvendor	= (__subven),					\
	.subdevice	= (__subdev),					\
	.driver_data	= (unsigned long) (__configptr)			\
}

#define CH_SHIFT    2                    /* need 2 bits for 0-3            */
#define CH_MASK     GENMASK(CH_SHIFT-1, 0)

static inline unsigned long pack_dev_ch(struct hws_pcie_dev *dev, u32 ch)
{
        return (unsigned long)dev | ch;          /* dev is pointer-aligned   */
}

static const struct pci_device_id hws_pci_table[] = {
	MAKE_ENTRY(0x8888, 0x9534, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8534, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8554, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8524, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x6524, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x6504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8532, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8512, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8501, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x6502, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8524, 0x8888, 0x0007, NULL),

	{ }
};

static void enable_pcie_relaxed_ordering(struct pci_dev *dev)
{
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}

static void hws_configure_hardware_capabilities(struct hws_pcie_dev *hdev)
{
	u16 id = hdev->device_id;

	/* select per-chip channel counts */
	switch (id) {
	case 0x9534: case 0x6524: case 0x8524:
		hdev->cur_max_video_ch   = 4;
		hdev->cur_max_linein_ch  = 1;
		break;
	case 0x8532:
		hdev->cur_max_video_ch   = 2;
		hdev->cur_max_linein_ch  = 1;
		break;
	case 0x8512: case 0x6502:
		hdev->cur_max_video_ch   = 2;
		hdev->cur_max_linein_ch  = 0;
		break;
	case 0x8501:
		hdev->cur_max_video_ch   = 1;
		hdev->cur_max_linein_ch  = 0;
		break;
	default:
		hdev->cur_max_video_ch   = 4;
		hdev->cur_max_linein_ch  = 0;
		break;
	}

	/* universal buffer capacity */
	hdev->max_hw_video_buf_sz = MAX_MM_VIDEO_SIZE;

	/* decide hardware-version and program DMA max size if needed */
	if (hdev->device_ver > 121) {
		if (id == 0x8501 && hdev->device_ver == 122) {
			hdev->hw_ver = 0;
		} else {
			hdev->hw_ver = 1;
			/* DMA max size is scaler size / 16 */
			// FIXME: not compiling
			// writel(MAX_VIDEO_SCALER_SIZE >> 4, hdev->bar0_base + HWS_REG_DMA_MAX_SIZE);
		}
	} else {
		hdev->hw_ver = 0;
	}
}

static int read_chip_id(struct hws_pcie_dev *hdev)
{
	u32   reg;
	int   i;

	/* ── read the on-chip device-info register ─────────────────── */
	reg = readl(hdev->bar0_base + HWS_REG_DEVICE_INFO);

	hdev->device_ver      = FIELD_GET(DEVINFO_VER,   reg);
	hdev->sub_ver         = FIELD_GET(DEVINFO_SUBVER, reg);
	hdev->support_yv12    = FIELD_GET(DEVINFO_YV12,   reg);
	hdev->port_id         = FIELD_GET(DEVINFO_PORTID, reg);

	/* ── fill in static capabilities ───────────────────────────── */
	hdev->max_hw_video_buf_sz = MAX_MM_VIDEO_SIZE;
	hdev->max_channels        = 4;
	hdev->buf_allocated       = false;
	hdev->main_task           = NULL;
	hdev->audio_pkt_size      = MAX_DMA_AUDIO_PK_SIZE;
	hdev->start_run           = false;
	hdev->pci_lost            = 0;

	/* ── per-channel defaults ──────────────────────────────────── */
	for (i = 0; i < hdev->max_channels; i++)
		set_video_format_size(hdev, i, 1920, 1080);

	/* ── reset decoder core ───────────────────────────────────── */
	writel(0x00, hdev->bar0_base + HWS_REG_DEC_MODE);
	writel(0x10, hdev->bar0_base + HWS_REG_DEC_MODE);

	hws_configure_hardware_capabilities(hdev);

	dev_info(&hdev->pdev->dev,
		 "chip detected: ver=%u subver=%u port=%u yv12=%u\n",
		 hdev->device_ver, hdev->sub_ver,
		 hdev->port_id, hdev->support_yv12);

	return 0;
}

static int hws_video_init_channel(struct hws_pcie_dev *dev, int idx);
static int hws_audio_init_channel(struct hws_pcie_dev *dev, int idx);
static int hws_irq_setup(struct hws_pcie_dev *hws);
static struct hws_pcie_dev *hws_alloc_dev_instance(struct pci_dev *pdev);
static void hws_audio_cleanup_channel(struct hws_pcie_dev *pdev, int ch);
static int probe_scan_for_msi(struct hws_pcie_dev *hws, struct pci_dev *pdev);
static void hws_disable_msi(struct hws_pcie_dev *hws_dev);
static void hws_video_cleanup_channel(struct hws_pcie_dev *pdev, int ch);
static void hws_remove(struct pci_dev *pdev);

#ifndef arch_msi_check_device
int arch_msi_check_device(struct pci_dev *dev, int nvec, int type);
#endif

static int main_ks_thread_handle(void *data)
{
    struct hws_pcie_dev *pdx = data;
    int i;
    bool need_check;

    while (!kthread_should_stop()) {
        need_check = false;

        /* See if any channel is running */
        for (i = 0; i < pdx->max_channels; i++) {
            if (pdx->video[i].cap_active) {
                need_check = true;
                break;
            }
        }

        if (need_check)
            // FIXME: figure out if we can check update_hpd_status early and exit fast
            check_video_format(pdx);

        /* Sleep 1s or until signaled to wake/stop */
        schedule_timeout_interruptible(msecs_to_jiffies(1000));
    }

    pr_debug("%s: exiting\n", __func__);
    return 0;
}


/*
 * alloc_dev → 
 * request_regions → 
 * ioremap → 
 * MSI scan / IRQ setup → 
 * chip-id + channel init → 
 * helper kthread → 
 * V4L2 / ALSA registration
 */
static int hws_probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id)
{
	struct hws_pcie_dev *hws_dev;
	int ret = -ENODEV;
	int j, i;

	hws_dev = hws_alloc_dev_instance(pci_dev);

	if (!hws_dev) {
		dev_err(&pci_dev->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}
	hws_dev->bar0_base = NULL;

	hws_dev->pdev = pci_dev;

	hws_dev->device_id = hws_dev->pdev->device;
	hws_dev->vendor_id = hws_dev->pdev->vendor;
	dev_info(&pci_dev->dev, "Device VID=0x%04x, DID=0x%04x\n",
		 pci_dev->vendor, pci_dev->device);

	// chatgpt recommends pci_enable_device_mem?
	ret = pci_enable_device(pci_dev);
	if (ret) {
		dev_err(&pci_dev->dev, "%s: pci_enable_device failed: %d\n",
			__func__, ret);
		goto err_free_dev;
	}

    ret = pci_request_regions(pci_dev, DRV_NAME);
    if (ret) {
        dev_err(&pci_dev->dev, "pci_request_regions failed: %d\n", ret);
        goto err_disable_device;
    }

    pci_set_drvdata(pci_dev, hws_dev);

    /* map BAR0 via the PCI core: */
    hws_dev->bar0_base = pci_iomap(pci_dev, 0,
                                pci_resource_len(pci_dev, 0));
    if (!hws_dev->bar0_base) {
        dev_err(&pci_dev->dev, "pci_iomap failed\n");
        ret = -ENOMEM;
        goto err_release_regions;
    }


	enable_pcie_relaxed_ordering(pci_dev);
	pci_set_master(pci_dev);

#ifdef CONFIG_ARCH_TI816X
	pcie_set_readrq(pci_dev, 128);
#endif

	read_chip_id(hws_dev);
    /* Initialize each video/audio channel */
    for (i = 0; i < hws_dev->max_channels; i++) {
	// FIXME: Fields have changed
        ret = hws_video_init_channel(hws_dev, i);
        if (ret)
            goto err_cleanup_channels;

	// FIXME: Fields have changed
        ret = hws_audio_init_channel(hws_dev, i);
        if (ret)
            goto err_cleanup_channels;
    }

	// FIXME: making changes in DMA register setting
	hws_init_video_sys(hws_dev, 0);

    // NOTE: there are two loops, need to check if the interrupt loop can see if the capture is running and signal 
    // is lost. This was what it was: `video_data_process` where there were periodic checks
    // that we can see no video. `main_ks_thread_handle` calls `check_video_format` calls get_video_status`
   
    // FIXME: figure out if we can check update_hpd_status early and exit fast
    hws_dev->main_task = kthread_run(main_ks_thread_handle, (void *)hws_dev, "start_ks_thread_task");
    if (IS_ERR(hws_dev->main_task)) {
            ret = PTR_ERR(hws_dev->main_task);
            hws_dev->main_task = NULL;
            goto err_cleanup_channels;
    }

	ret = probe_scan_for_msi(hws_dev, pci_dev);

	if (ret < 0) {
        dev_err(&pci_dev->dev, "%s: MSI setup failed: %d\n",
                __func__, ret);
		goto err_free_irq;
	}

	// FIXME: Clear/ack any pending bits in the device before requesting irq and enabling MSI
	ret = hws_irq_setup(hws_dev);
	if (ret) {
        dev_err(&pci_dev->dev, "%s: IRQ setup failed: %d\n",
                __func__, ret);
		goto err_free_irq;
	}


	// FIXME: figure out if this hardware only supports 4 GB RAM
	if (hws_video_register(hws_dev))
		goto err_free_irq;

    // FIXME: `audio_data_process` which gets set/called from this func sucks
    // this is where the audio devices get created, if we need to set the DMA address for the 
    // it would be a good point to check here
	if (hws_audio_register(hws_dev))
		goto err_unregister_video;
	return 0;

err_unregister_video:
	hws_video_unregister(hws_dev);
err_free_irq:
    hws_disable_msi(hws_dev);
    /* devm_free_irq() is implicit */
err_stop_thread:
        if (!IS_ERR_OR_NULL(hws_dev->main_task))
                kthread_stop(hws_dev->main_task);
err_free_dma:
err_cleanup_channels:
    /* undo channels [0 .. i-1] */
    for (j = i - 1; j >= 0; j--) {
        hws_video_cleanup_channel(hws_dev, j);
	hws_audio_cleanup_channel(hws_dev, j);
    }
err_unmap_bar:
        if (hws_dev->bar0_base) {          /* mapped → unmap & NULL it     */
                pci_iounmap(pci_dev, hws_dev->bar0_base);
                hws_dev->bar0_base = NULL;
        }
err_release_regions:
        pci_release_regions(pci_dev);
err_disable_device:
        pci_disable_device(pci_dev);
err_free_dev:
        return ret;
}

static int hws_check_busy(struct hws_pcie_dev *pdx)
{
    void __iomem *reg = pdx->bar0_base + HWS_REG_SYS_STATUS;
    u32 val;
    int ret;

    /* poll until !(val & BUSY_BIT), sleeping HWS_BUSY_POLL_DELAY_US between reads */
    ret = readl_poll_timeout(reg, val,
                             !(val & HWS_SYS_DMA_BUSY_BIT),
                             HWS_BUSY_POLL_DELAY_US,
                             HWS_BUSY_POLL_TIMEOUT_US);
    if (ret) {
        dev_err(&pdx->pdev->dev,
                "SYS_STATUS busy bit never cleared (0x%08x)\n", val);
        return -ETIMEDOUT;
    }

    return 0;
}

static void hws_stop_dsp(struct hws_pcie_dev *hws)
{
    u32 status;

    /* Read the decoder mode/status register */
    status = readl(hws->bar0_base + HWS_REG_DEC_MODE);
    dev_dbg(&hws->pdev->dev, "hws_stop_dsp: status=0x%08x\n", status);

    /* If the device looks unplugged/stuck, bail out */
    if (status == 0xFFFFFFFF)
        return;

    /* Tell the DSP to stop */
    writel(0x10, hws->bar0_base + HWS_REG_DEC_MODE);

    /* FIXME: hws_check_busy() should return an error if it times out */
    hws_check_busy(hws);

    /* Disable video capture engine in the DSP */
    writel(0x0, hws->bar0_base + HWS_REG_VCAP_ENABLE);
}

static void hws_stop_device(struct hws_pcie_dev *hws)
{
    unsigned int i;
    u32 status;

    /* 1) Stop the on-board DSP */
    hws_stop_dsp(hws);

    /* 2) Check for a lost PCI device */
    status = readl(hws->bar0_base + HWS_REG_SYS_STATUS);
    dev_dbg(&hws->pdev->dev, "hws_stop_device: status=0x%08x\n", status);
    if (status == 0xFFFFFFFF) {
        hws->pci_lost = true;
    } else {
        /* 3) Tear down each video/audio channel */
        for (i = 0; i < hws->max_channels; ++i) {
            hws_enable_video_capture(hws, i, false);
            hws_enable_audio_capture(hws, i, false);
        }
    }

    /* 4) Mark the device as no longer running */
    hws->start_run = false;

    dev_dbg(&hws->pdev->dev, "hws_stop_device: complete\n");
}

static void hws_remove(struct pci_dev *pdev)
{
    struct hws_pcie_dev *hdev = pci_get_drvdata(pdev);
    struct video_device *vdev;
    int i;
    if (!hdev || !hdev->bar0_base)
        return;

	if (hdev->main_task) {
		kthread_stop(hdev->main_task);
	}

	hws_stop_device(hdev);
	/* disable interrupts */
	// hws_free_irqs(hdev);

	for (i = 0; i < hdev->cur_max_video_ch; i++) {
		if (hdev->audio[i].sound_card) {
			snd_card_free(hdev->audio[i].sound_card);
			hdev->audio[i].sound_card = NULL;
		}
	}

	for (i = 0; i < hdev->cur_max_video_ch; i++) {
		vdev = hdev->video[i].video_device;
		video_unregister_device(vdev);
		// FIXME: need to understand if we're cleaning this up correctly
		// v4l2_device_unregister(&hdev->video[i].video_device);
		v4l2_ctrl_handler_free(&hdev->video[i].control_handler);
	}

	if (hdev->msix_enabled) {
		pci_disable_msix(pdev);
		hdev->msix_enabled = 0;
	} else if (hdev->msi_enabled) {
		pci_disable_msi(pdev);
		hdev->msi_enabled = 0;
	}
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

/* ─────────────────────────────────────────────────────────── */
/* Per-video-channel initialisation                            */
/* ------------------------------------------------------------------ */
/*  Initialise one video channel                                      */
/* ------------------------------------------------------------------ */
static int hws_video_init_channel(struct hws_pcie_dev *pdev, int ch)
{
	struct hws_video *vid;
	struct v4l2_ctrl_handler *hdl;

	/* basic sanity */
	if (!pdev || ch < 0 || ch >= pdev->max_channels)
		return -EINVAL;

	vid = &pdev->video[ch];

	/* hard reset the per-channel struct */
	memset(vid, 0, sizeof(*vid));

	/* identity */
	vid->parent        = pdev;
	vid->channel_index = ch;

	/* locks & lists */
	mutex_init(&vid->state_lock);
	spin_lock_init(&vid->irq_lock);
	INIT_LIST_HEAD(&vid->capture_queue);
	vid->cur = NULL;

	/* default format (safe 1080p; adjust to your real HW default) */
	vid->fmt_curr.width  = 1920;
	vid->fmt_curr.height = 1080;

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

	/* V4L2 controls:
	 * Expose +5V and HPD as volatile, read-only detection bits.
	 * (Skip/adjust if your HW doesn’t support these or uses different CIDs.)
	 */
	hdl = &vid->control_handler;
	v4l2_ctrl_handler_init(hdl, 2);

	vid->detect_tx_5v_control = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
		V4L2_CID_DV_RX_POWER_PRESENT, 0, 1, 1, 0);
	if (vid->detect_tx_5v_control)
		vid->detect_tx_5v_control->flags |=
			V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;

	vid->hotplug_detect_control = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
		V4L2_CID_DV_RX_HOTPLUG, 0, 1, 1, 0);
	if (vid->hotplug_detect_control)
		vid->hotplug_detect_control->flags |=
			V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		int err = hdl->error;
		dev_err(&pdev->pdev->dev,
			"v4l2 ctrl init failed on ch%d: %d\n", ch, err);
		v4l2_ctrl_handler_free(hdl);
		return err;
	}

	return 0;
}

static void hws_video_drain_queue_locked(struct hws_video *vid)
{
	while (!list_empty(&vid->capture_queue)) {
		struct hwsvideo_buffer *b =
			list_first_entry(&vid->capture_queue,
					 struct hwsvideo_buffer, list);
		list_del_init(&b->list);

		/* Assuming your buffer wraps a vb2 buffer like:
		 *   struct hwsvideo_buffer { struct vb2_v4l2_buffer vb; ... }
		 * Adjust if your field is different.
		 */
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	vid->cur = NULL;
}

void hws_video_cleanup_channel(struct hws_pcie_dev *pdev, int ch)
{
	struct hws_video *vid;
	unsigned long flags;

	if (!pdev || ch < 0 || ch >= pdev->max_channels)
		return;

	vid = &pdev->video[ch];

	/* --- Stop capture best-effort (device-specific hook if you have one) --- */
	if (vid->cap_active) {
		vid->stop_requested = true;
		/* If you have a per-channel stop: hws_hw_stop_channel(pdev, ch); */
		vid->cap_active = false;
	}

	/* --- Drain SW capture queue and in-flight buffer --- */
	spin_lock_irqsave(&vid->irq_lock, flags);
	hws_video_drain_queue_locked(vid);
	spin_unlock_irqrestore(&vid->irq_lock, flags);

	/* --- Release VB2 queue if it was initialized elsewhere --- */
	/* Heuristic: ops non-NULL -> queue was set up. Safe to call once. */
	if (vid->buffer_queue.ops)
		vb2_queue_release(&vid->buffer_queue);

	/* --- Free V4L2 controls --- */
	v4l2_ctrl_handler_free(&vid->control_handler);

	/* --- Optionally unregister the video_device if this function owns it --- */
	if (vid->video_device) {
		/* Only if you registered it with video_register_device() */
		if (video_is_registered(vid->video_device))
			video_unregister_device(vid->video_device);

		/* Only if you allocated with video_device_alloc() */
		/* video_device_release(vid->video_device); */
		vid->video_device = NULL;
	}

	/* --- Reset cheap state (don’t memset the whole struct here) --- */
	mutex_destroy(&vid->state_lock);
	/* spinlocks don’t need explicit destruction */
	INIT_LIST_HEAD(&vid->capture_queue);
	vid->cur                 = NULL;
	vid->stop_requested      = false;
	vid->last_buf_half_toggle = 0;
	vid->half_seen           = false;
	vid->signal_loss_cnt     = 0;
}



/*  Initialise one audio channel                                       */
/* ------------------------------------------------------------------ */
static int hws_audio_init_channel(struct hws_pcie_dev *pdev, int ch)
{
	struct hws_audio *aud = &pdev->audio[ch];
	int               q, err;

	/* ── zero entire per-channel struct ─────────────────────────── */
	memset(aud, 0, sizeof(*aud));

	/* ── identify parent + channel index ───────────────────────── */
	aud->parent        = pdev;
	aud->channel_index = ch;

	/* ── default PCM parameters (48-kHz / stereo / 16-bit) ─────── */
	aud->output_sample_rate  = 48000;
	aud->channel_count       = 2;
	aud->bits_per_sample     = 16;

	/* ── capture-state flags ───────────────────────────────────── */
	aud->cap_active      = false;
	aud->stream_running  = false;
	aud->stop_requested  = false;

	/* HW period tracking sentinel (optional) */
	aud->last_period_toggle = 0xFF;  /* means “never toggled yet” */

	return 0;
}

static void hws_audio_cleanup_channel(struct hws_pcie_dev *pdev, int ch)
{
	struct hws_audio *aud;
	unsigned long pcm_flags;

	if (!pdev)
		return;

	aud = &pdev->audio[ch];

	/* 1) Stop the hardware stream if it's running (your stop routine). */
	if (READ_ONCE(aud->stream_running)) {
		/* Must disable channel IRQs, stop DMA, flush/ack status, etc. */
		hws_audio_hw_stop(pdev, ch);   /* implement in your HW layer */
		WRITE_ONCE(aud->stream_running, false);
	}

	/* 2) Clear runtime flags. */
	aud->cap_active     = false;
	aud->stop_requested = false;

	/* 3) If the device is going away while ALSA stream is open, notify ALSA. */
	if (device_going_away && aud->pcm_substream) {
		snd_pcm_stream_lock_irqsave(aud->pcm_substream, pcm_flags);
		snd_pcm_stop(aud->pcm_substream, SNDRV_PCM_STATE_DISCONNECTED);
		snd_pcm_stream_unlock_irqrestore(aud->pcm_substream, pcm_flags);
		aud->pcm_substream = NULL;
	}

	/* 4) Reset optional book-keeping. */
	aud->last_period_toggle = 0xFF;   /* sentinel: “never toggled” */

	/* format defaults (rate/ch/bits) can be left as-is or reset if you prefer */
}
}

static struct hws_pcie_dev *hws_alloc_dev_instance(struct pci_dev *pdev)
{
    struct hws_pcie_dev *lro;

    if (WARN_ON(!pdev))
        return ERR_PTR(-EINVAL);

    lro = devm_kzalloc(&pdev->dev, sizeof(*lro), GFP_KERNEL);
    if (!lro) {
        dev_err(&pdev->dev, "failed to alloc hws_pcie_dev\n");
        return ERR_PTR(-ENOMEM);
    }

    /* no IRQ yet */
    lro->irq_line = -1;

    dev_set_drvdata(&pdev->dev, lro);
    lro->pdev = pdev;


    return lro;
}

/* type = PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX */
int msi_msix_capable(struct pci_dev *dev, int type)
{
	struct pci_bus *bus;
	int ret;
	//printk("msi_msix_capable in \n");
	if (!dev || dev->no_msi) {
		printk("msi_msix_capable no_msi exit \n");
		return 0;
	}

	for (bus = dev->bus; bus; bus = bus->parent) {
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI) {
			printk("msi_msix_capable PCI_BUS_FLAGS_NO_MSI \n");
			return 0;
		}
	}
	ret = arch_msi_check_device(dev, 1, type);
	if (ret) {
		return 0;
	}
	ret = pci_find_capability(dev, type);
	if (!ret) {
		printk("msi_msix_capable pci_find_capability =%d\n", ret);
		return 0;
	}

	return 1;
}

static int probe_scan_for_msi(struct hws_pcie_dev *hws, struct pci_dev *pdev)
{
    int rc, nvec;

    if (WARN_ON(!hws || !pdev))
        return -EINVAL;

#ifdef CONFIG_PCI_IRQ_VECTOR
    /* 1) Try to allocate MSI-X vectors */
    if (pci_find_capability(pdev, PCI_CAP_ID_MSIX)) {
        nvec = ARRAY_SIZE(hws->msix_entries);
        rc = pci_alloc_irq_vectors(pdev, nvec, nvec, PCI_IRQ_MSIX);
        if (rc == nvec) {
            hws->msix_enabled = true;
            hws->msi_enabled  = false;
            dev_info(&pdev->dev, "MSI-X x%d enabled\n", nvec);
            return 0;
        }
        dev_err(&pdev->dev,
                "MSI-X x%d allocation failed (%d), falling back\n",
                nvec, rc);
    }

    /* 2) Try to allocate a single MSI vector */
    if (pci_find_capability(pdev, PCI_CAP_ID_MSI)) {
        rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
        if (rc == 1) {
            hws->msi_enabled  = true;
            hws->msix_enabled = false;
            dev_info(&pdev->dev, "MSI x1 enabled\n");
            return 0;
        }
        dev_err(&pdev->dev,
                "MSI x1 allocation failed (%d), falling back\n",
                rc);
    }
#endif

    /* 3) Legacy INTx */
    hws->msi_enabled  = false;
    hws->msix_enabled = false;
    dev_info(&pdev->dev, "using legacy INTx interrupts\n");
    return 0;
}

static void hws_disable_msi(struct hws_pcie_dev *hws_dev)
{
    /* only free if we actually enabled MSI‑X or MSI */
    if (hws_dev->msix_enabled || hws_dev->msi_enabled) {
        pci_free_irq_vectors(hws_dev->pdev);
        hws_dev->msix_enabled = false;
        hws_dev->msi_enabled  = false;
    }
}

static int hws_irq_setup(struct hws_pcie_dev *hws)
{
    struct pci_dev *pdev = hws->pdev;
    int irq, rc;
    unsigned long flags = 0;

    if (WARN_ON(!hws || !pdev))
        return -EINVAL;

    /*
     * If neither MSI nor MSI-X got enabled in probe(), we’re stuck on
     * legacy INTx—mark it shared and log the pin/line.
     */
    if (!hws->msi_enabled && !hws->msix_enabled) {
        u8 pin;

        pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &pin);
        dev_info(&pdev->dev,
                 "no MSI/MSI-X; using legacy INTx (pin %u, line %d)\n",
                 pin, pdev->irq);
        flags |= IRQF_SHARED;
    }

    /* Get the vector we allocated in probe_scan_for_msi() */
    irq = pci_irq_vector(pdev, 0);
    if (irq < 0) {
        dev_err(&pdev->dev,
                "pci_irq_vector() failed: %d\n", irq);
        return irq;
    }

    rc = devm_request_irq(&pdev->dev, irq, irqhandler,
                          flags, dev_name(&pdev->dev), hws);
    if (rc) {
        dev_err(&pdev->dev,
                "devm_request_irq(%d) failed: %d\n", irq, rc);
        return rc;
    }

    hws->irq_line = irq;
    dev_info(&pdev->dev,
             "IRQ %d registered (msi=%u)\n",
             irq, hws->msi_enabled);

    return 0;
}

static struct pci_driver hws_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hws_pci_table,
	.probe = hws_probe,
	.remove = hws_remove,
};

#ifndef arch_msi_check_device
int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
	return 0;
}
#endif

MODULE_DEVICE_TABLE(pci, hws_pci_table);

static int __init pcie_hws_init(void)
{
        return pci_register_driver(&hws_pci_driver);
}

static void __exit pcie_hws_exit(void)
{
        pci_unregister_driver(&hws_pci_driver);
}

module_init(pcie_hws_init);
module_exit(pcie_hws_exit);

MODULE_DESCRIPTION(DRV_NAME);
MODULE_AUTHOR("Ben Hoff <hoff.benjamin.k@gmail.com>");
MODULE_AUTHOR("Sales <sales@avmatrix.com>");
MODULE_LICENSE("GPL");
