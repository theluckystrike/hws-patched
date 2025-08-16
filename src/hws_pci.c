/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/iopoll.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>


#include <media/v4l2-ctrls.h>

#include "hws.h"
#include "hws_audio.h"
#include "hws_reg.h"
#include "hws_video.h"
#include "hws_v4l2_ioctl.h"

#define DRV_NAME "hws"
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
static void hws_audio_cleanup_channel(struct hws_pcie_dev *pdev, int ch);
static void hws_video_cleanup_channel(struct hws_pcie_dev *pdev, int ch);
static void hws_remove(struct pci_dev *pdev);

static void hws_free_irq_vectors_action(void *data)
{
	pci_free_irq_vectors((struct pci_dev *)data);
}

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

static void hws_stop_kthread_action(void *data)
{
	struct task_struct *t = data;
	if (!IS_ERR_OR_NULL(t))
		kthread_stop(t);
}


static int hws_probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id)
{
	struct hws_pcie_dev *hws;
	void __iomem *bar0;
	int i, ret, nvec, irq;

	/* devres-backed device object */
	hws = devm_kzalloc(&pdev->dev, sizeof(*hws), GFP_KERNEL);
	if (!hws)
		return -ENOMEM;

	hws->pdev = pdev;
	pci_set_drvdata(pdev, hws);

	/* 1) Managed enable + bus mastering */
	ret = pcim_enable_device(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "pcim_enable_device\n");

	pci_set_master(pdev);

	/* 2) Map BAR0 with PCIM (auto request_regions + iounmap on detach) */
	bar0 = pcim_iomap_region(pdev, 0, KBUILD_MODNAME);
	if (IS_ERR(bar0))
		return dev_err_probe(&pdev->dev, PTR_ERR(bar0),
				     "pcim_iomap_region(BAR0)\n");
	hws->bar0_base = bar0;

	/* 3) DMA mask (try 64-bit, fall back to 32-bit) */
	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "No suitable DMA mask\n");
	}

	/* 4) Relaxed Ordering, ReadRQ, etc. if you need them */
	enable_pcie_relaxed_ordering(pdev);
#ifdef CONFIG_ARCH_TI816X
	pcie_set_readrq(pdev, 128);
#endif

	/* 5) Identify chip & set capabilities */
	read_chip_id(hws_dev);

	hws_dev->device_id = hws_dev->pdev->device;
	hws_dev->vendor_id = hws_dev->pdev->vendor;
	dev_info(&pci_dev->dev, "Device VID=0x%04x, DID=0x%04x\n",
		 pci_dev->vendor, pci_dev->device);

	/* 6) Init channels (explicit unwind on failure is fine here) */
	for (i = 0; i < hws->max_channels; i++) {
		ret = hws_video_init_channel(hws, i);
		if (ret)
			goto err_unwind_channels;
		ret = hws_audio_init_channel(hws, i);
		if (ret)
			goto err_unwind_channels;
	}

	/* 8) Allocate IRQ vector(s) the modern way; free via devm action */
	nvec = pci_alloc_irq_vectors(pdev, 1, 1,
		PCI_IRQ_ALL_TYPES | PCI_IRQ_AFFINITY);
	if (nvec < 0) {
		ret = nvec;
		dev_err(&pdev->dev, "pci_alloc_irq_vectors: %d\n", ret);
		goto err_unwind_channels;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       hws_free_irq_vectors_action, pdev);
	if (ret) {
		dev_err(&pdev->dev, "devm_add_action: free_irq_vectors: %d\n", ret);
		goto err_unwind_channels; /* add_action already called reset */
	}

	irq = pci_irq_vector(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, irqhandler, 0,
			       dev_name(&pdev->dev), hws);
	if (ret) {
		dev_err(&pdev->dev, "request_irq(%d): %d\n", irq, ret);
		goto err_unwind_channels;
	}

	/* 10) Register V4L2/ALSA */
	ret = hws_video_register(hws);
	if (ret) {
		dev_err(&pdev->dev, "video_register: %d\n", ret);
		goto err_unwind_channels;
	}
	ret = hws_audio_register(hws);
	if (ret) {
		dev_err(&pdev->dev, "audio_register: %d\n", ret);
		hws_video_unregister(hws);
		goto err_unwind_channels;
	}

	/* 11) Background monitor thread (managed stop via devm action) */
	hws->main_task = kthread_run(main_ks_thread_handle, hws, "hws-mon");
	if (IS_ERR(hws->main_task)) {
		ret = PTR_ERR(hws->main_task);
		hws->main_task = NULL;
		dev_err(&pdev->dev, "kthread_run: %d\n", ret);
		goto err_unregister_va;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       hws_stop_kthread_action, hws->main_task);
	if (ret) {
		dev_err(&pdev->dev, "devm_add_action: kthread_stop: %d\n", ret);
		goto err_unregister_va; /* reset already stopped the thread */
	}

	return 0;

err_unregister_va:
	hws_stop_device(hws);
	hws_audio_unregister(hws);
	hws_video_unregister(hws);
err_unwind_channels:
	/* explicit per-channel teardown for any initted channels */
	while (--i >= 0) {
		hws_video_cleanup_channel(hws, i);
		hws_audio_cleanup_channel(hws, i);
	}
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
    struct hws_pcie_dev *hws = pci_get_drvdata(pdev);
    int i;

    if (!hws)
        return;

    /* Stop hardware / capture cleanly (your helper) */
    hws_stop_device(hws);

    /* Unregister subsystems you registered */
    hws_audio_unregister(hws);
    hws_video_unregister(hws);

    /* Per-channel teardown */
    for (i = 0; i < hws->max_channels; i++) {
        hws_video_cleanup_channel(hws, i);
        hws_audio_cleanup_channel(hws, i);
    }
    /* kthread is stopped by the devm action you added in probe */
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


static struct pci_driver hws_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hws_pci_table,
	.probe = hws_probe,
	.remove = hws_remove,
};


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
