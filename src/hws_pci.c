/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_pci.h"

#include <media/v4l2-ctrls.h>

#include "hws.h"
#include "hws_init.h"
#include "hws_dma.h"
#include "hws_video_pipeline.h"
#include "hws_audio_pipeline.h"
#include "hws_interrupt.h"
#include "hws_video.h"

#define DRV_NAME "HWS driver"
#define HWS_REG_DEVICE_INFO   0x0000
#define HWS_REG_DEC_MODE      0x0004

/* register layout inside HWS_REG_DEVICE_INFO */
#define DEVINFO_VER          GENMASK( 7,  0)
#define DEVINFO_SUBVER       GENMASK(15,  8)
#define DEVINFO_YV12         GENMASK(31, 28)
#define DEVINFO_HWKEY        GENMASK(27, 24)
#define DEVINFO_PORTID       GENMASK(25, 24)   /* low 2 bits of HW-key */

#define MAX_MM_VIDEO_SIZE     (1920 * 1080 * 2)
#define MAX_DMA_AUDIO_PK_SIZE (128 * 16 * 4)

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
			hws_write32(hdev, HWS_REG_DMA_MAX_SIZE,
				    MAX_VIDEO_SCALER_SIZE >> 4);
		}
	} else {
		hdev->hw_ver = 0;
	}
}

static int read_chip_id(struct hws_pcie_dev *pdx)
{
	u32   reg;
	int   i;

	/* ── read the on-chip device-info register ─────────────────── */
	reg = hws_read32(hdev, HWS_REG_DEVICE_INFO);

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
	hws_write32(hdev, HWS_REG_DEC_MODE, 0x00);
	hws_write32(hdev, HWS_REG_DEC_MODE, 0x10);

	hws_configure_hardware_capabilities(hdev);

	dev_info(&hdev->pdev->dev,
		 "chip detected: ver=%u subver=%u port=%u yv12=%u\n",
		 hdev->device_ver, hdev->sub_ver,
		 hdev->port_id, hdev->support_yv12);

	return 0;
}

int hws_video_init_channel(struct hws_pcie_dev *dev, int idx);
int hws_audio_init_channel(struct hws_pcie_dev *dev, int idx);
int hws_irq_setup(struct hws_pcie_dev *hws);

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

static int hws_probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id)
{
	struct hws_pcie_dev *hws_dev= NULL;
	int err = 0, ret = -ENODEV;
	int j, i;

	hws_dev = hws_alloc_dev_instance(pci_dev);
	hws_dev->pdev = pci_dev;

	hws_dev->device_id = hws_dev->pci_dev->device;
	hws_dev->vendor_id = hws_dev->pci_dev->vendor;
	dev_info(&pci_dev->dev, "Device VID=0x%04x, DID=0x%04x\n",
		 pci_dev->vendor, pci_dev->device);

	err = pci_enable_device(pci_dev);
	if (err) {
		dev_err(&pci_dev->dev, "%s: pci_enable_device failed: %d\n",
			__func__, err);
		goto err_free_dev;
	}

    err = pci_request_regions(pci_dev, DRV_NAME);
    if (err) {
        dev_err(&pci_dev->dev, "pci_request_regions failed: %d\n", err);
        goto err_disable_device;
    }

    /* map BAR0 via the PCI core: */
    hws_dev->bar0_base = pci_iomap(pci_dev, 0,
                                pci_resource_len(pci_dev, 0));
    if (!hws_dev->bar0_base) {
        dev_err(&pci_dev->dev, "pci_iomap failed\n");
        err = -ENOMEM;
        goto err_release_regions;
    }


	enable_pcie_relaxed_ordering(pci_dev);
	pci_set_master(pci_dev);

	ret = probe_scan_for_msi(hws_dev, pci_dev);

	if (ret < 0) {
        dev_err(&pci_dev->dev, "%s: MSI setup failed: %d\n",
                __func__, ret);
		goto disable_msi;
	}

#ifdef CONFIG_ARCH_TI816X
	pcie_set_readrq(pci_dev, 128);
#endif

	hws_dev->video_wq = NULL;
	hws_dev->audio_w = NULL;

	ret = hws_irq_setup(hws_dev, pci_dev);
	if (ret) {
        dev_err(&pci_dev->dev, "%s: IRQ setup failed: %d\n",
                __func__, ret);
		goto err_disable_msi;
	}

	pci_set_drvdata(pci_dev, hws_dev);
	read_chip_id(hws_dev);
    /* Initialize each video/audio channel */
    for (i = 0; i < dev->max_channels; i++) {
        ret = hws_video_init_channel(dev, i);
        if (ret)
            goto err_cleanup_channels;
        ret = hws_audio_init_channel(dev, i);
        if (ret)
            goto err_cleanup_channels;
    }

	ret = hws_dma_mem_alloc(hws_dev);
	if (ret != 0) {
		goto err_free_dma;
	}

	hws_init_video_sys(hws_dev, 0);

    // NOTE: there are two loops, the `video_data_process` and this where we have periodic checks
    // that we can see no video. `main_ks_thread_handle` calls `check_video_format` calls get_video_status`
    // whereas the `video_data_process` checks a m_curr_No_Video instance which has since been refactored
   
    // FIXME: figure out if we can check update_hpd_status early and exit fast
    hws->main_task = kthread_run(main_ks_thread_handle, (void *)hws_dev, "start_ks_thread_task")

	// NOTE: loops around `hws_get_video_param`, which sets values based on vcap status height/width
	hws_adapters_init(hws_dev);
	hws_dev->video_wq = create_singlethread_workqueue("hws");
	hws_dev->audio_wq = create_singlethread_workqueue("hws-audio");

    // FIXME: This func sucks
	if (hws_video_register(hws_dev))
        // FIXME: not sure this goto makes sense
		goto err_mem_alloc;

    // FIXME: This func sucks
	if (hws_audio_register(hws_dev))
        // FIXME: not sure this goto makes sense
		goto err_mem_alloc;
	return 0;
err_unregister_video:
        hws_video_unregister(hws);
err_destroy_wq:
        destroy_workqueue(hws->video_wq);
        destroy_workqueue(hws->audio_wq);
err_stop_thread:
        if (!IS_ERR_OR_NULL(hws_dev->main_task))
                kthread_stop(hws_dev->main_task);
	// NOTE: GOOD BELOW HERE
err_free_dma:
        hws_dma_mem_free(hws);
err_cleanup_channels:
    /* undo channels [0 .. i-1] */
    for (j = i - 1; j >= 0; j--) {
        hws_video_cleanup_channel(hws_dev, j);
	hws_audio_cleanup_channel(hws_dev, j);
    }
err_disable_msi:
    hws_disable_msi(hws_dev);
err_unmap_bar:
        pci_iounmap(pdev, hws->bar0_base);
err_release_regions:
        pci_release_regions(pdev);
err_disable_device:
        pci_disable_device(pdev);
err_free_dev:
        kfree(hws);
        return ret;
}

void hws_remove(struct pci_dev *pdev)
{
	int i;
	struct video_device *vdev;
	struct hws_pcie_dev *dev = (struct hws_pcie_dev *)pci_get_drvdata(pdev);
	//----------------------------
	if (dev->map_bar0_addr == NULL)
		return;
	//StopSys(dev);
	StopDevice(dev);
	/* disable interrupts */
	hws_free_irqs(dev);

    // FIXME 
	if (pdx->mMain_tsk) {
		kthread_stop(pdx->mMain_tsk);
	}

	//printk("hws_remove  0\n");
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		tasklet_kill(&dev->dpc_video_tasklet[i]);
		tasklet_kill(&dev->dpc_audio_tasklet[i]);
	}
	//-------------------------
	//printk("hws_remove  1\n");
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		if (dev->audio[i].resampled_buf) {
			vfree(dev->audio[i].resampled_buf);
			dev->audio[i].resampled_buf = NULL;
		}
		if (dev->audio[i].card) {
			snd_card_free(dev->audio[i].card);
			dev->audio[i].card = NULL;
		}
	}
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		vdev = &dev->video[i].vdev;
		video_unregister_device(vdev);
		v4l2_device_unregister(&dev->video[i].v4l2_dev);
		v4l2_ctrl_handler_free(&dev->video[i].ctrl_handler);
	}
	//-----------------
	if (dev->wq) {
		destroy_workqueue(dev->wq);
	}

	if (dev->auwq) {
		destroy_workqueue(dev->auwq);
	}
	dev->wq = NULL;
	dev->auwq = NULL;
	//free_irq(dev->pdev->irq, dev);

	iounmap(dev->info.mem[0].internal_addr);

	//pci_disable_device(pdev);
	if (dev->msix_enabled) {
		pci_disable_msix(pdev);
		dev->msix_enabled = 0;
	} else if (dev->msi_enabled) {
		pci_disable_msi(pdev);
		dev->msi_enabled = 0;
	}
	// FIXME: is this no longer needed because we're using device managed memory?
	kfree(dev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	//printk("hws_remove  Done\n");
}

/* ─────────────────────────────────────────────────────────── */
/* Per-video-channel initialisation                            */
/* ------------------------------------------------------------------ */
/*  Initialise one video channel                                      */
/* ------------------------------------------------------------------ */
static int hws_video_init_channel(struct hws_pcie_dev *pdev, int ch)
{
	struct hws_video         *vid = &pdev->video[ch];
	struct v4l2_ctrl_handler *hdl = &vid->control_handler;
	int                       q;

	/* ── hard-reset the whole per-channel struct ─────────────────── */
	memset(vid, 0, sizeof(*vid));

	/* ── basic identity / defaults ───────────────────────────────── */
	vid->parent              = pdev;
	vid->channel_index       = ch;
	vid->query_index         = 0;

	/* default incoming signal info */
	vid->tv_standard         = V4L2_STD_NTSC_M;
	vid->pixel_format        = V4L2_PIX_FMT_YUYV;

	/* default outgoing (scaled) geometry */
	vid->output_width        = 1920;
	vid->output_height       = 1080;
	vid->output_frame_rate   = 60;
	vid->output_pixel_format = V4L2_PIX_FMT_YUYV;
	vid->output_size_index   = 0;

	/* colour controls : mid-range baseline (0x80) */
	vid->current_brightness  =
	vid->current_contrast    =
	vid->current_saturation  =
	vid->current_hue         = 0x80;

	/* ── kernel synchronisation primitives ───────────────────────── */
	mutex_init(&vid->state_lock);
	mutex_init(&vid->capture_queue_lock);
	spin_lock_init(&vid->irq_lock);

	INIT_LIST_HEAD(&vid->capture_queue);

	/* ── DMA bookkeeping is “empty” for now ─────────────────────── */
	vid->buf_phys_addr   = 0;
	vid->buf_virt        = NULL;
	vid->buf_size_bytes  = 0;
	vid->buf_high_wmark  = 0;

	/* ── capture-queue / VCAP status defaults ───────────────────── */
	for (q = 0; q < MAX_VIDEO_QUEUE; q++) {
		/* HW status mirror */
		vid->queue_status[q].lock       = MEM_UNLOCK;
		// vid->queue_status[q].channel    = ;
		// vid->queue_status[q].size       = ;
		vid->queue_status[q].field      = 0;
		vid->queue_status[q].path       = 2;
		vid->queue_status[q].width      = 1920;
		vid->queue_status[q].height     = 1080;
		vid->queue_status[q].interlace  = 0;

		// FIXME: not in original?
		vid->queue_status[q].fps        = 60;
		vid->queue_status[q].out_width  = 1920;
		vid->queue_status[q].out_height = 1080;
		// vid->queue_status[q].hdcp       = ;
		// FIXME: not in original?

		/* software helper struct (acap_video_info) */
		vid->chan_info.status[q].lock     = MEM_UNLOCK;
		vid->chan_info.video_buf[q]       = NULL;
	}

	/* ── per-channel runtime flags / counters ───────────────────── */
	vid->cap_active       = false;
	vid->dma_busy         = 0;
	vid->stop_requested   = false;
	vid->rd_idx           = 0;
	vid->wr_idx           = 0;
	vid->half_done_cnt    = 0;
	vid->irq_event        = 0;
	vid->irq_done_flag    = false;

	vid->signal_loss_cnt  = 0;
	vid->sw_fps           = 0;
	vid->sequence_number  = 0;

	/* ── V4L2 control handler (optional but mirrors old code) ───── */
	v4l2_ctrl_handler_init(hdl, 1);

	vid->detect_tx_5v_control = v4l2_ctrl_new_std(hdl, &hws_ctrl_ops,
					V4L2_CID_DV_RX_POWER_PRESENT,
					0, 1, 1, 0);
	if (vid->detect_tx_5v_control)
		vid->detect_tx_5v_control->flags |=
			V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		dev_err(&pdev->pdev->dev,
			"V4L2 ctrl init failed on ch%d: %d\n",
			ch, hdl->error);
		return hdl->error;
	}

	/* ── per-channel locks kept in the parent dev ──────────────── */
	spin_lock_init(&pdev->videoslock[ch]);

	return 0;
}

static void hws_video_cleanup_channel(struct hws_pcie_dev *pdev, int ch)
{
    struct hws_video *vid = &pdev->video[ch];

    /* 1) Free all V4L2 controls */
    v4l2_ctrl_handler_free(&vid->control_handler);
    // FIXME: this only handles the memory created by init channel

    /*
     * 2) If you ever allocate a DMA buffer later (via
     *    dma_alloc_coherent), free it here:
     *
     * if (vid->buf_virt) {
     *     dma_free_coherent(pdev->pdev,
     *                       vid->buf_size_bytes,
     *                       vid->buf_virt,
     *                       vid->buf_phys_addr);
     *     vid->buf_virt       = NULL;
     *     vid->buf_phys_addr  = 0;
     *     vid->buf_size_bytes = 0;
     * }
     *
     * 3) Any pending work on your capture queue list should be
     *    drained or cancelled here.  For example, if you ever
     *    queue work to a WQ you might do:
     *
     *    flush_workqueue(vid->your_workqueue);
     *
     * 4) Note: kernel mutexes, spinlocks and list heads
     *    do not need explicit “destroy” calls.
     */
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

	/* ── synchronisation primitives / workers ──────────────────── */
	spin_lock_init(&aud->ring_lock);

	/* ── ring-buffer bookkeeping defaults ──────────────────────── */
	aud->ring_size_frames      = 0;
	aud->ring_write_pos_frames = 0;
	aud->period_size_frames    = 0;
	aud->period_used_frames    = 0;
	aud->ring_offset_bytes     = 0;
	aud->ring_overflow_bytes   = 0;

	/* ── DMA pointers zero-ed until buffer alloc happens ───────── */
	aud->buf_phys_addr   = 0;
	aud->buf_virt        = NULL;
	aud->data_buf        = NULL;
	aud->data_area       = NULL;
	aud->buf_size_bytes  = 0;
	aud->buf_high_wmark  = 0;

	/* ── capture-state flags ───────────────────────────────────── */
	aud->cap_active      = false;
	aud->dma_busy        = 0;
	aud->stream_running  = false;
	aud->stop_requested  = false;
	aud->wr_idx          = 0;
	aud->rd_idx          = 0;
	aud->irq_event       = 0;

	/* ── resampling workspace cleared ──────────────────────────── */
	aud->resampled_buffer      = NULL;
	aud->resampled_buffer_size = 0;

	/* ── ALSA card skeleton (optional for minimal build) ───────── */
	err = snd_card_new(&pdev->pdev->dev, -1, NULL,
			   THIS_MODULE, 0, &aud->sound_card);
	if (err)
		return err;

	/* ── per-queue software status (if you keep acap_audio_info) – */
	for (q = 0; q < MAX_AUDIO_QUEUE; q++) {
		aud->chan_info.status[q].lock = MEM_UNLOCK;
		aud->chan_info.audio_buf[q]   = NULL;
	}

	/* ── channel-specific spin-lock on parent dev ──────────────── */
	spin_lock_init(&pdev->audiolock[ch]);

	/* ── per-channel tasklet for ISR bottom-half ──────────────── */
	tasklet_init(&pdev->dpc_audio_tasklet[ch],
		     ch == 0 ? DpcForIsr_Audio0 :
		     ch == 1 ? DpcForIsr_Audio1 :
		     ch == 2 ? DpcForIsr_Audio2 :
		               DpcForIsr_Audio3,
		     (unsigned long) pdev);

	return 0;
}

static void hws_audio_cleanup_channel(struct hws_pcie_dev *pdev, int ch)
{
    struct hws_audio *aud = &pdev->audio[ch];
    int q;

    /* ── kill the tasklet (bottom-half) ───────────────────────── */
    tasklet_kill(&pdev->dpc_audio_tasklet[ch]);

    /* ── unregister & free the ALSA card ───────────────────────── */
    if (aud->sound_card) {
        snd_card_free(aud->sound_card);
        aud->sound_card = NULL;
    }

    /* ── free DMA buffer if allocated ──────────────────────────── */
    if (aud->data_buf) {
        /* if you used pci_alloc_consistent / dma_alloc_coherent: */
        pci_free_consistent(pdev->pdev,
                            aud->buf_size_bytes,
                            aud->data_buf,
                            aud->buf_phys_addr);
        aud->data_buf = NULL;
        aud->buf_phys_addr = 0;
        aud->buf_size_bytes = 0;
    }

    /* ── free resampling workspace ─────────────────────────────── */
    kfree(aud->resampled_buffer);
    aud->resampled_buffer = NULL;
    aud->resampled_buffer_size = 0;

    /* ── clear per-queue info ──────────────────────────────────── */
    for (q = 0; q < MAX_AUDIO_QUEUE; q++) {
        aud->chan_info.audio_buf[q] = NULL;
        aud->chan_info.status[q].lock = MEM_UNLOCK;
    }

    /* ── (Optional) zero the struct so repeated init/cleanup safe ─ */
    memset(aud, 0, sizeof(*aud));
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
//--------------------------------------

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

    /* Managed IRQ — we don’t need an explicit free_irq() on error/remove */
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


static inline void hws_write32(struct hws_pcie_dev *hdev, u32 off, u32 val)
{
	writel(val, hdev->bar0_base + off);
}

static inline u32 hws_read32(struct hws_pcie_dev *hdev, u32 off)
{
	return readl(hdev->bar0_base + off);
}

static struct pci_driver hws_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hws_pci_table,
	.probe = hws_probe,
	.remove = hws_remove,
};
/*
*/
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
