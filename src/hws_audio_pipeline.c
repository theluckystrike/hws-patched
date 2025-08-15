/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_audio_pipeline.h"

#include "hws.h"
#include "hws_reg.h"
#include "hws_pci.h"
#include "hws_init.h"

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>


struct snd_pcm_hardware audio_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_RESUME |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_min = 512,
	.period_bytes_max = 16 * 1024,
	.periods_min = 2,
	.periods_max = 255,
};

int hws_start_audio_capture(struct hws_pcie_dev *hws, unsigned int index)
{
	int j, ret;

	if (hws->cap_active[ch]) {
		ret = hws_check_audio_capture(hws, ch);
		if (ret == 0) {
		    hws_check_card_status(hws);
		    hws_enable_audio_capture(hws, ch, true);
		}
		dev_dbg(&hws->pdev->dev,
			"audio channel %u already active, re‑enabled\n", ch);
		return -EBUSY;
	}
    /* Make sure card is healthy */
    hws_check_card_status(hws);

    /* Mark stream running and clear stop flag */
    hws->stream_running[ch] = true;
    atomic_set(&hws->audio[ch].stop_requested, 0);

    /* Reset ring pointers and data pointer */

    hws->audio[ch].wr_idx   = 0;
    hws->audio[ch].rd_idx   = 0;
    hws->data_buf[ch] = NULL;

    /* Unlock any leftover buffers in the queue */
    for (j = 0; j < MAX_AUDIO_QUEUE; ++j)
        hws->audio_info[ch].status[j].lock = MEM_UNLOCK;

    /* Tell the work handler the queue is live */
    hws->audio_info[ch].is_running = true;

    /* Kick off the hardware DMA */
    hws_enable_audio_capture(hws, ch, true);

    dev_dbg(&hws->pdev->dev,
            "audio capture started on channel %u\n", ch);
    return 0;
}

static void hws_stop_audio_capture(struct hws_pcie_dev *hws,
                                   unsigned int ch)
{
    /* nothing to do if not currently running */
    if (!hws->stream_running[ch])
        return;

    /* mark stream stopped */
    hws->stream_running[ch] = false;
    atomic_set(&hws->audio[ch].stop_requested, 1);

    /* reset write pointer for a fresh restart */
    hws->audio[ch].wr_idx = 0;

    /* disable the work‑queue handler */
    hws->audio_info[ch].is_running = false;

    /* shut off the DMA in hardware */
    hws_enable_audio_capture(hws, ch, false);

    dev_dbg(&hws->pdev->dev,
            "audio capture stopped on channel %u\n", ch);
}

void hws_enable_audio_capture(struct hws_pcie_dev *hws,
                                            unsigned int ch,
                                            bool enable)
{
    u32 reg, mask = BIT(ch);

    /* no-op if the card has been lost */
    if (hws->pci_lost)
        return;

    /* read current enable bitmap */
    reg = readl(hws->bar0_base + HWS_REG_ACAP_ENABLE);

    if (enable)
        reg |= mask;
    else
        reg &= ~mask;

    /* update our software state */
    hws->cap_active[ch] = enable;

    /* write back to HW */
    writel(reg, hws->bar0_base + HWS_REG_ACAP_ENABLE);

    dev_dbg(&hws->pdev->dev,
            "audio capture %s on ch %u, reg=0x%08x\n",
            enable ? "enabled" : "disabled", ch, reg);
}

static inline bool hws_check_audio_capture(struct hws_pcie_dev *hws,
                                           unsigned int ch)
{
    u32 reg;

    /* read back enable bitmap */
    reg = readl(hws->bar0_base + HWS_REG_ACAP_ENABLE);

    return !!(reg & BIT(ch));
}

//-------------------------------------------------
static snd_pcm_uframes_t
hws_pcie_audio_pointer(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	//struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t pos;
	int dwAudioCh;
	unsigned long flags;
	dwAudioCh = drv->index;
	//printk(KERN_INFO "%s() index:%x\n",__func__,dwAudioCh);
	spin_lock_irqsave(&drv->ring_lock, flags); //spin_lock
	pos = drv->ring_wpos_byframes;
	spin_unlock_irqrestore(&drv->ring_lock, flags); //spin_unlock
	return pos;
}

int hws_pcie_audio_open(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	drv->sample_rate_out = 48000;
	drv->channels = 2;
	//printk(KERN_INFO "%s() index:%x\n",__func__,drv->index);
	runtime->hw = audio_pcm_hardware;
	drv->substream = substream;
	//snd_pcm_hw_constraint_minmax(runtime,SNDRV_PCM_HW_PARAM_RATE,setrate,setrate);
	return 0;
}

int hws_pcie_audio_close(struct snd_pcm_substream *substream)
{
	//	struct hws_audio *chip = snd_pcm_substream_chip(substream);
	//printk(KERN_INFO "%s() \n",__func__);
	return 0;
}
int hws_pcie_audio_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params)
{
	//printk(KERN_INFO "%s() \n",__func__);
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

int hws_pcie_audio_hw_free(struct snd_pcm_substream *substream)
{
	//printk(KERN_INFO "%s() \n",__func__);
	return snd_pcm_lib_free_pages(substream);
}

int hws_pcie_audio_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	//struct hws_pcie_dev *dev= drv->dev;
	//int i;
	unsigned long flags;
	//printk(KERN_INFO "%s() index:%x\n",__func__,drv->index);

	spin_lock_irqsave(&drv->ring_lock, flags);
	drv->ring_size_byframes = runtime->buffer_size;
	drv->ring_wpos_byframes = 0;
	drv->period_size_byframes = runtime->period_size;
	drv->period_used_byframes = 0;
	drv->ring_offsize = 0;
	drv->ring_over_size = 0;
	spin_unlock_irqrestore(&drv->ring_lock, flags);

	return 0;
}

int hws_pcie_audio_trigger(struct snd_pcm_substream *substream, int cmd)
{
    struct hws_audio    *audio = snd_pcm_substream_chip(substream);
    struct hws_pcie_dev *hws   = audio->parent;           /* was dev */
    unsigned int         ch    = audio->channel_index;    /* was index */
    int                  err   = 0;

    dev_dbg(&hws->pdev->dev,
            "audio trigger %d on channel %u\n", cmd, ch);
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        audio->ring_write_pos_frames = 0;  /* was ring_wpos_byframes */
        audio->period_used_frames    = 0;  /* was period_used_byframes */
	// FIXME
        hws_start_audio_capture(hws, ch);
        break;

    case SNDRV_PCM_TRIGGER_STOP:
	// FIXME
        hws_stop_audio_capture(hws, ch);
        break;

    default:
        err = -EINVAL;
        break;
    }

    return err;
}

struct snd_pcm_ops hws_pcie_pcm_ops = { .open = hws_pcie_audio_open,
					.close = hws_pcie_audio_close,
					.ioctl = snd_pcm_lib_ioctl,
					.hw_params = hws_pcie_audio_hw_params,
					.hw_free = hws_pcie_audio_hw_free,
					.prepare = hws_pcie_audio_prepare,
					.trigger = hws_pcie_audio_trigger,
					.pointer = hws_pcie_audio_pointer };

int hws_audio_register(struct hws_pcie_dev *hws)
{
	struct snd_card *card = NULL;
	struct snd_pcm  *pcm  = NULL;
	char card_id[16];
	char card_name[64];
	int i, ret;

	if (!hws)
		return -EINVAL;

	/* ---- Create a single ALSA card for this PCI function ---- */
	snprintf(card_id, sizeof(card_id), "hws%u", hws->port_id);     /* <=16 chars */
	snprintf(card_name, sizeof(card_name), "HWS HDMI Audio %u", hws->port_id);

	ret = snd_card_new(&hws->pdev->dev, -1 /* auto index */,
	                   card_id, THIS_MODULE, 0, &card);
	if (ret < 0) {
		dev_err(&hws->pdev->dev, "snd_card_new failed: %d\n", ret);
		return ret;
	}

	strscpy(card->driver,   KBUILD_MODNAME, sizeof(card->driver));
	strscpy(card->shortname, card_name,      sizeof(card->shortname));
	strscpy(card->longname,  card->shortname, sizeof(card->longname));

	/* ---- Create one PCM capture device per HDMI input ---- */
	for (i = 0; i < hws->max_channels; i++) {
		char pcm_name[32];

		snprintf(pcm_name, sizeof(pcm_name), "HDMI In %d", i);

		/* device number = i → userspace sees hw:X,i */
		ret = snd_pcm_new(card, pcm_name, i,
		                  0 /* playback */, 1 /* capture */, &pcm);
		if (ret < 0) {
			dev_err(&hws->pdev->dev, "snd_pcm_new(%d) failed: %d\n", i, ret);
			goto error_card;
		}

		/* Tie this PCM to channel i */
		hws->audio[i].parent        = hws;
		hws->audio[i].channel_index = i;
		hws->audio[i].pcm_substream = NULL;
		hws->audio[i].cap_active    = false;
		hws->audio[i].stream_running= false;
		hws->audio[i].stop_requested= false;
		hws->audio[i].last_period_toggle = 0;
		hws->audio[i].output_sample_rate = 48000;  /* will be set at open/prepare if HDMI varies */
		hws->audio[i].channel_count      = 2;
		hws->audio[i].bits_per_sample    = 16;

		pcm->private_data = &hws->audio[i];
		strscpy(pcm->name, pcm_name, sizeof(pcm->name));
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &hws_pcie_pcm_ops);

		/* ALSA-owned DMA buffer, device-visible (no scratch buffer) */
		ret = snd_pcm_lib_preallocate_pages_for_all(
			pcm,
			SNDRV_DMA_TYPE_DEV,
			&hws->pdev->dev,
			audio_pcm_hardware.buffer_bytes_max,
			audio_pcm_hardware.buffer_bytes_max);
		if (ret < 0) {
			dev_err(&hws->pdev->dev,
				"preallocate pages (dev) failed on ch %d: %d\n", i, ret);
			goto error_card;
		}
	}

	/* Register the card once all PCMs are created */
	ret = snd_card_register(card);
	if (ret < 0) {
		dev_err(&hws->pdev->dev, "snd_card_register failed: %d\n", ret);
		goto error_card;
	}

	/* Store the single card handle (optional: also mirror to each channel if you like) */
	hws->snd_card = card;
	dev_info(&hws->pdev->dev, "audio registration complete (%d HDMI inputs)\n",
	         hws->max_channels);
	return 0;

error_card:
	/* Frees all PCMs created on it as well */
	snd_card_free(card);
	return ret;
}

void hws_audio_unregister(struct hws_pcie_dev *hws)
{
	int i;

	if (!hws)
		return;

	/* Quiesce hardware per channel before tearing ALSA down. */
	for (i = 0; i < hws->max_channels; i++) {
		struct hws_audio *a = &hws->audio[i];

		/* Stop capture if it’s running (idempotent). */
		if (a->stream_running || a->cap_active)
			hws_enable_audio_capture(hws, i, false);

		a->cap_active      = false;
		a->stream_running  = false;
		a->stop_requested  = false;
		a->pcm_substream   = NULL;

	}

	/*
	 * Free the single ALSA card. This releases all PCM devices and their
	 * ALSA-owned DMA buffers that were preallocated with
	 * snd_pcm_lib_preallocate_pages_for_all().
	 */
	if (hws->snd_card) {
		snd_card_free(hws->snd_card);
		hws->snd_card = NULL;
	}
    dev_info(&hws->pdev->dev, "audio unregistered (%d channels)\n",
             hws->cur_max_video_ch);
}

