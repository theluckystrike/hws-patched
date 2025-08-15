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

// FIXME
extern size_t hws_audio_dma_wptr_bytes(struct hws_pcie_dev *hws, unsigned int ch);

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
	if (!hws || ch >= hws->max_channels)
		return -EINVAL;

	/* If already active, ensure HW bit is set and return success. */
	if (hws->audio[ch].stream_running) {
		if (!hws_check_audio_capture(hws, ch)) {
			hws_check_card_status(hws);
			hws_enable_audio_capture(hws, ch, true);
		}
		dev_dbg(&hws->pdev->dev,
			"audio ch %u already running, ensured enabled\n", ch);
		return 0;
	}

	/* Make sure card is healthy */
	hws_check_card_status(hws);

	hws->audio[ch].stop_requested = false;
	hws->audio[ch].stream_running = true;

	/* Kick off the hardware DMA */
	hws_enable_audio_capture(hws, ch, true);

	dev_dbg(&hws->pdev->dev, "audio capture started on ch %u\n", ch);
	return 0;
}

static void hws_stop_audio_capture(struct hws_pcie_dev *hws,
                                   unsigned int ch)
{
	if (!hws || ch >= hws->max_channels)
		return;

	if (!hws->audio[ch].stream_running)
		return;

	hws->audio[ch].stop_requested = true;
	hws->audio[ch].stream_running = false;

	/* Shut off the DMA in hardware */
	hws_enable_audio_capture(hws, ch, false);

	dev_dbg(&hws->pdev->dev, "audio capture stopped on ch %u\n", ch);
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
	struct hws_audio *a = snd_pcm_substream_chip(sub);
	struct snd_pcm_runtime *rt = sub->runtime;
	struct hws_pcie_dev *hws = a->parent;
	unsigned int ch = a->channel_index;

	/* Read HW write offset (bytes from base, already wrapped) */
	size_t off_bytes = hws_audio_dma_wptr_bytes(hws, ch);
	return bytes_to_frames(rt, off_bytes % rt->dma_bytes);
}

int hws_pcie_audio_open(struct snd_pcm_substream *sub)
{
	struct hws_audio *a = snd_pcm_substream_chip(sub);
	struct snd_pcm_runtime *rt = sub->runtime;

	rt->hw = audio_pcm_hardware;
	a->pcm_substream = sub;

	/* If HDMI exposes per-port caps, relax/override here (rates/formats/channels) */
	/* e.g., update rt->hw.rates |= SNDRV_PCM_RATE_44100; etc. */

	/* Integer periods are typical for DMA engines */
	snd_pcm_hw_constraint_integer(rt, SNDRV_PCM_HW_PARAM_PERIODS);

	/* If your HW needs period alignment (e.g., 32B), add a constraint here. */

	return 0;
}

int hws_pcie_audio_close(struct snd_pcm_substream *substream)
{
	struct hws_audio *a = snd_pcm_substream_chip(sub);
	a->pcm_substream = NULL;
	return 0;
}
int hws_pcie_audio_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params)
{
	/* Using preallocation done at registration time; nothing to do. */
	return 0;
}

int hws_pcie_audio_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

int hws_pcie_audio_prepare(struct snd_pcm_substream *substream)
{
	// FIXME: program the card with DMA here
	struct hws_audio *a = snd_pcm_substream_chip(sub);
	struct hws_pcie_dev *hws = a->parent;
	struct snd_pcm_runtime *rt = sub->runtime;
	unsigned int ch = a->channel_index;

	/* Program your DMA with ALSA-owned buffer */
	dma_addr_t dma_base   = rt->dma_addr;
	u32        buf_bytes  = rt->dma_bytes;
	u32        per_bytes  = frames_to_bytes(rt, rt->period_size);

	/* Program HW: base address, buffer size, period size, and audio format */
	hws_prog_dma_base(hws, ch, dma_base, buf_bytes);
	hws_prog_dma_period(hws, ch, per_bytes);
	hws_prog_audio_fmt(hws, ch,
			   /* sample rate */ 48000,            /* or from HDMI caps */
			   /* channels    */ 2,
			   /* format      */ SNDRV_PCM_FORMAT_S16_LE);

	/* Clear any per-channel SW tracking if you keep it */
	a->last_period_toggle = 0;

	return 0;
}

int hws_pcie_audio_trigger(struct snd_pcm_substream *substream, int cmd)
{

	struct hws_audio *a = snd_pcm_substream_chip(sub);
	struct hws_pcie_dev *hws = a->parent;
	unsigned int ch = a->channel_index;

    dev_dbg(&hws->pdev->dev,
            "audio trigger %d on channel %u\n", cmd, ch);
    switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		return hws_start_audio_capture(hws, ch);
	case SNDRV_PCM_TRIGGER_STOP:
		hws_stop_audio_capture(hws, ch);
		return 0;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* if HW supports, re-enable; otherwise treat like START */
		return hws_start_audio_capture(hws, ch);
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* if HW supports pause, disable engine; otherwise treat like STOP */
		hws_stop_audio_capture(hws, ch);
		return 0;
	default:
		return -EINVAL;
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

