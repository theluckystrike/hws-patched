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
    hws->stop_requested[ch] = false;

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
    hws->stop_requested[ch] = true;

    /* reset write pointer for a fresh restart */
    hws->audio[ch].wr_idx = 0;

    /* disable the work‑queue handler */
    hws->audio_info[ch].is_running = false;

    /* shut off the DMA in hardware */
    hws_enable_audio_capture(hws, ch, false);

    dev_dbg(&hws->pdev->dev,
            "audio capture stopped on channel %u\n", ch);
}

static int hws_copy_audio_to_stream(struct hws_pcie_dev *hws,
                                    unsigned int ch)
{
    unsigned long flags;
    u8    *src, *dst = NULL;
    int    i, idx = -1;

    /* pick which half of the DMA buffer to copy */
    if (hws->wr_idx[ch] == 0)
        src = hws->buf_virt[ch] + hws->audio_pkt_size;
    else
        src = hws->buf_virt[ch];

    /* nothing to do if capture isn’t running */
    if (!hws->audio_info[ch].is_running)
        return 0;

    /* find next free slot in the software ring */
    for (i = hws->wr_idx[ch]; i < MAX_AUDIO_QUEUE; ++i) {
        if (hws->audio_info[ch].status[i].lock == MEM_UNLOCK) {
            idx = i;
            dst = hws->audio_info[ch].buf_data[i];
            break;
        }
    }
    if (idx < 0) {
        for (i = 0; i < hws->wr_idx[ch]; ++i) {
            if (hws->audio_info[ch].status[i].lock == MEM_UNLOCK) {
                idx = i;
                dst = hws->audio_info[ch].buf_data[i];
                break;
            }
        }
    }

    if (idx < 0 || !dst) {
        dev_warn(&hws->pdev->dev,
                 "no free audio buffer on channel %u\n", ch);
        return -ENOMEM;
    }

    /* sync the DMA page into CPU domain */
    dma_sync_single_for_cpu(&hws->pdev->dev,
                            hws->buf_phys_addr[ch],
                            MAX_AUDIO_CAP_SIZE,
                            DMA_FROM_DEVICE);

    /* copy the packet into our slot */
    memcpy(dst, src, hws->audio_pkt_size);

    /* advance write pointer */
    hws->wr_idx[ch] = (idx + 1) % MAX_AUDIO_QUEUE;

    /* mark the slot with length & locked */
    spin_lock_irqsave(&hws->audiolock[ch], flags);
    hws->audio_info[ch].status[idx].length = hws->audio_pkt_size;
    hws->audio_info[ch].status[idx].lock   = MEM_LOCK;
    spin_unlock_irqrestore(&hws->audiolock[ch], flags);

    /* kick the workqueue to deliver to ALSA */
    queue_work(hws->audio_wq, &hws->audio[ch].audio_work);

    return 0;
}

int hws_set_audio_queue(struct hws_pcie_dev *hws, unsigned int ch)
{
    int ret = 0;

    dev_dbg(&hws->pdev->dev,
            "set audio queue on channel %u\n", ch);

    /* no DMA until capture has been enabled */
    if (!hws->cap_active[ch])
        return -ENODEV;

    /* if stream is stopped, clear stop flag and exit */
    if (!hws->stream_running[ch]) {
        if (hws->stop_requested[ch]) {
            hws->stop_requested[ch] = false;
            dev_dbg(&hws->pdev->dev,
                    "cleared stop flag on channel %u\n", ch);
        }
        hws->dma_busy[ch] = false;
        return 0;
    }

    /* mark DMA busy while copying */
    hws->dma_busy[ch] = true;
    ret = hws_copy_audio_to_stream(hws, ch);
    hws->dma_busy[ch] = false;

    return ret;
}


static int _deliver_samples(struct hws_audio *drv, void *aud_data, u32 aud_len)
{
	struct snd_pcm_substream *substream = drv->substream;
	int elapsed = 0;
	unsigned int frames = aud_len / (2 * drv->channels);
	int copy_bytes = 0;
	unsigned long flags;

	if (frames == 0)
		return 0;

	if (drv->ring_wpos_byframes + frames > drv->ring_size_byframes) {
		copy_bytes =
			(drv->ring_size_byframes - drv->ring_wpos_byframes) *
			2 * drv->channels;
		memcpy(substream->runtime->dma_area +
			       drv->ring_wpos_byframes * 2 * drv->channels,
		       aud_data, copy_bytes);
		memcpy(substream->runtime->dma_area, aud_data + copy_bytes,
		       aud_len - copy_bytes);
	} else {
		memcpy(substream->runtime->dma_area +
			       drv->ring_wpos_byframes * 2 * drv->channels,
		       aud_data, aud_len);
	}

	spin_lock_irqsave(&drv->ring_lock, flags);
	drv->ring_wpos_byframes += frames;
	drv->ring_wpos_byframes %= drv->ring_size_byframes;
	/* check if a period available */
	elapsed = (drv->period_used_byframes + frames) /
		  drv->period_size_byframes;
	drv->period_used_byframes += frames;
	drv->period_used_byframes %= drv->period_size_byframes;
	spin_unlock_irqrestore(&drv->ring_lock, flags);

	if (elapsed && substream) {
		/* snd_pcm_period_elapsed will call SNDRV_PCM_TRIGGER_STOP in somecase */
		snd_pcm_period_elapsed(drv->substream);
	}

	return frames * 2 * drv->channels;
}

void audio_data_process(struct work_struct *work)
{
	struct hws_audio     *chan = container_of(work,
	                                          struct hws_audio,
	                                          audio_work);      /* was audiowork */
	struct hws_pcie_dev  *hws  = chan->parent;                /* was dev  */
	const unsigned int    ch   = chan->channel_index;         /* was index */
	unsigned long flags;
	int          i;
	int          rd          = hws->audio[ch].rd_idx;
	u8          *buf         = NULL;
	u32          len         = 0;

	spin_lock_irqsave(&hws->audiolock[ch], flags);
	for (i = 0; i < MAX_AUDIO_QUEUE; ++i) {
		int idx = (rd + i) % MAX_AUDIO_QUEUE;

		if (hws->audio_info[ch].status[idx].lock == MEM_LOCK) {
			buf = hws->audio_info[ch].buf_data[idx];
			len = hws->audio_info[ch].status[idx].length;

			/* Hand the buffer over to userspace and release it          */
			hws->audio_info[ch].status[idx].lock = MEM_UNLOCK;
			hws->audio[ch].rd_idx = (idx + 1) % MAX_AUDIO_QUEUE;
			break;
		}
	}
	spin_unlock_irqrestore(&hws->audiolock[ch], flags);

	/* Copy samples to ALSA only after the lock is dropped */
	if (buf && len)
		_deliver_samples(chan, buf, len);
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
	struct snd_card *card;
	struct snd_pcm *pcm;
	int ret, i;
	char name[32];
	if (!hws)
		return -EINVAL;

	for (i = 0; i < hws->max_channels; i++) {
		/* build a unique audio device name */
		ret = snprintf(name, sizeof(name), "%s %d",
		               HWS_AUDIO_NAME,
		               hws->port_id * hws->cur_max_video_ch +
		               i + 1);
		if (ret < 0 || ret >= sizeof(name)) {
			dev_err(&hws->pdev->dev,
			        "audio_register: name snprintf failed\n");
			ret = -EINVAL;
			goto error;
		}

		ret = snd_card_new(&hws->pdev->dev, -1, name, THIS_MODULE,
				   sizeof(struct hws_audio), &card);
		if (ret < 0) {
			dev_err(&hws->pdev->dev,
			        "snd_card_new failed: %d\n", ret);
			goto error;
		}

		/* set up card fields */
		strscpy(card->driver,   KBUILD_MODNAME, sizeof(card->driver));
		strscpy(card->shortname, name,           sizeof(card->shortname));
		strscpy(card->longname,  card->shortname,
		        sizeof(card->longname));

		ret = snd_pcm_new(card, name, 0, 0, 1, &pcm);
		if (ret < 0) {
			dev_err(&hws->pdev->dev,
			        "snd_pcm_new failed: %d\n", ret);
			snd_card_free(card);
			goto error;
		}

		hws->audio[i].index = i;
		hws->audio[i].parent= hws;
		pcm->private_data = &hws->audio[i];
		strscpy(pcm->name, name, sizeof(pcm->name));
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
		                &hws_pcie_pcm_ops);

		/* set up DMA buffers */
		ret = snd_pcm_lib_preallocate_pages_for_all(
			pcm, SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			audio_pcm_hardware.buffer_bytes_max,
			audio_pcm_hardware.buffer_bytes_max);

		if (ret < 0) {
			dev_err(&hws->pdev->dev,
			        "pcm preallocate failed: %d\n", ret);
			snd_card_free(card);
			goto error;
		}

		//----------------------------
		dev->audio[i].output_sample_rate = 48000;
		dev->audio[i].channel_count = 2;
		dev->audio[i].resampled_buffer_size =
			dev->audio[i].output_sample_rate *
			2 *
			dev->audio[i].channel_count;

		hws->audio[i].resampled_buf =
			kzalloc(hws->audio[i].resampled_buf_size, GFP_KERNEL);
		if (!hws->audio[i].resampled_buf) {
			dev_err(&hws->pdev->dev,
			        "resample buffer alloc failed\n");
			snd_card_free(card);
			ret = -ENOMEM;
			goto error;
		}

		spin_lock_init(&hws->audio[i].ring_lock);
		INIT_WORK(&hws->audio[i].audio_work, audio_data_process);
		ret = snd_card_register(card);
		if (ret < 0) {
			dev_err(&hws->pdev->dev,
			        "snd_card_register failed: %d\n", ret);
			snd_card_free(card);
			goto error;
		}
		hws->audio[i].card = card;
	}
	dev_info(&hws->pdev->dev, "audio registration complete (%d channels)\n",
	         hws->cur_max_video_ch);
	return 0;
error:
	/* unwind any channels that were successfully set up */
	while (--i >= 0) {
		if (hws->audio[i].card) {
			snd_card_free(hws->audio[i].card);
			hws->audio[i].card = NULL;
		}
		kfree(hws->audio[i].resampled_buf);
		hws->audio[i].resampled_buf = NULL;
	}
	return ret;
}

void hws_audio_unregister(struct hws_pcie_dev *hws)
{
    int i;

    if (!hws)
        return;

    /* For each channel, cancel work, free the ALSA card, and drop the resample buffer */
    for (i = 0; i < hws->cur_max_video_ch; i++) {
        struct hws_audio *aud = &hws->audio[i];

        /* Make sure any in-flight work is done */
        cancel_work_sync(&aud->audiowork);

        /* Unregister and free the ALSA card (safe to call even if never registered) */
        if (aud->card) {
            snd_card_free(aud->card);
            aud->card = NULL;
        }

        /* Free our software resample buffer */
        kfree(aud->resampled_buf);
        aud->resampled_buf = NULL;
    }

    dev_info(&hws->pdev->dev, "audio unregistered (%d channels)\n",
             hws->cur_max_video_ch);
}

