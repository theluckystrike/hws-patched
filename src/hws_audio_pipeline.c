#include "hws_audio_pipeline.h"

#if 1
static struct snd_pcm_hardware audio_pcm_hardware = {
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
#else
static struct snd_pcm_hardware audio_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.period_bytes_min = HWS_AUDIO_CELL_SIZE,
	.period_bytes_max = HWS_AUDIO_CELL_SIZE,
	.periods_min = 4,
	.periods_max = 4,
	.buffer_bytes_max = HWS_AUDIO_CELL_SIZE * 4,
};
#endif
int StartAudioCapture(struct hws_pcie_dev *pdx, int index)
{
	int j;

	if (pdx->m_bACapStarted[index] == 1) {
		if (CheckAudioCapture(pdx, index) == 0) {
			CheckCardStatus(pdx);
			EnableAudioCapture(pdx, index, 1);
		}
		//DbgPrint("Re StartAudioCapture =%d",index);
		return -1;
	}
	CheckCardStatus(pdx);
	pdx->m_bAudioRun[index] = 1;
	pdx->m_bAudioStop[index] = 0;
	pdx->m_nAudioBufferIndex[index] = 0;
	pdx->audio_data[index] = 0;
	pdx->m_nRDAudioIndex[index] = 0;
	for (j = 0; j < MAX_AUDIO_QUEUE; j++) {
		pdx->m_AudioInfo[index].pStatusInfo[j].byLock = MEM_UNLOCK;
	}
	pdx->m_AudioInfo[index].dwisRuning = 1;
	EnableAudioCapture(pdx, index, 1);
	return 0;
}

void StopAudioCapture(struct hws_pcie_dev *pdx, int index)
{
	//int inc=0;
	if (pdx->m_bAudioRun[index] == 0)
		return;
	pdx->m_bAudioRun[index] = 0;
	pdx->m_bAudioStop[index] = 1;
	pdx->m_nAudioBufferIndex[index] = 0;
	pdx->m_AudioInfo[index].dwisRuning = 0;
#if 0
	while(1)
	{
		if(pdx->m_bAudioStop[index] ==0)
		{
			break;
		}
		inc++;
		if(inc >2000)
		{
			break;
		}
		msleep(10);
	}
#endif
	EnableAudioCapture(pdx, index, 0);
}

//------------------------------------
int MemCopyAudioToSteam(struct hws_pcie_dev *pdx, int dwAudioCh)
{
	int i = 0;
	BYTE *bBuf = NULL;
	BYTE *pSrcBuf = NULL;
	int nIndex = -1;
	unsigned long flags;
	//int status =-1;
	//printk("MemCopyAudioToSteam =%d",dwAudioCh);
	if (pdx->m_nAudioBufferIndex[dwAudioCh] == 0) {
		pSrcBuf = pdx->m_pbyAudioBuffer[dwAudioCh] +
			  pdx->m_dwAudioPTKSize;
	} else {
		pSrcBuf = pdx->m_pbyAudioBuffer[dwAudioCh];
	}

	//-----------------------------------------------------
	nIndex = -1;
	if (pdx->m_AudioInfo[dwAudioCh].dwisRuning == 1) {
		for (i = pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex;
		     i < MAX_AUDIO_QUEUE; i++) {
			if (pdx->m_AudioInfo[dwAudioCh].pStatusInfo[i].byLock ==
			    MEM_UNLOCK) {
				nIndex = i;
				bBuf = pdx->m_AudioInfo[dwAudioCh]
					       .m_pAudioBufData[i];
				break;
			}
		}
		if (nIndex == -1) {
			for (i = 0;
			     i < pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex;
			     i++) {
				if (pdx->m_AudioInfo[dwAudioCh]
					    .pStatusInfo[i]
					    .byLock == MEM_UNLOCK) {
					nIndex = i;
					bBuf = pdx->m_AudioInfo[dwAudioCh]
						       .m_pAudioBufData[i];
					break;
				}
			}
		}

		if ((nIndex != -1) && bBuf) {
			//pci_dma_sync_single_for_cpu(pdx->pdev,pdx->m_pbyAudio_phys[dwAudioCh],MAX_AUDIO_CAP_SIZE,2);
			dma_sync_single_for_cpu(&pdx->pdev->dev,
						pdx->m_pbyAudio_phys[dwAudioCh],
						MAX_AUDIO_CAP_SIZE, 2);
			memcpy(bBuf, pSrcBuf, pdx->m_dwAudioPTKSize);

			pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex = nIndex + 1;
			if (pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex >=
			    MAX_AUDIO_QUEUE) {
				pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex = 0;
			}
			spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
			pdx->m_AudioInfo[dwAudioCh]
				.pStatusInfo[nIndex]
				.dwLength = pdx->m_dwAudioPTKSize;
			pdx->m_AudioInfo[dwAudioCh].pStatusInfo[nIndex].byLock =
				MEM_LOCK;
			spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh],
					       flags);
			//KeSetEvent(& pdx->m_AudioInfo[dwAudioCh].m_pAudioEvent[audio_index],IO_NO_INCREMENT,FALSE);
			//printk("Set Audio Event %d\n",dwAudioCh);
			//pdx->audio[dwAudioCh].pos = pdx->m_dwAudioPTKSize;
			//snd_pcm_period_elapsed(pdx->audio[dwAudioCh].substream);
			queue_work(pdx->auwq, &pdx->audio[dwAudioCh].audiowork);
			//pdx->m_AudioInfo[dwAudioCh].pStatusInfo[nIndex].byLock = MEM_UNLOCK;

		} else {
			printk("No Audio Buffer Write %d", dwAudioCh);
		}
	}
	return 0;
}

int SetAudioQueue(struct hws_pcie_dev *pdx, int dwAudioCh)
{
	int status = -1;
	//int i;
	//BYTE *bBuf = NULL;
	//BYTE *pSrcBuf= NULL;
	//int nIndex = -1;
	//printk("SetAudioQuene =%d",dwAudioCh);
	if (!pdx->m_bACapStarted[dwAudioCh]) {
		return -1;
	}
	if (!pdx->m_bAudioRun[dwAudioCh]) {
		if (pdx->m_bAudioStop[dwAudioCh] == 1) {
			pdx->m_bAudioStop[dwAudioCh] = 0;
			//DbgPrint("DpcForIsr_Audio0 Exit Event[%d]\n",dwAudioCh);
		}
		pdx->m_nAudioBusy[dwAudioCh] = 0;
		return status;
	}

	pdx->m_nAudioBusy[dwAudioCh] = 1;

	status = MemCopyAudioToSteam(pdx, dwAudioCh);

	pdx->m_nAudioBusy[dwAudioCh] = 0;

	return status;
}

//---------------------------------------------------
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

void audio_data_process(struct work_struct *p_work)
{
	struct hws_audio *drv =
		container_of(p_work, struct hws_audio, audiowork);
	//struct snd_pcm_substream *substream = drv->substream;
	struct hws_pcie_dev *pdx = drv->dev;
	int nIndex;
	int i;
	int delived = 0;
	//unsigned int frames;
	BYTE *bBuf = NULL;
	//int copysize =0;
	//int rev_size=0;
	int dwAudioCh;
	int aud_len;
	unsigned long flags;
	//int avail = 0;
	dwAudioCh = drv->index;
	nIndex = -1;
	//printk("audio_data_process %d\n",dwAudioCh);
	spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
	for (i = pdx->m_nRDAudioIndex[dwAudioCh]; i < MAX_AUDIO_QUEUE; i++) {
		if (pdx->m_AudioInfo[dwAudioCh].pStatusInfo[i].byLock ==
		    MEM_LOCK) {
			nIndex = i;
			bBuf = pdx->m_AudioInfo[dwAudioCh].m_pAudioBufData[i];
			aud_len = pdx->m_AudioInfo[dwAudioCh]
					  .pStatusInfo[i]
					  .dwLength;
			break;
		}
	}
	if (nIndex == -1) {
		for (i = 0; i < pdx->m_nRDAudioIndex[dwAudioCh]; i++) {
			if (pdx->m_AudioInfo[dwAudioCh].pStatusInfo[i].byLock ==
			    MEM_LOCK) {
				nIndex = i;
				bBuf = pdx->m_AudioInfo[dwAudioCh]
					       .m_pAudioBufData[i];
				aud_len = pdx->m_AudioInfo[dwAudioCh]
						  .pStatusInfo[i]
						  .dwLength;
				break;
			}
		}
	}
	if ((nIndex != -1) && (bBuf)) {
		//--- send data -------------
		delived = _deliver_samples(drv, bBuf, aud_len);
#if 0
		 avail = aud_len - delived;
		 if(avail)
		 {
		 	printk("Sample Rate is wrong  =%d \n",avail);
		 }
#endif
		//----------------------------------------------
		//spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
		pdx->m_AudioInfo[dwAudioCh].pStatusInfo[nIndex].byLock =
			MEM_UNLOCK;
		//spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);
		nIndex++;
		if (nIndex >= MAX_AUDIO_QUEUE) {
			nIndex = 0;
		}
		pdx->m_nRDAudioIndex[dwAudioCh] = nIndex;
	}
	spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);
}

void EnableAudioCapture(struct hws_pcie_dev *pdx, int index, int en)
{
	ULONG status;
	int enable;
	if (pdx->m_PciDeviceLost)
		return;

	status = READ_REGISTER_ULONG(pdx, HWS_REG_ACAP_ENABLE);

	if (en) {
		enable = 1;
		enable = enable << index;
		status = status | enable;
	} else {
		enable = 1;
		enable = enable << index;
		enable = ~enable;
		status = status & enable;
	}
	pdx->m_bACapStarted[index] = en;
	WRITE_REGISTER_ULONG(pdx, HWS_REG_ACAP_ENABLE, status);
	//printk("EnableAudioCapture =%X",status);
}

int CheckAudioCapture(struct hws_pcie_dev *pdx, int index)
{
	ULONG status;
	int enable;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_ACAP_ENABLE);

	enable = (status >> index) & 0x01;
	return enable;
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


int hws_audio_register(struct hws_pcie_dev *dev)
{
	struct snd_pcm *pcm;
	struct snd_card *card;
	int ret;
	int i;
	int ai_index;
	char audioname[100];
	//printk("hws_audio_register Start\n");
	ai_index = dev->m_Device_PortID * dev->m_nCurreMaxVideoChl + 1;
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		sprintf(audioname, "%s %d", HWS_AUDOI_NAME, i + ai_index);
		//printk("%s\n",audioname);
		ret = snd_card_new(&dev->pdev->dev, -1, audioname, THIS_MODULE,
				   sizeof(struct hws_audio), &card);
		// ret = snd_card_new(&dev->pdev->dev, audio_index[i], audio_id[i], THIS_MODULE,	sizeof(struct hws_audio), &card);
		if (ret < 0) {
			printk(KERN_ERR
			       "%s() ERROR: snd_card_new failed <%d>\n",
			       __func__, ret);
			goto fail0;
		}
		strcpy(card->driver, KBUILD_MODNAME);
		sprintf(card->shortname, "%s", audioname);
		sprintf(card->longname, "%s", card->shortname);

		ret = snd_pcm_new(card, audioname, 0, 0, 1, &pcm);
		if (ret < 0) {
			printk(KERN_ERR "%s() ERROR: snd_pcm_new failed <%d>\n",
			       __func__, ret);
			goto fail1;
		}
		dev->audio[i].index = i;
		dev->audio[i].dev = dev;
		pcm->private_data = &dev->audio[i];
		strcpy(pcm->name, audioname);
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
				&hws_pcie_pcm_ops);
		//snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,snd_dma_pci_data(dev->pdev), HWS_AUDIO_CELL_SIZE*4, HWS_AUDIO_CELL_SIZE*4);
		snd_pcm_lib_preallocate_pages_for_all(
			pcm, SNDRV_DMA_TYPE_CONTINUOUS,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
			card->dev,
#else
			snd_dma_continuous_data(GFP_KERNEL),
#endif
			audio_pcm_hardware.buffer_bytes_max,
			audio_pcm_hardware.buffer_bytes_max);
		//----------------------------
		dev->audio[i].sample_rate_out = 48000;
		dev->audio[i].channels = 2;
		dev->audio[i].resampled_buf_size =
			dev->audio[i].sample_rate_out * 2 /* sample bytes */ *
			dev->audio[i].channels /* channels */;
		//dev->audio[i].resampled_buf = vmalloc(dev->audio[i].resampled_buf_size);
		//if(dev->audio[i].resampled_buf == NULL)
		//	goto fail1;
		//-----------------
		spin_lock_init(&dev->audio[i].ring_lock);
		INIT_WORK(&dev->audio[i].audiowork, audio_data_process);
		ret = snd_card_register(card);
		if (ret < 0) {
			printk(KERN_ERR
			       "%s() ERROR: snd_card_register failed\n",
			       __func__);
			goto fail1;
		}
		dev->audio[i].card = card;
	}
	//printk("hws_audio_register End\n");
	return 0;
fail1:
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		if (dev->audio[i].card) {
			snd_card_free(dev->audio[i].card);
			dev->audio[i].card = NULL;
		}
		if (dev->audio[i].resampled_buf) {
			vfree(dev->audio[i].resampled_buf);
			dev->audio[i].resampled_buf = NULL;
		}
	}
fail0:
	return -1;
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
	struct hws_audio *chip = snd_pcm_substream_chip(substream);
	struct hws_pcie_dev *dev = chip->dev;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		//HWS_PCIE_READ(HWS_DMA_BASE(chip->index), HWS_DMA_STATUS);
		//start dma
		//HWS_PCIE_WRITE(HWS_INT_BASE, HWS_DMA_MASK(chip->index), 0x00000001);
		//HWS_PCIE_WRITE(HWS_DMA_BASE(chip->index), HWS_DMA_START, 0x00000001);
		//printk(KERN_INFO "SNDRV_PCM_TRIGGER_START index:%x\n",chip->index);
		chip->ring_wpos_byframes = 0;
		chip->period_used_byframes = 0;
		StartAudioCapture(dev, chip->index);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		//stop dma
		//HWS_PCIE_WRITE(HWS_INT_BASE, HWS_DMA_MASK(chip->index), 0x000000000);
		//HWS_PCIE_WRITE(HWS_DMA_BASE(chip->index), HWS_DMA_START, 0x00000000);
		//printk(KERN_INFO "SNDRV_PCM_TRIGGER_STOP index:%x\n",chip->index);
		StopAudioCapture(dev, chip->index);
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}
//-------------------------------------------------

struct snd_pcm_ops hws_pcie_pcm_ops = { .open = hws_pcie_audio_open,
					.close = hws_pcie_audio_close,
					.ioctl = snd_pcm_lib_ioctl,
					.hw_params = hws_pcie_audio_hw_params,
					.hw_free = hws_pcie_audio_hw_free,
					.prepare = hws_pcie_audio_prepare,
					.trigger = hws_pcie_audio_trigger,
					.pointer = hws_pcie_audio_pointer };
