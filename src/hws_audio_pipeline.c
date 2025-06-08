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

int SetAudioQuene(struct hws_pcie_dev *pdx, int dwAudioCh)
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
