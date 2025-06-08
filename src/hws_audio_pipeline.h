#ifndef HWS_AUDIO_PIPELINE_H
#define HWS_AUDIO_PIPELINE_H

#include <linux/workqueue.h>    /* for struct work_struct */
#include <sound/pcm.h>          /* for struct snd_pcm_hardware */
#include "hws_common.h"         /* for struct hws_pcie_dev, HWS_AUDIO_CELL_SIZE */

/* PCM hardware capabilities (defined/initialized in hws_audio_pipeline.c) */
extern struct snd_pcm_hardware audio_pcm_hardware;

/* Audio capture pipeline APIs */
int  StartAudioCapture(struct hws_pcie_dev *pdx, int index);
void StopAudioCapture(struct hws_pcie_dev *pdx, int index);
int  MemCopyAudioToSteam(struct hws_pcie_dev *pdx, int dwAudioCh);
void audio_data_process(struct work_struct *p_work);
void EnableAudioCapture(struct hws_pcie_dev *pdx, int index, int en);
int CheckAudioCapture(struct hws_pcie_dev *pdx, int index);

#endif /* HWS_AUDIO_PIPELINE_H */
