/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_AUDIO_PIPELINE_H
#define HWS_AUDIO_PIPELINE_H

#include <linux/workqueue.h>    /* for struct work_struct */
#include <sound/pcm.h>          /* for struct snd_pcm_hardware */
#include "hws.h"         /* for struct hws_pcie_dev, HWS_AUDIO_CELL_SIZE */

/* PCM hardware capabilities (defined/initialized in hws_audio_pipeline.c) */
extern struct snd_pcm_hardware audio_pcm_hardware;
int hws_audio_register(struct hws_pcie_dev *dev);
void hws_audio_unregister(struct hws_pcie_dev *hws);
void hws_enable_audio_capture(struct hws_pcie_dev *hws,
                                            unsigned int ch,
                                            bool enable);

/* Audio capture pipeline APIs */
int  hws_start_audio_capture(struct hws_pcie_dev *pdx, unsigned int index);
void hws_stop_audio_capture(struct hws_pcie_dev *pdx, unsigned int index);
int  hws_copy_audio_to_stream(struct hws_pcie_dev *pdx, unsigned int ch);
void audio_data_process(struct work_struct *p_work);
int hws_set_audio_queue(struct hws_pcie_dev *pdx, unsigned int ch);

#endif /* HWS_AUDIO_PIPELINE_H */
