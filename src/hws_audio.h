/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_AUDIO_PIPELINE_H
#define HWS_AUDIO_PIPELINE_H

#include <linux/workqueue.h>    /* for struct work_struct */
#include <sound/pcm.h>          /* for struct snd_pcm_hardware */
#include "hws.h"         /* for struct hws_pcie_dev, HWS_AUDIO_CELL_SIZE */

/* PCM hardware capabilities (defined/initialized in hws_audio_pipeline.c) */
void hws_audio_program_next_period(struct hws_pcie_dev *hws, unsigned int ch);
extern struct snd_pcm_hardware audio_pcm_hardware;
int hws_audio_register(struct hws_pcie_dev *dev);
void hws_audio_unregister(struct hws_pcie_dev *hws);
void hws_enable_audio_capture(struct hws_pcie_dev *hws,
                                            unsigned int ch,
                                            bool enable);

/* Audio capture pipeline APIs */
int  hws_start_audio_capture(struct hws_pcie_dev *pdx, unsigned int index);
void hws_stop_audio_capture(struct hws_pcie_dev *pdx, unsigned int index);
int hws_audio_init_channel(struct hws_pcie_dev *pdev, int ch);
void hws_audio_cleanup_channel(struct hws_pcie_dev *pdev, int ch, bool device_removal);

#endif /* HWS_AUDIO_PIPELINE_H */
