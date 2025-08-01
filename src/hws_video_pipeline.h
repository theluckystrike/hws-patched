/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_VIDEO_PIPELINE_H
#define HWS_VIDEO_PIPELINE_H


#include <linux/types.h>
#include "hws.h"


void change_video_size(struct hws_pcie_dev *pdx, int ch, int w, int h, int interlace);
int check_video_capture(struct hws_pcie_dev *pdx, int index);
void hws_enable_video_capture(struct hws_pcie_dev *pdx, int index, int en);
int set_video_format_size(struct hws_pcie_dev *pdx, int ch, int w, int h);

int StartVideoCapture(struct hws_pcie_dev *pdx, int index);
void StopVideoCapture(struct hws_pcie_dev *pdx, int index);

void CheckVideoFmt(struct hws_pcie_dev *pdx);
void InitVideoSys(struct hws_pcie_dev *pdx, int set);
int Get_Video_Status(struct hws_pcie_dev *pdx, unsigned int ch);
int MemCopyVideoToSteam(struct hws_pcie_dev *pdx, int nDecoder);
void video_data_process(struct work_struct *p_work);

#endif /* HWS_VIDEO_PIPELINE_H */
