/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_VIDEO_H
#define HWS_VIDEO_H

void StopKSThread(struct hws_pcie_dev *pdx);
void StartKSThread(struct hws_pcie_dev *pdx);
int hws_video_register(struct hws_pcie_dev *dev);
void hws_video_unregister(struct hws_pcie_dev *dev);


#endif // HWS_VIDEO_H
