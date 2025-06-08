#ifndef HWS_VIDEO_H
#define HWS_VIDEO_H

int hws_video_register(struct hws_pcie_dev *dev);
void StopKSThread(struct hws_pcie_dev *pdx);
void StartKSThread(struct hws_pcie_dev *pdx);
int SetVideoFormatSize(struct hws_pcie_dev *pdx, int ch, int w, int h);

#endif // HWS_VIDEO_H
