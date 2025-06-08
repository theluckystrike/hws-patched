#ifndef HWS_VIDEO_PIPELINE_H
#define HWS_VIDEO_PIPELINE_H


#include <linux/types.h>
#include "hws.h"


void ChangeVideoSize(struct hws_pcie_dev *pdx, int ch, int w, int h, int interlace);
int CheckVideoCapture(struct hws_pcie_dev *pdx, int index);
void EnableVideoCapture(struct hws_pcie_dev *pdx, int index, int en);

int StartVideoCapture(struct hws_pcie_dev *pdx, int index);
void StopVideoCapture(struct hws_pcie_dev *pdx, int index);

void CheckVideoFmt(struct hws_pcie_dev *pdx);
void InitVideoSys(struct hws_pcie_dev *pdx, int set);
int Get_Video_Status(struct hws_pcie_dev *pdx, unsigned int ch);
int MemCopyVideoToSteam(struct hws_pcie_dev *pdx, int nDecoder);
void video_data_process(struct work_struct *p_work);

#endif /* HWS_VIDEO_PIPELINE_H */
