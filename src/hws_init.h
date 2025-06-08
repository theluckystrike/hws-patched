#ifndef HWS_INIT_H
#define HWS_INIT_H

#include "hws_common.h"

int ReadChipId(struct hws_pcie_dev *pdx);
void SetHardWareInfo(struct hws_pcie_dev *pdx);
void CheckCardStatus(struct hws_pcie_dev *pdx);
void hws_adapters_init(struct hws_pcie_dev *dev);
void StopDevice(struct hws_pcie_dev *pdx);

#endif /* HWS_INIT_H */
