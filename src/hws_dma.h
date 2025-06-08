#ifndef HWS_DMA_H
#define HWS_DMA_H

int DmaMemAllocPool(struct hws_pcie_dev *pdx);
void DmaMemFreePool(struct hws_pcie_dev *pdx);
void SetDMAAddress(struct hws_pcie_dev *pdx);

#endif // HWS_DMA_H
