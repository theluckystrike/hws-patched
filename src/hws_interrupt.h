/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_INTERRUPT_H
#define HWS_INTERRUPT_H

#include <linux/pci.h>
#include "hws.h"


/**
 * hws_free_irqs() – free IRQs and tear down all DPC threads
 * @dev:    your driver’s private hws_pcie_dev
 */
void hws_free_irqs(struct hws_pcie_dev *dev);
int hws_irq_setup(struct hws_pcie_dev *lro, struct pci_dev *pdev);

void DpcForIsr_Audio0(unsigned long data);
void DpcForIsr_Audio1(unsigned long data);
void DpcForIsr_Audio2(unsigned long data);
void DpcForIsr_Audio3(unsigned long data);

void DpcForIsr_Video0(unsigned long data);
void DpcForIsr_Video1(unsigned long data);
void DpcForIsr_Video2(unsigned long data);
void DpcForIsr_Video3(unsigned long data);

#endif /* HWS_INTERRUPT_H */

