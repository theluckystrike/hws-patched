#ifndef HWS_PCI_H
#define HWS_PCI_H

#include <linux/pci.h>
#include <linux/types.h>
#include "hws.h"


#define MAKE_ENTRY( __vend, __chip, __subven, __subdev, __configptr) {	\
	.vendor		= (__vend),					\
	.device		= (__chip),					\
	.subvendor	= (__subven),					\
	.subdevice	= (__subdev),					\
	.driver_data	= (unsigned long) (__configptr)			\
}

int hws_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id);
void hws_remove(struct pci_dev *pdev);
struct hws_pcie_dev *alloc_dev_instance(struct pci_dev *pdev);
int __init pcie_hws_init(void);
void __exit pcie_hws_exit(void);
// int msi_msix_capable(struct pci_dev *dev, int type);
// int probe_scan_for_msi(struct hws_pcie_dev *lro, struct pci_dev *pdev);
void WRITE_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset,
				 u32 Value);
u32 READ_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset);

#ifndef arch_msi_check_device
int  arch_msi_check_device(struct pci_dev *dev, int nvec, int type);
#endif

int msi_msix_capable(struct pci_dev *dev, int type);
int probe_scan_for_msi(struct hws_pcie_dev *lro, struct pci_dev *pdev);

#endif /* HWS_PCI_H */
