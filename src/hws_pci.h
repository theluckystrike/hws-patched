#ifndef HWS_PCI_H
#define HWS_PCI_H

#include <linux/pci.h>

int hws_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id);
void hws_remove(struct pci_dev *pdev);
struct hws_pcie_dev *alloc_dev_instance(struct pci_dev *pdev);
int __init pcie_hws_init(void);
void __exit pcie_hws_exit(void);

#endif /* HWS_PCI_H */
