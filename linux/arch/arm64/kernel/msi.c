/*
 * ARM64 architectural MSI implemention
 *
 * Support for Message Signalelled Interrupts for systems that
 * implement ARM Generic Interrupt Controller: GICv2m.
 *
 * Copyright (C) 2014 Advanced Micro Devices, Inc.
 * Authors: Suravee Suthikulpanit <suravee.suthikulpanit@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/pci.h>

#include <asm/msi.h>

/*
 * ARM64 function for seting up MSI irqs.
 * Copied from driver/pci/msi.c: arch_setup_msi_irqs().
 */
int arm64_setup_msi_irqs(struct pci_dev *dev, int nvec, int type)
{
	struct msi_desc *entry;
	int ret;

	if (type == PCI_CAP_ID_MSI && nvec > 1)
		return 1;

	list_for_each_entry(entry, &dev->msi_list, list) {
		ret = arch_setup_msi_irq(dev, entry);
		if (ret < 0)
			return ret;
		if (ret > 0)
			return -ENOSPC;
	}

	return 0;
}

struct arm64_msi_ops arm64_msi = {
	.setup_msi_irqs         = arm64_setup_msi_irqs,
	.teardown_msi_irqs      = default_teardown_msi_irqs,
};

int arch_setup_msi_irqs(struct pci_dev *dev, int nvec, int type)
{
	return arm64_msi.setup_msi_irqs(dev, nvec, type);
}

void arch_teardown_msi_irqs(struct pci_dev *dev)
{
	arm64_msi.teardown_msi_irqs(dev);
}