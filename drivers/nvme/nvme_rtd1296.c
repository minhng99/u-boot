// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <pci.h>
#include <linux/err.h>
#include <asm/arch/pcie.h>
#include "nvme.h"

struct rtd_nvme {
	struct nvme_dev dev;
	bool failed;
};

static int rtd_nvme_doorbell(struct nvme_queue *queue, bool completion, u16 value)
{
	struct rtd_nvme *priv = dev_get_priv(queue->dev->udev);
	u64 offset = 0x1000 + ((u64)queue->qid * 2 + completion) *
		queue->dev->db_stride * sizeof(u32);
	int ret;

	if (priv->failed)
		return -EIO;
	if (offset > U32_MAX || value >= queue->q_depth)
		return -ERANGE;
	ret = rtd1296_pcie_bar0_write(queue->dev->udev, offset, value);
	if (ret)
		priv->failed = true;
	return ret;
}

static int rtd_nvme_probe(struct udevice *dev)
{
	struct rtd_nvme *priv = dev_get_priv(dev);
	struct nvme_dev *ndev = &priv->dev;
	struct pci_child_plat *pci = dev_get_parent_plat(dev);
	struct udevice *bus = pci_get_controller(dev);
	int ret;

	if (!bus || (!device_is_compatible(bus, "realtek,rtd1296-pcie-gen1") &&
		     !device_is_compatible(bus, "realtek,rtd1296-pcie-gen2")))
		return -ENODEV;
	ndev->instance = dev_seq(dev);
	snprintf(ndev->vendor, sizeof(ndev->vendor), "0x%04x", pci->vendor);
	ndev->bar = dm_pci_map_bar(dev, PCI_BASE_ADDRESS_0, 0, 0x800,
				   PCI_REGION_TYPE, PCI_REGION_MEM);
	if (!ndev->bar || IS_ERR(ndev->bar))
		return -ENXIO;
	ret = dm_pci_clrset_config16(dev, PCI_COMMAND, 0, PCI_COMMAND_MASTER);
	if (ret)
		return ret;
	ret = nvme_init(dev);
	if (ret)
		dm_pci_clrset_config16(dev, PCI_COMMAND, PCI_COMMAND_MASTER, 0);
	return ret;
}

static int rtd_nvme_remove(struct udevice *dev)
{
	int ret;

	ret = nvme_shutdown(dev);
	if (ret)
		return ret;
	ret = dm_pci_clrset_config16(dev, PCI_COMMAND, PCI_COMMAND_MASTER, 0);
	if (ret)
		return ret;
	nvme_free_resources(dev);
	return 0;
}

static const struct nvme_ops rtd_nvme_ops = {
	.ring_doorbell = rtd_nvme_doorbell,
};

U_BOOT_DRIVER(rtd1296_nvme) = {
	.name = "rtd1296_nvme",
	.id = UCLASS_NVME,
	.probe = rtd_nvme_probe,
	.remove = rtd_nvme_remove,
	.ops = &rtd_nvme_ops,
	.priv_auto = sizeof(struct rtd_nvme),
	.flags = DM_FLAG_ACTIVE_DMA | DM_FLAG_OS_PREPARE,
};

static struct pci_device_id rtd_nvme_ids[] = {
	{ PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, ~0) },
	{ }
};

U_BOOT_PCI_DEVICE(rtd1296_nvme, rtd_nvme_ids);
