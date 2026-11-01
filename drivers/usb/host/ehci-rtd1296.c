// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <usb.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <power/regulator.h>
#include "ehci.h"

struct rtd_ehci {
	struct ehci_ctrl ctrl;
	struct ehci_hcor *hcor;
	struct udevice *supply;
	bool registered;
	bool vbus_on;
};

static int rtd_ehci_halt(struct rtd_ehci *priv)
{
	u32 status;

	if (!priv->hcor)
		return 0;
	clrbits_le32(&priv->hcor->or_usbcmd, CMD_RUN);
	return readl_poll_timeout(&priv->hcor->or_usbsts, status,
				 status & STS_HALT, 500000);
}

static int rtd_ehci_remove(struct udevice *dev)
{
	struct rtd_ehci *priv = dev_get_priv(dev);
	int ret;

	ret = rtd_ehci_halt(priv);
	if (ret)
		return ret;
	if (priv->registered) {
		ret = ehci_deregister(dev);
		if (ret)
			return ret;
		priv->registered = false;
	}
	if (priv->vbus_on) {
		ret = regulator_set_enable(priv->supply, false);
		if (ret)
			return ret;
		priv->vbus_on = false;
	}
	return 0;
}

static int rtd_ehci_probe(struct udevice *dev)
{
	struct rtd_ehci *priv = dev_get_priv(dev);
	struct ehci_hccr *hccr;
	void __iomem *wrapper;
	u32 length;
	int ret, err;

	hccr = dev_remap_addr_name(dev, "core");
	wrapper = dev_remap_addr_name(dev, "wrapper");
	if (!hccr || !wrapper)
		return -EINVAL;
	setbits_le32(wrapper + 0x2c, BIT(0));
	clrbits_le32(wrapper + 0x40, BIT(0));
	clrbits_le32(wrapper + 0x44, GENMASK(8, 7));
	udelay(1000);
	length = HC_LENGTH(readl(&hccr->cr_capbase));
	if (length < 0x10 || !IS_ALIGNED(length, 4))
		return -EINVAL;
	priv->hcor = (void __iomem *)hccr + length;
	ret = device_get_supply_regulator(dev, "vbus-supply", &priv->supply);
	if (ret)
		return ret;
	ret = regulator_set_enable(priv->supply, true);
	if (ret)
		return ret;
	priv->vbus_on = true;
	ret = ehci_register(dev, hccr, priv->hcor, NULL, 0, USB_INIT_HOST);
	if (ret) {
		err = rtd_ehci_remove(dev);
		return err ? err : ret;
	}
	priv->registered = true;
	return 0;
}

static int rtd_ehci_of_to_plat(struct udevice *dev)
{
	struct usb_plat *plat = dev_get_plat(dev);

	plat->init_type = USB_INIT_HOST;
	return 0;
}

static const struct udevice_id rtd_ehci_ids[] = {
	{ .compatible = "realtek,rtd1296-ehci" },
	{ }
};

U_BOOT_DRIVER(rtd1296_ehci) = {
	.name = "rtd1296_ehci",
	.id = UCLASS_USB,
	.of_match = rtd_ehci_ids,
	.of_to_plat = rtd_ehci_of_to_plat,
	.probe = rtd_ehci_probe,
	.remove = rtd_ehci_remove,
	.ops = &ehci_usb_ops,
	.plat_auto = sizeof(struct usb_plat),
	.priv_auto = sizeof(struct rtd_ehci),
	.flags = DM_FLAG_ALLOC_PRIV_DMA | DM_FLAG_OS_PREPARE,
};
