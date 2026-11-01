// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <generic-phy.h>
#include <usb.h>
#include <asm/arch/usb.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <power/regulator.h>
#include <usb/xhci.h>
#include "../dwc3/core.h"
#include "../dwc3/io.h"

#define RTD_DWC3_GLOBALS	0x8100
#define RTD_USB_HOST_MODE	0x70

struct rtd_xhci {
	struct xhci_ctrl ctrl;
	struct dwc3 dwc;
	struct phy_bulk phys;
	struct xhci_hcor *hcor;
	struct udevice *supply;
	struct udevice *typec;
	uint initialized;
	uint powered;
	bool core_ready;
	bool registered;
	bool vbus_on;
};

static int rtd_xhci_halt(struct rtd_xhci *priv)
{
	u32 status;

	if (!priv->hcor)
		return 0;
	clrbits_le32(&priv->hcor->or_usbcmd, CMD_RUN);
	return readl_poll_timeout(&priv->hcor->or_usbsts, status,
				 status & STS_HALT, 500000);
}

static int rtd_xhci_cleanup(struct udevice *dev)
{
	struct rtd_xhci *priv = dev_get_priv(dev);
	int ret;

	ret = rtd_xhci_halt(priv);
	if (ret)
		return ret;
	if (priv->registered) {
		ret = xhci_deregister(dev);
		if (ret)
			return ret;
		priv->registered = false;
	} else if (priv->ctrl.dcbaa) {
		xhci_cleanup(&priv->ctrl);
	}
	if (priv->core_ready) {
		dwc3_remove(&priv->dwc);
		priv->core_ready = false;
	}
	while (priv->powered) {
		ret = generic_phy_power_off(&priv->phys.phys[priv->powered - 1]);
		if (ret)
			return ret;
		priv->powered--;
	}
	while (priv->initialized) {
		ret = generic_phy_exit(&priv->phys.phys[priv->initialized - 1]);
		if (ret)
			return ret;
		priv->initialized--;
	}
	if (priv->typec) {
		ret = rtd1296_typec_put(priv->typec);
		if (ret)
			return ret;
		priv->typec = NULL;
	}
	if (priv->vbus_on) {
		ret = regulator_set_enable(priv->supply, false);
		if (ret)
			return ret;
		priv->vbus_on = false;
	}
	return 0;
}

static int rtd_xhci_phys(struct udevice *dev)
{
	struct rtd_xhci *priv = dev_get_priv(dev);
	void __iomem *regs = priv->dwc.regs;
	int ret;

	dwc3_writel(regs, DWC3_GCTL, dwc3_readl(regs, DWC3_GCTL) |
		    DWC3_GCTL_CORESOFTRESET);
	udelay(1);
	dwc3_writel(regs, DWC3_GUSB2PHYCFG(0),
		    dwc3_readl(regs, DWC3_GUSB2PHYCFG(0)) |
		    DWC3_GUSB2PHYCFG_PHYSOFTRST);
	dwc3_writel(regs, DWC3_GUSB3PIPECTL(0),
		    dwc3_readl(regs, DWC3_GUSB3PIPECTL(0)) |
		    DWC3_GUSB3PIPECTL_PHYSOFTRST);
	ret = generic_phy_get_bulk(dev, &priv->phys);
	if (ret)
		return ret;
	for (; priv->initialized < priv->phys.count; priv->initialized++) {
		ret = generic_phy_init(&priv->phys.phys[priv->initialized]);
		if (ret)
			return ret;
	}
	udelay(100000);
	dwc3_writel(regs, DWC3_GUSB2PHYCFG(0),
		    dwc3_readl(regs, DWC3_GUSB2PHYCFG(0)) &
		    ~DWC3_GUSB2PHYCFG_PHYSOFTRST);
	dwc3_writel(regs, DWC3_GUSB3PIPECTL(0),
		    dwc3_readl(regs, DWC3_GUSB3PIPECTL(0)) &
		    ~DWC3_GUSB3PIPECTL_PHYSOFTRST);
	udelay(100000);
	dwc3_writel(regs, DWC3_GCTL, dwc3_readl(regs, DWC3_GCTL) &
		    ~DWC3_GCTL_CORESOFTRESET);
	for (; priv->powered < priv->phys.count; priv->powered++) {
		ret = generic_phy_power_on(&priv->phys.phys[priv->powered]);
		if (ret)
			return ret;
	}
	return 0;
}

static int rtd_xhci_vbus(struct udevice *dev)
{
	struct rtd_xhci *priv = dev_get_priv(dev);
	int ret;

	if (dev_read_prop(dev, "realtek,typec", NULL))
		return rtd1296_typec_get(dev, &priv->typec);
	ret = device_get_supply_regulator(dev, "vbus-supply", &priv->supply);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return ret;
	ret = regulator_set_enable(priv->supply, true);
	if (!ret)
		priv->vbus_on = true;
	return ret;
}

static int rtd_xhci_probe(struct udevice *dev)
{
	struct rtd_xhci *priv = dev_get_priv(dev);
	struct xhci_hccr *hccr;
	void __iomem *wrapper;
	u32 value, length;
	int ret, err;

	hccr = dev_remap_addr_name(dev, "core");
	wrapper = dev_remap_addr_name(dev, "wrapper");
	if (!hccr || !wrapper)
		return -EINVAL;
	priv->dwc.dev = dev;
	priv->dwc.regs = (void __iomem *)hccr + RTD_DWC3_GLOBALS;
	priv->dwc.dr_mode = USB_DR_MODE_HOST;
	priv->dwc.maximum_speed = usb_get_maximum_speed(dev_ofnode(dev));
	if (priv->dwc.maximum_speed == USB_SPEED_UNKNOWN)
		priv->dwc.maximum_speed = USB_SPEED_SUPER;
	dwc3_of_parse(&priv->dwc);
	clrsetbits_le32(wrapper + RTD_USB_HOST_MODE, 0x707, 0x606);
	value = dwc3_readl(priv->dwc.regs, DWC3_DCTL);
	value = (value & ~DWC3_DCTL_RUN_STOP) | DWC3_DCTL_CSFTRST;
	dwc3_writel(priv->dwc.regs, DWC3_DCTL, value);
	ret = readl_poll_timeout(priv->dwc.regs + DWC3_DCTL - DWC3_GLOBALS_REGS_START,
				 value, !(value & DWC3_DCTL_CSFTRST), 500000);
	if (ret)
		return ret;
	ret = rtd_xhci_phys(dev);
	if (ret)
		goto cleanup;
	ret = dwc3_init(&priv->dwc);
	if (ret)
		goto cleanup;
	priv->core_ready = true;
	length = HC_LENGTH(readl(&hccr->cr_capbase));
	if (length < 0x20 || !IS_ALIGNED(length, 4)) {
		ret = -EINVAL;
		goto cleanup;
	}
	priv->hcor = (void __iomem *)hccr + length;
	ret = rtd_xhci_vbus(dev);
	if (ret)
		goto cleanup;
	ret = xhci_register(dev, hccr, priv->hcor);
	if (ret)
		goto cleanup;
	priv->registered = true;
	return 0;
cleanup:
	err = rtd_xhci_cleanup(dev);
	return err ? err : ret;
}

static int rtd_xhci_of_to_plat(struct udevice *dev)
{
	struct usb_plat *plat = dev_get_plat(dev);

	if (usb_get_dr_mode(dev_ofnode(dev)) != USB_DR_MODE_HOST)
		return -EINVAL;
	plat->init_type = USB_INIT_HOST;
	return 0;
}

static const struct udevice_id rtd_xhci_ids[] = {
	{ .compatible = "realtek,rtd1296-dwc3" },
	{ }
};

U_BOOT_DRIVER(rtd1296_xhci) = {
	.name = "rtd1296_xhci",
	.id = UCLASS_USB,
	.of_match = rtd_xhci_ids,
	.of_to_plat = rtd_xhci_of_to_plat,
	.probe = rtd_xhci_probe,
	.remove = rtd_xhci_cleanup,
	.ops = &xhci_usb_ops,
	.plat_auto = sizeof(struct usb_plat),
	.priv_auto = sizeof(struct rtd_xhci),
	.flags = DM_FLAG_ALLOC_PRIV_DMA | DM_FLAG_OS_PREPARE,
};
