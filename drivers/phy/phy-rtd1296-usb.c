// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <event.h>
#include <generic-phy.h>
#include <asm/io.h>
#include <dm/read.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>

#define USB2_REQUEST	BIT(25)
#define USB2_BUSY	BIT(23)
#define USB2_PAGE	0xf4
#define USB2_PAGE0	0x9b
#define USB2_PAGE1	0xbb
#define USB3_BUSY	BIT(7)
#define USB3_WRITE	BIT(0)
#define USB3_RX		0x09
#define USB3_RX_CALIBRATE BIT(9)

enum rtd_usb_phy_type {
	RTD_USB2,
	RTD_USB3,
};

struct rtd_usb_phy {
	void __iomem *port;
	void __iomem *viewport;
	enum rtd_usb_phy_type type;
	u32 drive;
	u32 coarse;
	u32 fine;
	bool has_coarse;
	bool has_fine;
	bool initialized;
	bool powered;
};

static int rtd_usb2_wait(struct rtd_usb_phy *priv)
{
	u32 value;

	return readl_poll_timeout(priv->viewport, value, !(value & USB2_BUSY), 50000);
}

static int rtd_usb2_address(struct rtd_usb_phy *priv, u8 address)
{
	int ret;

	for (uint shift = 0; shift < 8; shift += 4) {
		ret = rtd_usb2_wait(priv);
		if (ret)
			return ret;
		writel(USB2_REQUEST | (((address >> shift) & 0xf) << 8), priv->viewport);
		ret = rtd_usb2_wait(priv);
		if (ret)
			return ret;
	}
	return 0;
}

static int rtd_usb2_read(struct rtd_usb_phy *priv, u8 address, u8 *value)
{
	int ret;

	if (address < 0x20)
		return -EINVAL;
	ret = rtd_usb2_address(priv, address - 0x20);
	if (!ret)
		*value = readl(priv->viewport);
	return ret;
}

static int rtd_usb2_write(struct rtd_usb_phy *priv, u8 address, u8 value)
{
	int ret = rtd_usb2_wait(priv);

	if (ret)
		return ret;
	writel(value, priv->port);
	return rtd_usb2_address(priv, address);
}

static int rtd_usb2_configure(struct rtd_usb_phy *priv)
{
	static const struct {
		u8 address;
		u8 value;
	} profile[] = {
		{ 0xe0, 0xe0 }, { 0xe2, 0x3a }, { 0xe4, 0xa8 },
		{ 0xe6, 0x91 }, { 0xf5, 0x81 }, { 0xf7, 0x02 },
	};
	u8 value, wanted;
	int ret, restore;

	ret = rtd_usb2_write(priv, USB2_PAGE, USB2_PAGE0);
	if (ret)
		return ret;
	for (uint i = 0; i < ARRAY_SIZE(profile); i++) {
		wanted = profile[i].value;
		if (profile[i].address == 0xe4)
			wanted = (wanted & 0xf0) | priv->drive;
		ret = rtd_usb2_write(priv, profile[i].address, wanted);
		if (!ret)
			ret = rtd_usb2_read(priv, profile[i].address, &value);
		if (ret)
			return ret;
		if (value != wanted)
			return -EIO;
	}
	ret = rtd_usb2_write(priv, USB2_PAGE, USB2_PAGE1);
	if (ret)
		goto page0;
	ret = rtd_usb2_read(priv, 0xe0, &value);
	if (ret)
		goto page0;
	ret = rtd_usb2_write(priv, 0xe0, value & ~BIT(2));
	if (ret)
		goto page0;
	udelay(1000);
	ret = rtd_usb2_write(priv, 0xe0, value | BIT(2));
page0:
	restore = rtd_usb2_write(priv, USB2_PAGE, USB2_PAGE0);
	return ret ? ret : restore;
}

static int rtd_usb3_wait(struct rtd_usb_phy *priv)
{
	u32 value;

	return readl_poll_timeout(priv->port, value, !(value & USB3_BUSY), 50000);
}

static int rtd_usb3_read(struct rtd_usb_phy *priv, u8 address, u16 *value)
{
	int ret = rtd_usb3_wait(priv);

	if (ret)
		return ret;
	writel((u32)address << 8, priv->port);
	ret = rtd_usb3_wait(priv);
	if (!ret)
		*value = readl(priv->port) >> 16;
	return ret;
}

static int rtd_usb3_write(struct rtd_usb_phy *priv, u8 address, u16 value)
{
	int ret = rtd_usb3_wait(priv);

	if (ret)
		return ret;
	writel((u32)value << 16 | (u32)address << 8 | USB3_WRITE, priv->port);
	return rtd_usb3_wait(priv);
}

static int rtd_usb3_update(struct rtd_usb_phy *priv, u8 address, u16 mask, u16 value)
{
	u16 previous;
	int ret = rtd_usb3_read(priv, address, &previous);

	if (ret)
		return ret;
	value = (previous & ~mask) | (value & mask);
	return previous == value ? 0 : rtd_usb3_write(priv, address, value);
}

static int rtd_usb3_configure(struct rtd_usb_phy *priv)
{
	static const struct {
		u8 address;
		u16 value;
	} profile[] = {
		{ 0x00, 0x4008 }, { 0x01, 0xe046 }, { 0x03, 0x2779 },
		{ 0x06, 0x000e }, { 0x0a, 0xa600 }, { 0x0b, 0xa904 },
		{ 0x19, 0x4004 }, { 0x1f, 0x2807 }, { 0x20, 0x947a },
		{ 0x21, 0x88aa }, { 0x28, 0xf8c2 }, { 0x2f, 0x0040 },
	};
	int ret;

	for (uint i = 0; i < ARRAY_SIZE(profile); i++) {
		ret = rtd_usb3_write(priv, profile[i].address, profile[i].value);
		if (ret)
			return ret;
	}
	if (priv->has_coarse) {
		ret = rtd_usb3_update(priv, 0x20, 0xff, priv->coarse);
		if (ret)
			return ret;
	}
	if (priv->has_fine) {
		ret = rtd_usb3_update(priv, 0x21, 0xffff, priv->fine);
		if (ret)
			return ret;
	}
	return rtd_usb3_update(priv, USB3_RX, BIT(6), BIT(6));
}

static int rtd_usb_phy_init(struct phy *phy)
{
	struct rtd_usb_phy *priv = dev_get_priv(phy->dev);
	int ret = 0;

	if (priv->type == RTD_USB3)
		ret = rtd_usb3_configure(priv);
	if (!ret)
		priv->initialized = true;
	return ret;
}

static int rtd_usb_phy_on(struct phy *phy)
{
	struct rtd_usb_phy *priv = dev_get_priv(phy->dev);
	int ret;

	if (!priv->initialized)
		return -EINVAL;
	if (priv->type == RTD_USB2) {
		ret = rtd_usb2_configure(priv);
	} else {
		ret = rtd_usb3_update(priv, USB3_RX, USB3_RX_CALIBRATE, 0);
		if (ret)
			return ret;
		udelay(1000);
		ret = rtd_usb3_update(priv, USB3_RX, USB3_RX_CALIBRATE, USB3_RX_CALIBRATE);
	}
	if (!ret)
		priv->powered = true;
	return ret;
}

static int rtd_usb_phy_off(struct phy *phy)
{
	struct rtd_usb_phy *priv = dev_get_priv(phy->dev);

	priv->powered = false;
	return 0;
}

static int rtd_usb_phy_exit(struct phy *phy)
{
	struct rtd_usb_phy *priv = dev_get_priv(phy->dev);

	priv->initialized = false;
	return 0;
}

static int rtd_usb_phy_probe(struct udevice *dev)
{
	struct rtd_usb_phy *priv = dev_get_priv(dev);
	struct resource res;
	int ret;

	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < sizeof(u32))
		return -EINVAL;
	priv->port = dev_remap_addr_index(dev, 0);
	if (!priv->port)
		return -EINVAL;
	priv->type = dev_get_driver_data(dev);
	if (priv->type == RTD_USB2) {
		ret = dev_read_resource(dev, 1, &res);
		if (ret)
			return ret;
		if (resource_size(&res) < sizeof(u32))
			return -EINVAL;
		priv->viewport = dev_remap_addr_index(dev, 1);
		if (!priv->viewport)
			return -EINVAL;
		priv->drive = dev_read_u32_default(dev, "realtek,driving-level", 8);
		return priv->drive > 15 ? -EINVAL : 0;
	}
	priv->has_coarse = dev_read_prop(dev, "realtek,amplitude-control-coarse-tuning", NULL);
	priv->has_fine = dev_read_prop(dev, "realtek,amplitude-control-fine-tuning", NULL);
	if (priv->has_coarse) {
		ret = dev_read_u32(dev, "realtek,amplitude-control-coarse-tuning", &priv->coarse);
		if (ret)
			return ret;
	}
	if (priv->has_fine) {
		ret = dev_read_u32(dev, "realtek,amplitude-control-fine-tuning", &priv->fine);
		if (ret)
			return ret;
	}
	return priv->coarse > 0xff || priv->fine > 0xffff ? -EINVAL : 0;
}

static int rtd_usb_phy_pre_remove(void *ctx, struct event *event)
{
	struct udevice *dev = event->data.dm.dev;
	struct rtd_usb_phy *priv;

	if (device_get_uclass_id(dev) != UCLASS_PHY ||
	    (!device_is_compatible(dev, "realtek,rtd1296-usb2phy") &&
	     !device_is_compatible(dev, "realtek,rtd1296-usb3phy")))
		return 0;
	priv = dev_get_priv(dev);
	return priv->initialized || priv->powered ? -EPROBE_DEFER : 0;
}

EVENT_SPY_FULL(EVT_DM_PRE_REMOVE, rtd_usb_phy_pre_remove);

static const struct phy_ops rtd_usb_phy_ops = {
	.init = rtd_usb_phy_init,
	.exit = rtd_usb_phy_exit,
	.power_on = rtd_usb_phy_on,
	.power_off = rtd_usb_phy_off,
};

static const struct udevice_id rtd_usb_phy_ids[] = {
	{ .compatible = "realtek,rtd1296-usb2phy", .data = RTD_USB2 },
	{ .compatible = "realtek,rtd1296-usb3phy", .data = RTD_USB3 },
	{ }
};

U_BOOT_DRIVER(rtd1296_usb_phy) = {
	.name = "rtd1296_usb_phy",
	.id = UCLASS_PHY,
	.of_match = rtd_usb_phy_ids,
	.probe = rtd_usb_phy_probe,
	.ops = &rtd_usb_phy_ops,
	.priv_auto = sizeof(struct rtd_usb_phy),
};
