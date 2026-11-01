// SPDX-License-Identifier: GPL-2.0+
#include <clk.h>
#include <dm.h>
#include <reset.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <linux/err.h>

#define RTD_USB_PHY_RESETS	6
#define RTD_USB_RESETS		9

struct rtd_usb_bus {
	struct clk clock;
	struct reset_ctl_bulk *resets;
	bool clock_on;
};

static int rtd_usb_clock(struct rtd_usb_bus *priv, bool on)
{
	int ret;

	if (on == priv->clock_on)
		return 0;
	ret = on ? clk_enable(&priv->clock) : clk_disable(&priv->clock);
	if (!ret)
		priv->clock_on = on;
	return ret;
}

static int rtd_usb_disable(struct rtd_usb_bus *priv)
{
	int ret = reset_assert_bulk(priv->resets);

	if (ret)
		return ret;
	return rtd_usb_clock(priv, false);
}

static int rtd_usb_release(struct rtd_usb_bus *priv, uint first, uint end)
{
	int ret;

	for (uint i = first; i < end; i++) {
		ret = reset_deassert(&priv->resets->resets[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int rtd_usb_bus_probe(struct udevice *dev)
{
	struct rtd_usb_bus *priv = dev_get_priv(dev);
	int ret, err;

	ret = clk_get_by_index(dev, 0, &priv->clock);
	if (ret)
		return ret;
	priv->resets = devm_reset_bulk_get(dev);
	if (IS_ERR(priv->resets))
		return PTR_ERR(priv->resets);
	if (priv->resets->count != RTD_USB_RESETS)
		return -EINVAL;
	ret = rtd_usb_clock(priv, true);
	if (ret)
		return ret;
	ret = rtd_usb_disable(priv);
	if (ret)
		return ret;
	ret = rtd_usb_release(priv, 0, RTD_USB_PHY_RESETS);
	if (ret)
		goto disable;
	udelay(200000);
	ret = rtd_usb_clock(priv, true);
	if (ret)
		goto disable;
	ret = rtd_usb_clock(priv, false);
	if (ret)
		goto disable;
	udelay(10000);
	ret = rtd_usb_release(priv, RTD_USB_PHY_RESETS, RTD_USB_RESETS);
	if (ret)
		goto disable;
	udelay(10000);
	ret = rtd_usb_clock(priv, true);
	if (ret)
		goto disable;
	udelay(10000);
	return 0;
disable:
	err = rtd_usb_disable(priv);
	return err ? err : ret;
}

static int rtd_usb_bus_remove(struct udevice *dev)
{
	return rtd_usb_disable(dev_get_priv(dev));
}

static const struct udevice_id rtd_usb_bus_ids[] = {
	{ .compatible = "realtek,rtd1296-usb-bus" },
	{ }
};

U_BOOT_DRIVER(rtd1296_usb_bus) = {
	.name = "rtd1296_usb_bus",
	.id = UCLASS_SIMPLE_BUS,
	.of_match = rtd_usb_bus_ids,
	.probe = rtd_usb_bus_probe,
	.remove = rtd_usb_bus_remove,
	.priv_auto = sizeof(struct rtd_usb_bus),
	.flags = DM_FLAG_OS_PREPARE,
};
