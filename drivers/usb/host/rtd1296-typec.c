// SPDX-License-Identifier: GPL-2.0+
#include <cyclic.h>
#include <dm.h>
#include <event.h>
#include <limits.h>
#include <time.h>
#include <asm/arch/usb.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <dm/read.h>
#include <dm/uclass.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <power/regulator.h>

#define CC1_CONTROL	0x00
#define CC1_REFERENCE	0x04
#define CC2_CONTROL	0x08
#define CC2_REFERENCE	0x0c
#define CC_STATUS	0x10
#define CC_DEBOUNCE	0x14
#define CC_SWITCH	GENMASK(29, 27)
#define CC_SWITCH_ENABLE BIT(29)
#define CC_SWITCH_CC2	GENMASK(28, 27)
#define CC_FLAGS	GENMASK(6, 0)
#define CC_SOURCE	(BIT(5) | BIT(3) | BIT(0))
#define CC_RD		3
#define CC_ATTACH_MS	100
#define CC_DETACH_MS	20

struct rtd_typec_source {
	struct udevice *dev;
	struct udevice *vbus;
	struct gpio_desc *rd;
	void __iomem *regs;
	struct cyclic_info poll;
	uint references;
	int attached;
	int candidate;
	ulong changed;
	bool vbus_on;
	bool polling;
	bool faulted;
	bool updating;
};

static int rtd_typec_power(struct rtd_typec_source *priv, bool on)
{
	int ret;

	if (priv->vbus_on == on)
		return 0;
	ret = regulator_set_enable_if_allowed(priv->vbus, on);
	if (!ret)
		priv->vbus_on = on;
	return ret;
}

static int rtd_typec_orientation(u32 status)
{
	bool cc1 = (status & 7) == CC_RD;
	bool cc2 = ((status >> 3) & 7) == CC_RD;

	return cc1 == cc2 ? -1 : cc2;
}

static int rtd_typec_step(struct rtd_typec_source *priv)
{
	int candidate = rtd_typec_orientation(readl(priv->regs + CC_STATUS));
	u32 route;
	int ret;

	if (candidate != priv->candidate) {
		priv->candidate = candidate;
		priv->changed = get_timer(0);
	}
	if (priv->attached >= 0) {
		if (candidate == priv->attached || get_timer(priv->changed) < CC_DETACH_MS)
			return 0;
		ret = rtd_typec_power(priv, false);
		if (ret)
			return ret;
		clrbits_le32(priv->regs + CC1_CONTROL, CC_SWITCH);
		priv->attached = -1;
		priv->changed = get_timer(0);
		return 0;
	}
	if (candidate < 0 || get_timer(priv->changed) < CC_ATTACH_MS)
		return 0;
	route = CC_SWITCH_ENABLE | (candidate ? CC_SWITCH_CC2 : 0);
	clrsetbits_le32(priv->regs + CC1_CONTROL, CC_SWITCH, route);
	ret = rtd_typec_power(priv, true);
	if (ret) {
		clrbits_le32(priv->regs + CC1_CONTROL, CC_SWITCH);
		return ret;
	}
	priv->attached = candidate;
	return 0;
}

static int rtd_typec_update(struct rtd_typec_source *priv)
{
	int ret;

	if (priv->updating)
		return 0;
	priv->updating = true;
	ret = rtd_typec_step(priv);
	priv->updating = false;
	return ret;
}

static void rtd_typec_poll(struct cyclic_info *poll)
{
	struct rtd_typec_source *priv = container_of(poll, struct rtd_typec_source, poll);
	int ret;

	if (priv->faulted)
		return;
	ret = rtd_typec_update(priv);
	if (ret) {
		priv->faulted = true;
		dev_err(priv->dev, "VBUS update failed: %d\n", ret);
	}
}

static int rtd_typec_stop(struct rtd_typec_source *priv)
{
	int ret;

	if (priv->polling) {
		cyclic_unregister(&priv->poll);
		priv->polling = false;
	}
	ret = rtd_typec_power(priv, false);
	if (ret)
		return ret;
	clrbits_le32(priv->regs + CC1_CONTROL, CC_FLAGS | CC_SWITCH);
	clrbits_le32(priv->regs + CC2_CONTROL, CC_FLAGS);
	priv->attached = -1;
	priv->candidate = -1;
	priv->faulted = false;
	return 0;
}

static int rtd_typec_start(struct rtd_typec_source *priv)
{
	ulong start;
	int ret, err;

	ret = rtd_typec_power(priv, false);
	if (ret)
		return ret;
	writel((0xb << 22) | (0x17 << 17) | (0x10 << 12) | CC_SOURCE,
	       priv->regs + CC1_CONTROL);
	writel((0xc << 22) | (0x17 << 17) | (0x12 << 12) | CC_SOURCE,
	       priv->regs + CC2_CONTROL);
	writel((3 << 18) | (4 << 8), priv->regs + CC1_REFERENCE);
	writel((2 << 26) | (3 << 18) | (5 << 8), priv->regs + CC2_REFERENCE);
	clrsetbits_le32(priv->regs + CC_DEBOUNCE, 0xff, 0x7f);
	priv->attached = -1;
	priv->candidate = -2;
	priv->faulted = false;
	start = get_timer(0);
	do {
		ret = rtd_typec_update(priv);
		if (ret) {
			err = rtd_typec_stop(priv);
			return err ? err : ret;
		}
		if (priv->attached >= 0)
			break;
		udelay(5000);
	} while (get_timer(start) < 300);
	cyclic_register(&priv->poll, rtd_typec_poll, 20000, "rtd1296-typec");
	priv->polling = true;
	return 0;
}

int rtd1296_typec_get(struct udevice *host, struct udevice **port)
{
	struct rtd_typec_source *priv;
	struct udevice *dev;
	int ret;

	*port = NULL;
	ret = uclass_get_device_by_phandle(UCLASS_MISC, host, "realtek,typec", &dev);
	if (ret)
		return ret;
	if (!device_is_compatible(dev, "realtek,rtd1296-typec-source"))
		return -ENODEV;
	priv = dev_get_priv(dev);
	if (priv->references == UINT_MAX)
		return -EOVERFLOW;
	if (!priv->references) {
		ret = rtd_typec_start(priv);
		if (ret)
			return ret;
	}
	priv->references++;
	*port = dev;
	return 0;
}

int rtd1296_typec_put(struct udevice *port)
{
	struct rtd_typec_source *priv = dev_get_priv(port);
	int ret;

	if (!priv->references)
		return -EINVAL;
	if (priv->references == 1) {
		ret = rtd_typec_stop(priv);
		if (ret)
			return ret;
	}
	priv->references--;
	return 0;
}

static int rtd_typec_probe(struct udevice *dev)
{
	struct rtd_typec_source *priv = dev_get_priv(dev);
	struct resource res;
	ofnode connector = dev_read_subnode(dev, "connector");
	const char *role, *data;
	int ret;

	if (!ofnode_valid(connector))
		return -EINVAL;
	role = ofnode_read_string(connector, "power-role");
	data = ofnode_read_string(connector, "data-role");
	if (!role || strcmp(role, "source") || !data || strcmp(data, "host") ||
	    !ofnode_read_bool(connector, "pd-disable"))
		return -EOPNOTSUPP;
	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < 0x18)
		return -EINVAL;
	priv->regs = dev_remap_addr(dev);
	if (!priv->regs)
		return -EINVAL;
	priv->dev = dev;
	ret = device_get_supply_regulator(dev, "vbus-supply", &priv->vbus);
	if (ret)
		return ret;
	priv->rd = devm_gpiod_get_optional(dev, "realtek,rd-ctrl",
					   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	return IS_ERR(priv->rd) ? PTR_ERR(priv->rd) : 0;
}

static int rtd_typec_pre_remove(void *ctx, struct event *event)
{
	struct udevice *dev = event->data.dm.dev;
	struct rtd_typec_source *priv;

	if (device_get_uclass_id(dev) != UCLASS_MISC ||
	    !device_is_compatible(dev, "realtek,rtd1296-typec-source"))
		return 0;
	priv = dev_get_priv(dev);
	return priv->references ? -EPROBE_DEFER : 0;
}

EVENT_SPY_FULL(EVT_DM_PRE_REMOVE, rtd_typec_pre_remove);

static int rtd_typec_remove(struct udevice *dev)
{
	return rtd_typec_stop(dev_get_priv(dev));
}

static const struct udevice_id rtd_typec_ids[] = {
	{ .compatible = "realtek,rtd1296-typec-source" },
	{ }
};

U_BOOT_DRIVER(rtd1296_typec_source) = {
	.name = "rtd1296_typec_source",
	.id = UCLASS_MISC,
	.of_match = rtd_typec_ids,
	.probe = rtd_typec_probe,
	.remove = rtd_typec_remove,
	.priv_auto = sizeof(struct rtd_typec_source),
};
