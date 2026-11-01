// SPDX-License-Identifier: GPL-2.0+
#include <clk.h>
#include <dm.h>
#include <event.h>
#include <generic-phy.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <dm/read.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>

#define SATA_MDIO	0x00
#define SATA_SELECT	0x04
#define SATA_RATE	0x08
#define MDIO_BUSY	BIT(7)
#define MDIO_REGISTER	GENMASK(13, 8)
#define MDIO_RATE	GENMASK(15, 14)
#define MDIO_DATA	GENMASK(31, 16)
#define SATA_OOB	0x980

struct rtd_sata_phy {
	void __iomem *regs;
	struct regmap *sb2;
	struct clk_bulk clocks;
	struct reset_ctl *resets[2];
	uint enabled;
	u8 initialized;
	u8 powered;
};

struct rtd_sata_analog {
	u8 reg;
	u16 value[3];
};

static const struct rtd_sata_analog rtd_sata_analog[] = {
	{ 0x11, { 0x0000, 0x0000, 0x0000 } },
	{ 0x05, { 0x336a, 0x336a, 0x336a } },
	{ 0x01, { 0xe070, 0xe05c, 0xe04a } },
	{ 0x06, { 0x0015, 0x0015, 0x0015 } },
	{ 0x0a, { 0xc600, 0xc600, 0xc600 } },
	{ 0x02, { 0x7000, 0x7000, 0x7000 } },
	{ 0x0a, { 0xc660, 0xc660, 0xc660 } },
	{ 0x19, { 0x2004, 0x2004, 0x2004 } },
	{ 0x20, { 0x94aa, 0x94aa, 0x94aa } },
	{ 0x15, { 0x1717, 0x1717, 0x1717 } },
	{ 0x16, { 0x0770, 0x0770, 0x0770 } },
	{ 0x2a, { 0x4000, 0x4000, 0x4000 } },
	{ 0x10, { 0x2900, 0x2900, 0x2900 } },
	{ 0x0c, { 0x4000, 0x4000, 0x4000 } },
	{ 0x17, { 0x0027, 0x0027, 0x0027 } },
	{ 0x20, { 0x94a7, 0x94a7, 0x94a7 } },
	{ 0x21, { 0x587a, 0x587a, 0x587a } },
	{ 0x04, { 0x538e, 0x538e, 0x538e } },
	{ 0x09, { 0x4210, 0x4210, 0x4210 } },
	{ 0x03, { 0x276f, 0x276d, 0x276d } },
	{ 0x2a, { 0x7c00, 0x7c00, 0x7c00 } },
};

static int rtd_sata_mdio(struct rtd_sata_phy *priv, u8 reg, uint rate, u16 value)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(priv->regs + SATA_MDIO, status,
				 !(status & MDIO_BUSY), 10000);
	if (ret)
		return ret;
	writel(BIT(4) | BIT(0) | FIELD_PREP(MDIO_REGISTER, reg) |
	       FIELD_PREP(MDIO_RATE, rate) | FIELD_PREP(MDIO_DATA, value),
	       priv->regs + SATA_MDIO);
	udelay(1000);
	return readl_poll_timeout(priv->regs + SATA_MDIO, status,
				  !(status & MDIO_BUSY), 10000);
}

static int rtd_sata_phy_init(struct phy *phy)
{
	struct rtd_sata_phy *priv = dev_get_priv(phy->dev);
	u32 mask;
	int ret;

	ret = reset_assert(priv->resets[phy->id]);
	if (ret)
		return ret;
	mask = phy->id ? BIT(1) | BIT(3) | BIT(5) :
			BIT(0) | BIT(2) | BIT(4) | BIT(8);
	ret = regmap_update_bits(priv->sb2, SATA_OOB, mask, mask);
	if (ret)
		return ret;
	writel(phy->id, priv->regs + SATA_SELECT);
	for (uint i = 0; i < ARRAY_SIZE(rtd_sata_analog); i++) {
		const struct rtd_sata_analog *step = &rtd_sata_analog[i];

		for (uint rate = 0; rate < ARRAY_SIZE(step->value); rate++) {
			ret = rtd_sata_mdio(priv, step->reg, rate, step->value[rate]);
			if (ret)
				return ret;
		}
	}
	clrsetbits_le32(priv->regs + SATA_RATE, GENMASK(3, 0), 0xa);
	priv->initialized |= BIT(phy->id);
	return 0;
}

static int rtd_sata_phy_on(struct phy *phy)
{
	struct rtd_sata_phy *priv = dev_get_priv(phy->dev);
	int ret;

	if (!(priv->initialized & BIT(phy->id)))
		return -EINVAL;
	ret = reset_deassert(priv->resets[phy->id]);
	if (!ret)
		priv->powered |= BIT(phy->id);
	return ret;
}

static int rtd_sata_phy_off(struct phy *phy)
{
	struct rtd_sata_phy *priv = dev_get_priv(phy->dev);
	int ret = reset_assert(priv->resets[phy->id]);

	if (!ret)
		priv->powered &= ~BIT(phy->id);
	return ret;
}

static int rtd_sata_phy_exit(struct phy *phy)
{
	struct rtd_sata_phy *priv = dev_get_priv(phy->dev);
	int ret = rtd_sata_phy_off(phy);

	if (!ret)
		priv->initialized &= ~BIT(phy->id);
	return ret;
}

static int rtd_sata_phy_xlate(struct phy *phy, struct ofnode_phandle_args *args)
{
	if (args->args_count != 1 || args->args[0] > 1)
		return -EINVAL;
	phy->id = args->args[0];
	return 0;
}

static int rtd_sata_phy_disable(struct rtd_sata_phy *priv)
{
	int ret;

	while (priv->enabled) {
		ret = clk_disable(&priv->clocks.clks[priv->enabled - 1]);
		if (ret)
			return ret;
		priv->enabled--;
	}
	return 0;
}

static int rtd_sata_phy_probe(struct udevice *dev)
{
	static const char * const reset_names[] = { "port0", "port1" };
	struct rtd_sata_phy *priv = dev_get_priv(dev);
	struct resource res;
	int ret, err;

	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < 0xc)
		return -EINVAL;
	priv->regs = dev_remap_addr(dev);
	if (!priv->regs)
		return -EINVAL;
	priv->sb2 = syscon_regmap_lookup_by_phandle(dev, "realtek,sb2");
	if (IS_ERR(priv->sb2))
		return PTR_ERR(priv->sb2);
	for (uint i = 0; i < ARRAY_SIZE(priv->resets); i++) {
		priv->resets[i] = devm_reset_control_get(dev, reset_names[i]);
		if (IS_ERR(priv->resets[i]))
			return PTR_ERR(priv->resets[i]);
	}
	ret = clk_get_bulk(dev, &priv->clocks);
	if (ret)
		return ret;
	if (priv->clocks.count != 2)
		return -EINVAL;
	while (priv->enabled < priv->clocks.count) {
		ret = clk_enable(&priv->clocks.clks[priv->enabled]);
		if (ret) {
			err = rtd_sata_phy_disable(priv);
			return err ? err : ret;
		}
		priv->enabled++;
	}
	return 0;
}

static int rtd_sata_phy_pre_remove(void *ctx, struct event *event)
{
	struct udevice *dev = event->data.dm.dev;
	struct rtd_sata_phy *priv;

	if (device_get_uclass_id(dev) != UCLASS_PHY ||
	    !device_is_compatible(dev, "realtek,rtd1296-sata-phy"))
		return 0;
	priv = dev_get_priv(dev);
	return priv->initialized || priv->powered ? -EPROBE_DEFER : 0;
}

EVENT_SPY_FULL(EVT_DM_PRE_REMOVE, rtd_sata_phy_pre_remove);

static int rtd_sata_phy_remove(struct udevice *dev)
{
	struct rtd_sata_phy *priv = dev_get_priv(dev);
	int ret;

	for (uint i = 0; i < ARRAY_SIZE(priv->resets); i++) {
		ret = reset_assert(priv->resets[i]);
		if (ret)
			return ret;
	}
	return rtd_sata_phy_disable(priv);
}

static const struct phy_ops rtd_sata_phy_ops = {
	.init = rtd_sata_phy_init,
	.exit = rtd_sata_phy_exit,
	.power_on = rtd_sata_phy_on,
	.power_off = rtd_sata_phy_off,
	.of_xlate = rtd_sata_phy_xlate,
};

static const struct udevice_id rtd_sata_phy_ids[] = {
	{ .compatible = "realtek,rtd1296-sata-phy" },
	{ }
};

U_BOOT_DRIVER(rtd1296_sata_phy) = {
	.name = "rtd1296_sata_phy",
	.id = UCLASS_PHY,
	.of_match = rtd_sata_phy_ids,
	.probe = rtd_sata_phy_probe,
	.remove = rtd_sata_phy_remove,
	.ops = &rtd_sata_phy_ops,
	.priv_auto = sizeof(struct rtd_sata_phy),
	.flags = DM_FLAG_OS_PREPARE,
};
