// SPDX-License-Identifier: GPL-2.0+
#include <clk.h>
#include <dm.h>
#include <wdt.h>
#include <asm/io.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/math64.h>

#define RTD_WDT_CONTROL	0x00
#define RTD_WDT_RELOAD	0x04
#define RTD_WDT_LIMIT	0x0c
#define RTD_WDT_KEY_MASK	GENMASK(7, 0)
#define RTD_WDT_ENABLE	0xff
#define RTD_WDT_DISABLE	0xa5

struct rtd_wdt {
	struct clk clock;
	void __iomem *regs;
	ulong rate;
};

static int rtd_wdt_stop(struct udevice *dev)
{
	struct rtd_wdt *priv = dev_get_priv(dev);

	clrsetbits_le32(priv->regs + RTD_WDT_CONTROL,
			RTD_WDT_KEY_MASK, RTD_WDT_DISABLE);
	return 0;
}

static int rtd_wdt_reset(struct udevice *dev)
{
	struct rtd_wdt *priv = dev_get_priv(dev);

	writel(BIT(0), priv->regs + RTD_WDT_RELOAD);
	return 0;
}

static int rtd_wdt_start(struct udevice *dev, u64 timeout_ms, ulong flags)
{
	struct rtd_wdt *priv = dev_get_priv(dev);
	struct wdt_uc_plat *plat = dev_get_uclass_plat(dev);
	u32 ticks;

	if (flags)
		return -EOPNOTSUPP;
	if (!timeout_ms || timeout_ms > plat->max_timeout_ms)
		return -ERANGE;
	ticks = DIV_ROUND_UP_ULL(timeout_ms * priv->rate, 1000);
	rtd_wdt_stop(dev);
	writel(ticks, priv->regs + RTD_WDT_LIMIT);
	rtd_wdt_reset(dev);
	clrsetbits_le32(priv->regs + RTD_WDT_CONTROL,
			RTD_WDT_KEY_MASK, RTD_WDT_ENABLE);
	return 0;
}

static int rtd_wdt_probe(struct udevice *dev)
{
	struct rtd_wdt *priv = dev_get_priv(dev);
	struct wdt_uc_plat *plat = dev_get_uclass_plat(dev);
	struct resource res;
	u64 maximum;
	int ret;

	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < 0x10)
		return -EINVAL;
	priv->regs = dev_remap_addr(dev);
	if (!priv->regs)
		return -EINVAL;
	ret = clk_get_by_index(dev, 0, &priv->clock);
	if (ret)
		return ret;
	priv->rate = clk_get_rate(&priv->clock);
	if (IS_ERR_VALUE(priv->rate))
		return (int)priv->rate;
	if (!priv->rate)
		return -EINVAL;
	maximum = div64_u64((u64)U32_MAX * 1000, priv->rate);
	if (!maximum)
		return -ERANGE;
	plat->max_timeout_ms = min_t(u64, maximum, U32_MAX);
	ret = clk_enable(&priv->clock);
	if (ret)
		return ret;
	return rtd_wdt_stop(dev);
}

static int rtd_wdt_remove(struct udevice *dev)
{
	struct rtd_wdt *priv = dev_get_priv(dev);

	rtd_wdt_stop(dev);
	return clk_disable(&priv->clock);
}

static const struct wdt_ops rtd_wdt_ops = {
	.start = rtd_wdt_start,
	.stop = rtd_wdt_stop,
	.reset = rtd_wdt_reset,
};

static const struct udevice_id rtd_wdt_ids[] = {
	{ .compatible = "realtek,rtd1296-watchdog" },
	{ }
};

U_BOOT_DRIVER(rtd1296_wdt) = {
	.name = "rtd1296_wdt",
	.id = UCLASS_WDT,
	.of_match = rtd_wdt_ids,
	.probe = rtd_wdt_probe,
	.remove = rtd_wdt_remove,
	.ops = &rtd_wdt_ops,
	.priv_auto = sizeof(struct rtd_wdt),
	.flags = DM_FLAG_OS_PREPARE,
};
