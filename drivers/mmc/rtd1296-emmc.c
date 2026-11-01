// SPDX-License-Identifier: GPL-2.0+
#include <clk.h>
#include <dm.h>
#include <dwmmc.h>
#include <limits.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <dm/read.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/ioport.h>

#define EMMC_DDR		0x10c
#define EMMC_PAD		0x474
#define EMMC_CLOCK	0x478
#define EMMC_SAMPLE	GENMASK(13, 12)
#define EMMC_PUSH	GENMASK(9, 8)
#define EMMC_PLL_PHASE	0x1f0
#define PLL_BIAS		BIT(0)
#define PLL_RESET_N	BIT(1)
#define PLL_PUSH	GENMASK(7, 3)
#define PLL_SAMPLE	GENMASK(12, 8)
#define EMMC_MIN_HZ	300000
#define EMMC_MAX_HZ	20000000

struct rtd_emmc_plat {
	struct mmc mmc;
	struct mmc_config cfg;
};

struct rtd_emmc {
	struct dwmci_host host;
	struct clk gates[3];
	struct reset_ctl *reset;
	struct regmap *crt;
	uint enabled;
};

static int rtd_emmc_disable(struct rtd_emmc *priv)
{
	int ret = reset_assert(priv->reset);

	if (ret)
		return ret;
	while (priv->enabled) {
		ret = clk_disable(&priv->gates[priv->enabled - 1]);
		if (ret)
			return ret;
		priv->enabled--;
	}
	return 0;
}

static int rtd_emmc_phase(struct rtd_emmc *priv)
{
	int ret;

	ret = regmap_update_bits(priv->crt, EMMC_PLL_PHASE,
				 PLL_BIAS | PLL_RESET_N, PLL_BIAS);
	if (ret)
		return ret;
	ret = regmap_update_bits(priv->crt, EMMC_PLL_PHASE, PLL_PUSH | PLL_SAMPLE,
				 FIELD_PREP(PLL_SAMPLE, 16));
	if (ret)
		return ret;
	ret = regmap_update_bits(priv->crt, EMMC_PLL_PHASE, PLL_RESET_N, PLL_RESET_N);
	if (ret)
		return ret;
	udelay(200);
	return reset_deassert(priv->reset);
}

static int rtd_emmc_probe(struct udevice *dev)
{
	static const char * const gate_names[] = { "card", "biu", "ip" };
	struct rtd_emmc_plat *plat = dev_get_plat(dev);
	struct rtd_emmc *priv = dev_get_priv(dev);
	struct dwmci_host *host = &priv->host;
	struct mmc_uclass_priv *uc = dev_get_uclass_priv(dev);
	struct resource res;
	struct clk input;
	ulong rate;
	int ret, err;

	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < EMMC_CLOCK + sizeof(u32))
		return -EINVAL;
	host->ioaddr = dev_remap_addr(dev);
	if (!host->ioaddr)
		return -EINVAL;
	host->name = dev->name;
	host->buswidth = dev_read_u32_default(dev, "bus-width", 1);
	if (host->buswidth != 1 && host->buswidth != 4 && host->buswidth != 8)
		return -EINVAL;
	priv->crt = syscon_regmap_lookup_by_phandle(dev, "realtek,crt");
	if (IS_ERR(priv->crt))
		return PTR_ERR(priv->crt);
	priv->reset = devm_reset_control_get(dev, "emmc");
	if (IS_ERR(priv->reset))
		return PTR_ERR(priv->reset);
	ret = clk_get_by_name(dev, "ciu", &input);
	if (ret)
		return ret;
	rate = clk_get_rate(&input);
	if (IS_ERR_VALUE(rate))
		return (int)rate;
	if (rate < EMMC_MIN_HZ || rate > UINT_MAX)
		return -EINVAL;
	host->bus_hz = rate;
	for (uint i = 0; i < ARRAY_SIZE(gate_names); i++) {
		ret = clk_get_by_name(dev, gate_names[i], &priv->gates[i]);
		if (ret)
			return ret;
	}
	ret = reset_assert(priv->reset);
	if (ret)
		return ret;
	while (priv->enabled < ARRAY_SIZE(priv->gates)) {
		ret = clk_enable(&priv->gates[priv->enabled]);
		if (ret)
			goto disable;
		priv->enabled++;
	}
	ret = rtd_emmc_phase(priv);
	if (ret)
		goto disable;
	dwmci_writel(host, EMMC_DDR, 0);
	dwmci_writel(host, EMMC_PAD, 0);
	dwmci_writel(host, EMMC_CLOCK,
		     FIELD_PREP(EMMC_SAMPLE, 2) | FIELD_PREP(EMMC_PUSH, 1));
	dwmci_writel(host, DWMCI_UHS_REG, BIT(0));
	dwmci_setup_cfg(&plat->cfg, host, EMMC_MAX_HZ, EMMC_MIN_HZ);
	plat->cfg.host_caps = MMC_MODE_1BIT | MMC_CAP(MMC_HS);
	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret)
		goto disable;
	plat->cfg.host_caps &= ~(UHS_CAPS | MMC_MODE_HS_52MHz |
				 MMC_MODE_DDR_52MHz | MMC_MODE_HS200 |
				 MMC_MODE_HS400 | MMC_MODE_HS400_ES);
	plat->cfg.f_max = min_t(uint, plat->cfg.f_max, EMMC_MAX_HZ);
	if (plat->cfg.f_max < EMMC_MIN_HZ) {
		ret = -EINVAL;
		goto disable;
	}
	host->mmc = &plat->mmc;
	plat->mmc.cfg = &plat->cfg;
	plat->mmc.priv = host;
	plat->mmc.dev = dev;
	uc->mmc = &plat->mmc;
	ret = dwmci_probe(dev);
	if (!ret)
		return 0;
disable:
	err = rtd_emmc_disable(priv);
	return err ? err : ret;
}

static int rtd_emmc_bind(struct udevice *dev)
{
	struct rtd_emmc_plat *plat = dev_get_plat(dev);

	return dwmci_bind(dev, &plat->mmc, &plat->cfg);
}

static int rtd_emmc_remove(struct udevice *dev)
{
	return rtd_emmc_disable(dev_get_priv(dev));
}

static const struct udevice_id rtd_emmc_ids[] = {
	{ .compatible = "realtek,rtd1296-dw-mshc" },
	{ }
};

U_BOOT_DRIVER(rtd1296_emmc) = {
	.name = "rtd1296_emmc",
	.id = UCLASS_MMC,
	.of_match = rtd_emmc_ids,
	.bind = rtd_emmc_bind,
	.probe = rtd_emmc_probe,
	.remove = rtd_emmc_remove,
	.ops = &dm_dwmci_ops,
	.priv_auto = sizeof(struct rtd_emmc),
	.plat_auto = sizeof(struct rtd_emmc_plat),
	.flags = DM_FLAG_ACTIVE_DMA | DM_FLAG_OS_PREPARE,
};
