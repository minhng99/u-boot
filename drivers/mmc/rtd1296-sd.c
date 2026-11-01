// SPDX-License-Identifier: GPL-2.0+
#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <malloc.h>
#include <memalign.h>
#include <mmc.h>
#include <regmap.h>
#include <syscon.h>
#include <time.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <dm/read.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/sizes.h>
#include <power/regulator.h>

#define SD_DMA_ADDRESS	0x004
#define SD_DMA_COUNT	0x008
#define SD_DMA_CONTROL	0x00c
#define SD_CONTROL	0x020
#define SD_IRQ_STATUS	0x024
#define SD_IRQ_ENABLE	0x028
#define SD_DUMMY		0x02c
#define SD_PAD		0x074
#define SD_CLOCK	0x078
#define SD_STOP		0x103
#define SD_OUTPUT	0x104
#define SD_SELECT	0x10e
#define SD_PRESENT	0x11f
#define SD_CARD_CLOCK	0x129
#define SD_CONFIG1	0x180
#define SD_CONFIG2	0x181
#define SD_CONFIG3	0x182
#define SD_STATUS2	0x184
#define SD_COMMAND	0x189
#define SD_BYTE_COUNT	0x18f
#define SD_BLOCK_COUNT	0x191
#define SD_TRANSFER	0x193
#define SD_BUS_TA	0x197
#define SD_DMA_ENABLE	BIT(0)
#define SD_DMA_READ	BIT(1)
#define SD_DMA_R2	BIT(4)
#define SD_DMA_SHORT	BIT(5)
#define SD_START	BIT(7)
#define SD_COMPLETE	(BIT(6) | BIT(5))
#define SD_ERROR	BIT(4)
#define SD_CRC16_OFF	BIT(6)
#define SD_CRC7_OFF	BIT(2)
#define SD_DIV256	GENMASK(7, 6)
#define SD_WIDTH	GENMASK(1, 0)
#define SD_AUX_CLOCK	GENMASK(18, 16)
#define SD_PLL1		0x1e0
#define SD_PLL2		0x1e4
#define SD_PLL3		0x1e8
#define SD_PLL4		0x1ec
#define SD_MAX_HZ	25000000
#define SD_MAX_BLOCKS	128

struct rtd_sd_plat {
	struct mmc mmc;
	struct mmc_config cfg;
};

struct rtd_sd {
	void __iomem *regs;
	struct regmap *crt;
	struct regmap *sb2;
	struct clk_bulk clocks;
	struct udevice *supply;
	u8 *buffer;
	uint enabled;
	bool supplied;
	bool accessible;
	bool dummy;
	bool failed;
};

struct rtd_sd_transfer {
	u32 dma;
	uint bytes;
	uint span;
	u16 byte_count;
	u16 blocks;
	u8 command;
	u8 response;
	bool long_response;
	bool use_dma;
};

static int rtd_sd_plan(struct mmc_cmd *cmd, struct mmc_data *data,
		       struct rtd_sd_transfer *xfer)
{
	if (cmd->cmdidx > 63)
		return -EINVAL;
	memset(xfer, 0, sizeof(*xfer));
	xfer->long_response = cmd->resp_type & MMC_RSP_136;
	xfer->response = SD_CRC16_OFF;
	xfer->command = 0x08;
	xfer->blocks = 1;
	xfer->byte_count = 512;
	xfer->span = 512;
	xfer->dma = SD_DMA_ENABLE;
	if (cmd->resp_type & MMC_RSP_PRESENT)
		xfer->response |= xfer->long_response ? 2 : 1;
	if (!(cmd->resp_type & MMC_RSP_CRC))
		xfer->response |= SD_CRC7_OFF;
	if (cmd->resp_type & MMC_RSP_BUSY)
		xfer->response |= BIT(3);
	if (!data) {
		xfer->use_dma = xfer->long_response;
		if (xfer->long_response)
			xfer->dma |= SD_DMA_READ | SD_DMA_R2;
		return 0;
	}
	if (xfer->long_response || !data->blocks || data->blocks > SD_MAX_BLOCKS ||
	    !data->blocksize || data->blocksize > 512 ||
	    (data->flags != MMC_DATA_READ && data->flags != MMC_DATA_WRITE))
		return -EINVAL;
	if (data->blocksize != 512 && (data->blocks != 1 || data->blocksize > 64))
		return -EOPNOTSUPP;
	if (data->flags == MMC_DATA_WRITE && data->blocksize != 512)
		return -EOPNOTSUPP;
	xfer->use_dma = true;
	xfer->blocks = data->blocks;
	xfer->bytes = data->blocks * data->blocksize;
	xfer->byte_count = data->blocksize == 512 ? 512 : 64;
	xfer->span = ALIGN(max_t(uint, xfer->bytes, xfer->byte_count), ARCH_DMA_MINALIGN);
	if (data->blocksize >= 64)
		xfer->response &= ~SD_CRC16_OFF;
	if (xfer->byte_count == 64)
		xfer->dma |= SD_DMA_SHORT;
	if (data->flags == MMC_DATA_READ) {
		xfer->dma |= SD_DMA_READ;
		xfer->command = xfer->byte_count == 64 ? 0x0c : 0x0e;
	} else {
		xfer->command = 0x0a;
	}
	return 0;
}

static int rtd_sd_cancel(struct rtd_sd *priv)
{
	u32 status;
	int ret;

	writel(0, priv->regs + SD_DMA_CONTROL);
	writeb(0, priv->regs + SD_TRANSFER);
	writeb(0xff, priv->regs + SD_STOP);
	writeb(0, priv->regs + SD_STOP);
	clrbits_le32(priv->regs + SD_CONTROL, BIT(0));
	readl(priv->regs + SD_CONTROL);
	udelay(1);
	setbits_le32(priv->regs + SD_CONTROL, BIT(0));
	readl(priv->regs + SD_CONTROL);
	ret = readl_poll_timeout(priv->regs + SD_DMA_CONTROL, status,
				 !(status & SD_DMA_ENABLE), 1000000);
	if (ret)
		priv->failed = true;
	return ret;
}

static int rtd_sd_wait(struct rtd_sd *priv)
{
	ulong start = get_timer(0);
	u8 status;

	do {
		if (!(readb(priv->regs + SD_PRESENT) & BIT(2)))
			return -ENOMEDIUM;
		status = readb(priv->regs + SD_TRANSFER);
		if (readb(priv->regs + SD_STATUS2) & BIT(0))
			return -ETIMEDOUT;
		if (status & SD_ERROR)
			return -EILSEQ;
		if ((status & (SD_START | SD_COMPLETE)) == SD_COMPLETE)
			return 0;
		udelay(1);
	} while (get_timer(start) < 3000);
	return -ETIMEDOUT;
}

static void rtd_sd_count(struct rtd_sd *priv, uint reg, u16 count)
{
	writeb(count, priv->regs + reg);
	writeb(count >> 8, priv->regs + reg + 1);
}

static int rtd_sd_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
			   struct mmc_data *data)
{
	struct rtd_sd *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct rtd_sd_transfer xfer;
	u32 status;
	int ret;

	if (!priv->accessible || priv->failed)
		return -EIO;
	ret = rtd_sd_plan(cmd, data, &xfer);
	if (ret)
		return ret;
	if (!(readb(priv->regs + SD_PRESENT) & BIT(2)))
		return -ENOMEDIUM;
	if (data && data->flags == MMC_DATA_WRITE)
		memcpy(priv->buffer, data->src, xfer.bytes);
	writeb(0x40 | cmd->cmdidx, priv->regs + SD_COMMAND);
	for (uint i = 0; i < 4; i++)
		writeb(cmd->cmdarg >> (24 - 8 * i), priv->regs + SD_COMMAND + i + 1);
	writeb(0, priv->regs + SD_COMMAND + 5);
	writeb(xfer.response, priv->regs + SD_CONFIG2);
	writeb(5, priv->regs + SD_CONFIG3);
	writel(0x16, priv->regs + SD_IRQ_STATUS);
	if (xfer.use_dma) {
		rtd_sd_count(priv, SD_BYTE_COUNT, xfer.byte_count);
		rtd_sd_count(priv, SD_BLOCK_COUNT, xfer.blocks);
		flush_dcache_range((ulong)priv->buffer, (ulong)priv->buffer + xfer.span);
		writel(virt_to_phys(priv->buffer) >> 3, priv->regs + SD_DMA_ADDRESS);
		writel(xfer.blocks, priv->regs + SD_DMA_COUNT);
		writel(xfer.dma, priv->regs + SD_DMA_CONTROL);
	}
	/* Complete buffer and command writes before starting the card engine. */
	mb();
	ret = regmap_write(priv->sb2, 0x20, 0);
	if (ret)
		goto cancel;
	writeb(SD_START | xfer.command, priv->regs + SD_TRANSFER);
	ret = regmap_write(priv->sb2, 0x20, 0);
	if (ret)
		goto cancel;
	ret = rtd_sd_wait(priv);
	if (ret)
		goto cancel;
	if (xfer.use_dma) {
		ret = readl_poll_timeout(priv->regs + SD_DMA_CONTROL, status,
					 !(status & SD_DMA_ENABLE), 1000000);
		if (ret)
			goto cancel;
		writel(0, priv->regs + SD_DMA_CONTROL);
		invalidate_dcache_range((ulong)priv->buffer,
					(ulong)priv->buffer + xfer.span);
	}
	if (xfer.long_response) {
		priv->buffer[16] = readb(priv->regs + SD_COMMAND + 5);
		for (uint i = 0; i < ARRAY_SIZE(cmd->response); i++)
			cmd->response[i] = get_unaligned_be32(priv->buffer + 1 + 4 * i);
	} else if (cmd->resp_type & MMC_RSP_PRESENT) {
		cmd->response[0] = 0;
		for (uint i = 0; i < 4; i++)
			cmd->response[0] = (cmd->response[0] << 8) |
				readb(priv->regs + SD_COMMAND + i + 1);
	}
	if (data && data->flags == MMC_DATA_READ)
		memcpy(data->dest, priv->buffer, xfer.bytes);
	if (cmd->cmdidx == MMC_CMD_SELECT_CARD && cmd->cmdarg && mmc->legacy_speed)
		return mmc_set_clock(mmc, min(mmc->legacy_speed, mmc->cfg->f_max),
				     MMC_CLK_ENABLE);
	return 0;
cancel:
	rtd_sd_cancel(priv);
	return ret;
}

static int rtd_sd_set_ios(struct udevice *dev)
{
	struct rtd_sd *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	u8 config;
	u32 divider;

	if (!priv->accessible)
		return -EIO;
	if (mmc->clk_disable) {
		writeb(0, priv->regs + SD_CARD_CLOCK);
		return 0;
	}
	if (mmc->bus_width != 1 && mmc->bus_width != 4)
		return -EINVAL;
	config = readb(priv->regs + SD_CONFIG1) & ~(SD_DIV256 | SD_WIDTH);
	if (mmc->bus_width == 4)
		config |= 1;
	if (mmc->clock >= SD_MAX_HZ) {
		divider = 1;
	} else if (mmc->clock >= SD_MAX_HZ / 2) {
		divider = 2;
	} else if (mmc->clock >= SD_MAX_HZ / 4) {
		divider = 3;
	} else {
		divider = 0;
		config |= SD_DIV256;
	}
	writel(0x2100 | divider, priv->regs + SD_CLOCK);
	writeb(config, priv->regs + SD_CONFIG1);
	writeb(mmc->clock && !mmc->clk_disable ? BIT(2) : 0,
	       priv->regs + SD_CARD_CLOCK);
	return 0;
}

static int rtd_sd_get_cd(struct udevice *dev)
{
	struct rtd_sd *priv = dev_get_priv(dev);

	return priv->accessible && (readb(priv->regs + SD_PRESENT) & BIT(2));
}

static int rtd_sd_get_wp(struct udevice *dev)
{
	struct rtd_sd *priv = dev_get_priv(dev);

	return priv->accessible && (readb(priv->regs + SD_PRESENT) & BIT(5));
}

static int rtd_sd_pll(struct rtd_sd *priv, bool auxiliary)
{
	int ret;

	if (priv->dummy)
		writel(BIT(30), priv->regs + SD_DUMMY);
	ret = regmap_write(priv->crt, SD_PLL4, 6);
	if (ret)
		goto release;
	if (auxiliary) {
		setbits_le32(priv->regs + SD_CLOCK, SD_AUX_CLOCK);
		ret = regmap_write(priv->crt, SD_PLL2, 0x04515893);
		if (!ret)
			ret = regmap_write(priv->crt, SD_PLL3, 0x00564388);
	} else {
		clrbits_le32(priv->regs + SD_CLOCK, SD_AUX_CLOCK);
	}
	if (ret)
		goto release;
	udelay(2000);
	ret = regmap_write(priv->crt, SD_PLL4, 7);
release:
	if (priv->dummy)
		writel(0, priv->regs + SD_DUMMY);
	return ret;
}

static int rtd_sd_initialize(struct rtd_sd *priv)
{
	int ret;

	ret = regmap_update_bits(priv->crt, SD_PLL4, 7, 7);
	if (ret)
		return ret;
	ret = regmap_write(priv->crt, SD_PLL1, 0x2003);
	if (ret)
		return ret;
	ret = rtd_sd_pll(priv, true);
	if (ret)
		return ret;
	setbits_le32(priv->regs + SD_CONTROL, BIT(1));
	ret = rtd_sd_cancel(priv);
	if (ret)
		return ret;
	writeb(3, priv->regs + SD_BUS_TA);
	ret = rtd_sd_pll(priv, false);
	if (ret)
		return ret;
	writel(0, priv->regs + SD_PAD);
	writel(0x16, priv->regs + SD_IRQ_ENABLE);
	writel(0x16, priv->regs + SD_IRQ_STATUS);
	writeb(0xd0, priv->regs + SD_CONFIG1);
	writeb(2, priv->regs + SD_SELECT);
	writeb(BIT(2), priv->regs + SD_OUTPUT);
	writeb(BIT(2), priv->regs + SD_CARD_CLOCK);
	priv->failed = false;
	return 0;
}

static int rtd_sd_reinit(struct udevice *dev)
{
	struct rtd_sd *priv = dev_get_priv(dev);
	int ret;

	ret = rtd_sd_initialize(priv);
	if (ret)
		priv->failed = true;
	return ret;
}

static int rtd_sd_disable(struct rtd_sd *priv)
{
	int ret;

	if (priv->accessible) {
		ret = rtd_sd_cancel(priv);
		if (ret)
			return ret;
		writeb(0, priv->regs + SD_CARD_CLOCK);
		writeb(0, priv->regs + SD_OUTPUT);
		clrbits_le32(priv->regs + SD_CONTROL, BIT(0));
		readl(priv->regs + SD_CONTROL);
	}
	priv->accessible = false;
	udelay(10);
	while (priv->enabled) {
		ret = clk_disable(&priv->clocks.clks[priv->enabled - 1]);
		if (ret)
			return ret;
		priv->enabled--;
	}
	if (priv->supplied) {
		ret = regulator_set_enable_if_allowed(priv->supply, false);
		if (ret)
			return ret;
		priv->supplied = false;
	}
	return 0;
}

static int rtd_sd_probe(struct udevice *dev)
{
	struct rtd_sd *priv = dev_get_priv(dev);
	struct rtd_sd_plat *plat = dev_get_plat(dev);
	struct mmc_uclass_priv *uc = dev_get_uclass_priv(dev);
	struct resource res;
	u32 revision;
	phys_addr_t address;
	int ret, err;

	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < 0x200)
		return -EINVAL;
	priv->regs = dev_remap_addr(dev);
	if (!priv->regs)
		return -EINVAL;
	ret = dev_read_u32_default(dev, "bus-width", 1);
	if (ret != 1 && ret != 4)
		return -EINVAL;
	priv->crt = syscon_regmap_lookup_by_phandle(dev, "realtek,crt");
	if (IS_ERR(priv->crt))
		return PTR_ERR(priv->crt);
	priv->sb2 = syscon_regmap_lookup_by_phandle(dev, "realtek,sb2");
	if (IS_ERR(priv->sb2))
		return PTR_ERR(priv->sb2);
	ret = regmap_read(priv->sb2, 0x204, &revision);
	if (ret)
		return ret;
	priv->dummy = revision != 0;
	ret = clk_get_bulk(dev, &priv->clocks);
	if (ret)
		return ret;
	if (priv->clocks.count != 2)
		return -EINVAL;
	ret = device_get_supply_regulator(dev, "vmmc-supply", &priv->supply);
	if (ret)
		return ret;
	priv->buffer = memalign(ARCH_DMA_MINALIGN, SZ_64K);
	if (!priv->buffer)
		return -ENOMEM;
	address = virt_to_phys(priv->buffer);
	if (address > SZ_2G - SZ_64K) {
		ret = -ERANGE;
		goto free_buffer;
	}
	ret = regulator_set_enable_if_allowed(priv->supply, true);
	if (ret)
		goto free_buffer;
	priv->supplied = true;
	while (priv->enabled < priv->clocks.count) {
		ret = clk_enable(&priv->clocks.clks[priv->enabled]);
		if (ret)
			goto disable;
		priv->enabled++;
	}
	priv->accessible = true;
	ret = rtd_sd_initialize(priv);
	if (ret)
		goto disable;
	plat->cfg.name = dev->name;
	plat->cfg.voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	plat->cfg.f_min = 400000;
	plat->cfg.f_max = SD_MAX_HZ;
	plat->cfg.b_max = SD_MAX_BLOCKS;
	plat->cfg.host_caps = MMC_MODE_1BIT;
	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret)
		goto disable;
	plat->cfg.host_caps &= ~(UHS_CAPS | MMC_MODE_HS | MMC_MODE_HS_52MHz |
				 MMC_MODE_DDR_52MHz | MMC_MODE_HS200 |
				 MMC_MODE_HS400 | MMC_MODE_HS400_ES | MMC_MODE_8BIT);
	plat->cfg.f_max = min_t(uint, plat->cfg.f_max, SD_MAX_HZ);
	if (plat->cfg.f_max < plat->cfg.f_min) {
		ret = -EINVAL;
		goto disable;
	}
	plat->mmc.cfg = &plat->cfg;
	plat->mmc.priv = priv;
	plat->mmc.dev = dev;
	uc->mmc = &plat->mmc;
	return 0;
disable:
	err = rtd_sd_disable(priv);
	if (err)
		return err;
free_buffer:
	free(priv->buffer);
	priv->buffer = NULL;
	return ret;
}

static int rtd_sd_bind(struct udevice *dev)
{
	struct rtd_sd_plat *plat = dev_get_plat(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static int rtd_sd_remove(struct udevice *dev)
{
	struct rtd_sd *priv = dev_get_priv(dev);
	int ret;

	ret = rtd_sd_disable(priv);
	if (ret)
		return ret;
	free(priv->buffer);
	priv->buffer = NULL;
	return 0;
}

static const struct dm_mmc_ops rtd_sd_ops = {
	.reinit = rtd_sd_reinit,
	.send_cmd = rtd_sd_send_cmd,
	.set_ios = rtd_sd_set_ios,
	.get_cd = rtd_sd_get_cd,
	.get_wp = rtd_sd_get_wp,
};

static const struct udevice_id rtd_sd_ids[] = {
	{ .compatible = "realtek,rtd1296-sdmmc" },
	{ }
};

U_BOOT_DRIVER(rtd1296_sd) = {
	.name = "rtd1296_sd",
	.id = UCLASS_MMC,
	.of_match = rtd_sd_ids,
	.bind = rtd_sd_bind,
	.probe = rtd_sd_probe,
	.remove = rtd_sd_remove,
	.ops = &rtd_sd_ops,
	.priv_auto = sizeof(struct rtd_sd),
	.plat_auto = sizeof(struct rtd_sd_plat),
	.flags = DM_FLAG_ACTIVE_DMA | DM_FLAG_OS_PREPARE,
};
