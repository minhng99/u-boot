// SPDX-License-Identifier: GPL-2.0+
#include <clk.h>
#include <dm.h>
#include <pci.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <asm/arch/pcie.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm/read.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/log2.h>
#include <linux/sizes.h>

#define PCIE_CONTROL	0xc00
#define PCIE_REQUEST	0xc14
#define PCIE_MDIO	0xc1c
#define PCIE_CFG_START	0xc38
#define PCIE_CFG_ENABLE	0xc3c
#define PCIE_CFG_STATUS	0xc40
#define PCIE_CFG_ADDRESS	0xc44
#define PCIE_CFG_WRITE	0xc48
#define PCIE_CFG_READ	0xc4c
#define PCIE_IO_STATUS	0xc74
#define PCIE_IO_TIMEOUT	0xc78
#define PCIE_DLLP	0xc7c
#define PCIE_LINK	0xcb4
#define PCIE_BASE	0xcfc
#define PCIE_MASK	0xd00
#define PCIE_TARGET	0xd04
#define PCIE_TRANSLATE	BIT(20)
#define PCIE_MEMORY	BIT(19)
#define PCIE_MDIO_ENABLE	BIT(18)
#define PCIE_MDIO_RESET_N	BIT(17)
#define PCIE_INDIRECT_CFG	BIT(5)
#define PCIE_DIRECT_CFG	BIT(4)
#define PCIE_LTSSM	BIT(1)
#define PCIE_LINK_UP	BIT(11)
#define PCIE_MDIO_BUSY	BIT(7)
#define PCIE_CFG_DONE	BIT(0)
#define PCIE_CFG_ERROR	BIT(1)
#define PCIE_INTX_DISABLE BIT(10)
#define PCIE_WINDOW	SZ_2K
#define PCIE_BAR_SPACE	SZ_16M
#define PCIE_MEMORY_MODE	(PCIE_TRANSLATE | PCIE_MEMORY | PCIE_MDIO_ENABLE | \
			 PCIE_MDIO_RESET_N | PCIE_LTSSM)

struct rtd_pcie_step {
	u8 reg;
	u16 value;
};

static const struct rtd_pcie_step rtd_gen1_phy[] = {
	{ 0x08, 0x3181 }, { 0x11, 0xb200 }, { 0x00, 0x0010 },
};

static const struct rtd_pcie_step rtd_gen2_phy[] = {
	{ 0x03, 0x27e9 }, { 0x04, 0x52f5 }, { 0x05, 0xead7 },
	{ 0x06, 0x000c }, { 0x0a, 0xa653 }, { 0x01, 0xa84a },
	{ 0x20, 0xc466 }, { 0x2b, 0xb803 },
	{ 0x43, 0x27e9 }, { 0x44, 0x52f5 }, { 0x45, 0xead7 },
	{ 0x46, 0x000c }, { 0x4a, 0xa653 }, { 0x41, 0xa84a },
	{ 0x60, 0xd477 }, { 0x6b, 0xa803 },
	{ 0x5a, 0x0122 }, { 0x09, 0x521c }, { 0x09, 0x501c },
	{ 0x09, 0x521c }, { 0x24, 0x0828 }, { 0x64, 0x0828 },
};

struct rtd_pcie_variant {
	const struct rtd_pcie_step *phy;
	uint count;
	u32 lane_mask;
};

static const struct rtd_pcie_variant rtd_gen1 = {
	.phy = rtd_gen1_phy,
	.count = ARRAY_SIZE(rtd_gen1_phy),
	.lane_mask = GENMASK(3, 0),
};

static const struct rtd_pcie_variant rtd_gen2 = {
	.phy = rtd_gen2_phy,
	.count = ARRAY_SIZE(rtd_gen2_phy),
	.lane_mask = GENMASK(19, 16),
};

struct rtd_pcie {
	void __iomem *regs;
	void __iomem *window;
	struct regmap *sds;
	struct clk clock;
	struct reset_ctl_bulk *resets;
	struct gpio_desc *perst;
	const struct rtd_pcie_variant *variant;
	u32 prefix;
	u32 bar_base[6];
	u32 bar_size[6];
	bool clock_on;
	bool accessible;
	bool in_access;
};

static bool rtd_pcie_link(struct rtd_pcie *priv)
{
	return priv->accessible && (readl(priv->regs + PCIE_LINK) & PCIE_LINK_UP);
}

static void rtd_pcie_restore(struct rtd_pcie *priv)
{
	writel(priv->prefix, priv->regs + PCIE_TARGET);
	writel(PCIE_MEMORY_MODE, priv->regs + PCIE_CONTROL);
	readl(priv->regs + PCIE_CONTROL);
	priv->in_access = false;
}

static int rtd_pcie_config(struct rtd_pcie *priv, uint offset,
			   enum pci_size_t size, ulong *value, bool write)
{
	u32 enables, status;
	uint shift;
	int ret;

	if (size < PCI_SIZE_8 || size > PCI_SIZE_32 ||
	    offset >= PCI_CFG_SPACE_EXP_SIZE || (offset & ((1U << size) - 1)))
		return -EINVAL;
	if (!rtd_pcie_link(priv))
		return -ENODEV;
	if (priv->in_access)
		return -EBUSY;
	priv->in_access = true;
	shift = (offset & 3) * 8;
	enables = ((1U << (1U << size)) - 1) << (offset & 3);
	writel(PCIE_MDIO_ENABLE | PCIE_LTSSM | PCIE_INDIRECT_CFG,
	       priv->regs + PCIE_CONTROL);
	writel(write ? 0x12 : 0x10, priv->regs + PCIE_REQUEST);
	writel(PCIE_CFG_DONE | PCIE_CFG_ERROR, priv->regs + PCIE_CFG_STATUS);
	writel(offset & ~3, priv->regs + PCIE_CFG_ADDRESS);
	if (write)
		writel(*value << shift, priv->regs + PCIE_CFG_WRITE);
	writel((enables << 4) | BIT(1) | write, priv->regs + PCIE_CFG_ENABLE);
	writel(1, priv->regs + PCIE_CFG_START);
	ret = readl_poll_timeout(priv->regs + PCIE_CFG_STATUS, status,
				 status & PCIE_CFG_DONE, 100000);
	if (!ret && (status & PCIE_CFG_ERROR))
		ret = -EIO;
	if (!ret && !write)
		*value = pci_conv_32_to_size(readl(priv->regs + PCIE_CFG_READ), offset, size);
	writel(PCIE_CFG_DONE | PCIE_CFG_ERROR, priv->regs + PCIE_CFG_STATUS);
	rtd_pcie_restore(priv);
	return ret;
}

static bool rtd_pcie_function(const struct udevice *bus, pci_dev_t bdf)
{
	return (int)PCI_BUS(bdf) == dev_seq(bus) && !PCI_DEV(bdf) && !PCI_FUNC(bdf);
}

static int rtd_pcie_read_config(const struct udevice *bus, pci_dev_t bdf,
				uint offset, ulong *value, enum pci_size_t size)
{
	struct rtd_pcie *priv = dev_get_priv(bus);

	if (size < PCI_SIZE_8 || size > PCI_SIZE_32)
		return -EINVAL;
	*value = pci_get_ff(size);
	if (!rtd_pcie_function(bus, bdf) || !rtd_pcie_link(priv))
		return 0;
	return rtd_pcie_config(priv, offset, size, value, false);
}

static int rtd_pcie_write_config(struct udevice *bus, pci_dev_t bdf, uint offset,
				 ulong value, enum pci_size_t size)
{
	struct rtd_pcie *priv = dev_get_priv(bus);

	if (size < PCI_SIZE_8 || size > PCI_SIZE_32)
		return -EINVAL;
	if (!rtd_pcie_function(bus, bdf) || !rtd_pcie_link(priv))
		return 0;
	return rtd_pcie_config(priv, offset, size, &value, true);
}

static int rtd_pcie_cfg_read(struct rtd_pcie *priv, uint offset, u32 *value)
{
	ulong data;
	int ret = rtd_pcie_config(priv, offset, PCI_SIZE_32, &data, false);

	if (!ret)
		*value = data;
	return ret;
}

static int rtd_pcie_cfg_write(struct rtd_pcie *priv, uint offset, u32 value)
{
	ulong data = value;

	return rtd_pcie_config(priv, offset, PCI_SIZE_32, &data, true);
}

static int rtd_pcie_allocate_bars(struct rtd_pcie *priv)
{
	u64 next = priv->prefix, limit = next + PCIE_BAR_SPACE;
	u64 mask, size, address;
	u32 original, high, low_mask, high_mask;
	ulong command;
	bool wide;
	uint offset;
	int ret;

	ret = rtd_pcie_config(priv, PCI_COMMAND, PCI_SIZE_16, &command, false);
	if (ret)
		return ret;
	command &= ~(PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	ret = rtd_pcie_config(priv, PCI_COMMAND, PCI_SIZE_16, &command, true);
	if (ret)
		return ret;
	for (uint bar = 0; bar < ARRAY_SIZE(priv->bar_base); bar++) {
		offset = PCI_BASE_ADDRESS_0 + bar * sizeof(u32);
		ret = rtd_pcie_cfg_read(priv, offset, &original);
		if (ret)
			return ret;
		wide = (original & PCI_BASE_ADDRESS_MEM_TYPE_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64;
		if ((original & PCI_BASE_ADDRESS_SPACE_IO) || (wide && bar == 5) ||
		    (!wide && (original & PCI_BASE_ADDRESS_MEM_TYPE_MASK)))
			return -EOPNOTSUPP;
		high = 0;
		high_mask = ~0U;
		if (wide) {
			ret = rtd_pcie_cfg_read(priv, offset + sizeof(u32), &high);
			if (ret)
				return ret;
		}
		ret = rtd_pcie_cfg_write(priv, offset, ~0U);
		if (!ret && wide)
			ret = rtd_pcie_cfg_write(priv, offset + sizeof(u32), ~0U);
		if (!ret)
			ret = rtd_pcie_cfg_read(priv, offset, &low_mask);
		if (!ret && wide)
			ret = rtd_pcie_cfg_read(priv, offset + sizeof(u32), &high_mask);
		if (ret)
			return ret;
		ret = rtd_pcie_cfg_write(priv, offset, original);
		if (!ret && wide)
			ret = rtd_pcie_cfg_write(priv, offset + sizeof(u32), high);
		if (ret)
			return ret;
		if (!low_mask)
			continue;
		mask = (u64)high_mask << 32 | (low_mask & PCI_BASE_ADDRESS_MEM_MASK);
		size = ~mask + 1;
		if (!size || size > PCIE_BAR_SPACE || !is_power_of_2(size))
			return -ENOSPC;
		address = ALIGN(next, size);
		if (address > limit || size > limit - address)
			return -ENOSPC;
		ret = rtd_pcie_cfg_write(priv, offset,
					 address | (original & ~PCI_BASE_ADDRESS_MEM_MASK));
		if (!ret && wide)
			ret = rtd_pcie_cfg_write(priv, offset + sizeof(u32), 0);
		if (ret)
			return ret;
		priv->bar_base[bar] = address;
		priv->bar_size[bar] = size;
		next = address + size;
		if (wide)
			bar++;
	}
	command |= PCI_COMMAND_MEMORY | PCIE_INTX_DISABLE;
	return rtd_pcie_config(priv, PCI_COMMAND, PCI_SIZE_16, &command, true);
}

int rtd1296_pcie_bar0_write(struct udevice *endpoint, u32 offset, u32 value)
{
	struct udevice *bus = pci_get_controller(endpoint);
	struct rtd_pcie *priv;
	u32 status, dllp;

	if (!bus || (!device_is_compatible(bus, "realtek,rtd1296-pcie-gen1") &&
		     !device_is_compatible(bus, "realtek,rtd1296-pcie-gen2")))
		return -ENODEV;
	priv = dev_get_priv(bus);
	if ((offset & 3) || priv->bar_size[0] < sizeof(u32) ||
	    offset > priv->bar_size[0] - sizeof(u32))
		return -ERANGE;
	if (!rtd_pcie_link(priv))
		return -ENODEV;
	if (priv->in_access)
		return -EBUSY;
	priv->in_access = true;
	writel(BIT(0), priv->regs + PCIE_IO_STATUS);
	writel(0x1f, priv->regs + PCIE_DLLP);
	writel((priv->bar_base[0] + offset) & ~(PCIE_WINDOW - 1),
	       priv->regs + PCIE_TARGET);
	readl(priv->regs + PCIE_TARGET);
	writel(value, priv->window + (offset & (PCIE_WINDOW - 1)));
	status = readl(priv->regs + PCIE_IO_STATUS);
	dllp = readl(priv->regs + PCIE_DLLP);
	rtd_pcie_restore(priv);
	return (status & BIT(0)) || (dllp & 0x1f) ? -EIO : 0;
}

static int rtd_pcie_mdio_wait(struct rtd_pcie *priv)
{
	u32 value;

	return readl_poll_timeout(priv->regs + PCIE_MDIO, value,
				  !(value & PCIE_MDIO_BUSY), 50000);
}

static int rtd_pcie_phy_init(struct rtd_pcie *priv)
{
	const struct rtd_pcie_step *step;
	int ret;

	writel(3, priv->regs + PCIE_MDIO);
	udelay(1000);
	writel(0, priv->regs + PCIE_MDIO);
	udelay(1000);
	for (uint i = 0; i < priv->variant->count; i++) {
		step = &priv->variant->phy[i];
		ret = rtd_pcie_mdio_wait(priv);
		if (ret)
			return ret;
		writel((u32)step->value << 16 | (u32)step->reg << 8 | BIT(0),
		       priv->regs + PCIE_MDIO);
		udelay(1000);
		ret = rtd_pcie_mdio_wait(priv);
		if (ret)
			return ret;
	}
	return 0;
}

static int rtd_pcie_stop(struct udevice *dev)
{
	struct rtd_pcie *priv = dev_get_priv(dev);
	int ret;

	priv->accessible = false;
	ret = dm_gpio_set_value(priv->perst, 1);
	if (ret)
		return ret;
	udelay(1000);
	ret = reset_assert_bulk(priv->resets);
	if (ret)
		return ret;
	if (priv->clock_on) {
		ret = clk_disable(&priv->clock);
		if (ret)
			return ret;
		priv->clock_on = false;
	}
	return 0;
}

static int rtd_pcie_regions(struct udevice *dev, phys_addr_t window)
{
	struct rtd_pcie *priv = dev_get_priv(dev);
	struct pci_controller *hose = dev_get_uclass_priv(dev);
	const struct pci_region *region;
	bool found = false;

	for (int i = 0; i < hose->region_count; i++) {
		region = &hose->regions[i];
		if (region->flags & PCI_REGION_SYS_MEMORY)
			continue;
		if (region->flags != PCI_REGION_MEM || region->size != PCIE_WINDOW ||
		    region->phys_start != window || (region->bus_start & (PCIE_BAR_SPACE - 1)) ||
		    region->bus_start > U32_MAX - PCIE_BAR_SPACE + 1 || found)
			return -EINVAL;
		priv->prefix = region->bus_start;
		found = true;
	}
	return found ? 0 : -EINVAL;
}

static int rtd_pcie_probe(struct udevice *dev)
{
	struct rtd_pcie *priv = dev_get_priv(dev);
	struct regmap *sb2;
	struct resource control, window;
	u32 revision, value;
	int ret, err;

	ret = dev_read_resource(dev, 0, &control);
	if (ret)
		return ret;
	ret = dev_read_resource(dev, 1, &window);
	if (ret)
		return ret;
	if (resource_size(&control) < PCIE_TARGET + sizeof(u32) ||
	    resource_size(&window) < PCIE_WINDOW || window.start > U32_MAX ||
	    (window.start & (PCIE_WINDOW - 1)))
		return -EINVAL;
	priv->regs = dev_remap_addr_index(dev, 0);
	priv->window = dev_remap_addr_index(dev, 1);
	if (!priv->regs || !priv->window)
		return -EINVAL;
	priv->variant = (const void *)dev_get_driver_data(dev);
	ret = rtd_pcie_regions(dev, window.start);
	if (ret)
		return ret;
	priv->sds = syscon_regmap_lookup_by_phandle(dev, "realtek,sds");
	if (IS_ERR(priv->sds))
		return PTR_ERR(priv->sds);
	sb2 = syscon_regmap_lookup_by_phandle(dev, "realtek,sb2");
	if (IS_ERR(sb2))
		return PTR_ERR(sb2);
	ret = regmap_read(sb2, 0x204, &revision);
	if (ret)
		return ret;
	ret = clk_get_by_index(dev, 0, &priv->clock);
	if (ret)
		return ret;
	priv->resets = devm_reset_bulk_get(dev);
	if (IS_ERR(priv->resets))
		return PTR_ERR(priv->resets);
	if (priv->resets->count != 7)
		return -EINVAL;
	priv->perst = devm_gpiod_get(dev, "reset", GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (IS_ERR(priv->perst))
		return PTR_ERR(priv->perst);
	ret = reset_assert_bulk(priv->resets);
	if (ret)
		return ret;
	ret = clk_enable(&priv->clock);
	if (ret)
		return ret;
	priv->clock_on = true;
	if (revision != 0x20000) {
		ret = regmap_update_bits(priv->sds, 0x14, GENMASK(2, 0), 1);
		if (ret)
			goto stop;
		ret = regmap_update_bits(priv->sds, 0, priv->variant->lane_mask, 0);
		if (ret)
			goto stop;
		ret = regmap_update_bits(priv->sds, 8, 0xff, 0x51);
		if (ret)
			goto stop;
	}
	ret = reset_deassert_bulk(priv->resets);
	if (ret)
		goto stop;
	priv->accessible = true;
	writel(PCIE_TRANSLATE | PCIE_MDIO_ENABLE | PCIE_DIRECT_CFG,
	       priv->regs + PCIE_CONTROL);
	readl(priv->regs + PCIE_CONTROL);
	ret = rtd_pcie_phy_init(priv);
	if (ret)
		goto stop;
	udelay(100000);
	ret = dm_gpio_set_value(priv->perst, 0);
	if (ret)
		goto stop;
	writel(PCIE_MDIO_ENABLE | PCIE_DIRECT_CFG | PCIE_LTSSM,
	       priv->regs + PCIE_CONTROL);
	udelay(50000);
	writel(BIT(16) | BIT(8) | BIT(5), priv->regs + 0x710);
	ret = readl_poll_timeout(priv->regs + PCIE_LINK, value,
				 value & PCIE_LINK_UP, 1000000);
	if (!ret)
		udelay(100000);
	dev_info(dev, "link %s\n", ret ? "down" : "up");
	writel(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER, priv->regs + PCI_COMMAND);
	writel(window.start, priv->regs + PCIE_BASE);
	writel(~(PCIE_WINDOW - 1), priv->regs + PCIE_MASK);
	writel(BIT(24) | BIT(0), priv->regs + PCIE_IO_TIMEOUT);
	rtd_pcie_restore(priv);
	if (!rtd_pcie_link(priv))
		return 0;
	ret = rtd_pcie_allocate_bars(priv);
	if (!ret)
		return 0;
stop:
	err = rtd_pcie_stop(dev);
	return err ? err : ret;
}

static const struct dm_pci_ops rtd_pcie_ops = {
	.read_config = rtd_pcie_read_config,
	.write_config = rtd_pcie_write_config,
};

static const struct udevice_id rtd_pcie_ids[] = {
	{ .compatible = "realtek,rtd1296-pcie-gen1", .data = (ulong)&rtd_gen1 },
	{ .compatible = "realtek,rtd1296-pcie-gen2", .data = (ulong)&rtd_gen2 },
	{ }
};

U_BOOT_DRIVER(rtd1296_pcie) = {
	.name = "rtd1296_pcie",
	.id = UCLASS_PCI,
	.of_match = rtd_pcie_ids,
	.probe = rtd_pcie_probe,
	.remove = rtd_pcie_stop,
	.ops = &rtd_pcie_ops,
	.priv_auto = sizeof(struct rtd_pcie),
	.flags = DM_FLAG_OS_PREPARE | DM_FLAG_ACTIVE_DMA,
};
