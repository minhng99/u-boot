// SPDX-License-Identifier: GPL-2.0+
#include <ahci.h>
#include <blk.h>
#include <bootdev.h>
#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <fis.h>
#include <generic-phy.h>
#include <libata.h>
#include <malloc.h>
#include <memalign.h>
#include <part.h>
#include <reset.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/read.h>
#include <dm/uclass.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/log2.h>
#include <linux/sizes.h>
#include <power/regulator.h>

#define RTD_SATA_PORTS	2
#define RTD_SATA_FIS	1024
#define RTD_SATA_TABLE	1280
#define RTD_SATA_DATA	2048
#define RTD_SATA_BUFFER	SZ_64K
#define RTD_SATA_MEMORY	(RTD_SATA_DATA + RTD_SATA_BUFFER)
#define RTD_SATA_READY_US	20000000

struct rtd_sata_port {
	struct phy phy;
	struct udevice *supply;
	ofnode node;
	u8 *memory;
	u8 flush;
	bool initialized;
	bool powered;
	bool supplied;
	bool ready;
	bool failed;
	bool write_cache;
};

struct rtd_sata {
	void __iomem *regs;
	struct clk_bulk clocks;
	struct reset_ctl_bulk *resets;
	struct rtd_sata_port ports[RTD_SATA_PORTS];
	u32 mask;
	uint enabled;
	bool accessible;
};

static void __iomem *rtd_sata_port_regs(struct rtd_sata *priv, uint port)
{
	return priv->regs + 0x100 + port * 0x80;
}

static int rtd_sata_stop(struct rtd_sata *priv, uint port)
{
	void __iomem *regs = rtd_sata_port_regs(priv, port);
	u32 status;
	int ret;

	writel(0, regs + PORT_IRQ_MASK);
	clrbits_le32(regs + PORT_CMD, PORT_CMD_START);
	ret = readl_poll_timeout(regs + PORT_CMD, status,
				 !(status & PORT_CMD_LIST_ON), 500000);
	if (ret)
		return ret;
	clrbits_le32(regs + PORT_CMD, PORT_CMD_FIS_RX);
	return readl_poll_timeout(regs + PORT_CMD, status,
				  !(status & PORT_CMD_FIS_ON), 500000);
}

static int rtd_sata_exec(struct rtd_sata *priv, uint index,
			 const struct sata_fis_h2d *fis, uint bytes, bool write)
{
	struct rtd_sata_port *port;
	void __iomem *regs;
	struct ahci_cmd_hdr *header;
	struct ahci_sg *prd;
	phys_addr_t address;
	u32 status, irq;
	int ret;

	if (!priv->accessible || index >= RTD_SATA_PORTS)
		return -ENODEV;
	port = &priv->ports[index];
	if (!port->memory || !port->ready || port->failed)
		return -ENODEV;
	regs = rtd_sata_port_regs(priv, index);
	header = (void *)port->memory;
	prd = (void *)(port->memory + RTD_SATA_TABLE + AHCI_CMD_TBL_HDR);
	if (bytes > RTD_SATA_BUFFER)
		return -E2BIG;
	if (readl(regs + PORT_CMD_ISSUE) & BIT(0))
		return -EBUSY;
	ret = readl_poll_timeout(regs + PORT_TFDATA, status,
				 !(status & (ATA_BUSY | ATA_DRQ)), 1000000);
	if (ret)
		goto fail;
	memset(header, 0, sizeof(*header));
	memset(port->memory + RTD_SATA_TABLE, 0, 256);
	memcpy(port->memory + RTD_SATA_TABLE, fis, sizeof(*fis));
	header->opts = cpu_to_le32(sizeof(*fis) / sizeof(u32) |
				   (write ? AHCI_CMD_WRITE : 0) | (bytes ? BIT(16) : 0));
	address = virt_to_phys(port->memory + RTD_SATA_TABLE);
	header->tbl_addr = cpu_to_le32(lower_32_bits(address));
	header->tbl_addr_hi = cpu_to_le32(upper_32_bits(address));
	if (bytes) {
		address = virt_to_phys(port->memory + RTD_SATA_DATA);
		prd->addr = cpu_to_le32(lower_32_bits(address));
		prd->addr_hi = cpu_to_le32(upper_32_bits(address));
		prd->flags_size = cpu_to_le32(bytes - 1);
		flush_dcache_range((ulong)port->memory + RTD_SATA_DATA,
				   (ulong)port->memory + RTD_SATA_DATA +
				   ALIGN(bytes, ARCH_DMA_MINALIGN));
	}
	flush_dcache_range((ulong)header, (ulong)header + ARCH_DMA_MINALIGN);
	flush_dcache_range((ulong)port->memory + RTD_SATA_TABLE,
			   (ulong)port->memory + RTD_SATA_TABLE + 256);
	writel(~0U, regs + PORT_SCR_ERR);
	writel(~0U, regs + PORT_IRQ_STAT);
	/* Publish command storage before granting the engine ownership. */
	mb();
	writel(BIT(0), regs + PORT_CMD_ISSUE);
	ret = readl_poll_timeout(regs + PORT_CMD_ISSUE, status, !(status & BIT(0)),
				 10000000);
	if (ret)
		goto fail;
	irq = readl(regs + PORT_IRQ_STAT);
	if ((irq & (PORT_IRQ_FATAL)) || (readl(regs + PORT_TFDATA) & (ATA_ERR | ATA_DF))) {
		ret = -EIO;
		goto fail;
	}
	invalidate_dcache_range((ulong)header, (ulong)header + ARCH_DMA_MINALIGN);
	if (bytes && le32_to_cpu(header->status) != bytes) {
		ret = -EIO;
		goto fail;
	}
	if (bytes && !write)
		invalidate_dcache_range((ulong)port->memory + RTD_SATA_DATA,
					(ulong)port->memory + RTD_SATA_DATA +
					ALIGN(bytes, ARCH_DMA_MINALIGN));
	writel(irq, regs + PORT_IRQ_STAT);
	return 0;
fail:
	port->failed = true;
	rtd_sata_stop(priv, index);
	return ret;
}

static int rtd_sata_fis(struct sata_fis_h2d *fis, lbaint_t start, uint count,
			bool lba48, bool write)
{
	u64 limit = lba48 ? BIT_ULL(48) : BIT_ULL(28);
	uint max_count = lba48 ? 65536 : 256;

	if (!count || count > max_count || start >= limit || count > limit - start)
		return -ERANGE;
	memset(fis, 0, sizeof(*fis));
	fis->fis_type = 0x27;
	fis->pm_port_c = BIT(7);
	fis->device = BIT(6);
	fis->lba_low = start;
	fis->lba_mid = start >> 8;
	fis->lba_high = start >> 16;
	fis->sector_count = count;
	if (lba48) {
		fis->command = write ? ATA_CMD_WRITE_EXT : ATA_CMD_READ_EXT;
		fis->lba_low_exp = start >> 24;
		fis->lba_mid_exp = start >> 32;
		fis->lba_high_exp = start >> 40;
		fis->sector_count_exp = count >> 8;
	} else {
		fis->command = write ? ATA_CMD_WRITE : ATA_CMD_READ;
		fis->device |= (start >> 24) & 0xf;
	}
	return 0;
}

static ulong rtd_sata_rw(struct udevice *dev, lbaint_t start, lbaint_t count,
			 void *buffer, bool write)
{
	struct rtd_sata *priv = dev_get_priv(dev->parent);
	struct blk_desc *desc = dev_get_uclass_plat(dev);
	struct rtd_sata_port *port;
	struct sata_fis_h2d fis;
	lbaint_t done = 0;
	u8 *data;
	uint blocks, bytes;
	int ret;

	if (!priv->accessible || desc->target >= RTD_SATA_PORTS)
		return 0;
	port = &priv->ports[desc->target];
	if (!port->memory || start >= desc->lba || count > desc->lba - start || !desc->blksz ||
	    (write && port->write_cache && !port->flush))
		return 0;
	data = port->memory + RTD_SATA_DATA;
	while (done < count) {
		blocks = min_t(lbaint_t, count - done, RTD_SATA_BUFFER / desc->blksz);
		if (!blocks)
			break;
		bytes = blocks * desc->blksz;
		ret = rtd_sata_fis(&fis, start + done, blocks, desc->lba48, write);
		if (ret)
			break;
		if (write)
			memcpy(data, buffer, bytes);
		ret = rtd_sata_exec(priv, desc->target, &fis, bytes, write);
		if (ret)
			break;
		if (!write)
			memcpy(buffer, data, bytes);
		buffer += bytes;
		done += blocks;
	}
	if (write && done) {
		if (done != count)
			return 0;
		if (port->write_cache) {
			memset(&fis, 0, sizeof(fis));
			fis.fis_type = 0x27;
			fis.pm_port_c = BIT(7);
			fis.command = port->flush;
			if (rtd_sata_exec(priv, desc->target, &fis, 0, false))
				return 0;
		}
	}
	return done;
}

static ulong rtd_sata_read(struct udevice *dev, lbaint_t start, lbaint_t count,
			   void *buffer)
{
	return rtd_sata_rw(dev, start, count, buffer, false);
}

static ulong rtd_sata_write(struct udevice *dev, lbaint_t start, lbaint_t count,
			    const void *buffer)
{
	return rtd_sata_rw(dev, start, count, (void *)buffer, true);
}

static const struct blk_ops rtd_sata_blk_ops = {
	.read = rtd_sata_read,
	.write = rtd_sata_write,
};

U_BOOT_DRIVER(rtd1296_sata_disk) = {
	.name = "rtd1296_sata_disk",
	.id = UCLASS_BLK,
	.ops = &rtd_sata_blk_ops,
};

static int rtd_sata_start(struct rtd_sata *priv, uint index)
{
	struct rtd_sata_port *port = &priv->ports[index];
	void __iomem *regs = rtd_sata_port_regs(priv, index);
	phys_addr_t address;
	u32 status;
	int ret;

	port->ready = false;
	port->failed = false;
	ret = rtd_sata_stop(priv, index);
	if (ret)
		return ret;
	memset(port->memory, 0, RTD_SATA_DATA);
	flush_dcache_range((ulong)port->memory, (ulong)port->memory + RTD_SATA_DATA);
	address = virt_to_phys(port->memory);
	writel(lower_32_bits(address), regs + PORT_LST_ADDR);
	writel(upper_32_bits(address), regs + PORT_LST_ADDR_HI);
	address += RTD_SATA_FIS;
	writel(lower_32_bits(address), regs + PORT_FIS_ADDR);
	writel(upper_32_bits(address), regs + PORT_FIS_ADDR_HI);
	writel(0, regs + PORT_IRQ_MASK);
	writel(~0U, regs + PORT_SCR_ERR);
	writel(~0U, regs + PORT_IRQ_STAT);
	clrsetbits_le32(regs + PORT_CMD, GENMASK(31, 28), PORT_CMD_FIS_RX |
		       PORT_CMD_POWER_ON | PORT_CMD_SPIN_UP | PORT_CMD_ICC_ACTIVE);
	clrsetbits_le32(regs + PORT_SCR_CTL, GENMASK(3, 0), 1);
	udelay(1000);
	clrbits_le32(regs + PORT_SCR_CTL, GENMASK(3, 0));
	ret = readl_poll_timeout(regs + PORT_SCR_STAT, status,
				 (status & GENMASK(3, 0)) == 3, 1000000);
	if (ret)
		return 0;
	setbits_le32(regs + PORT_CMD, PORT_CMD_START);
	ret = readl_poll_timeout(regs + PORT_TFDATA, status,
				 !(status & (ATA_BUSY | ATA_DRQ)), RTD_SATA_READY_US);
	if (ret)
		return ret;
	if (ata_dev_classify(readl(regs + PORT_SIG)) != ATA_DEV_ATA)
		return 0;
	port->ready = true;
	return 0;
}

static int rtd_sata_reset(struct udevice *dev)
{
	struct rtd_sata *priv = dev_get_priv(dev);
	u32 cap, ports, status;
	int ret;

	if (!priv->accessible)
		return -EIO;
	for (uint i = 0; i < RTD_SATA_PORTS; i++) {
		priv->ports[i].ready = false;
		ret = rtd_sata_stop(priv, i);
		if (ret)
			return ret;
	}
	cap = readl(priv->regs + HOST_CAP);
	ports = readl(priv->regs + HOST_PORTS_IMPL) | priv->mask;
	if ((cap & GENMASK(4, 0)) != RTD_SATA_PORTS - 1)
		return -ENODEV;
	writel(HOST_AHCI_EN, priv->regs + HOST_CTL);
	writel(HOST_AHCI_EN | HOST_RESET, priv->regs + HOST_CTL);
	ret = readl_poll_timeout(priv->regs + HOST_CTL, status,
				 !(status & HOST_RESET), 1000000);
	if (ret)
		return ret;
	writel(cap, priv->regs + HOST_CAP);
	writel(ports, priv->regs + HOST_PORTS_IMPL);
	writel(HOST_AHCI_EN, priv->regs + HOST_CTL);
	writel(~0U, priv->regs + HOST_IRQ_STAT);
	for (uint i = 0; i < RTD_SATA_PORTS; i++) {
		if (!(priv->mask & BIT(i)))
			continue;
		ret = rtd_sata_start(priv, i);
		if (ret)
			return ret;
	}
	return 0;
}

static int rtd_sata_sector_size(const u16 *id, uint *size)
{
	u32 words;

	*size = 512;
	if ((id[106] & (GENMASK(15, 14) | BIT(12))) != (BIT(14) | BIT(12)))
		return 0;
	words = ata_id_u32(id, 117);
	if (words < 256 || words > 2048 || !is_power_of_2(words))
		return -EOPNOTSUPP;
	*size = words * sizeof(u16);
	return 0;
}

static int rtd_sata_identify(struct rtd_sata *priv, uint index,
			     struct blk_desc *desc)
{
	struct rtd_sata_port *port = &priv->ports[index];
	struct sata_fis_h2d fis = { .fis_type = 0x27, .pm_port_c = BIT(7),
				  .command = ATA_CMD_ID_ATA };
	u16 id[256];
	u64 sectors;
	uint size;
	int ret;

	ret = rtd_sata_exec(priv, index, &fis, sizeof(id), false);
	if (ret)
		return ret;
	memcpy(id, port->memory + RTD_SATA_DATA, sizeof(id));
	for (uint i = 0; i < ARRAY_SIZE(id); i++)
		id[i] = le16_to_cpu(id[i]);
	if (!ata_id_has_lba(id) || !(id[49] & BIT(8)))
		return -EOPNOTSUPP;
	sectors = ata_id_n_sectors(id);
	if (!sectors || sectors > BIT_ULL(48))
		return -ERANGE;
	ret = rtd_sata_sector_size(id, &size);
	if (ret)
		return ret;
	desc->target = index;
	desc->lba48 = ata_id_has_lba48(id);
	if (!desc->lba48 && sectors > BIT_ULL(28))
		return -ERANGE;
	desc->lba = sectors;
	desc->blksz = size;
	desc->log2blksz = ilog2(size);
	desc->type = DEV_TYPE_HARDDISK;
	desc->removable = ata_id_removeable(id) != 0;
	ata_id_c_string(id, (u8 *)desc->vendor, ATA_ID_PROD, sizeof(desc->vendor));
	ata_id_c_string(id, (u8 *)desc->product, ATA_ID_SERNO, sizeof(desc->product));
	ata_id_c_string(id, (u8 *)desc->revision, ATA_ID_FW_REV, sizeof(desc->revision));
	port->write_cache = ata_id_wcache_enabled(id);
	port->flush = 0;
	if (desc->lba48 && ata_id_has_flush_ext(id))
		port->flush = ATA_CMD_FLUSH_EXT;
	else if (ata_id_has_flush(id))
		port->flush = ATA_CMD_FLUSH;
	return 0;
}

static int rtd_sata_scan(struct udevice *dev)
{
	struct rtd_sata *priv = dev_get_priv(dev);
	struct udevice *child;
	struct blk_desc *desc;
	bool found;
	char name[8];
	int ret;

	if (!priv->accessible)
		return -EIO;
	for (uint i = 0; i < RTD_SATA_PORTS; i++) {
		if (!priv->ports[i].ready || priv->ports[i].failed)
			continue;
		found = false;
		device_foreach_child(child, dev) {
			if (device_get_uclass_id(child) != UCLASS_BLK)
				continue;
			desc = dev_get_uclass_plat(child);
			if (desc->target == i)
				found = true;
		}
		if (found)
			continue;
		snprintf(name, sizeof(name), "port%u", i);
		ret = blk_create_devicef(dev, "rtd1296_sata_disk", name, UCLASS_AHCI,
					 -1, 512, 0, &child);
		if (ret)
			return ret;
		desc = dev_get_uclass_plat(child);
		ret = rtd_sata_identify(priv, i, desc);
		if (ret) {
			device_unbind(child);
			return ret;
		}
		ret = blk_probe_or_unbind(child);
		if (ret)
			return ret;
		ret = bootdev_setup_for_sibling_blk(child, "sata_bootdev");
		if (ret)
			return ret;
	}
	return 0;
}

static int rtd_sata_status(struct udevice *dev, int index)
{
	struct rtd_sata *priv = dev_get_priv(dev);

	if (index < 0 || index >= RTD_SATA_PORTS || !priv->accessible)
		return -EINVAL;
	return (readl(rtd_sata_port_regs(priv, index) + PORT_SCR_STAT) & 0xf) == 3 ?
		0 : -ENXIO;
}

static int rtd_sata_disable(struct rtd_sata *priv)
{
	struct rtd_sata_port *port;
	int ret;

	if (priv->accessible) {
		for (uint i = 0; i < RTD_SATA_PORTS; i++) {
			ret = rtd_sata_stop(priv, i);
			if (ret)
				return ret;
		}
		priv->accessible = false;
	}
	ret = reset_assert_bulk(priv->resets);
	if (ret)
		return ret;
	for (uint i = RTD_SATA_PORTS; i; i--) {
		port = &priv->ports[i - 1];
		if (port->powered) {
			ret = generic_phy_power_off(&port->phy);
			if (ret)
				return ret;
			port->powered = false;
		}
		if (port->initialized) {
			ret = generic_phy_exit(&port->phy);
			if (ret)
				return ret;
			port->initialized = false;
		}
	}
	while (priv->enabled) {
		ret = clk_disable(&priv->clocks.clks[priv->enabled - 1]);
		if (ret)
			return ret;
		priv->enabled--;
	}
	for (uint i = RTD_SATA_PORTS; i; i--) {
		port = &priv->ports[i - 1];
		if (!port->supplied)
			continue;
		ret = regulator_set_enable_if_allowed(port->supply, false);
		if (ret)
			return ret;
		port->supplied = false;
	}
	for (uint i = 0; i < RTD_SATA_PORTS; i++) {
		free(priv->ports[i].memory);
		priv->ports[i].memory = NULL;
	}
	return 0;
}

static int rtd_sata_get_ports(struct udevice *dev)
{
	struct rtd_sata *priv = dev_get_priv(dev);
	struct rtd_sata_port *port;
	ofnode node;
	u32 index, phandle;
	int ret;

	dev_for_each_subnode(node, dev) {
		if (!ofnode_is_enabled(node))
			continue;
		ret = ofnode_read_u32(node, "reg", &index);
		if (ret || index >= RTD_SATA_PORTS || (priv->mask & BIT(index)))
			return -EINVAL;
		priv->mask |= BIT(index);
		port = &priv->ports[index];
		port->node = node;
		ret = generic_phy_get_by_index_nodev(node, 0, &port->phy);
		if (ret)
			return ret;
		if (!ofnode_read_prop(node, "target-supply", NULL))
			continue;
		ret = ofnode_read_u32(node, "target-supply", &phandle);
		if (ret)
			return ret;
		ret = uclass_get_device_by_phandle_id(UCLASS_REGULATOR, phandle, &port->supply);
		if (ret)
			return ret;
	}
	return priv->mask ? 0 : -ENODEV;
}

static int rtd_sata_probe(struct udevice *dev)
{
	struct rtd_sata *priv = dev_get_priv(dev);
	struct rtd_sata_port *port;
	struct resource res;
	int ret, err;

	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < 0x200)
		return -EINVAL;
	priv->regs = dev_remap_addr(dev);
	if (!priv->regs)
		return -EINVAL;
	priv->resets = devm_reset_bulk_get(dev);
	if (IS_ERR(priv->resets))
		return PTR_ERR(priv->resets);
	if (priv->resets->count != 4)
		return -EINVAL;
	ret = clk_get_bulk(dev, &priv->clocks);
	if (ret)
		return ret;
	if (priv->clocks.count != 2)
		return -EINVAL;
	ret = rtd_sata_get_ports(dev);
	if (ret)
		return ret;
	ret = reset_assert_bulk(priv->resets);
	if (ret)
		return ret;
	for (uint i = 0; i < RTD_SATA_PORTS; i++) {
		if (!(priv->mask & BIT(i)))
			continue;
		port = &priv->ports[i];
		if (port->supply) {
			ret = regulator_set_enable_if_allowed(port->supply, true);
			if (ret)
				goto disable;
			port->supplied = true;
		}
		port->memory = memalign(1024, RTD_SATA_MEMORY);
		if (!port->memory) {
			ret = -ENOMEM;
			goto disable;
		}
		if (virt_to_phys(port->memory) > 0xffffffffULL - RTD_SATA_MEMORY + 1) {
			ret = -ERANGE;
			goto disable;
		}
	}
	while (priv->enabled < priv->clocks.count) {
		ret = clk_enable(&priv->clocks.clks[priv->enabled]);
		if (ret)
			goto disable;
		priv->enabled++;
	}
	ret = reset_deassert_bulk(priv->resets);
	if (ret)
		goto disable;
	priv->accessible = true;
	for (uint i = 0; i < RTD_SATA_PORTS; i++) {
		if (!(priv->mask & BIT(i)))
			continue;
		port = &priv->ports[i];
		ret = generic_phy_init(&port->phy);
		if (ret)
			goto disable;
		port->initialized = true;
	}
	for (uint i = 0; i < RTD_SATA_PORTS; i++) {
		if (!(priv->mask & BIT(i)))
			continue;
		port = &priv->ports[i];
		ret = generic_phy_power_on(&port->phy);
		if (ret)
			goto disable;
		port->powered = true;
	}
	ret = rtd_sata_reset(dev);
	if (!ret)
		return 0;
disable:
	err = rtd_sata_disable(priv);
	return err ? err : ret;
}

static int rtd_sata_remove(struct udevice *dev)
{
	return rtd_sata_disable(dev_get_priv(dev));
}

static const struct ahci_ops rtd_sata_ops = {
	.reset = rtd_sata_reset,
	.scan = rtd_sata_scan,
	.port_status = rtd_sata_status,
};

static const struct udevice_id rtd_sata_ids[] = {
	{ .compatible = "realtek,rtd1296-ahci" },
	{ }
};

U_BOOT_DRIVER(rtd1296_sata) = {
	.name = "rtd1296_sata",
	.id = UCLASS_AHCI,
	.of_match = rtd_sata_ids,
	.probe = rtd_sata_probe,
	.remove = rtd_sata_remove,
	.ops = &rtd_sata_ops,
	.priv_auto = sizeof(struct rtd_sata),
	.flags = DM_FLAG_ACTIVE_DMA | DM_FLAG_OS_PREPARE,
};
