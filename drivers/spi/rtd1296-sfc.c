// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <regmap.h>
#include <spi.h>
#include <spi-mem.h>
#include <syscon.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <dm/read.h>
#include <linux/ioport.h>
#include <linux/mtd/spi-nor.h>

#define SFC_OPCODE	0x00
#define SFC_FORMAT	0x04
#define SFC_AUTO_WAIT	0x18
#define SFC_AUTO_WREN	0x1c
#define SFC_ADDR_MODE	0x28
#define SFC_DATA		BIT(4)
#define SFC_ADDRESS	BIT(3)
#define SFC_DUMMY	GENMASK(2, 0)
#define SFC_LIMIT	BIT(24)
#define SB2_SYNC	0x20

struct rtd_sfc {
	void __iomem *regs;
	void __iomem *aperture;
	struct regmap *sb2;
	size_t size;
};

static int rtd_sfc_sync(struct rtd_sfc *sfc)
{
	int ret;

	/* Publish controller configuration before flushing the SB2 bridge. */
	mb();
	ret = regmap_write(sfc->sb2, SB2_SYNC, 0);
	/* Complete the flush before issuing an aperture access. */
	mb();
	return ret;
}

static int rtd_sfc_command(struct rtd_sfc *sfc, u8 opcode, u32 format)
{
	writel(opcode, sfc->regs + SFC_OPCODE);
	writel(format, sfc->regs + SFC_FORMAT);
	return rtd_sfc_sync(sfc);
}

static bool rtd_sfc_supports_op(struct spi_slave *slave,
				const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(slave, op) ||
	    op->cmd.nbytes != 1 || op->cmd.buswidth != 1 || op->cmd.dtr ||
	    (op->addr.nbytes && (op->addr.nbytes != 3 ||
				op->addr.buswidth != 1 || op->addr.dtr)) ||
	    (op->dummy.nbytes && (op->dummy.buswidth != 1 || op->dummy.dtr)) ||
	    (op->data.nbytes && (op->data.buswidth != 1 || op->data.dtr)))
		return false;
	if (op->dummy.nbytes > SFC_DUMMY || op->addr.val >= SFC_LIMIT)
		return false;
	if (op->addr.nbytes || !op->data.nbytes)
		return true;
	if (op->data.dir == SPI_MEM_DATA_IN)
		return op->data.nbytes <= 8 - op->dummy.nbytes;
	return op->data.nbytes == 1 || op->data.nbytes == 2 ||
		op->data.nbytes == 4;
}

static int rtd_sfc_adjust_op_size(struct spi_slave *slave, struct spi_mem_op *op)
{
	if (op->addr.nbytes && op->data.dir == SPI_MEM_DATA_OUT &&
	    op->data.nbytes)
		op->data.nbytes = IS_ALIGNED(op->addr.val, 4) &&
			op->data.nbytes >= 4 ? 4 : 1;
	return 0;
}

static void rtd_sfc_read(struct rtd_sfc *sfc, const struct spi_mem_op *op)
{
	u8 *dst = op->data.buf.in;
	void __iomem *src = sfc->aperture + op->addr.val;
	size_t count = op->data.nbytes;

	while (count) {
		if (IS_ALIGNED((uintptr_t)src, 4) && count >= 4) {
			put_unaligned_le32(readl(src), dst);
			src += 4;
			dst += 4;
			count -= 4;
		} else {
			*dst++ = readb(src++);
			count--;
		}
	}
}

static int rtd_sfc_read_status(struct rtd_sfc *sfc,
			       const struct spi_mem_op *op, u32 format)
{
	u8 *dst = op->data.buf.in;
	u32 value;
	uint offset;
	int ret;

	for (offset = 0; offset < op->data.nbytes; offset += 4) {
		ret = rtd_sfc_command(sfc, op->cmd.opcode, format + offset);
		if (ret)
			return ret;
		value = readl(sfc->aperture);
		for (uint i = 0; i < min_t(uint, 4, op->data.nbytes - offset); i++)
			dst[offset + i] = value >> (i * 8);
	}
	return 0;
}

static int rtd_sfc_write(struct rtd_sfc *sfc, const struct spi_mem_op *op)
{
	const u8 *src = op->data.buf.out;
	void __iomem *dst = sfc->aperture + op->addr.val;

	if (!IS_ALIGNED(op->addr.val, op->data.nbytes))
		return -EINVAL;
	switch (op->data.nbytes) {
	case 1:
		writeb(*src, dst);
		break;
	case 2:
		writew(get_unaligned_le16(src), dst);
		break;
	case 4:
		writel(get_unaligned_le32(src), dst);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return rtd_sfc_sync(sfc);
}

static int rtd_sfc_exec_op(struct spi_slave *slave, const struct spi_mem_op *op)
{
	struct rtd_sfc *sfc = dev_get_priv(slave->dev->parent);
	u32 format = op->dummy.nbytes;
	int ret, restore;

	if (!rtd_sfc_supports_op(slave, op))
		return -EOPNOTSUPP;
	if (spi_chip_select(slave->dev))
		return -ENODEV;
	if (op->addr.val >= sfc->size ||
	    op->data.nbytes > sfc->size - op->addr.val)
		return -ERANGE;
	if (op->addr.nbytes)
		format |= SFC_ADDRESS;
	if (op->data.nbytes)
		format |= SFC_DATA;
	if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN &&
	    !op->addr.nbytes) {
		ret = rtd_sfc_read_status(sfc, op, format);
	} else {
		ret = rtd_sfc_command(sfc, op->cmd.opcode, format);
		if (ret)
			goto out;
		if (!op->data.nbytes) {
			readb(sfc->aperture + op->addr.val);
			ret = rtd_sfc_sync(sfc);
		} else if (op->data.dir == SPI_MEM_DATA_OUT) {
			ret = rtd_sfc_write(sfc, op);
		} else {
			rtd_sfc_read(sfc, op);
		}
	}
out:
	restore = rtd_sfc_command(sfc, SPINOR_OP_READ, SFC_ADDRESS | SFC_DATA);
	return ret ? ret : restore;
}

static int rtd_sfc_speed(struct udevice *dev, uint hz)
{
	/* Keep the clock divider and pad timing established at entry. */
	return hz ? 0 : -EINVAL;
}

static int rtd_sfc_mode(struct udevice *dev, uint mode)
{
	return mode & (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST |
		       SPI_3WIRE) ? -EOPNOTSUPP : 0;
}

static int rtd_sfc_probe(struct udevice *dev)
{
	struct rtd_sfc *sfc = dev_get_priv(dev);
	struct resource res;
	int ret;

	ret = dev_read_resource_byname(dev, "ctrl", &res);
	if (ret)
		return ret;
	if (resource_size(&res) < SFC_ADDR_MODE + sizeof(u32))
		return -EINVAL;
	ret = dev_read_resource_byname(dev, "flash", &res);
	if (ret)
		return ret;
	sfc->size = resource_size(&res);
	if (!sfc->size || sfc->size > SFC_LIMIT)
		return -EINVAL;
	sfc->regs = dev_remap_addr_name(dev, "ctrl");
	sfc->aperture = dev_remap_addr_name(dev, "flash");
	if (!sfc->regs || !sfc->aperture)
		return -EINVAL;
	sfc->sb2 = syscon_regmap_lookup_by_phandle(dev, "realtek,sb2");
	if (IS_ERR(sfc->sb2))
		return PTR_ERR(sfc->sb2);
	/* The NOR layer supplies write enable and bounded busy polling. */
	writel(SPINOR_OP_WREN, sfc->regs + SFC_AUTO_WREN);
	writel(SPINOR_OP_RDSR, sfc->regs + SFC_AUTO_WAIT);
	writel(0, sfc->regs + SFC_ADDR_MODE);
	return rtd_sfc_command(sfc, SPINOR_OP_READ, SFC_ADDRESS | SFC_DATA);
}

static const struct spi_controller_mem_ops rtd_sfc_mem_ops = {
	.supports_op = rtd_sfc_supports_op,
	.adjust_op_size = rtd_sfc_adjust_op_size,
	.exec_op = rtd_sfc_exec_op,
};

static const struct dm_spi_ops rtd_sfc_ops = {
	.set_speed = rtd_sfc_speed,
	.set_mode = rtd_sfc_mode,
	.mem_ops = &rtd_sfc_mem_ops,
};

static const struct udevice_id rtd_sfc_ids[] = {
	{ .compatible = "realtek,rtd1296-sfc" },
	{ }
};

U_BOOT_DRIVER(rtd1296_sfc) = {
	.name = "rtd1296_sfc",
	.id = UCLASS_SPI,
	.of_match = rtd_sfc_ids,
	.probe = rtd_sfc_probe,
	.ops = &rtd_sfc_ops,
	.priv_auto = sizeof(struct rtd_sfc),
};
