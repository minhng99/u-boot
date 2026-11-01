// SPDX-License-Identifier: GPL-2.0+
#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <malloc.h>
#include <miiphy.h>
#include <net.h>
#include <phy.h>
#include <phys2bus.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <dm/device_compat.h>
#include <dm/read.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>

#define GMAC_MAC		0x00
#define GMAC_MULTICAST	0x08
#define GMAC_TX_RING	0x20
#define GMAC_COMMAND	0x37
#define GMAC_TX_POLL	0x38
#define GMAC_IRQ_MASK	0x3c
#define GMAC_IRQ_STATUS	0x3e
#define GMAC_TX_CONFIG	0x40
#define GMAC_RX_CONFIG	0x44
#define GMAC_UNLOCK	0x50
#define GMAC_MULTI_IRQ	0x5c
#define GMAC_MDIO	0x60
#define GMAC_PHY_STATUS	0x6c
#define GMAC_OCP		0xb0
#define GMAC_RX_MAX	0xda
#define GMAC_RX_RING	0xe4
#define GMAC_TX_EARLY	0xec
#define GMAC_MISC	0xf0
#define GMAC_RESET	BIT(4)
#define GMAC_RX_ENABLE	BIT(3)
#define GMAC_TX_ENABLE	BIT(2)
#define GMAC_MDIO_FLAG	BIT(31)
#define GMAC_INTERFACE	0xea34
#define GMAC_RX_KEEP	0xff7e1880
#define DESC_OWN		BIT(31)
#define DESC_END		BIT(30)
#define DESC_FIRST	BIT(29)
#define DESC_LAST	BIT(28)
#define DESC_RX_ERROR	BIT(21)
#define DESC_LENGTH	GENMASK(13, 0)
#define RTD_RX_COUNT	16
#define RTD_BUFFER_SIZE	2048
#define RTD_FRAME_MAX	1536
#define RTD_RING_ALIGN	256
#define ISO_GPHY_CONTROL	0x60
#define ISO_GMAC_CONTROL	0xfc0

struct rtd_desc {
	__le32 status;
	__le32 vlan;
	__le32 address;
	__le32 address_hi;
};

struct rtd_eth_plat {
	struct eth_pdata eth;
	void *rings;
};

struct rtd_eth {
	void __iomem *regs;
	struct regmap *iso;
	struct clk_bulk clocks;
	struct reset_ctl *mac_reset;
	struct reset_ctl *phy_reset;
	struct mii_dev *mdio;
	struct phy_device *phy;
	struct rtd_desc *tx;
	struct rtd_desc *rx;
	u8 *tx_buffer;
	u8 *rx_buffers;
	uint enabled;
	uint rx_index;
	bool accessible;
	bool running;
	bool rx_held;
};

static int rtd_clocks_off(struct rtd_eth *priv)
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

static int rtd_clocks_on(struct rtd_eth *priv)
{
	int ret;

	while (priv->enabled < priv->clocks.count) {
		ret = clk_enable(&priv->clocks.clks[priv->enabled]);
		if (ret)
			return ret;
		priv->enabled++;
	}
	return 0;
}

static int rtd_power_off(struct rtd_eth *priv)
{
	int ret;

	ret = reset_assert(priv->mac_reset);
	if (ret)
		return ret;
	priv->accessible = false;
	udelay(10);
	ret = reset_assert(priv->phy_reset);
	return ret ? ret : rtd_clocks_off(priv);
}

static int rtd_power_on(struct rtd_eth *priv)
{
	int ret;

	ret = reset_assert(priv->mac_reset);
	if (ret)
		return ret;
	ret = reset_assert(priv->phy_reset);
	if (ret)
		return ret;
	ret = reset_deassert(priv->phy_reset);
	if (ret)
		return ret;
	udelay(200);
	ret = regmap_update_bits(priv->iso, ISO_GPHY_CONTROL, BIT(1), 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(priv->iso, ISO_GMAC_CONTROL, BIT(0), 0);
	if (ret)
		return ret;
	udelay(200);
	ret = rtd_clocks_on(priv);
	if (ret)
		return ret;
	ret = rtd_clocks_off(priv);
	if (ret)
		return ret;
	ret = reset_deassert(priv->mac_reset);
	if (ret)
		return ret;
	ret = rtd_clocks_on(priv);
	if (ret)
		return ret;
	mdelay(100);
	priv->accessible = true;
	return 0;
}

static int rtd_mac_reset(struct rtd_eth *priv)
{
	u8 command;

	writew(0, priv->regs + GMAC_IRQ_MASK);
	writeb(GMAC_RESET, priv->regs + GMAC_COMMAND);
	return readb_poll_timeout(priv->regs + GMAC_COMMAND, command,
				 !(command & GMAC_RESET), 100000);
}

static void rtd_select_phy(struct rtd_eth *priv)
{
	u32 address = (GMAC_INTERFACE / 2) << 16;
	u16 value;

	writel(address, priv->regs + GMAC_OCP);
	value = readl(priv->regs + GMAC_OCP);
	value = (value & ~GENMASK(1, 0)) | BIT(1);
	writel(BIT(31) | address | value, priv->regs + GMAC_OCP);
	clrbits_le32(priv->regs + GMAC_MISC, BIT(19));
}

static int rtd_mdio_read(struct mii_dev *bus, int addr, int devad, int reg)
{
	struct rtd_eth *priv = bus->priv;
	u32 value;
	int ret;

	if (!priv->accessible)
		return -ENODEV;
	if (addr || devad != MDIO_DEVAD_NONE || reg < 0 || reg > 31)
		return -EOPNOTSUPP;
	writel(reg << 16, priv->regs + GMAC_MDIO);
	ret = readl_poll_timeout(priv->regs + GMAC_MDIO, value,
				 value & GMAC_MDIO_FLAG, 50000);
	if (ret)
		return ret;
	return value & 0xffff;
}

static int rtd_mdio_write(struct mii_dev *bus, int addr, int devad, int reg,
			  u16 value)
{
	struct rtd_eth *priv = bus->priv;
	u32 status;

	if (!priv->accessible)
		return -ENODEV;
	if (addr || devad != MDIO_DEVAD_NONE || reg < 0 || reg > 31)
		return -EOPNOTSUPP;
	writel(GMAC_MDIO_FLAG | (reg << 16) | value, priv->regs + GMAC_MDIO);
	return readl_poll_timeout(priv->regs + GMAC_MDIO, status,
				 !(status & GMAC_MDIO_FLAG), 50000);
}

static void rtd_rx_recycle(struct rtd_eth *priv)
{
	u8 *buffer = priv->rx_buffers + priv->rx_index * RTD_BUFFER_SIZE;
	u32 status = DESC_OWN | RTD_BUFFER_SIZE;

	if (priv->rx_index == RTD_RX_COUNT - 1)
		status |= DESC_END;
	invalidate_dcache_range((ulong)buffer, (ulong)buffer + RTD_BUFFER_SIZE);
	/* Finish buffer maintenance before returning ownership to DMA. */
	mb();
	WRITE_ONCE(priv->rx[priv->rx_index].status, cpu_to_le32(status));
	priv->rx_index = (priv->rx_index + 1) % RTD_RX_COUNT;
	priv->rx_held = false;
}

static void rtd_init_rings(struct udevice *dev)
{
	struct rtd_eth *priv = dev_get_priv(dev);
	u64 address;

	memset(priv->tx, 0, sizeof(*priv->tx));
	memset(priv->rx, 0, RTD_RX_COUNT * sizeof(*priv->rx));
	address = dev_phys_to_bus(dev, virt_to_phys(priv->tx_buffer));
	priv->tx->address = cpu_to_le32(lower_32_bits(address));
	priv->tx->address_hi = cpu_to_le32(upper_32_bits(address));
	priv->tx->status = cpu_to_le32(DESC_END);
	for (uint i = 0; i < RTD_RX_COUNT; i++) {
		address = dev_phys_to_bus(dev, virt_to_phys(priv->rx_buffers +
						 i * RTD_BUFFER_SIZE));
		priv->rx[i].address = cpu_to_le32(lower_32_bits(address));
		priv->rx[i].address_hi = cpu_to_le32(upper_32_bits(address));
	}
	priv->rx_held = false;
	priv->rx_index = 0;
	for (uint i = 0; i < RTD_RX_COUNT; i++)
		rtd_rx_recycle(priv);
}

static int rtd_write_hwaddr(struct udevice *dev)
{
	struct rtd_eth *priv = dev_get_priv(dev);
	struct eth_pdata *plat = dev_get_plat(dev);

	if (!priv->accessible)
		return -EIO;
	writeb(0xc0, priv->regs + GMAC_UNLOCK);
	writel(get_unaligned_le32(plat->enetaddr), priv->regs + GMAC_MAC);
	writew(get_unaligned_le16(plat->enetaddr + 4), priv->regs + GMAC_MAC + 4);
	writeb(0, priv->regs + GMAC_UNLOCK);
	return 0;
}

static int rtd_start(struct udevice *dev)
{
	struct rtd_eth *priv = dev_get_priv(dev);
	u64 tx, rx;
	u32 value;
	int ret;

	if (!priv->accessible)
		return -ENETDOWN;
	priv->running = false;
	ret = rtd_mac_reset(priv);
	if (ret)
		return ret;
	rtd_select_phy(priv);
	ret = phy_startup(priv->phy);
	if (ret)
		return ret;
	if (!priv->phy->link)
		return -ENETDOWN;
	rtd_init_rings(dev);
	ret = rtd_write_hwaddr(dev);
	if (ret)
		return ret;
	tx = dev_phys_to_bus(dev, virt_to_phys(priv->tx));
	rx = dev_phys_to_bus(dev, virt_to_phys(priv->rx));
	writeb(0xc0, priv->regs + GMAC_UNLOCK);
	writeb(0x3f, priv->regs + GMAC_TX_EARLY);
	writew(RTD_BUFFER_SIZE, priv->regs + GMAC_RX_MAX);
	value = readl(priv->regs + GMAC_RX_CONFIG) & GMAC_RX_KEEP;
	value |= (7 << 13) | (6 << 8) | BIT(1) | BIT(2) | BIT(3);
	writel(value, priv->regs + GMAC_RX_CONFIG);
	writel((3 << 24) | (3 << 8), priv->regs + GMAC_TX_CONFIG);
	writel(lower_32_bits(tx), priv->regs + GMAC_TX_RING);
	writel(upper_32_bits(tx), priv->regs + GMAC_TX_RING + 4);
	writel(lower_32_bits(rx), priv->regs + GMAC_RX_RING);
	writel(upper_32_bits(rx), priv->regs + GMAC_RX_RING + 4);
	writel(~0U, priv->regs + GMAC_MULTICAST);
	writel(~0U, priv->regs + GMAC_MULTICAST + 4);
	writew(0xffff, priv->regs + GMAC_IRQ_STATUS);
	clrbits_le16(priv->regs + GMAC_MULTI_IRQ, 0x0fff);
	/* Publish complete rings before starting the receive and transmit engines. */
	mb();
	writeb(GMAC_RX_ENABLE | GMAC_TX_ENABLE, priv->regs + GMAC_COMMAND);
	writeb(0, priv->regs + GMAC_UNLOCK);
	priv->running = true;
	return 0;
}

static void rtd_stop(struct udevice *dev)
{
	struct rtd_eth *priv = dev_get_priv(dev);

	priv->running = false;
	if (priv->accessible && rtd_mac_reset(priv))
		dev_warn(dev, "MAC reset timed out\n");
	priv->rx_held = false;
}

static int rtd_send(struct udevice *dev, void *packet, int length)
{
	struct rtd_eth *priv = dev_get_priv(dev);
	u32 status;
	uint size;

	if (!priv->running)
		return -ENETDOWN;
	if (!packet || length <= 0 || length > RTD_FRAME_MAX)
		return -EMSGSIZE;
	if (le32_to_cpu(READ_ONCE(priv->tx->status)) & DESC_OWN)
		return -EBUSY;
	size = max_t(uint, length, ETH_ZLEN);
	memcpy(priv->tx_buffer, packet, length);
	memset(priv->tx_buffer + length, 0, size - length);
	flush_dcache_range((ulong)priv->tx_buffer,
			   (ulong)priv->tx_buffer + ALIGN(size, ARCH_DMA_MINALIGN));
	/* Complete payload writes before publishing the transmit descriptor. */
	mb();
	WRITE_ONCE(priv->tx->status,
		   cpu_to_le32(DESC_OWN | DESC_END | DESC_FIRST | DESC_LAST | size));
	/* The descriptor must reach memory before the polling request. */
	mb();
	writeb(BIT(6), priv->regs + GMAC_TX_POLL);
	return readl_poll_timeout(&priv->tx->status, status,
				 !(status & DESC_OWN), 1000000);
}

static int rtd_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct rtd_eth *priv = dev_get_priv(dev);
	u32 status, length;
	u8 *buffer;

	if (!priv->running || priv->rx_held)
		return -EAGAIN;
	writew(readw(priv->regs + GMAC_IRQ_STATUS), priv->regs + GMAC_IRQ_STATUS);
	for (uint i = 0; i < RTD_RX_COUNT; i++) {
		status = le32_to_cpu(READ_ONCE(priv->rx[priv->rx_index].status));
		if (status & DESC_OWN)
			return -EAGAIN;
		/* Observe DMA completion before accessing the received payload. */
		mb();
		length = status & DESC_LENGTH;
		if ((status & DESC_RX_ERROR) || length < ETH_HLEN + ETH_FCS_LEN ||
		    length > RTD_FRAME_MAX + ETH_FCS_LEN ||
		    (status & (DESC_FIRST | DESC_LAST)) != (DESC_FIRST | DESC_LAST)) {
			rtd_rx_recycle(priv);
			continue;
		}
		buffer = priv->rx_buffers + priv->rx_index * RTD_BUFFER_SIZE;
		invalidate_dcache_range((ulong)buffer, (ulong)buffer + RTD_BUFFER_SIZE);
		*packetp = buffer;
		priv->rx_held = true;
		return length - ETH_FCS_LEN;
	}
	return -EAGAIN;
}

static int rtd_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct rtd_eth *priv = dev_get_priv(dev);

	if (!priv->rx_held || packet != priv->rx_buffers +
	    priv->rx_index * RTD_BUFFER_SIZE)
		return -EINVAL;
	rtd_rx_recycle(priv);
	return 0;
}

static void rtd_free_buffers(struct rtd_eth *priv)
{
	free(priv->tx_buffer);
	free(priv->rx_buffers);
	priv->tx_buffer = NULL;
	priv->rx_buffers = NULL;
}

static int rtd_eth_probe(struct udevice *dev)
{
	struct rtd_eth_plat *plat = dev_get_plat(dev);
	struct rtd_eth *priv = dev_get_priv(dev);
	struct resource res;
	int ret, err;

	if (dev_read_phy_mode(dev) != PHY_INTERFACE_MODE_INTERNAL)
		return -EOPNOTSUPP;
	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < 0x1000)
		return -EINVAL;
	plat->eth.iobase = res.start;
	priv->regs = dev_remap_addr(dev);
	if (!priv->regs)
		return -EINVAL;
	priv->iso = syscon_regmap_lookup_by_phandle(dev, "realtek,iso");
	if (IS_ERR(priv->iso))
		return PTR_ERR(priv->iso);
	priv->mac_reset = devm_reset_control_get(dev, "gmac");
	if (IS_ERR(priv->mac_reset))
		return PTR_ERR(priv->mac_reset);
	priv->phy_reset = devm_reset_control_get(dev, "gphy");
	if (IS_ERR(priv->phy_reset))
		return PTR_ERR(priv->phy_reset);
	ret = clk_get_bulk(dev, &priv->clocks);
	if (ret)
		return ret;
	if (priv->clocks.count != 2)
		return -EINVAL;
	ret = rtd_power_on(priv);
	if (ret)
		goto power_off;
	ret = rtd_mac_reset(priv);
	if (ret)
		goto power_off;
	rtd_select_phy(priv);
	priv->mdio = mdio_alloc();
	if (!priv->mdio) {
		ret = -ENOMEM;
		goto power_off;
	}
	strlcpy(priv->mdio->name, dev->name, sizeof(priv->mdio->name));
	priv->mdio->priv = priv;
	priv->mdio->read = rtd_mdio_read;
	priv->mdio->write = rtd_mdio_write;
	ret = mdio_register(priv->mdio);
	if (ret)
		goto free_mdio;
	priv->phy = phy_connect(priv->mdio, 0, dev, PHY_INTERFACE_MODE_INTERNAL);
	if (!priv->phy) {
		ret = -ENODEV;
		goto unregister;
	}
	ret = phy_config(priv->phy);
	if (ret)
		goto free_phy;
	if (!plat->rings)
		plat->rings = (void *)noncached_alloc(RTD_RING_ALIGN +
			ALIGN(RTD_RX_COUNT * sizeof(struct rtd_desc), RTD_RING_ALIGN),
			RTD_RING_ALIGN);
	priv->tx_buffer = memalign(ARCH_DMA_MINALIGN, RTD_BUFFER_SIZE);
	priv->rx_buffers = memalign(ARCH_DMA_MINALIGN, RTD_RX_COUNT * RTD_BUFFER_SIZE);
	if (!plat->rings || !priv->tx_buffer || !priv->rx_buffers) {
		ret = -ENOMEM;
		rtd_free_buffers(priv);
		goto free_phy;
	}
	priv->tx = plat->rings;
	priv->rx = plat->rings + RTD_RING_ALIGN;
	return 0;

free_phy:
	free(priv->phy);
unregister:
	mdio_unregister(priv->mdio);
free_mdio:
	mdio_free(priv->mdio);
power_off:
	err = rtd_power_off(priv);
	return err ? err : ret;
}

static int rtd_eth_remove(struct udevice *dev)
{
	struct rtd_eth *priv = dev_get_priv(dev);
	int ret;

	priv->running = false;
	if (priv->accessible) {
		ret = rtd_mac_reset(priv);
		if (ret)
			return ret;
		priv->rx_held = false;
		phy_shutdown(priv->phy);
	}
	ret = rtd_power_off(priv);
	if (ret)
		return ret;
	rtd_free_buffers(priv);
	free(priv->phy);
	mdio_unregister(priv->mdio);
	mdio_free(priv->mdio);
	return 0;
}

static const struct eth_ops rtd_eth_ops = {
	.start = rtd_start,
	.stop = rtd_stop,
	.send = rtd_send,
	.recv = rtd_recv,
	.free_pkt = rtd_free_pkt,
	.write_hwaddr = rtd_write_hwaddr,
};

static const struct udevice_id rtd_eth_ids[] = {
	{ .compatible = "realtek,rtd1296-gmac" },
	{ }
};

U_BOOT_DRIVER(rtd1296_eth) = {
	.name = "rtd1296_eth",
	.id = UCLASS_ETH,
	.of_match = rtd_eth_ids,
	.probe = rtd_eth_probe,
	.remove = rtd_eth_remove,
	.ops = &rtd_eth_ops,
	.priv_auto = sizeof(struct rtd_eth),
	.plat_auto = sizeof(struct rtd_eth_plat),
	.flags = DM_FLAG_ACTIVE_DMA | DM_FLAG_OS_PREPARE,
};
