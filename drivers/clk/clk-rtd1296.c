// SPDX-License-Identifier: GPL-2.0+
#include <clk-uclass.h>
#include <dm.h>
#include <limits.h>
#include <asm/io.h>
#include <dm/read.h>
#include <linux/ioport.h>

struct rtd_gates {
	void __iomem *base;
	uint count;
	uint refs[64];
};

static int rtd_gate_request(struct clk *clk)
{
	struct rtd_gates *priv = dev_get_priv(clk->dev);

	return (clk->id & CLK_ID_MSK) < priv->count ? 0 : -EINVAL;
}

static int rtd_gate_enable(struct clk *clk)
{
	struct rtd_gates *priv = dev_get_priv(clk->dev);
	uint id = clk->id & CLK_ID_MSK;
	void __iomem *reg;
	int ret = rtd_gate_request(clk);

	if (ret)
		return ret;
	if (priv->refs[id] == UINT_MAX)
		return -EOVERFLOW;
	if (!priv->refs[id]++) {
		reg = priv->base + (id / 32) * sizeof(u32);
		setbits_le32(reg, BIT(id % 32));
		readl(reg);
	}
	return 0;
}

static int rtd_gate_disable(struct clk *clk)
{
	struct rtd_gates *priv = dev_get_priv(clk->dev);
	uint id = clk->id & CLK_ID_MSK;
	void __iomem *reg;
	int ret = rtd_gate_request(clk);

	if (ret)
		return ret;
	if (priv->refs[id] && !--priv->refs[id]) {
		reg = priv->base + (id / 32) * sizeof(u32);
		clrbits_le32(reg, BIT(id % 32));
		readl(reg);
	}
	return 0;
}

static int rtd_gates_probe(struct udevice *dev)
{
	struct rtd_gates *priv = dev_get_priv(dev);
	struct resource res;
	int ret = dev_read_resource(dev, 0, &res);

	if (ret)
		return ret;
	if (resource_size(&res) != 4 && resource_size(&res) != 8)
		return -EINVAL;
	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;
	priv->count = resource_size(&res) * 8;
	return 0;
}

static int rtd_gates_remove(struct udevice *dev)
{
	struct rtd_gates *priv = dev_get_priv(dev);

	for (uint id = 0; id < priv->count; id++) {
		if (priv->refs[id])
			return -EBUSY;
	}
	return 0;
}

static const struct clk_ops rtd_gate_ops = {
	.request = rtd_gate_request,
	.enable = rtd_gate_enable,
	.disable = rtd_gate_disable,
};

static const struct udevice_id rtd_gates_ids[] = {
	{ .compatible = "realtek,rtd1296-clk-gates" },
	{ }
};

U_BOOT_DRIVER(rtd1296_gates) = {
	.name = "rtd1296_gates",
	.id = UCLASS_CLK,
	.of_match = rtd_gates_ids,
	.probe = rtd_gates_probe,
	.remove = rtd_gates_remove,
	.ops = &rtd_gate_ops,
	.priv_auto = sizeof(struct rtd_gates),
};
