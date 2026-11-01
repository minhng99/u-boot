// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm/read.h>
#include <linux/ioport.h>

struct rtd_gpio_bank {
	u16 pins;
	u8 stride;
	u8 data_out;
	u8 data_in;
};

struct rtd_gpio {
	void __iomem *base;
	const struct rtd_gpio_bank *bank;
};

static void __iomem *rtd_gpio_reg(struct rtd_gpio *priv, uint offset, uint reg)
{
	return priv->base + reg + (offset / 32) * priv->bank->stride;
}

static int rtd_gpio_set(struct udevice *dev, uint offset, int value)
{
	struct rtd_gpio *priv = dev_get_priv(dev);
	u32 bit = BIT(offset % 32);

	if (offset >= priv->bank->pins)
		return -EINVAL;
	clrsetbits_le32(rtd_gpio_reg(priv, offset, priv->bank->data_out),
			bit, value ? bit : 0);
	return 0;
}

static int rtd_gpio_direction_input(struct udevice *dev, uint offset)
{
	struct rtd_gpio *priv = dev_get_priv(dev);

	if (offset >= priv->bank->pins)
		return -EINVAL;
	clrbits_le32(rtd_gpio_reg(priv, offset, 0), BIT(offset % 32));
	return 0;
}

static int rtd_gpio_direction_output(struct udevice *dev, uint offset, int value)
{
	struct rtd_gpio *priv = dev_get_priv(dev);
	int ret = rtd_gpio_set(dev, offset, value);

	if (ret)
		return ret;
	setbits_le32(rtd_gpio_reg(priv, offset, 0), BIT(offset % 32));
	return 0;
}

static int rtd_gpio_function(struct udevice *dev, uint offset)
{
	struct rtd_gpio *priv = dev_get_priv(dev);

	if (offset >= priv->bank->pins)
		return -EINVAL;
	return readl(rtd_gpio_reg(priv, offset, 0)) & BIT(offset % 32) ?
		GPIOF_OUTPUT : GPIOF_INPUT;
}

static int rtd_gpio_get(struct udevice *dev, uint offset)
{
	struct rtd_gpio *priv = dev_get_priv(dev);
	int function = rtd_gpio_function(dev, offset);
	u32 reg;

	if (function < 0)
		return function;
	reg = function == GPIOF_OUTPUT ? priv->bank->data_out : priv->bank->data_in;
	return !!(readl(rtd_gpio_reg(priv, offset, reg)) & BIT(offset % 32));
}

static int rtd_gpio_probe(struct udevice *dev)
{
	struct rtd_gpio *priv = dev_get_priv(dev);
	struct gpio_dev_priv *uc = dev_get_uclass_priv(dev);
	struct resource res;
	int ret;

	priv->bank = (const void *)dev_get_driver_data(dev);
	ret = dev_read_resource(dev, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < priv->bank->data_in + sizeof(u32) +
	    ((priv->bank->pins - 1) / 32) * priv->bank->stride)
		return -EINVAL;
	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;
	uc->gpio_count = priv->bank->pins;
	uc->bank_name = dev->name;
	return 0;
}

static const struct rtd_gpio_bank rtd1296_gpio_bank = { 101, 4, 0x10, 0x20 };
static const struct rtd_gpio_bank rtd1296_iso_bank = { 35, 0x18, 4, 8 };

static const struct udevice_id rtd_gpio_ids[] = {
	{ "realtek,rtd1296-gpio", (ulong)&rtd1296_gpio_bank },
	{ "realtek,rtd1296-iso-gpio", (ulong)&rtd1296_iso_bank },
	{ }
};

static const struct dm_gpio_ops rtd_gpio_ops = {
	.direction_input = rtd_gpio_direction_input,
	.direction_output = rtd_gpio_direction_output,
	.get_value = rtd_gpio_get,
	.set_value = rtd_gpio_set,
	.get_function = rtd_gpio_function,
	.xlate = gpio_xlate_offs_flags,
};

U_BOOT_DRIVER(rtd1296_gpio) = {
	.name = "rtd1296_gpio",
	.id = UCLASS_GPIO,
	.of_match = rtd_gpio_ids,
	.probe = rtd_gpio_probe,
	.ops = &rtd_gpio_ops,
	.priv_auto = sizeof(struct rtd_gpio),
};
