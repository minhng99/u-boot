// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <event.h>
#include <power-domain-uclass.h>
#include <reset.h>

struct rtd_reset_domain {
	struct reset_ctl *reset;
	bool active;
};

static int rtd_domain_request(struct power_domain *domain)
{
	return domain->id ? -EINVAL : 0;
}

static int rtd_domain_on(struct power_domain *domain)
{
	struct rtd_reset_domain *priv = dev_get_priv(domain->dev);
	int ret = reset_deassert(priv->reset);

	if (!ret)
		priv->active = true;
	return ret;
}

static int rtd_domain_off(struct power_domain *domain)
{
	struct rtd_reset_domain *priv = dev_get_priv(domain->dev);
	int ret = reset_assert(priv->reset);

	if (!ret)
		priv->active = false;
	return ret;
}

static int rtd_domain_pre_remove(void *ctx, struct event *event)
{
	struct udevice *dev = event->data.dm.dev;
	struct rtd_reset_domain *priv;

	if (device_get_uclass_id(dev) != UCLASS_POWER_DOMAIN ||
	    !device_is_compatible(dev, "realtek,rtd1296-reset-domain"))
		return 0;
	priv = dev_get_priv(dev);
	return priv->active ? -EPROBE_DEFER : 0;
}

EVENT_SPY_FULL(EVT_DM_PRE_REMOVE, rtd_domain_pre_remove);

static int rtd_domain_probe(struct udevice *dev)
{
	struct rtd_reset_domain *priv = dev_get_priv(dev);
	struct power_domain_plat *plat = dev_get_uclass_plat(dev);

	priv->reset = devm_reset_control_get(dev, "domain");
	if (IS_ERR(priv->reset))
		return PTR_ERR(priv->reset);
	plat->subdomains = 1;
	return 0;
}

static const struct power_domain_ops rtd_domain_ops = {
	.request = rtd_domain_request,
	.on = rtd_domain_on,
	.off = rtd_domain_off,
};

static const struct udevice_id rtd_domain_ids[] = {
	{ .compatible = "realtek,rtd1296-reset-domain" },
	{ }
};

U_BOOT_DRIVER(rtd1296_reset_domain) = {
	.name = "rtd1296_reset_domain",
	.id = UCLASS_POWER_DOMAIN,
	.of_match = rtd_domain_ids,
	.probe = rtd_domain_probe,
	.ops = &rtd_domain_ops,
	.priv_auto = sizeof(struct rtd_reset_domain),
};
