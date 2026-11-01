// SPDX-License-Identifier: GPL-2.0+
#include <dm.h>
#include <event.h>
#include <spi_flash.h>
#include <ubi_uboot.h>
#include <dm/device_compat.h>
#include <linux/list.h>

static bool rtd_flash_busy(struct mtd_info *mtd)
{
	struct mtd_info *part;

	if (mtd->usecount)
		return true;
	list_for_each_entry(part, &mtd->partitions, node) {
		if (rtd_flash_busy(part))
			return true;
	}
	return false;
}

static int rtd_flash_pre_remove(void *ctx, struct event *event)
{
	struct udevice *dev = event->data.dm.dev;
	struct spi_nor *nor;

	if (device_get_uclass_id(dev) != UCLASS_SPI_FLASH || !dev->parent ||
	    !device_is_compatible(dev->parent, "realtek,rtd1296-sfc"))
		return 0;
	nor = dev_get_uclass_priv(dev);
	return rtd_flash_busy(&nor->mtd) ? -EBUSY : 0;
}

EVENT_SPY_FULL(EVT_DM_PRE_REMOVE, rtd_flash_pre_remove);

void board_quiesce_devices(void)
{
	if (IS_ENABLED(CONFIG_CMD_UBI))
		ubi_detach();
}
