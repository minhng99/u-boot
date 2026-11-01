// SPDX-License-Identifier: GPL-2.0+
#include <fdt_support.h>
#include <fdtdec.h>
#include <init.h>
#include <dm/ofnode.h>

int board_init(void)
{
	return 0;
}

static int setup_psci(void *blob)
{
	static const char compatible[] = "arm,psci-1.0\0arm,psci-0.2";
	int node, ret, count = 0;

	node = fdt_find_or_add_subnode(blob, 0, "psci");
	if (node < 0)
		return node;
	ret = fdt_setprop(blob, node, "compatible", compatible, sizeof(compatible));
	if (ret)
		return ret;
	ret = fdt_setprop_string(blob, node, "method", "smc");
	if (ret)
		return ret;
	ret = fdt_setprop_string(blob, node, "status", "okay");
	if (ret)
		return ret;

	node = -1;
	for (;;) {
		node = fdt_node_offset_by_prop_value(blob, node, "device_type",
						     "cpu", sizeof("cpu"));
		if (node < 0)
			return node == -FDT_ERR_NOTFOUND && count ? 0 : node;
		ret = fdt_setprop_string(blob, node, "enable-method", "psci");
		if (ret)
			return ret;
		count++;
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	struct fdt_memory region;
	fdt_size_t size;
	int ret;
	ofnode node = ofnode_path("/reserved-memory/bl31@10000000");

	region.start = ofnode_get_addr_size_index(node, 0, &size);
	if (region.start == FDT_ADDR_T_NONE || !size ||
	    region.start > (phys_addr_t)-1 - (size - 1))
		return -EINVAL;
	region.end = region.start + size - 1;
	ret = setup_psci(blob);
	if (ret)
		return ret;
	return fdtdec_add_reserved_memory(blob, "bl31", &region, NULL, 0, NULL,
					 FDTDEC_RESERVED_MEMORY_NO_MAP);
}
