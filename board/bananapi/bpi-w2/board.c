// SPDX-License-Identifier: GPL-2.0+
#include <fdt_support.h>
#include <fdtdec.h>
#include <init.h>
#include <dm/ofnode.h>

int board_init(void)
{
	return 0;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	struct fdt_memory region;
	fdt_size_t size;
	ofnode node = ofnode_path("/reserved-memory/bl31@10000000");

	region.start = ofnode_get_addr_size_index(node, 0, &size);
	if (region.start == FDT_ADDR_T_NONE || !size ||
	    region.start > (phys_addr_t)-1 - (size - 1))
		return -EINVAL;
	region.end = region.start + size - 1;
	return fdtdec_add_reserved_memory(blob, "bl31", &region, NULL, 0, NULL,
					 FDTDEC_RESERVED_MEMORY_NO_MAP);
}
