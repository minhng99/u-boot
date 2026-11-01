// SPDX-License-Identifier: GPL-2.0+
#include <errno.h>
#include <fdtdec.h>
#include <init.h>
#include <asm/armv8/mmu.h>
#include <asm/global_data.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

#define RTD_DEVICE_MEMORY (PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) | \
			  PTE_BLOCK_NON_SHARE | PTE_BLOCK_PXN | PTE_BLOCK_UXN)

static struct mm_region rtd1296_map[] = {
	{
		.size = SZ_2G,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) | PTE_BLOCK_INNER_SHARE,
	}, {
		.virt = 0x88000000,
		.phys = 0x88000000,
		.size = SZ_32M,
		.attrs = RTD_DEVICE_MEMORY,
	}, {
		.virt = 0x98000000,
		.phys = 0x98000000,
		.size = SZ_2M,
		.attrs = RTD_DEVICE_MEMORY,
	}, {
		.virt = 0xff000000,
		.phys = 0xff000000,
		.size = SZ_16M,
		.attrs = RTD_DEVICE_MEMORY,
	}, { }
};

struct mm_region *mem_map = rtd1296_map;

int dram_init(void)
{
	int ret = fdtdec_setup_mem_size_base();

	if (ret)
		return ret;
	if (gd->ram_base || !gd->ram_size || gd->ram_size > SZ_2G)
		return -EINVAL;
	rtd1296_map[0].size = gd->ram_size;
	return 0;
}

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}
