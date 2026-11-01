/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __RTD1296_PCIE_H
#define __RTD1296_PCIE_H

#include <linux/types.h>

struct udevice;

int rtd1296_pcie_bar0_write(struct udevice *endpoint, u32 offset, u32 value);

#endif
