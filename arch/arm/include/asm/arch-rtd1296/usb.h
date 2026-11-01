/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __RTD1296_USB_H
#define __RTD1296_USB_H

struct udevice;

int rtd1296_typec_get(struct udevice *host, struct udevice **port);
int rtd1296_typec_put(struct udevice *port);

#endif
