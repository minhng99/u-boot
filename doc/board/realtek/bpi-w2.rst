.. SPDX-License-Identifier: GPL-2.0+

Banana Pi BPI-W2
================

The Banana Pi BPI-W2 uses an RTD1296 with four Cortex-A53 cores and 2 GiB
of DRAM. U-Boot runs as BL33 at non-secure EL2. Boot firmware initializes
DRAM and supplies the SMC PSCI interface before entering U-Boot at
``0x00200000``.

Building
--------

Use an AArch64 cross compiler::

    make CROSS_COMPILE=aarch64-none-linux-gnu- bananapi_w2_defconfig
    make CROSS_COMPILE=aarch64-none-linux-gnu-

The resulting ``u-boot.bin`` is the BL33 payload.

SPI NOR layout
--------------

The 16 MiB NOR contains boot firmware, two firmware slots and a UBI
partition for the environment.

.. list-table::
   :header-rows: 1

   * - Partition
     - Offset
     - Size
   * - ``atf``
     - ``0x000000``
     - ``0x040000``
   * - ``uboot_a``
     - ``0x040000``
     - ``0x400000``
   * - ``uboot_b``
     - ``0x440000``
     - ``0x400000``
   * - ``ubi_env``
     - ``0x840000``
     - ``0x7c0000``

Each firmware slot contains a wrapped FIP with BL31 and BL33. The SPI
controller retains the clock divider and sampling configuration established
by boot firmware.

Environment
-----------

The environment uses two 16 KiB dynamic UBI volumes, ``env_a`` and
``env_b``, in ``ubi_env``. Saves alternate between the copies. The loader
selects a valid copy using its CRC and sequence number.

An empty environment partition initializes both volumes. Invalid copies
fall back to the default environment, which can then be saved. Save,
reload, single-copy recovery and initialization of an erased partition
have been exercised on the board.

Interfaces
----------

.. list-table::
   :header-rows: 1

   * - Interface
     - Validation
   * - Native Ethernet
     - DHCP, ping and 16 MiB transfers in both directions
   * - eMMC, ``mmc 0``
     - Identification and 16 MiB reads
   * - microSD, ``mmc 1``
     - 16 MiB write/readback and controller reprobe
   * - PCIe/NVMe
     - 16 MiB write/readback in both storage slots
   * - SATA
     - 16 MiB write/readback on both connectors
   * - USB 2 Type-A
     - High-speed storage and 16 MiB write/readback
   * - USB 3 Type-A
     - SuperSpeed storage and 16 MiB write/readback
   * - USB Type-C
     - SuperSpeed storage and 16 MiB write/readback
   * - Internal M.2 USB path
     - High-speed hub and WWAN device enumeration

SATA uses the native ``sata`` command interface. NVMe uses the common
command and queue engine, with banked host access for doorbell writes.
The USB controllers share a parent-owned clock and reset sequence.
Type-C operates as a fixed 5 V source and routes SuperSpeed lanes from
CC attachment detection.

The native Ethernet jack shows green for link and amber for activity.
The fan supply uses main GPIO 15 with active-high enable.

Operating-system handoff
------------------------

Board setup supplies PSCI CPU enable methods and marks the 256 KiB BL31
region at ``0x10000000`` as ``no-map`` in the operating-system device tree.
Four-core Linux boot, CPU hotplug and PSCI reset have been exercised.

During removal, storage and USB controllers stop DMA before releasing
controller storage. Shared reset domains remain available until their
consumers have stopped.

The watchdog exposes the standard ``wdt`` commands with automatic startup
disabled. Start, servicing, stop and timed expiry have been exercised.
