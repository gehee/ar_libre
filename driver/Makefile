# SPDX-License-Identifier: GPL-2.0-or-later
#
# Out-of-tree build:
#   make KERNEL_DIR=<configured kernel tree> ARCH=<arm64|arm> CROSS_COMPILE=<prefix>
# One module for both buses; each is built when the kernel has it.
obj-m := arlink.o
arlink-y := core.o
arlink-$(CONFIG_USB) += usb.o
arlink-$(CONFIG_MMC) += sdio.o

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) clean
