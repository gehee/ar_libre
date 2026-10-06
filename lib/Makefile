# SPDX-License-Identifier: GPL-2.0-or-later
# libarlink: a drop-in for the commonly used subset of libar8030_client.so.
# Built under the vendor library's name so a program linked against it loads
# this one instead. CROSS_COMPILE points at an aarch64 toolchain.
CC = $(CROSS_COMPILE)gcc
CFLAGS += -O2 -Wall -Wextra -fPIC -fvisibility=hidden -pthread
LIB = libar8030_client.so

all: $(LIB)

$(LIB): arlink.c arlink.h ioctl_sizes.inc
	$(CC) $(CFLAGS) -shared -Wl,-soname,$(LIB) -o $@ arlink.c -pthread

clean:
	rm -f $(LIB)

.PHONY: all clean
