/* SPDX-License-Identifier: GPL-2.0-or-later WITH Linux-syscall-note */
/*
 * arlink.ko's interface beyond read/write/poll on /dev/arlinkN.
 *
 * ARLINK_IOC_SOCK_ATTACH, on /dev/arlinkN: from now on the chip's data for
 * socket (slot, port) goes to a file of its own instead of /dev/arlinkN, and
 * the ioctl returns that file's descriptor. Attach before opening the socket
 * on the chip, so no data goes the other way. Closing the descriptor detaches.
 *
 * read() on that descriptor returns one of the chip's data frames at a time:
 * a struct arlink_sock_rec, then the frame's payload (the socket's bytes).
 * -EMSGSIZE if the buffer is too small for it; it stays queued. poll() works
 * as on /dev/arlinkN.
 */
#ifndef ARLINK_UAPI_H
#define ARLINK_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct arlink_sock_attach {
	__u8 slot;
	__u8 port;
	__u8 pad[2];
};

struct arlink_sock_rec {
	/* CLOCK_MONOTONIC, in ns, when the USB transfer that completed the
	 * frame finished: before any scheduling of the reader */
	__u64 rx_ns;
};

#define ARLINK_IOC_MAGIC        'A'
#define ARLINK_IOC_SOCK_ATTACH  _IOW(ARLINK_IOC_MAGIC, 1, struct arlink_sock_attach)

#endif
