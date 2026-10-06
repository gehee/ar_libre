// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * arlink on USB: an AR8030 attached over USB (a ground unit).
 *
 * See ../docs/driver.md.
 *
 *   4152:8030  the boot ROM. The firmware goes up as vendor control writes
 *              (bmRequestType 0x40, bRequest 0x0c), one 'U' 'S' block each.
 *   1d6b:8030  the running firmware: one interface, a bulk endpoint each way,
 *              carrying the host protocol's frames.
 */
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/ktime.h>

#include "arlink_core.h"

#define ROM_REQ        0x0c
#define RX_URBS        8
#define RX_URB_SIZE    8192
#define RX_ERROR_RUN   50               /* errors in a row before resetting the chip */

struct arlink_usb {
	struct arlink *d;
	struct usb_device *udev;
	struct usb_interface *intf;     /* held, as d->dev */
	u8 ep_in, ep_out;
	struct urb *rx_urb[RX_URBS];
	struct urb *tx_urb;
	struct completion tx_done;
};

/* ---- boot ROM ------------------------------------------------------------- */

static u32 rom_chunk(u32 left)
{
	return left;                    /* any size up to ROM_BLOCK_MAX */
}

static int rom_send(void *ctx, const u8 *block, u32 len)
{
	struct usb_device *udev = ctx;
	int r = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), ROM_REQ,
				USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE, 0, 0,
				(void *)block, len, 1000);

	return r < 0 ? r : 0;
}

static int rom_upload(struct usb_interface *intf)
{
	struct arlink_rom rom = {
		.dev = &intf->dev,
		.magic = { 'U', 'S' },
		.chunk = rom_chunk,
		.send = rom_send,
		.ctx = interface_to_usbdev(intf),
	};

	return arlink_rom_upload(&rom);
}

/* ---- the running chip ------------------------------------------------------- */

static void rx_done(struct urb *urb)
{
	struct arlink_usb *u = urb->context;
	u64 rx_ns = ktime_get_ns();     /* CLOCK_MONOTONIC */
	unsigned int run;
	int r;

	switch (urb->status) {
	case 0:
		arlink_rx_stream(u->d, urb->transfer_buffer, urb->actual_length, rx_ns);
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -ENODEV:
		return;                 /* killed, or the chip is gone */
	default:
		/* -EPROTO and the like: a glitch, or the chip on its way out.
		 * Too many in a row, or a stall (clearing a halt sleeps, which a
		 * completion cannot): give up on this chip - its readers and
		 * writers fail from now on - and reset it, which clears a halt
		 * too, so that it re-enumerates and is probed afresh. */
		run = arlink_rx_error(u->d);
		if (urb->status == -EPIPE || run >= RX_ERROR_RUN) {
			if (arlink_gone(u->d)) {
				dev_err(&u->intf->dev, "rx: error %d (%u in a row), resetting the chip\n",
					urb->status, run);
				usb_queue_reset_device(u->intf);
			}
			return;
		}
	}
	if (READ_ONCE(u->d->gone))
		return;                 /* given up on, or unplugged: no more transfers */
	r = usb_submit_urb(urb, GFP_ATOMIC);
	if (r && r != -ENODEV && r != -EPERM)
		dev_err_ratelimited(&u->intf->dev, "rx resubmit: %d\n", r);
}

static void tx_done(struct urb *urb)
{
	struct arlink_usb *u = urb->context;

	complete(&u->tx_done);
}

static int usb_tx(struct arlink *d, u32 len)
{
	struct arlink_usb *u = d->bus_priv;
	int r;

	reinit_completion(&u->tx_done);
	u->tx_urb->transfer_buffer_length = len;
	r = usb_submit_urb(u->tx_urb, GFP_KERNEL);
	if (r)
		return r;
	if (!wait_for_completion_timeout(&u->tx_done, msecs_to_jiffies(TX_TIMEOUT_MS))) {
		usb_kill_urb(u->tx_urb);
		return -ETIMEDOUT;
	}
	return u->tx_urb->status;
}

static void usb_release(struct arlink *d)
{
	struct arlink_usb *u = d->bus_priv;
	int i;

	if (!u)
		return;
	for (i = 0; i < RX_URBS; i++) {
		if (!u->rx_urb[i])
			continue;
		kfree(u->rx_urb[i]->transfer_buffer);
		usb_free_urb(u->rx_urb[i]);
	}
	usb_free_urb(u->tx_urb);
	usb_put_dev(u->udev);
	kfree(u);
}

static const struct arlink_bus usb_bus = {
	.name = "usb",
	.tx = usb_tx,
	.release = usb_release,
	.stream_rx = true,
};

static int probe_running(struct usb_interface *intf)
{
	struct usb_endpoint_descriptor *in, *out;
	struct arlink_usb *u;
	struct arlink *d;
	int i, r;

	if (usb_find_common_endpoints(intf->cur_altsetting, &in, &out, NULL, NULL))
		return -ENODEV;
	u = kzalloc(sizeof(*u), GFP_KERNEL);
	if (!u)
		return -ENOMEM;
	d = arlink_alloc(&intf->dev, &usb_bus, u);
	if (!d) {
		kfree(u);
		return -ENOMEM;
	}
	u->d = d;
	u->udev = usb_get_dev(interface_to_usbdev(intf));
	u->intf = intf;
	u->ep_in = in->bEndpointAddress;
	u->ep_out = out->bEndpointAddress;
	init_completion(&u->tx_done);

	r = -ENOMEM;
	u->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!u->tx_urb)
		goto fail;
	usb_fill_bulk_urb(u->tx_urb, u->udev, usb_sndbulkpipe(u->udev, u->ep_out),
			  d->tx_buf, 0, tx_done, u);
	u->tx_urb->transfer_flags |= URB_ZERO_PACKET;
	for (i = 0; i < RX_URBS; i++) {
		void *b = kmalloc(RX_URB_SIZE, GFP_KERNEL);

		u->rx_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!u->rx_urb[i] || !b) {
			kfree(b);
			goto fail;
		}
		usb_fill_bulk_urb(u->rx_urb[i], u->udev, usb_rcvbulkpipe(u->udev, u->ep_in),
				  b, RX_URB_SIZE, rx_done, u);
	}

	usb_set_intfdata(intf, u);
	/* Drain the chip from the start, whether or not anyone has it open. */
	for (i = 0; i < RX_URBS; i++) {
		r = usb_submit_urb(u->rx_urb[i], GFP_KERNEL);
		if (r)
			goto fail_urbs;
	}
	r = arlink_register(d);
	if (r)
		goto fail_urbs;
	dev_info(&intf->dev, "bulk in %02x out %02x\n", u->ep_in, u->ep_out);
	return 0;

fail_urbs:
	for (i = 0; i < RX_URBS; i++)
		usb_kill_urb(u->rx_urb[i]);
	usb_set_intfdata(intf, NULL);
fail:
	arlink_put(d);
	return r;
}

enum { CHIP_ROM, CHIP_RUNNING };

static int arlink_usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	/* For the boot ROM there is nothing to keep: it leaves the bus once
	 * the upload is done. */
	if (id->driver_info == CHIP_ROM)
		return rom_upload(intf);
	return probe_running(intf);
}

static void arlink_usb_disconnect(struct usb_interface *intf)
{
	struct arlink_usb *u = usb_get_intfdata(intf);
	int i;

	usb_set_intfdata(intf, NULL);
	if (!u)
		return;                 /* the boot ROM */
	arlink_unregister(u->d);
	for (i = 0; i < RX_URBS; i++)
		usb_kill_urb(u->rx_urb[i]);
	usb_kill_urb(u->tx_urb);
	arlink_put(u->d);
}

static const struct usb_device_id arlink_usb_ids[] = {
	{ USB_DEVICE(0x4152, 0x8030), .driver_info = CHIP_ROM },
	{ USB_DEVICE(0x1d6b, 0x8030), .driver_info = CHIP_RUNNING },
	{ }
};
MODULE_DEVICE_TABLE(usb, arlink_usb_ids);

static struct usb_driver arlink_usb_driver = {
	.name = DRV_NAME,
	.probe = arlink_usb_probe,
	.disconnect = arlink_usb_disconnect,
	.id_table = arlink_usb_ids,
};

int arlink_usb_init(void)
{
	return usb_register(&arlink_usb_driver);
}

void arlink_usb_exit(void)
{
	usb_deregister(&arlink_usb_driver);
}
