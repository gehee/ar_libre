// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * arlink on SDIO: an AR8030 attached over SDIO (an air unit), function 1.
 *
 * See ../docs/driver.md.
 *
 *   4152:8030  the boot ROM. The firmware goes up through the FIFO (function 1,
 *              address 0) as 'S' 'D' blocks; then the host is set to poll for
 *              the card, which leaves and comes back as
 *   4152:8031  the running firmware. Its interrupt says what it has: data to
 *              read and room to write, both in 512-byte blocks.
 *
 * SDIO moves whole blocks, and the chip takes each transfer as one packet:
 *   - what we read is one frame per run of blocks, starting on a block
 *     boundary, the rest of its last block stale;
 *   - what we write must be one transfer: up to a block (one byte-mode
 *     transfer) or a whole number of blocks, so a longer frame is padded.
 *     A frame split in two transfers is dropped.
 */
#include <linux/module.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/sysfs.h>
#include <linux/kthread.h>

#include "arlink_core.h"

#define SDIO_VENDOR_ARTOSYN 0x4152
#define SDIO_DEV_ROM        0x8030
#define SDIO_DEV_RUNNING    0x8031

#define BLOCK               512

/* Function 1 registers */
#define REG_INT_PENDING     0x13        /* non-zero while something is pending; bit 4: events */
#define REG_INT_ENABLE      0x14
#define REG_EVENTS          0x68        /* bit 0 mailbox 0, bit 1 mailbox 1, bit 2 data to read, bit 3 room to write */
#define REG_MBOX0           0x54
#define REG_MBOX1           0x58
#define REG_RX_BLOCKS       0x5c        /* data waiting, in blocks */
#define REG_TX_BLOCKS       0x60        /* room to write, in blocks */
#define EV_MBOX0            BIT(0)
#define EV_MBOX1            BIT(1)
#define EV_RX               BIT(2)
#define EV_TX               BIT(3)
#define PENDING_EVENTS      BIT(4)

#define RX_BUF_SIZE         (256 * BLOCK)       /* REG_RX_BLOCKS is one byte */
#define RX_READ_MAX         0x10000             /* per transfer, as stock reads */
#define POLL_MS             100                 /* a missed interrupt costs at most this; stock
                                                 * never polls, and polling more often disturbs the chip */

static bool sdio_debug;
module_param(sdio_debug, bool, 0644);
MODULE_PARM_DESC(sdio_debug, "log the SDIO interrupt and event registers as they are read");

struct arlink_sdio {
	struct arlink *d;
	struct sdio_func *func;         /* held, as d->dev */
	u8 *rx_buf;
	u32 tx_room;                    /* bytes the chip has room for; one write uses it up */
	wait_queue_head_t tx_wait;
	/* What the chip has for us, as the interrupt saw it; the reader thread reads
	 * it and sets it back to 0, with the host held, as stock's driver does. */
	u32 rx_len;
	u64 rx_ns;
	wait_queue_head_t rx_wait;
	struct task_struct *rx_task;
	bool gone;
	u8 mbox[2];
};

/* ---- passive trace ----------------------------------------------------------
 * Every transfer and grant, timestamped, in a ring read through sysfs
 * (/sys/bus/sdio/devices/<func>/arlink_trace, raw records). It touches no chip
 * register: only what the driver does anyway. For comparing the bus pattern
 * with the stock driver's (2026-10-04, the link drops on ar_libre). */
enum { TR_IRQ = 1, TR_RX = 2, TR_GRANT = 3, TR_TX = 4, TR_TX_DONE = 5, TR_TX_WAIT = 6 };
struct trace_rec { u64 ns; u16 type; u16 a; u32 b; };
#define TRACE_N 32768
static struct trace_rec *trace_ring;
static u32 trace_pos;
static DEFINE_SPINLOCK(trace_lock);

static void trace(u16 type, u16 a, u32 b)
{
	unsigned long f;

	if (!trace_ring)
		return;
	spin_lock_irqsave(&trace_lock, f);
	trace_ring[trace_pos % TRACE_N] = (struct trace_rec){ ktime_get_ns(), type, a, b };
	trace_pos++;
	spin_unlock_irqrestore(&trace_lock, f);
}

/* The ring, oldest first: a u32 count, then the records. */
static ssize_t trace_read(struct file *f, struct kobject *k, struct bin_attribute *attr,
			  char *buf, loff_t off, size_t count)
{
	size_t total = sizeof(u32) + TRACE_N * sizeof(struct trace_rec);
	unsigned long fl;
	u32 n, start;
	size_t i;

	if (!trace_ring || off >= total)
		return 0;
	if (off + count > total)
		count = total - off;
	spin_lock_irqsave(&trace_lock, fl);
	n = min_t(u32, trace_pos, TRACE_N);
	start = trace_pos - n;
	for (i = 0; i < count; i++) {
		loff_t o = off + i;
		u8 v;

		if (o < sizeof(u32)) {
			v = ((u8 *)&n)[o];
		} else {
			size_t r = (o - sizeof(u32)) / sizeof(struct trace_rec);
			size_t b = (o - sizeof(u32)) % sizeof(struct trace_rec);

			v = r < n ? ((u8 *)&trace_ring[(start + r) % TRACE_N])[b] : 0;
		}
		buf[i] = v;
	}
	spin_unlock_irqrestore(&trace_lock, fl);
	return count;
}
static BIN_ATTR(arlink_trace, 0444, trace_read, NULL, sizeof(u32) + TRACE_N * sizeof(struct trace_rec));

/* ---- the card ---------------------------------------------------------------- */

/*
 * Have the host poll for the card: after the upload the chip leaves and comes
 * back as 8031, and a chip that resets falls back to 8030. Stock does exactly
 * this (MMC_CAP_NEEDS_POLL on, MMC_CAP_NONREMOVABLE off, the host's
 * card_event, a detect).
 */
static void watch_card(struct sdio_func *func)
{
	struct mmc_host *host = func->card->host;

	host->caps |= MMC_CAP_NEEDS_POLL;
	host->caps &= ~MMC_CAP_NONREMOVABLE;
	if (host->ops->card_event)
		host->ops->card_event(host);
	mmc_detect_change(host, msecs_to_jiffies(600));
}

static int enable(struct sdio_func *func)
{
	int r;

	sdio_claim_host(func);
	r = sdio_enable_func(func);
	if (!r)
		r = sdio_set_block_size(func, BLOCK);
	if (!r)
		sdio_writeb(func, 1, REG_INT_ENABLE, &r);
	if (!r)
		sdio_writeb(func, 0xf0, REG_EVENTS, &r);
	sdio_release_host(func);
	return r;
}

/* ---- boot ROM --------------------------------------------------------------- */

/* A block is one transfer: at most one SDIO block, or a whole number of them. */
static u32 rom_chunk(u32 left)
{
	if (left == ROM_BLOCK_MAX || left + ROM_BLOCK_HDR <= BLOCK)
		return left;
	return (left + ROM_BLOCK_HDR) / BLOCK * BLOCK - ROM_BLOCK_HDR;
}

static int rom_send(void *ctx, const u8 *block, u32 len)
{
	struct sdio_func *func = ctx;
	int r;

	sdio_claim_host(func);
	r = sdio_memcpy_toio(func, 0, (void *)block, len);
	sdio_release_host(func);
	return r;
}

static int rom_upload(struct sdio_func *func)
{
	struct arlink_rom rom = {
		.dev = &func->dev,
		.magic = { 'S', 'D' },
		.chunk = rom_chunk,
		.send = rom_send,
		.ctx = func,
	};
	u8 ready = 0;
	int i, r = 0;

	/* function 1 ready (CCCR I/O ready) */
	for (i = 0; i < 100; i++) {
		sdio_claim_host(func);
		ready = sdio_f0_readb(func, SDIO_CCCR_IORx, &r);
		sdio_release_host(func);
		if (!r && (ready & BIT(func->num)))
			break;
		msleep(10);
	}
	if (r || !(ready & BIT(func->num))) {
		dev_err(&func->dev, "boot ROM not ready (%d, %02x)\n", r, ready);
		return -EIO;
	}
	return arlink_rom_upload(&rom);
}

/* ---- the running chip ------------------------------------------------------- */

/* Read what the chip has (blocks of it) and hand over the frames in it. */
static void rx(struct arlink_sdio *s, u32 len, u64 rx_ns)
{
	u32 o, n;
	int r;

	if (!len)
		return;
	for (o = 0; o < len; o += n) {
		n = min_t(u32, len - o, RX_READ_MAX);
		r = sdio_memcpy_fromio(s->func, s->rx_buf + o, 0, n);
		if (r) {
			if (arlink_rx_error(s->d) == 1)
				dev_err_ratelimited(&s->func->dev, "rx: %d\n", r);
			return;
		}
	}
	for (o = 0; o + FRAME_OVERHEAD <= len; ) {
		n = arlink_frame_len(s->rx_buf + o, len - o);
		if (!n) {
			arlink_rx_skip(s->d, BLOCK);    /* not a frame: the next block */
			o += BLOCK;
			continue;
		}
		arlink_rx_frame(s->d, s->rx_buf + o, n, rx_ns);
		o += ALIGN(n, BLOCK);
	}
}

/* Called with the host claimed: from the SDIO interrupt, or from a writer
 * waiting for room. */
static void service(struct arlink_sdio *s)
{
	struct sdio_func *func = s->func;
	u64 rx_ns = ktime_get_ns();
	bool tx_done = false, rx_done = false;
	int i, err;

	/* Every RX event is read at once: the chip offers one packet at a time and
	 * the next only after the host has read the last, so skipping one stalls it
	 * (tried 2026-10-04: no TX room ever came back). At most one new TX figure
	 * per pass, as stock's handler. */
	for (i = 0; i < 16; i++) {
		u8 pending = sdio_readb(func, REG_INT_PENDING, &err), ev;

		trace(TR_IRQ, pending, i);

		if (sdio_debug)
			dev_info_ratelimited(&func->dev, "pending %02x (%d)\n", pending, err);
		if (err || !pending)
			break;
		if (!(pending & PENDING_EVENTS))
			continue;
		ev = sdio_readb(func, REG_EVENTS, &err);
		if (sdio_debug)
			dev_info_ratelimited(&func->dev, "events %02x (%d) rx %02x tx %02x\n", ev, err,
					     sdio_readb(func, REG_RX_BLOCKS, NULL), sdio_readb(func, REG_TX_BLOCKS, NULL));
		if (err)
			break;
		if (ev & EV_MBOX0)
			s->mbox[0] = sdio_readb(func, REG_MBOX0, &err);
		if (ev & EV_MBOX1)
			s->mbox[1] = sdio_readb(func, REG_MBOX1, &err);
		if (ev & EV_RX) {
			u8 blocks = sdio_readb(func, REG_RX_BLOCKS, &err);

			/* As stock: the interrupt only takes note of what the chip has,
			 * and only once the last of it has been read; the reader thread
			 * reads it. The chip offers the next only after that read. */
			if (!err && blocks && !rx_done && !READ_ONCE(s->rx_len)) {
				rx_done = true;
				trace(TR_RX, blocks, 1);
				s->rx_ns = rx_ns;
				WRITE_ONCE(s->rx_len, blocks * BLOCK);
				wake_up(&s->rx_wait);
			} else if (!err) {
				trace(TR_RX, blocks, 0);        /* not taken */
			}
		}
		if (ev & EV_TX) {
			u8 blocks = sdio_readb(func, REG_TX_BLOCKS, &err);

			/* as stock: a new figure only once the last is used up */
			if (!err && !tx_done && !READ_ONCE(s->tx_room)) {
				tx_done = true;
				trace(TR_GRANT, blocks, 0);
				WRITE_ONCE(s->tx_room, blocks * BLOCK);
				wake_up(&s->tx_wait);
			}
		}
	}
}

/* The reader: what the interrupt noted, read with the host held, and the note
 * cleared before the host is let go, so the interrupt that follows the chip's
 * next offer finds it clear. */
static int rx_thread(void *arg)
{
	struct arlink_sdio *s = arg;

	while (!kthread_should_stop()) {
		u32 len;

		wait_event_interruptible(s->rx_wait, READ_ONCE(s->rx_len) || kthread_should_stop() ||
					 READ_ONCE(s->gone));
		if (kthread_should_stop())
			break;
		len = READ_ONCE(s->rx_len);
		if (!len || READ_ONCE(s->gone))
			continue;
		sdio_claim_host(s->func);
		rx(s, len, s->rx_ns);
		WRITE_ONCE(s->rx_len, 0);
		sdio_release_host(s->func);
	}
	return 0;
}

static void irq(struct sdio_func *func)
{
	struct arlink_sdio *s = sdio_get_drvdata(func);

	if (s && s->d)
		service(s);
}

static int sdio_tx(struct arlink *d, u32 len)
{
	struct arlink_sdio *s = d->bus_priv;
	u32 n = len > BLOCK ? ALIGN(len, BLOCK) : len;
	unsigned long until = jiffies + msecs_to_jiffies(TX_TIMEOUT_MS);
	int r;

	if (n > len)
		memset(d->tx_buf + len, 0, n - len);
	/* Room for all of it, in one transfer. Look for it ourselves too, every
	 * POLL_MS: an interrupt that does not come must not stall the writer. */
	if (READ_ONCE(s->tx_room) < n)
		trace(TR_TX_WAIT, 0, n);
	while (READ_ONCE(s->tx_room) < n) {
		if (READ_ONCE(s->gone))
			return -ENODEV;
		if (time_after(jiffies, until)) {
			dev_err_ratelimited(&s->func->dev, "tx: no room for %u bytes (has %u)\n",
					    n, READ_ONCE(s->tx_room));
			return -ETIMEDOUT;
		}
		wait_event_timeout(s->tx_wait, READ_ONCE(s->tx_room) >= n || READ_ONCE(s->gone),
				   msecs_to_jiffies(POLL_MS));
		if (READ_ONCE(s->tx_room) < n) {
			sdio_claim_host(s->func);
			service(s);
			sdio_release_host(s->func);
		}
	}
	sdio_claim_host(s->func);
	trace(TR_TX, 0, n);
	r = sdio_memcpy_toio(s->func, 0, d->tx_buf, n);
	trace(TR_TX_DONE, r < 0 ? -r : 0, n);
	if (!r)
		WRITE_ONCE(s->tx_room, 0);      /* used up, as stock */
	else
		d->bus_errors++;                /* the bus itself failed: see arlink_core.h */
	sdio_release_host(s->func);
	return r;
}

static void sdio_release(struct arlink *d)
{
	struct arlink_sdio *s = d->bus_priv;

	if (!s)
		return;
	kfree(s->rx_buf);
	kfree(s);
}

static const struct arlink_bus sdio_bus = {
	.name = "sdio",
	.tx = sdio_tx,
	.release = sdio_release,
	.stream_rx = false,
};

static int probe_running(struct sdio_func *func)
{
	struct arlink_sdio *s;
	struct arlink *d;
	int r;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->rx_buf = kmalloc(RX_BUF_SIZE, GFP_KERNEL);
	if (!s->rx_buf) {
		kfree(s);
		return -ENOMEM;
	}
	s->func = func;
	init_waitqueue_head(&s->tx_wait);
	init_waitqueue_head(&s->rx_wait);
	s->rx_task = kthread_run(rx_thread, s, "arlink-sdio-rx");
	if (IS_ERR(s->rx_task)) {
		kfree(s->rx_buf);
		kfree(s);
		return -ENOMEM;
	}
	d = arlink_alloc(&func->dev, &sdio_bus, s);
	if (!d) {
		kthread_stop(s->rx_task);
		kfree(s->rx_buf);
		kfree(s);
		return -ENOMEM;
	}
	sdio_set_drvdata(func, s);
	sdio_claim_host(func);
	r = sdio_claim_irq(func, irq);
	sdio_release_host(func);
	if (r) {
		dev_err(&func->dev, "irq: %d\n", r);
		goto fail;
	}
	s->d = d;                       /* the interrupt may serve it from now on */
	sdio_claim_host(func);
	service(s);                     /* anything already waiting */
	sdio_release_host(func);
	r = arlink_register(d);
	if (r)
		goto fail_irq;
	if (!trace_ring)
		trace_ring = vzalloc(TRACE_N * sizeof(struct trace_rec));
	if (trace_ring && sysfs_create_bin_file(&func->dev.kobj, &bin_attr_arlink_trace))
		dev_warn(&func->dev, "no trace file\n");
	return 0;

fail_irq:
	s->d = NULL;
	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_release_host(func);
fail:
	kthread_stop(s->rx_task);
	sdio_set_drvdata(func, NULL);
	arlink_put(d);
	return r;
}

static int arlink_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	int r = enable(func);

	if (r) {
		dev_err(&func->dev, "enable: %d\n", r);
		return r;
	}
	watch_card(func);
	/* For the boot ROM there is nothing to keep: it leaves once the upload
	 * is done, and comes back as the running firmware. */
	if (func->device == SDIO_DEV_ROM)
		return rom_upload(func);
	return probe_running(func);
}

static void arlink_sdio_remove(struct sdio_func *func)
{
	struct arlink_sdio *s = sdio_get_drvdata(func);

	if (s) {
		sysfs_remove_bin_file(&func->dev.kobj, &bin_attr_arlink_trace);
		WRITE_ONCE(s->gone, true);
		wake_up(&s->tx_wait);
		kthread_stop(s->rx_task);
		arlink_unregister(s->d);
	}
	sdio_claim_host(func);
	if (s)
		sdio_release_irq(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	sdio_set_drvdata(func, NULL);
	if (s)
		arlink_put(s->d);
}

static const struct sdio_device_id arlink_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ARTOSYN, SDIO_DEV_ROM) },
	{ SDIO_DEVICE(SDIO_VENDOR_ARTOSYN, SDIO_DEV_RUNNING) },
	{ }
};
MODULE_DEVICE_TABLE(sdio, arlink_sdio_ids);

static struct sdio_driver arlink_sdio_driver = {
	.name = DRV_NAME,
	.id_table = arlink_sdio_ids,
	.probe = arlink_sdio_probe,
	.remove = arlink_sdio_remove,
};

int arlink_sdio_init(void)
{
	return sdio_register_driver(&arlink_sdio_driver);
}

void arlink_sdio_exit(void)
{
	sdio_unregister_driver(&arlink_sdio_driver);
	vfree(trace_ring);
	trace_ring = NULL;
}
