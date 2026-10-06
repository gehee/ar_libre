// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * arlink: driver for the Artosyn AR8030 baseband - on USB (a ground unit)
 * and on SDIO (an air unit).
 *
 * See ../docs/protocol.md and ../docs/driver.md.
 *
 * The chip shows up twice on either bus: first its boot ROM, to which the
 * driver uploads the firmware image and the baseband config, then the running
 * firmware, carrying the host protocol's frames. This file is the part that
 * does not depend on the bus: the upload sequence, and the frame pipe the
 * running chip becomes, /dev/arlinkN:
 *
 *   read()   one whole frame, as the chip sent it (AA .. BB). -EMSGSIZE if
 *            the buffer is too small for it; the frame stays queued.
 *   write()  one whole frame, sent as one transfer. Returns once the chip has
 *            taken it.
 *   poll()   readable when a frame is queued.
 *
 * A program can also have one socket's data delivered to a file of its own,
 * each frame stamped with the time its transfer completed
 * (ARLINK_IOC_SOCK_ATTACH, see arlink_uapi.h). That spares a busy socket - the
 * video - the trip through the program's dispatch of everything else.
 *
 * The device can be open once at a time. Frames that arrive while it is
 * closed are dropped. When it is closed, the driver finishes what the program
 * left running, as a program that exits cleanly does: it closes the sockets
 * still open, then stops and deinitialises the baseband if it was started.
 * Without that, a program that dies leaves the chip in a state where the next
 * program's bring-up wedges its OUT endpoint until a power cycle - measured:
 * every second restart after a kill.
 */
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/log2.h>
#include <linux/uaccess.h>
#include <linux/idr.h>
#include <linux/jiffies.h>
#include <linux/anon_inodes.h>
#include <linux/ktime.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "arlink_core.h"
#include "arlink_uapi.h"

/* ---- module parameters --------------------------------------------------- */

static char *fw_name = "bb_demo_cx485_2PA.img";
module_param(fw_name, charp, 0444);
MODULE_PARM_DESC(fw_name, "firmware image to upload, from /lib/firmware");

static char *cfg_name = "bb_config_gnd_pro.json";
module_param(cfg_name, charp, 0444);
MODULE_PARM_DESC(cfg_name, "baseband config (JSON) to upload over the image's own; empty for none");

static unsigned int rx_queue_kb = 1024;
module_param(rx_queue_kb, uint, 0444);
MODULE_PARM_DESC(rx_queue_kb, "received frames waiting for read(), in KB (rounded down to a power of two)");

static unsigned int sock_queue_kb = 512;
module_param(sock_queue_kb, uint, 0444);
MODULE_PARM_DESC(sock_queue_kb, "an attached socket's frames waiting for read(), in KB (rounded down to a power of two)");

/* ---- the wire ------------------------------------------------------------ */

#define DOM_SOCKET     4
#define REQ_BB_START   0x0b000000       /* RPC_IOCTL: start, stop, init, deinit */
#define REQ_BB_STOP    0x0b000001
#define REQ_BB_INIT    0x0b000002
#define REQ_BB_DEINIT  0x0b000003
#define CLEANUP_REPLY_MS 1000
#define CLEANUP_LATE_MS  10000          /* a reply later than this is not coming */
#define SO_OPEN        0
#define SO_READ        2                /* the chip's data for a socket */
#define SO_CLOSE       3
#define SOCK_DATA_STA  0x12345678       /* the status of a data frame */

#define RX_PART_SIZE   (2 * FRAME_MAX)
#define MAX_DEVICES    8

static DEFINE_IDA(arlink_ida);

static u8 frame_xor(const u8 *f)
{
	u8 x = 0xff;
	int i;

	for (i = 0; i < FRAME_HDR - 1; i++)
		x ^= f[i];
	return x;
}

u32 arlink_frame_len(const u8 *p, u32 n)
{
	u32 plen;

	if (n < FRAME_OVERHEAD || p[0] != 0xaa || frame_xor(p) != p[FRAME_HDR - 1])
		return 0;
	plen = get_unaligned_le32(p + 1);
	if (plen > FRAME_MAX - FRAME_OVERHEAD || n - FRAME_OVERHEAD < plen ||
	    p[FRAME_HDR + plen] != 0xbb)
		return 0;
	return plen + FRAME_OVERHEAD;
}

/* ---- boot ROM upload ------------------------------------------------------ */

/*
 * The image: a 64-byte header, then three sections, each starting on a
 * 512-byte boundary in the file, in this order: the SPL, the image's own
 * baseband config, the firmware. The header gives each one's load address
 * and length (LE32 pairs): the firmware's at 0x18, the SPL's at 0x20, the
 * config's at 0x28. Same format on both sides (goggle and air unit).
 */
#define ROM_HDR_ADDR   0x002f0040       /* where the image header goes */
#define IMG_MAGIC      0x41528030
#define IMG_HDR_SIZE   64
#define IMG_ALIGN      512

struct img_section {
	const char *what;
	u32 addr, len;
	size_t off;
};

/* A chip that keeps falling back to the boot ROM (a config the firmware
 * cannot run, a firmware that crashes) is not fed for ever: after
 * UPLOAD_TRIES uploads in a row with no running chip in between that lasted
 * UPLOAD_GOOD_MS, the driver stops uploading until it is reloaded. Only
 * uploads that reached the chip count, not a missing firmware file. */
#define UPLOAD_TRIES     4
#define UPLOAD_GOOD_MS   30000
static unsigned int upload_count;       /* all of them, for the stats */
static unsigned int upload_tries;       /* in a row, as above */
static DEFINE_MUTEX(upload_lock);

static int parse_image(const struct firmware *img, struct img_section s[3])
{
	const u8 *p = img->data;
	size_t off = ALIGN(IMG_HDR_SIZE, IMG_ALIGN);
	int i;

	if (img->size < IMG_HDR_SIZE || get_unaligned_le32(p) != IMG_MAGIC ||
	    get_unaligned_le16(p + 6) != IMG_HDR_SIZE)
		return -EINVAL;
	s[0] = (struct img_section){ "spl",    get_unaligned_le32(p + 0x20), get_unaligned_le32(p + 0x24) };
	s[1] = (struct img_section){ "config", get_unaligned_le32(p + 0x28), get_unaligned_le32(p + 0x2c) };
	s[2] = (struct img_section){ "fw",     get_unaligned_le32(p + 0x18), get_unaligned_le32(p + 0x1c) };
	for (i = 0; i < 3; i++) {
		if (s[i].len > img->size || off > img->size - s[i].len)
			return -EINVAL;
		s[i].off = off;
		off = ALIGN(off + s[i].len, IMG_ALIGN);
	}
	return 0;
}

static bool upload_allowed(void)
{
	bool ok;

	mutex_lock(&upload_lock);
	ok = upload_tries < UPLOAD_TRIES;
	if (ok) {
		upload_tries++;
		upload_count++;
	}
	mutex_unlock(&upload_lock);
	return ok;
}

/* The running chip is going (unregister): if it ran long enough, the
 * uploads before it worked, and the count starts again. */
static void upload_ran(unsigned long since)
{
	if (time_before(jiffies, since + msecs_to_jiffies(UPLOAD_GOOD_MS)))
		return;
	mutex_lock(&upload_lock);
	upload_tries = 0;
	mutex_unlock(&upload_lock);
}

static int rom_block(const struct arlink_rom *rom, u8 *buf, u32 addr, const u8 *data, u32 len)
{
	buf[0] = rom->magic[0];
	buf[1] = rom->magic[1];
	put_unaligned_le16(len, buf + 2);
	put_unaligned_le32(addr, buf + 4);
	put_unaligned_le32(0, buf + 8);
	if (len)
		memcpy(buf + ROM_BLOCK_HDR, data, len);
	return rom->send(rom->ctx, buf, ROM_BLOCK_HDR + len);
}

static int rom_write(const struct arlink_rom *rom, u8 *buf, u32 addr, const u8 *data, size_t len)
{
	while (len) {
		u32 n = rom->chunk(min_t(size_t, len, ROM_BLOCK_MAX));
		int r = rom_block(rom, buf, addr, data, n);

		if (r)
			return r;
		addr += n;
		data += n;
		len -= n;
	}
	return 0;
}

int arlink_rom_upload(const struct arlink_rom *rom)
{
	struct device *dev = rom->dev;
	const struct firmware *img = NULL, *cfg = NULL;
	struct img_section s[3];
	u8 *buf;
	int i, r;

	buf = kmalloc(ROM_BLOCK_HDR + ROM_BLOCK_MAX, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	r = request_firmware(&img, fw_name, dev);
	if (r) {
		dev_err(dev, "firmware %s: %d\n", fw_name, r);
		goto out;
	}
	if (cfg_name && *cfg_name) {
		r = request_firmware(&cfg, cfg_name, dev);
		if (r) {
			dev_err(dev, "config %s: %d\n", cfg_name, r);
			goto out;
		}
	}
	r = parse_image(img, s);
	if (r) {
		dev_err(dev, "%s is not an AR8030 image\n", fw_name);
		goto out;
	}
	if (cfg && cfg->size > s[1].len) {
		dev_err(dev, "config %s: %zu bytes, the image has room for %u\n",
			cfg_name, cfg->size, s[1].len);
		r = -EFBIG;
		goto out;
	}
	if (!upload_allowed()) {
		dev_err(dev, "boot ROM again: %d uploads in a row, none of them running %d s, the chip cannot run this firmware/config; not uploading (reload the module to retry)\n",
			UPLOAD_TRIES, UPLOAD_GOOD_MS / 1000);
		r = -EIO;
		goto out;
	}

	r = rom_write(rom, buf, ROM_HDR_ADDR, img->data, IMG_HDR_SIZE);
	for (i = 0; i < 3 && !r; i++)
		r = rom_write(rom, buf, s[i].addr, img->data + s[i].off, s[i].len);
	/* the separate config goes over the image's own */
	if (!r && cfg)
		r = rom_write(rom, buf, s[1].addr, cfg->data, cfg->size);
	if (r) {
		dev_err(dev, "upload failed: %d\n", r);
		goto out;
	}
	dev_info(dev, "uploaded %s (spl %u, config %u, firmware %u bytes)%s%s\n",
		 fw_name, s[0].len, s[1].len, s[2].len, cfg ? " and " : "", cfg ? cfg_name : "");
	/* Run it. The chip may leave the bus before the request completes, so
	 * an error here does not say it failed to start - nor that it started:
	 * say so, and leave it to the chip coming back (or not) as running. */
	r = rom_block(rom, buf, 0, NULL, 0);
	if (r)
		dev_warn(dev, "run: %d (no matter if the chip has left the boot ROM already)\n", r);
	r = 0;
out:
	release_firmware(cfg);
	release_firmware(img);
	kfree(buf);
	return r;
}

/* ---- receive ---------------------------------------------------------------- */

/* Called under rx_lock with a reply whose status is 0: note what the chip has
 * set running - a socket open, the baseband initialised or started - once it
 * says it did, so that the cleanup on close does not act on what it refused.
 * What a program stops is noted as it asks (track_state). */
static void track_reply(struct arlink *d, u32 reqid)
{
	u16 key = reqid & 0xffff;
	int i;

	switch (reqid) {
	case REQ_BB_INIT:  d->bb_inited = true; return;
	case REQ_BB_START: d->bb_started = true; return;
	}
	if (reqid >> 24 != DOM_SOCKET || ((reqid >> 16) & 0xff) != SO_OPEN)
		return;
	for (i = 0; i < d->nsocks; i++)
		if (d->socks[i] == key)
			return;
	if (d->nsocks < MAX_SOCKS) {
		d->socks[d->nsocks++] = key;
	} else if (!d->socks_full) {
		d->socks_full = true;
		dev_warn(d->dev, "more than %d sockets open: slot %u port %u and later ones are not closed on close\n",
			 MAX_SOCKS, key >> 8, key & 0xff);
	}
}

/* Called under rx_lock. A data frame for an attached socket goes to that
 * socket's file. Its header is no longer needed then, so the record header
 * (the arrival time) is written over the end of it. */
static void rx_push(struct arlink *d, u8 *f, u32 len, u64 rx_ns)
{
	u32 reqid = get_unaligned_be32(f + 5);
	int i;

	if (d->await_reqid && reqid == d->await_reqid) {
		d->await_reqid = 0;
		complete(&d->await_done);
	} else if (d->late_reqid && reqid == d->late_reqid && !get_unaligned_be32(f + 9)) {
		/* The reply to a cleanup request that timed out (msgid 0, as
		 * the cleanup sends them; libarlink's requests never are): no
		 * one's, and the next program would take it for one of its own. */
		d->late_reqid = 0;
		if (time_before(jiffies, d->late_until)) {
			d->rx_closed++;
			return;
		}
	}
	if (!get_unaligned_be32(f + 13))
		track_reply(d, reqid);
	if (!d->open) {
		d->rx_closed++;
		return;
	}
	if (reqid >> 24 == DOM_SOCKET && ((reqid >> 16) & 0xff) == SO_READ &&
	    get_unaligned_be32(f + 13) == SOCK_DATA_STA) {
		for (i = 0; i < MAX_ATTACHED; i++) {
			struct arlink_sock *s = d->attached[i];
			struct arlink_sock_rec rec = { .rx_ns = rx_ns };
			u8 *r = f + FRAME_HDR - sizeof(rec);
			u32 n = len - FRAME_OVERHEAD + sizeof(rec);

			if (!s || s->key != (reqid & 0xffff))
				continue;
			memcpy(r, &rec, sizeof(rec));
			if (kfifo_in(&s->q, r, n) != n) {
				s->dropped++;
				return;
			}
			s->frames++;
			wake_up_interruptible(&s->wait);
			return;
		}
	}
	if (kfifo_in(&d->rxq, f, len) != len) {
		d->rx_dropped++;
		return;
	}
	d->rx_frames++;
}

/*
 * Take the whole frames off the front of p, skipping bytes that cannot start
 * one. Stops at a frame that is not all there yet. Returns the bytes used.
 */
static u32 rx_parse(struct arlink *d, u8 *p, u32 n, u64 rx_ns)
{
	u32 o = 0;

	while (o < n) {
		u8 *f = p + o;
		u32 left = n - o, plen;

		if (f[0] != 0xaa)
			goto skip;
		if (left < FRAME_HDR)
			break;
		plen = get_unaligned_le32(f + 1);
		if (frame_xor(f) != f[FRAME_HDR - 1] || plen > FRAME_MAX - FRAME_OVERHEAD)
			goto skip;
		if (left < plen + FRAME_OVERHEAD)
			break;
		if (f[FRAME_HDR + plen] != 0xbb)
			goto skip;
		rx_push(d, f, plen + FRAME_OVERHEAD, rx_ns);
		o += plen + FRAME_OVERHEAD;
		continue;
skip:
		d->rx_skipped++;
		o++;
	}
	return o;
}

/* Called under rx_lock. A frame normally arrives in one transfer; one that
 * does not is put together in ->part. */
static void rx_feed(struct arlink *d, u8 *p, u32 n, u64 rx_ns)
{
	if (!d->part_len) {
		u32 used = rx_parse(d, p, n, rx_ns);

		p += used;
		n -= used;
	}
	while (n) {
		u32 c = min_t(u32, n, RX_PART_SIZE - d->part_len), used;

		memcpy(d->part + d->part_len, p, c);
		d->part_len += c;
		p += c;
		n -= c;
		used = rx_parse(d, d->part, d->part_len, rx_ns);
		if (!used && d->part_len == RX_PART_SIZE)
			used = d->part_len;     /* cannot happen: a frame fits */
		memmove(d->part, d->part + used, d->part_len - used);
		d->part_len -= used;
	}
}

void arlink_rx_stream(struct arlink *d, u8 *p, u32 n, u64 rx_ns)
{
	unsigned long flags;
	u64 before;

	spin_lock_irqsave(&d->rx_lock, flags);
	d->rx_error_run = 0;
	before = d->rx_frames;
	d->rx_bytes += n;
	rx_feed(d, p, n, rx_ns);
	spin_unlock_irqrestore(&d->rx_lock, flags);
	if (d->rx_frames != before)
		wake_up_interruptible(&d->rx_wait);
}

void arlink_rx_frame(struct arlink *d, u8 *f, u32 len, u64 rx_ns)
{
	unsigned long flags;
	u64 before;

	spin_lock_irqsave(&d->rx_lock, flags);
	d->rx_error_run = 0;
	before = d->rx_frames;
	d->rx_bytes += len;
	rx_push(d, f, len, rx_ns);
	spin_unlock_irqrestore(&d->rx_lock, flags);
	if (d->rx_frames != before)
		wake_up_interruptible(&d->rx_wait);
}

void arlink_rx_skip(struct arlink *d, u32 n)
{
	unsigned long flags;

	spin_lock_irqsave(&d->rx_lock, flags);
	d->rx_skipped += n;
	spin_unlock_irqrestore(&d->rx_lock, flags);
}

unsigned int arlink_rx_error(struct arlink *d)
{
	unsigned long flags;
	unsigned int run;

	spin_lock_irqsave(&d->rx_lock, flags);
	d->rx_errors++;
	run = ++d->rx_error_run;
	spin_unlock_irqrestore(&d->rx_lock, flags);
	return run;
}

/* ---- transmit --------------------------------------------------------------- */

/* Called with io_mutex held, the frame in d->tx_buf. */
static int tx_frame(struct arlink *d, u32 len)
{
	int r = READ_ONCE(d->gone) ? -ENODEV : d->bus->tx(d, len);

	if (r) {
		d->tx_errors++;
		return r;
	}
	d->tx_frames++;
	d->tx_bytes += len;
	return 0;
}

/* Called with io_mutex held: note what the program stops - the sockets it
 * closes, the baseband stopped or deinitialised - as it asks. What it sets
 * running is noted from the chip's replies (track_reply). */
static void track_state(struct arlink *d, const u8 *f)
{
	u32 reqid = get_unaligned_be32(f + 5);
	u16 key = reqid & 0xffff;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&d->rx_lock, flags);
	switch (reqid) {
	case REQ_BB_DEINIT: d->bb_inited = false; d->bb_started = false; break;
	case REQ_BB_STOP:   d->bb_started = false; break;
	}
	if (reqid >> 24 == DOM_SOCKET && ((reqid >> 16) & 0xff) == SO_CLOSE) {
		for (i = 0; i < d->nsocks; i++) {
			if (d->socks[i] == key) {
				d->socks[i] = d->socks[--d->nsocks];
				break;
			}
		}
	}
	spin_unlock_irqrestore(&d->rx_lock, flags);
}

/* Called with io_mutex held: send a request with no payload and wait for its
 * reply, as a program would. 0, or why not. */
static int request_and_wait(struct arlink *d, u32 reqid)
{
	u8 *f = d->tx_buf;
	unsigned long flags;
	int r;

	memset(f, 0, FRAME_OVERHEAD);
	f[0] = 0xaa;
	put_unaligned_be32(reqid, f + 5);
	f[FRAME_HDR - 1] = frame_xor(f);
	f[FRAME_HDR] = 0xbb;
	spin_lock_irqsave(&d->rx_lock, flags);
	reinit_completion(&d->await_done);
	d->await_reqid = reqid;
	spin_unlock_irqrestore(&d->rx_lock, flags);
	r = tx_frame(d, FRAME_OVERHEAD);
	if (!r && !wait_for_completion_timeout(&d->await_done, msecs_to_jiffies(CLEANUP_REPLY_MS)))
		r = -ETIMEDOUT;
	spin_lock_irqsave(&d->rx_lock, flags);
	if (r && d->await_reqid) {
		/* given up on: should its reply come after all, drop it */
		d->late_reqid = reqid;
		d->late_until = jiffies + msecs_to_jiffies(CLEANUP_LATE_MS);
	}
	d->await_reqid = 0;
	spin_unlock_irqrestore(&d->rx_lock, flags);
	if (!r && READ_ONCE(d->gone))
		r = -ENODEV;            /* woken because the chip went */
	return r;
}

/* Called with io_mutex held: what a program exiting cleanly does, for one
 * that did not - its sockets closed, then the baseband stopped and
 * deinitialised. A chip that does not take or answer one of these is not
 * going to answer the rest: the first failure ends it, rather than hold the
 * device (opens get -EBUSY, a disconnect waits) for up to two seconds a
 * request. */
static void finish_program(struct arlink *d)
{
	static const struct { u32 reqid; const char *what; } bb[] = {
		{ REQ_BB_STOP,   "stopped the baseband" },
		{ REQ_BB_DEINIT, "deinitialised the baseband" },
	};
	u16 socks[MAX_SOCKS];
	unsigned long flags;
	bool todo[2];
	u32 reqid = 0;
	int n, i, r = 0;

	/* Taken over: a reply still on its way adds to the next program's. */
	spin_lock_irqsave(&d->rx_lock, flags);
	n = d->nsocks;
	memcpy(socks, d->socks, sizeof(socks));
	todo[0] = d->bb_started;
	todo[1] = d->bb_inited;
	d->nsocks = 0;
	d->socks_full = false;
	d->bb_started = d->bb_inited = false;
	spin_unlock_irqrestore(&d->rx_lock, flags);

	while (n && !r && !READ_ONCE(d->gone)) {
		u16 key = socks[--n];

		reqid = DOM_SOCKET << 24 | SO_CLOSE << 16 | key;
		r = request_and_wait(d, reqid);
		if (!r)
			dev_info(d->dev, "closed socket slot %u port %u left open\n",
				 key >> 8, key & 0xff);
	}
	for (i = 0; i < 2 && !r && !READ_ONCE(d->gone); i++) {
		if (!todo[i])
			continue;
		reqid = bb[i].reqid;
		r = request_and_wait(d, reqid);
		if (!r)
			dev_info(d->dev, "%s left running\n", bb[i].what);
	}
	if (r && r != -ENODEV)
		dev_warn(d->dev, "cleanup: request %08x: %d, the chip is not answering; skipped the rest\n",
			 reqid, r);
}

/* ---- the device file ---------------------------------------------------------- */

/* A queue of frames. Its buffer need not be physically contiguous, and a
 * large one that has to be may not be had once memory has fragmented: a
 * re-probe hours into a session would then leave no /dev/arlinkN. */
static int queue_alloc(struct kfifo_rec_ptr_2 *q, unsigned int kb)
{
	unsigned int size = rounddown_pow_of_two(kb * 1024);
	void *buf = kvmalloc(size, GFP_KERNEL);

	if (!buf)
		return -ENOMEM;
	if (kfifo_init(q, buf, size)) {
		kvfree(buf);
		return -ENOMEM;
	}
	return 0;
}

static void queue_free(struct kfifo_rec_ptr_2 *q)
{
	kvfree(q->kfifo.data);
}

static void arlink_free(struct kref *kref)
{
	struct arlink *d = container_of(kref, struct arlink, kref);

	if (d->bus->release)
		d->bus->release(d);
	kfree(d->tx_buf);
	kfree(d->part);
	queue_free(&d->rxq);
	if (d->index >= 0)
		ida_free(&arlink_ida, d->index);
	put_device(d->dev);
	kfree(d);
}

void arlink_put(struct arlink *d)
{
	kref_put(&d->kref, arlink_free);
}

static int arlink_open(struct inode *inode, struct file *file)
{
	/* misc_open() holds misc_mtx, so arlink_unregister() cannot run past
	 * misc_deregister() until this returns */
	struct arlink *d = container_of(file->private_data, struct arlink, misc);
	unsigned long flags;

	if (READ_ONCE(d->gone))
		return -ENODEV;
	if (test_and_set_bit(0, &d->busy))
		return -EBUSY;
	kref_get(&d->kref);
	spin_lock_irqsave(&d->rx_lock, flags);
	kfifo_reset(&d->rxq);           /* no reader yet: safe */
	d->open = true;
	spin_unlock_irqrestore(&d->rx_lock, flags);
	file->private_data = d;
	return stream_open(inode, file);
}

static int arlink_release(struct inode *inode, struct file *file)
{
	struct arlink *d = file->private_data;
	unsigned long flags;

	mutex_lock(&d->io_mutex);
	finish_program(d);
	mutex_unlock(&d->io_mutex);
	spin_lock_irqsave(&d->rx_lock, flags);
	d->open = false;
	spin_unlock_irqrestore(&d->rx_lock, flags);
	clear_bit(0, &d->busy);
	arlink_put(d);
	return 0;
}

static ssize_t arlink_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct arlink *d = file->private_data;
	unsigned int copied;
	ssize_t r;

	if (mutex_lock_interruptible(&d->read_mutex))
		return -ERESTARTSYS;
	while (kfifo_is_empty(&d->rxq)) {
		if (READ_ONCE(d->gone)) {
			r = -ENODEV;
			goto out;
		}
		if (file->f_flags & O_NONBLOCK) {
			r = -EAGAIN;
			goto out;
		}
		r = wait_event_interruptible(d->rx_wait,
					     !kfifo_is_empty(&d->rxq) || READ_ONCE(d->gone));
		if (r)
			goto out;
	}
	if (kfifo_peek_len(&d->rxq) > count) {
		r = -EMSGSIZE;
		goto out;
	}
	r = kfifo_to_user(&d->rxq, buf, count, &copied);
	if (!r)
		r = copied;
out:
	mutex_unlock(&d->read_mutex);
	return r;
}

static ssize_t arlink_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct arlink *d = file->private_data;
	u8 *f = d->tx_buf;
	ssize_t r;

	if (count < FRAME_OVERHEAD || count > FRAME_MAX)
		return -EINVAL;
	if (mutex_lock_interruptible(&d->io_mutex))
		return -ERESTARTSYS;
	if (copy_from_user(f, buf, count)) {
		r = -EFAULT;
		goto out;
	}
	if (f[0] != 0xaa || get_unaligned_le32(f + 1) != count - FRAME_OVERHEAD ||
	    f[count - 1] != 0xbb) {
		r = -EINVAL;
		goto out;
	}
	track_state(d, f);
	r = tx_frame(d, count);
	if (!r)
		r = count;
out:
	mutex_unlock(&d->io_mutex);
	return r;
}

static __poll_t arlink_poll(struct file *file, poll_table *wait)
{
	struct arlink *d = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &d->rx_wait, wait);
	if (!kfifo_is_empty(&d->rxq))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (READ_ONCE(d->gone))
		mask |= EPOLLERR | EPOLLHUP;
	return mask;
}

/* ---- attached sockets -------------------------------------------------------- */

static int sock_release(struct inode *inode, struct file *file)
{
	struct arlink_sock *s = file->private_data;
	struct arlink *d = s->d;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&d->rx_lock, flags);
	for (i = 0; i < MAX_ATTACHED; i++)
		if (d->attached[i] == s)
			d->attached[i] = NULL;
	spin_unlock_irqrestore(&d->rx_lock, flags);
	if (s->dropped)
		dev_info(d->dev, "socket slot %u port %u: %llu frames dropped (reader behind)\n",
			 s->key >> 8, s->key & 0xff, s->dropped);
	queue_free(&s->q);
	kfree(s);
	arlink_put(d);
	return 0;
}

static ssize_t sock_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct arlink_sock *s = file->private_data;
	struct arlink *d = s->d;
	unsigned int copied;
	ssize_t r;

	if (mutex_lock_interruptible(&s->read_mutex))
		return -ERESTARTSYS;
	while (kfifo_is_empty(&s->q)) {
		if (READ_ONCE(d->gone)) {
			r = -ENODEV;
			goto out;
		}
		if (file->f_flags & O_NONBLOCK) {
			r = -EAGAIN;
			goto out;
		}
		r = wait_event_interruptible(s->wait, !kfifo_is_empty(&s->q) || READ_ONCE(d->gone));
		if (r)
			goto out;
	}
	if (kfifo_peek_len(&s->q) > count) {
		r = -EMSGSIZE;
		goto out;
	}
	r = kfifo_to_user(&s->q, buf, count, &copied);
	if (!r)
		r = copied;
out:
	mutex_unlock(&s->read_mutex);
	return r;
}

static __poll_t sock_poll(struct file *file, poll_table *wait)
{
	struct arlink_sock *s = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &s->wait, wait);
	if (!kfifo_is_empty(&s->q))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (READ_ONCE(s->d->gone))
		mask |= EPOLLERR | EPOLLHUP;
	return mask;
}

static const struct file_operations arlink_sock_fops = {
	.owner = THIS_MODULE,
	.release = sock_release,
	.read = sock_read,
	.poll = sock_poll,
};

static long sock_attach(struct arlink *d, struct arlink_sock_attach __user *uarg)
{
	struct arlink_sock_attach a;
	struct arlink_sock *s;
	unsigned long flags;
	int i, slot = -1, fd;

	if (copy_from_user(&a, uarg, sizeof(a)))
		return -EFAULT;
	if (READ_ONCE(d->gone))
		return -ENODEV;
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	if (queue_alloc(&s->q, max(sock_queue_kb, 16u))) {
		kfree(s);
		return -ENOMEM;
	}
	s->d = d;
	s->key = a.slot << 8 | a.port;
	mutex_init(&s->read_mutex);
	init_waitqueue_head(&s->wait);

	spin_lock_irqsave(&d->rx_lock, flags);
	for (i = 0; i < MAX_ATTACHED; i++) {
		if (d->attached[i] && d->attached[i]->key == s->key) {
			slot = -EBUSY;
			break;
		}
		if (!d->attached[i] && slot == -1)
			slot = i;
	}
	if (slot >= 0)
		d->attached[slot] = s;
	spin_unlock_irqrestore(&d->rx_lock, flags);
	if (slot < 0) {
		queue_free(&s->q);
		kfree(s);
		return slot == -1 ? -ENOSPC : slot;
	}

	kref_get(&d->kref);
	fd = anon_inode_getfd("[arlink-socket]", &arlink_sock_fops, s, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		spin_lock_irqsave(&d->rx_lock, flags);
		d->attached[slot] = NULL;
		spin_unlock_irqrestore(&d->rx_lock, flags);
		queue_free(&s->q);
		kfree(s);
		arlink_put(d);
	}
	return fd;
}

static long arlink_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct arlink *d = file->private_data;

	switch (cmd) {
	case ARLINK_IOC_SOCK_ATTACH:
		return sock_attach(d, (struct arlink_sock_attach __user *)arg);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations arlink_fops = {
	.owner = THIS_MODULE,
	.open = arlink_open,
	.release = arlink_release,
	.read = arlink_read,
	.write = arlink_write,
	.poll = arlink_poll,
	.unlocked_ioctl = arlink_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/* ---- counters: /sys/class/misc/arlinkN/stats ---------------------------------- */

static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct miscdevice *m = dev_get_drvdata(dev);
	struct arlink *d = container_of(m, struct arlink, misc);
	unsigned long flags;
	u64 v[6], sf[MAX_ATTACHED], sd[MAX_ATTACHED];
	u16 sk[MAX_ATTACHED];
	int i, n, len;

	spin_lock_irqsave(&d->rx_lock, flags);
	v[0] = d->rx_frames; v[1] = d->rx_bytes; v[2] = d->rx_dropped;
	v[3] = d->rx_closed; v[4] = d->rx_skipped; v[5] = d->rx_errors;
	for (i = n = 0; i < MAX_ATTACHED; i++) {
		if (!d->attached[i])
			continue;
		sk[n] = d->attached[i]->key;
		sf[n] = d->attached[i]->frames;
		sd[n++] = d->attached[i]->dropped;
	}
	spin_unlock_irqrestore(&d->rx_lock, flags);
	len = sysfs_emit(buf,
		"bus %s\nrx_frames %llu\nrx_bytes %llu\nrx_dropped %llu\nrx_closed %llu\nrx_skipped %llu\nrx_errors %llu\n"
		"tx_frames %llu\ntx_bytes %llu\ntx_errors %llu\nbus_errors %llu\nuploads %u\n",
		d->bus->name, v[0], v[1], v[2], v[3], v[4], v[5],
		d->tx_frames, d->tx_bytes, d->tx_errors, d->bus_errors, upload_count);
	for (i = 0; i < n; i++)
		len += sysfs_emit_at(buf, len, "socket_%u_%u frames %llu dropped %llu\n",
				     sk[i] >> 8, sk[i] & 0xff, sf[i], sd[i]);
	return len;
}
static DEVICE_ATTR_RO(stats);

static struct attribute *arlink_attrs[] = {
	&dev_attr_stats.attr,
	NULL,
};
ATTRIBUTE_GROUPS(arlink);

/* ---- the running chip, for the buses ----------------------------------------- */

static const struct arlink_bus no_bus = { .name = "none" };

struct arlink *arlink_alloc(struct device *dev, const struct arlink_bus *bus, void *priv)
{
	struct arlink *d = kzalloc(sizeof(*d), GFP_KERNEL);

	if (!d)
		return NULL;
	kref_init(&d->kref);
	d->index = -1;
	/* Held until the last put: a program can keep the device open, and
	 * log through it, long after the bus device itself has gone. */
	d->dev = get_device(dev);
	d->bus = bus;
	d->bus_priv = priv;
	mutex_init(&d->io_mutex);
	mutex_init(&d->read_mutex);
	spin_lock_init(&d->rx_lock);
	init_waitqueue_head(&d->rx_wait);
	init_completion(&d->await_done);
	if (queue_alloc(&d->rxq, max(rx_queue_kb, 64u)))
		goto fail;
	d->tx_buf = kmalloc(TX_BUF_SIZE, GFP_KERNEL);
	if (!d->tx_buf)
		goto fail;
	if (bus->stream_rx) {
		d->part = kmalloc(RX_PART_SIZE, GFP_KERNEL);
		if (!d->part)
			goto fail;
	}
	return d;
fail:
	d->bus = &no_bus;               /* nothing of the bus's to free yet */
	arlink_put(d);
	return NULL;
}

int arlink_register(struct arlink *d)
{
	int r = ida_alloc_max(&arlink_ida, MAX_DEVICES - 1, GFP_KERNEL);

	if (r < 0)
		return r;
	d->index = r;
	snprintf(d->name, sizeof(d->name), DRV_NAME "%d", d->index);
	d->misc.minor = MISC_DYNAMIC_MINOR;
	d->misc.name = d->name;
	d->misc.fops = &arlink_fops;
	d->misc.mode = 0600;
	d->misc.parent = d->dev;
	d->misc.groups = arlink_groups;
	r = misc_register(&d->misc);
	if (r)
		return r;
	d->registered = true;
	d->up_since = jiffies;
	dev_info(d->dev, "/dev/%s: AR8030 running (%s)\n", d->name, d->bus->name);
	return 0;
}

/* Gone: no new opens, reads end in -ENODEV once the queue is empty (the
 * attached sockets' too), writes fail, poll reports EPOLLERR | EPOLLHUP, and
 * everyone waiting is woken to find it so - readers, a cleanup waiting for a
 * reply. */
bool arlink_gone(struct arlink *d)
{
	unsigned long flags;
	bool first;
	int i;

	spin_lock_irqsave(&d->rx_lock, flags);
	first = !d->gone;
	WRITE_ONCE(d->gone, true);
	for (i = 0; i < MAX_ATTACHED; i++)
		if (d->attached[i])
			wake_up_interruptible_all(&d->attached[i]->wait);
	if (d->await_reqid) {
		d->await_reqid = 0;
		complete(&d->await_done);
	}
	spin_unlock_irqrestore(&d->rx_lock, flags);
	wake_up_interruptible_all(&d->rx_wait);
	return first;
}

/* The chip is gone. The caller still holds its reference and drops it with
 * arlink_put() once its own transfers have stopped. */
void arlink_unregister(struct arlink *d)
{
	if (d->registered) {
		misc_deregister(&d->misc);
		upload_ran(d->up_since);
	}
	/* The number is free again at once: a chip that comes back while a
	 * program still has this one open is /dev/arlinkN again, not N + 1. */
	if (d->index >= 0) {
		ida_free(&arlink_ida, d->index);
		d->index = -1;
	}
	arlink_gone(d);
	/* A write that started before it was gone has finished once this
	 * returns, and none starts after: the bus may stop its transfers. */
	mutex_lock(&d->io_mutex);
	mutex_unlock(&d->io_mutex);
	dev_info(d->dev, "/dev/%s: gone\n", d->name);
}

/* ---- the module ----------------------------------------------------------------- */

static int __init arlink_init(void)
{
	int r = arlink_usb_init();

	if (r)
		return r;
	r = arlink_sdio_init();
	if (r)
		arlink_usb_exit();
	return r;
}

static void __exit arlink_exit(void)
{
	arlink_sdio_exit();
	arlink_usb_exit();
}

module_init(arlink_init);
module_exit(arlink_exit);

MODULE_DESCRIPTION("Artosyn AR8030 baseband: firmware upload and frame pipe, on USB and SDIO");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("bb_demo_cx485_2PA.img");
