/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Inside arlink.ko: what the core (core.c - the frame pipe, the firmware
 * upload) and the buses (usb.c, sdio.c) share.
 */
#ifndef ARLINK_CORE_H
#define ARLINK_CORE_H

#include <linux/device.h>
#include <linux/kfifo.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/completion.h>

#define DRV_NAME "arlink"

/*
 * A frame: AA | payload length, LE32 | reqid, BE32 | msgid, BE32 | status,
 * BE32 | xor of the 17 bytes before it, starting from FF | payload | BB.
 */
#define FRAME_HDR      18
#define FRAME_OVERHEAD (FRAME_HDR + 1)
#define FRAME_MAX      32768            /* the chip's are at most 10240 */
#define TX_BUF_SIZE    (FRAME_MAX + 512)        /* room for a bus to pad */
#define TX_TIMEOUT_MS  1000
#define MAX_SOCKS      16
#define MAX_ATTACHED   4

struct arlink;

/* What a bus does for the core. */
struct arlink_bus {
	const char *name;
	/* Send the frame in d->tx_buf (len bytes; the buffer has TX_BUF_SIZE
	 * room for padding). Called with io_mutex held; may sleep. */
	int (*tx)(struct arlink *d, u32 len);
	/* Free what the bus allocated: the last reference is gone. */
	void (*release)(struct arlink *d);
	/* Receive is a byte stream to cut into frames (USB), rather than
	 * frames the bus finds itself (SDIO). */
	bool stream_rx;
};

/* A socket whose data has a file of its own (ARLINK_IOC_SOCK_ATTACH). */
struct arlink_sock {
	struct arlink *d;
	u16 key;                        /* slot << 8 | port */
	struct kfifo_rec_ptr_2 q;       /* struct arlink_sock_rec + payload */
	struct mutex read_mutex;
	wait_queue_head_t wait;
	u64 frames, dropped;            /* under the device's rx_lock */
};

struct arlink {
	struct kref kref;
	struct device *dev;             /* the bus device, for messages; held until the last put */
	const struct arlink_bus *bus;
	void *bus_priv;
	struct miscdevice misc;
	char name[24];
	int index;                      /* -1 once given back (at unregister) */
	bool registered;                /* the misc device exists */
	unsigned long up_since;         /* jiffies, when it was registered */

	struct mutex io_mutex;          /* writes */
	bool gone;                      /* under rx_lock; see arlink_gone() */
	unsigned long busy;             /* bit 0: open */

	/* receive: bus -> (framer) -> queue of frames -> read() */
	spinlock_t rx_lock;             /* framer, queue producer, ->open, ->gone, ->socks */
	bool open;
	u8 *part;                       /* stream_rx: a frame split across transfers */
	u32 part_len;
	struct kfifo_rec_ptr_2 rxq;     /* one record per frame */
	struct mutex read_mutex;        /* queue consumer */
	wait_queue_head_t rx_wait;
	struct arlink_sock *attached[MAX_ATTACHED];     /* under rx_lock */

	u8 *tx_buf;                     /* TX_BUF_SIZE */

	/* what the program has left running, as the chip's replies said:
	 * sockets (slot << 8 | port), and the baseband initialised and started */
	u16 socks[MAX_SOCKS];
	int nsocks;
	bool socks_full;                /* one more than MAX_SOCKS: said so once */
	bool bb_inited, bb_started;

	/* release(): the reply it is waiting for, under rx_lock; and the last
	 * one it gave up waiting for, dropped should it come after all */
	u32 await_reqid;
	struct completion await_done;
	u32 late_reqid;
	unsigned long late_until;

	/* counters, under rx_lock / io_mutex */
	u64 rx_frames, rx_bytes, rx_dropped, rx_closed, rx_skipped, rx_errors;
	unsigned int rx_error_run;
	u64 tx_frames, tx_bytes, tx_errors;
	u64 bus_errors;         /* the bus itself failed (an SDIO transfer timed out or errored): the chip may be wedged */
};

/* The running chip: allocate, then register once the bus is ready for
 * traffic; unregister when it goes, then drop the bus's reference. */
struct arlink *arlink_alloc(struct device *dev, const struct arlink_bus *bus, void *priv);
int arlink_register(struct arlink *d);
void arlink_unregister(struct arlink *d);
void arlink_put(struct arlink *d);
/* The bus has given up on the chip (it is resetting it): from now on it is
 * as good as gone, though still registered until the bus lets go of it.
 * True the first time. Callable in atomic context. */
bool arlink_gone(struct arlink *d);

/* Received data. arlink_rx_stream cuts frames out of a byte stream;
 * arlink_rx_frame takes one the bus found (arlink_frame_len says whether p
 * starts one). Both may write into the buffer. Callable in atomic context. */
void arlink_rx_stream(struct arlink *d, u8 *p, u32 n, u64 rx_ns);
void arlink_rx_frame(struct arlink *d, u8 *f, u32 len, u64 rx_ns);
u32 arlink_frame_len(const u8 *p, u32 n);
/* Bytes received that were not a frame. */
void arlink_rx_skip(struct arlink *d, u32 n);
/* A receive error; returns how many in a row. */
unsigned int arlink_rx_error(struct arlink *d);

/*
 * The boot ROM upload. The core reads the image and config and drives the
 * sequence; the bus sends blocks: a 12-byte header (magic, length LE16,
 * address LE32, 0 LE32) and up to `chunk(left)` bytes of data. A block of
 * length 0 at address 0 starts what was uploaded.
 */
#define ROM_BLOCK_HDR  12
#define ROM_BLOCK_MAX  (4096 - ROM_BLOCK_HDR)
struct arlink_rom {
	struct device *dev;
	char magic[2];
	u32 (*chunk)(u32 left);
	int (*send)(void *ctx, const u8 *block, u32 len);       /* len includes the header */
	void *ctx;
};
int arlink_rom_upload(const struct arlink_rom *rom);

/* The buses. */
#if IS_ENABLED(CONFIG_USB)
int arlink_usb_init(void);
void arlink_usb_exit(void);
#else
static inline int arlink_usb_init(void) { return 0; }
static inline void arlink_usb_exit(void) { }
#endif
#if IS_ENABLED(CONFIG_MMC)
int arlink_sdio_init(void);
void arlink_sdio_exit(void);
#else
static inline int arlink_sdio_init(void) { return 0; }
static inline void arlink_sdio_exit(void) { }
#endif

#endif
