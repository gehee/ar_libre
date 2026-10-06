// SPDX-License-Identifier: GPL-2.0-or-later
//
// libarlink: an AR8030 host client without Artosyn's daemon.
//
// See ../docs/protocol.md.
//
// Artosyn's stack puts a daemon between the application and the chip: the
// client library (libar8030_client.so) sends each call over TCP to the
// daemon, which owns the chip's USB interface and relays the frames. This
// library does the daemon's job in-process instead. It talks to the chip
// through a frame pipe: arlink.ko's /dev/arlinkN (../driver), or else the
// /dev/ar_mdev0 of Artosyn's own driver. write() sends a frame to the bulk OUT
// endpoint, read() returns what the chip sent (see ../docs/protocol.md).
// Nothing here comes from Artosyn's daemon or client sources: it is written
// from USB captures of the stock daemon.
//
// What the daemon itself does, and so this library does:
//   - at connect: a burst of link resets, then hello until the chip answers;
//     the firmware log switched on; every event subscribed (13 down to 0);
//   - a heartbeat every 120 ms;
//   - replies matched to requests by msgid; socket data, which the chip
//     pushes unasked once a socket is open, buffered per socket; events
//     passed to the callbacks the client registered.
// A client's requests (bb_ioctl) go out unchanged: a request code is the
// wire reqid, domain << 24 | sub-command.
//
// On arlink.ko each socket's data comes from a file of its own
// (ARLINK_IOC_SOCK_ATTACH), stamped with when its USB transfer completed, so
// bb_socket_read() takes it straight from the kernel instead of through the
// reader thread and a ring here. ARLINK_NOFAST=1 turns that off.
//
// Environment: ARLINK_DEV (device), ARLINK_DEBUG=1|2 (log; 2 adds the chip's
// own log), ARLINK_CHIPLOG=<file> (the chip's log, rotated to <file>.1 at
// 1 MB), ARLINK_NOFAST=1.
#define _GNU_SOURCE
#include "arlink.h"

#include "../driver/arlink_uapi.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define API __attribute__((visibility("default")))

enum { DOM_CFG = 0, DOM_GET = 1, DOM_SET = 2, DOM_CB = 3, DOM_SOCKET = 4, DOM_DBG = 5,
       DOM_RPC_IOCTL = 11, DOM_LINK = 0xff };
enum { SO_OPEN = 0, SO_WRITE = 1, SO_READ = 2, SO_CLOSE = 3 };

#define REQ_EVENT_SUBSCRIBE 0x02000000u   // BB_SET_EVENT_SUBSCRIBE: handled here
#define REQ_INIT            0x0b000002u   // BB_INIT_REQ
#define REQ_START           0x0b000000u   // BB_START_REQ
#define SOCK_DATA_STA       0x12345678    // sta of a data frame
#define WRITE_PENDING_STA   (-262)        // a write's first reply; the second carries the count
#define DEFAULT_TIMEOUT_MS  3000
#define MAX_EVENTS          16
#define ARLINK_VERSION      "0.2"
#define STAGE_SIZE          (32768 + 64)  // a socket file's record: arlink.ko's FRAME_MAX
#define CHIPLOG_MAX         (1 << 20)

static int debug, nofast;

// Artosyn's SDIO driver (the air unit) moves whole 512-byte blocks: each read
// is one frame per block run, the rest of the last block stale bytes, and a
// write longer than a block goes out only up to a block boundary - the chip
// takes each write as one packet, so the tail must not follow separately.
// Frames longer than a block are padded to whole blocks, as the chip pads its
// own. 0 on other devices.
static uint32_t sdio_block;
#define LOG(...) do { if (debug) fprintf(stderr, "arlink: " __VA_ARGS__); } while (0)

// ---- request sizes --------------------------------------------------------------
// The input length of a request is not on the wire anywhere the library could
// learn it, so it comes from a table. A program that exports its own
// get_bb_ioctl_cmdiptlen() is asked first, as the vendor library asks it.
struct size_row { uint32_t req; int ipt, out; };
static const struct size_row sizes[] = {
#include "ioctl_sizes.inc"
    // RPC_IOCTL (domain 11): start, stop, init, deinit. Empty both ways on
    // the wire (usbmon of the stock daemon's bring-up).
    { 0x0b000000, 0, 0 }, { 0x0b000001, 0, 0 },
    { 0x0b000002, 0, 0 }, { 0x0b000003, 0, 0 },
};
extern int get_bb_ioctl_cmdiptlen(int req) __attribute__((weak));

static int request_sizes(uint32_t req, int *out) {
    int ipt = -1;
    *out = -1;
    for (size_t i = 0; i < sizeof sizes / sizeof sizes[0]; i++)
        if (sizes[i].req == req) { ipt = sizes[i].ipt; *out = sizes[i].out; break; }
    if (get_bb_ioctl_cmdiptlen) {
        int v = get_bb_ioctl_cmdiptlen((int)req);
        if (v >= 0) ipt = v;
    }
    return ipt;
}

// ---- state ----------------------------------------------------------------------
struct bb_host_t { int unused; };
struct bb_dev_t { int unused; };
struct bb_dev_handle_t { int unused; };
static struct bb_host_t the_host;
static struct bb_dev_t the_dev;
static struct bb_dev_handle_t the_handle;

static int devfd = -1;
static volatile int running;
static int dev_gone;                     // see device_gone()
static pthread_t rd_thread, hb_thread;
static int hb_started;                   // hb_thread exists, to be joined
#define RD_BUF_SIZE (1 << 18)
static uint8_t *rd_buf;                  // the reader's, freed once it is joined
static pthread_mutex_t tx_lock = PTHREAD_MUTEX_INITIALIZER;

// Requests waiting for their reply. A socket operation is also matched by
// (op, slot, port), in case its reply comes back without the msgid.
#define MAX_PENDING 32
struct pending {
    int used, done;
    uint32_t id, key;
    int is_write;
    int32_t sta;
    uint8_t *out;
    uint32_t cap, got;
};
static struct pending pend[MAX_PENDING];
static uint32_t next_id = 1;
static pthread_mutex_t st_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t st_cond = PTHREAD_COND_INITIALIZER;
static int hello_seen;

// Open sockets, each with a ring for the data the chip pushes.
#define MAX_SOCKETS 8
struct sock {
    int used;
    uint8_t slot, port;
    uint32_t written;              // data bytes written so far: the write header
    uint8_t *ring;
    uint32_t cap, head, count;
    unsigned long long dropped;
    pthread_mutex_t m;
    pthread_cond_t c;
    // arlink.ko's socket file instead of the ring, when it has one
    int sfd;
    uint8_t *stage;                // the frame being handed out
    uint32_t stage_off, stage_len;
    uint64_t rx_ns;                // when the data last returned arrived
    uint64_t stage_rx_ns;          // when the frame in the stage arrived
    pthread_mutex_t rd;            // held by the reader of the socket file
};
static struct sock socks[MAX_SOCKETS];

// Under st_lock: the caller sets them while the reader thread calls them.
static struct { bb_event_callback cb; arlink_event_cb cb_len; void *user; } events[MAX_EVENTS];

// The chip's own log (domain 5), when ARLINK_CHIPLOG names a file.
static int chiplog_fd = -1;
static const char *chiplog_path;
static size_t chiplog_size;

static void chiplog_open(void) {
    chiplog_fd = open(chiplog_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    chiplog_size = chiplog_fd >= 0 ? (size_t)lseek(chiplog_fd, 0, SEEK_END) : 0;
}

static void chiplog_write(const uint8_t *p, uint32_t n) {
    if (chiplog_fd < 0) return;
    if (chiplog_size + n > CHIPLOG_MAX) {
        char old[512];
        snprintf(old, sizeof old, "%s.1", chiplog_path);
        close(chiplog_fd);
        rename(chiplog_path, old);
        chiplog_open();
        if (chiplog_fd < 0) return;
    }
    if (write(chiplog_fd, p, n) == (ssize_t)n) chiplog_size += n;
}

static int gone(void) {
    return __atomic_load_n(&dev_gone, __ATOMIC_ACQUIRE);
}

// The device is gone - unplugged, reset, or given up on by its driver - and
// nothing more comes through this descriptor: the reader stops, every call
// waiting is woken and fails, and so does every call from now on (-ENODEV, or
// -1 where that is the call's failure), until the caller disconnects and
// connects again, to the chip as it comes back.
static void device_gone(const char *why) {
    if (__atomic_exchange_n(&dev_gone, 1, __ATOMIC_ACQ_REL)) return;
    fprintf(stderr, "arlink: the device is gone (%s); reconnect to use the chip again\n", why);
    pthread_mutex_lock(&st_lock);
    pthread_cond_broadcast(&st_cond);
    pthread_mutex_unlock(&st_lock);
    for (int i = 0; i < MAX_SOCKETS; i++) {
        struct sock *s = &socks[i];
        if (!s->ring) continue;                   // a socket file's reader polls
        pthread_mutex_lock(&s->m);
        pthread_cond_broadcast(&s->c);
        pthread_mutex_unlock(&s->m);
    }
}

// ---- framing --------------------------------------------------------------------
static int send_frame(uint32_t reqid, uint32_t msgid, const void *p, uint32_t n) {
    uint8_t stackbuf[2048];
    uint32_t len = n + 19;
    if (sdio_block && len > sdio_block) len = (len + sdio_block - 1) / sdio_block * sdio_block;
    uint8_t *buf = len <= sizeof stackbuf ? stackbuf : malloc(len);
    if (!buf) return -ENOMEM;
    if (len > n + 19) memset(buf + n + 19, 0, len - (n + 19));
    buf[0] = 0xAA;
    buf[1] = n; buf[2] = n >> 8; buf[3] = n >> 16; buf[4] = n >> 24;
    buf[5] = reqid >> 24; buf[6] = reqid >> 16; buf[7] = reqid >> 8; buf[8] = reqid;
    buf[9] = msgid >> 24; buf[10] = msgid >> 16; buf[11] = msgid >> 8; buf[12] = msgid;
    memset(buf + 13, 0, 4);
    uint8_t x = 0xFF;
    for (int i = 0; i < 17; i++) x ^= buf[i];
    buf[17] = x;
    if (n) memcpy(buf + 18, p, n);
    buf[18 + n] = 0xBB;
    // One write, one frame, on every driver.
    pthread_mutex_lock(&tx_lock);
    ssize_t w;
    do w = devfd >= 0 ? write(devfd, buf, len) : -1;
    while (w < 0 && errno == EINTR);
    int err = errno;
    pthread_mutex_unlock(&tx_lock);
    if (buf != stackbuf) free(buf);
    if (w != (ssize_t)len) {
        LOG("write of %u bytes failed: %s\n", len, w < 0 ? strerror(err) : "short");
        if (w < 0 && devfd >= 0 && err == ENODEV) device_gone(strerror(err));
        return -EIO;
    }
    return 0;
}

static uint32_t sock_key(uint32_t op, uint32_t slot, uint32_t port) {
    return op << 16 | (slot & 0xff) << 8 | (port & 0xff);
}

// A reply: complete the request it answers - by msgid, else by (op, slot,
// port). The chip does not always echo a socket request's msgid (a write is
// answered twice, and not both replies carry it), so a reply whose msgid
// matches nothing is matched by key too: dropping those leaves the video
// socket without its data. The cost: a late reply to a request that timed
// out can complete the next one of its kind.
static void complete(uint32_t msgid, uint32_t key, int32_t sta, const uint8_t *p, uint32_t n) {
    pthread_mutex_lock(&st_lock);
    struct pending *q = NULL;
    if (msgid)
        for (int i = 0; i < MAX_PENDING; i++)
            if (pend[i].used && !pend[i].done && pend[i].id == msgid) { q = &pend[i]; break; }
    if (!q && key)
        for (int i = 0; i < MAX_PENDING; i++)
            if (pend[i].used && !pend[i].done && pend[i].key == key) { q = &pend[i]; break; }
    // A write is answered twice; only the second reply carries the count.
    if (q && !(q->is_write && sta == WRITE_PENDING_STA)) {
        q->sta = sta;
        q->got = 0;
        if (q->out && n) {
            q->got = n < q->cap ? n : q->cap;
            memcpy(q->out, p, q->got);
        }
        q->done = 1;
        pthread_cond_broadcast(&st_cond);
    }
    pthread_mutex_unlock(&st_lock);
}

static void sock_push(uint32_t slot, uint32_t port, const uint8_t *p, uint32_t n) {
    for (int i = 0; i < MAX_SOCKETS; i++) {
        struct sock *s = &socks[i];
        if (!s->ring) continue;                   // never opened
        pthread_mutex_lock(&s->m);
        if (!s->used || !s->ring || s->slot != slot || s->port != port) {
            pthread_mutex_unlock(&s->m);
            continue;
        }
        if (n > s->cap - s->count) {          // the reader is behind: keep what fits
            s->dropped += n - (s->cap - s->count);
            n = s->cap - s->count;
        }
        uint32_t tail = (s->head + s->count) % s->cap;
        uint32_t first = n < s->cap - tail ? n : s->cap - tail;
        memcpy(s->ring + tail, p, first);
        memcpy(s->ring, p + first, n - first);
        s->count += n;
        pthread_cond_signal(&s->c);
        pthread_mutex_unlock(&s->m);
        return;
    }
}

static void on_frame(uint32_t reqid, uint32_t msgid, int32_t sta, const uint8_t *p, uint32_t n) {
    uint32_t dom = reqid >> 24, sub = reqid & 0xffffff;
    switch (dom) {
    case DOM_SOCKET: {
        uint32_t op = sub >> 16, slot = sub >> 8 & 0xff, port = sub & 0xff;
        if (op == SO_READ && sta == SOCK_DATA_STA) { sock_push(slot, port, p, n); return; }
        complete(msgid, sock_key(op, slot, port), sta, p, n);
        return;
    }
    case DOM_CB:
        if ((sub >> 16) == 2) {               // an event; sub 1 is a subscription ack
            uint32_t ev = sub & 0xff;
            if (ev >= MAX_EVENTS) return;
            pthread_mutex_lock(&st_lock);
            bb_event_callback cb = events[ev].cb;
            arlink_event_cb cb_len = events[ev].cb_len;
            void *user = events[ev].user;
            pthread_mutex_unlock(&st_lock);
            if (cb || cb_len) {
                // Zeroed past the payload: a callback reads its event's
                // struct whole, however short the chip's payload.
                uint8_t copy[256] = { 0 };
                uint32_t m = n < sizeof copy ? n : sizeof copy;
                memcpy(copy, p, m);
                if (cb_len) cb_len(copy, m, user);
                else cb(copy, user);
            }
        }
        return;
    case DOM_DBG:
        chiplog_write(p, n);
        if (debug > 1) fwrite(p, 1, n, stderr);
        return;
    case DOM_LINK:
        if (sub == 0 && n >= 8) {
            pthread_mutex_lock(&st_lock);
            hello_seen = 1;
            pthread_cond_broadcast(&st_cond);
            pthread_mutex_unlock(&st_lock);
        }
        return;
    default:
        complete(msgid, 0, sta, p, n);
    }
}

// arlink.ko hands over one frame per read; Artosyn's driver what each USB
// transfer carried, which can split or join frames. Treating both as a byte
// stream to split covers either.
static void *reader(void *arg) {
    uint8_t *acc = arg;                           // rd_buf: disconnect frees it, cancelled or not
    const size_t acc_size = RD_BUF_SIZE;
    size_t have = 0;
    unsigned long long bad = 0;
    while (running) {
        ssize_t r = read(devfd, acc + have, acc_size - have);
        if (r <= 0) {
            if (r < 0 && errno == EINTR) continue;
            if (!running) break;
            // Gone: retrying would only spin on a descriptor that stays dead,
            // while the chip comes back under a new one.
            if (r < 0 && (errno == ENODEV || errno == EIO || errno == EPIPE)) {
                device_gone(strerror(errno));
                break;
            }
            LOG("read: %s\n", r < 0 ? strerror(errno) : "end of file");
            usleep(10000);
            continue;
        }
        if (sdio_block) {                         // frames start on block boundaries
            for (size_t o = 0; o + 19 <= (size_t)r;) {
                const uint8_t *f = acc + o;
                uint32_t n = f[1] | f[2] << 8 | f[3] << 16 | (uint32_t)f[4] << 24;
                uint8_t x = 0xFF;
                for (int i = 0; i < 17; i++) x ^= f[i];
                if (f[0] != 0xAA || x != f[17] || n > (size_t)r - o - 19 || f[18 + n] != 0xBB) {
                    bad++;
                    o += sdio_block;              // not a frame: the next block
                    continue;
                }
                uint32_t reqid = (uint32_t)f[5] << 24 | f[6] << 16 | f[7] << 8 | f[8];
                uint32_t msgid = (uint32_t)f[9] << 24 | f[10] << 16 | f[11] << 8 | f[12];
                int32_t sta = (int32_t)((uint32_t)f[13] << 24 | f[14] << 16 | f[15] << 8 | f[16]);
                on_frame(reqid, msgid, sta, f + 18, n);
                o += (19 + n + sdio_block - 1) / sdio_block * sdio_block;
            }
            continue;
        }
        have += r;
        size_t o = 0;
        while (have - o >= 19) {
            const uint8_t *f = acc + o;
            if (f[0] != 0xAA) { o++; bad++; continue; }
            uint32_t n = f[1] | f[2] << 8 | f[3] << 16 | (uint32_t)f[4] << 24;
            uint8_t x = 0xFF;
            for (int i = 0; i < 17; i++) x ^= f[i];
            if (x != f[17] || n > acc_size - 19) { o++; bad++; continue; }
            if (have - o < 19 + (size_t)n) break;
            if (f[18 + n] != 0xBB) { o++; bad++; continue; }
            uint32_t reqid = (uint32_t)f[5] << 24 | f[6] << 16 | f[7] << 8 | f[8];
            uint32_t msgid = (uint32_t)f[9] << 24 | f[10] << 16 | f[11] << 8 | f[12];
            int32_t sta = (int32_t)((uint32_t)f[13] << 24 | f[14] << 16 | f[15] << 8 | f[16]);
            on_frame(reqid, msgid, sta, f + 18, n);
            o += 19 + n;
        }
        memmove(acc, acc + o, have - o);
        have -= o;
        if (have == acc_size) have = 0;          // no frame fits: start over
    }
    if (sdio_block) LOG("reader done, %llu blocks without a frame\n", bad);
    else LOG("reader done, %llu bytes skipped resyncing\n", bad);
    return NULL;
}

static void *heartbeat(void *arg) {
    (void)arg;
    while (running && !gone()) {
        send_frame((uint32_t)DOM_LINK << 24 | 2, 0, NULL, 0);
        usleep(120000);
    }
    return NULL;
}

// Send a request and wait for its reply. Returns 0 with the reply's status in
// *sta, or a negative errno when nothing came back in time.
static int request(uint32_t reqid, const void *in, uint32_t inlen, void *out, uint32_t cap,
                   int timeout_ms, uint32_t key, int is_write, int32_t *sta, uint32_t *got) {
    if (devfd < 0 || gone()) return -ENODEV;
    if (timeout_ms <= 0) timeout_ms = DEFAULT_TIMEOUT_MS;
    pthread_mutex_lock(&st_lock);
    struct pending *q = NULL;
    for (int i = 0; i < MAX_PENDING; i++)
        if (!pend[i].used) { q = &pend[i]; break; }
    if (!q) { pthread_mutex_unlock(&st_lock); return -EBUSY; }
    if (++next_id == 0) next_id = 1;
    *q = (struct pending){ .used = 1, .id = next_id, .key = key, .is_write = is_write,
                           .out = out, .cap = cap };
    uint32_t id = q->id;
    pthread_mutex_unlock(&st_lock);

    int r = send_frame(reqid, id, in, inlen);

    struct timespec dl;
    clock_gettime(CLOCK_REALTIME, &dl);
    dl.tv_sec += timeout_ms / 1000;
    dl.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (dl.tv_nsec >= 1000000000L) { dl.tv_sec++; dl.tv_nsec -= 1000000000L; }
    pthread_mutex_lock(&st_lock);
    while (r == 0 && !q->done && !gone())
        if (pthread_cond_timedwait(&st_cond, &st_lock, &dl) == ETIMEDOUT) break;
    if (r == 0 && !q->done) r = -ETIMEDOUT;
    if (r && gone()) r = -ENODEV;
    if (r == 0) {
        if (sta) *sta = q->sta;
        if (got) *got = q->got;
    }
    q->used = 0;
    pthread_mutex_unlock(&st_lock);
    if (r) LOG("request %08x: %s\n", reqid, strerror(-r));
    return r;
}

// ---- the API --------------------------------------------------------------------

// arlink.ko first: the lowest /dev/arlinkN that opens (a chip that came back
// while its old device was still open can have another number); if there is
// one but none opens, why not. Then Artosyn's USB (goggle) or SDIO (air)
// driver.
static int open_dev(const char **path) {
    static char name[32], first[32];
    int fd, err = 0;
    for (int i = 0; i < 16; i++) {                // arlink.ko makes at most 8
        snprintf(name, sizeof name, "/dev/arlink%d", i);
        fd = open(name, O_RDWR | O_CLOEXEC);
        if (fd >= 0) { *path = name; return fd; }
        if (errno != ENOENT && !err) { err = errno; memcpy(first, name, sizeof first); }
    }
    if (err) { *path = first; errno = err; return -1; }
    static const char *const devs[] = { "/dev/ar_mdev0", "/dev/artosyn_sdio" };
    for (size_t i = 0; i < sizeof devs / sizeof devs[0]; i++) {
        *path = devs[i];
        fd = open(*path, O_RDWR | O_CLOEXEC);
        if (fd >= 0 || errno != ENOENT) break;
    }
    return fd;
}

API int bb_host_connect(bb_host_t **phost, const char *addr, int port) {
    (void)addr; (void)port;                      // no daemon to connect to
    if (devfd >= 0 && gone()) bb_host_disconnect(&the_host);   // start again on the chip as it is now
    const char *d = getenv("ARLINK_DEBUG");
    debug = d ? atoi(d) : 0;
    nofast = getenv("ARLINK_NOFAST") != NULL;
    chiplog_path = getenv("ARLINK_CHIPLOG");
    if (chiplog_path && chiplog_fd < 0) chiplog_open();
    if (devfd >= 0) { *phost = &the_host; return 0; }
    const char *path = getenv("ARLINK_DEV");
    if (path) devfd = open(path, O_RDWR | O_CLOEXEC);
    else devfd = open_dev(&path);
    sdio_block = devfd >= 0 && strcmp(path, "/dev/artosyn_sdio") == 0 ? 512 : 0;
    if (devfd < 0) {
        fprintf(stderr, "arlink: %s: %s (is the AR8030 driver loaded, and nothing else using it?)\n",
                path, strerror(errno));
        return -1;
    }
    memset(socks, 0, sizeof socks);
    hello_seen = 0;
    __atomic_store_n(&dev_gone, 0, __ATOMIC_RELEASE);
    running = 1;
    rd_buf = malloc(RD_BUF_SIZE);
    int e = rd_buf ? pthread_create(&rd_thread, NULL, reader, rd_buf) : ENOMEM;
    if (e) {
        fprintf(stderr, "arlink: reader thread: %s\n", strerror(e));
        running = 0;
        free(rd_buf);
        rd_buf = NULL;
        close(devfd);
        devfd = -1;
        return -1;
    }

    // Link start, as the daemon does it: resets, then hello until answered.
    for (int i = 0; i < 10; i++) send_frame((uint32_t)DOM_LINK << 24 | 1, 0, NULL, 0);
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (;;) {
        send_frame((uint32_t)DOM_LINK << 24 | 0, 0, NULL, 0);
        usleep(20000);
        pthread_mutex_lock(&st_lock);
        int seen = hello_seen;
        pthread_mutex_unlock(&st_lock);
        if (seen) break;
        clock_gettime(CLOCK_MONOTONIC, &t);
        if ((t.tv_sec - t0.tv_sec) * 1000 + (t.tv_nsec - t0.tv_nsec) / 1000000 > 3000) {
            fprintf(stderr, "arlink: no hello from the chip on %s\n", path);
            bb_host_disconnect(&the_host);
            return -1;
        }
    }
    e = pthread_create(&hb_thread, NULL, heartbeat, NULL);
    if (e) {
        fprintf(stderr, "arlink: heartbeat thread: %s\n", strerror(e));
        bb_host_disconnect(&the_host);
        return -1;
    }
    hb_started = 1;

    // The firmware log on, and every event subscribed (the chip refuses
    // event 4; the daemon skips 5).
    static const uint8_t zero4[4];
    send_frame((uint32_t)DOM_DBG << 24, 0, zero4, 4);
    for (int ev = 13; ev >= 0; ev--) {
        if (ev == 5) continue;
        send_frame((uint32_t)DOM_CB << 24 | 0x010000 | ev, 0, NULL, 0);
        usleep(2000);
    }
    LOG("connected on %s\n", path);
    *phost = &the_host;
    return 0;
}

API int bb_host_disconnect(bb_host_t *phost) {
    (void)phost;
    if (devfd < 0) return 0;
    // Close the sockets cleanly: a chip left with open sockets can wedge its
    // OUT endpoint until it is power-cycled.
    for (int i = 0; i < MAX_SOCKETS; i++)
        if (socks[i].used) bb_socket_close(i);
    running = 0;
    // The reader sits in read(); the heartbeat's echo wakes it.
    send_frame((uint32_t)DOM_LINK << 24 | 2, 0, NULL, 0);
    struct timespec dl;
    clock_gettime(CLOCK_REALTIME, &dl);
    dl.tv_sec += 1;
    if (pthread_timedjoin_np(rd_thread, NULL, &dl) != 0) {
        pthread_cancel(rd_thread);
        pthread_join(rd_thread, NULL);
    }
    free(rd_buf);
    rd_buf = NULL;
    // Only a heartbeat that was started: a hello that comes in late, after
    // connect has given up and called this, does not make one.
    if (hb_started) {
        pthread_join(hb_thread, NULL);
        hb_started = 0;
    }
    close(devfd);
    devfd = -1;
    LOG("disconnected\n");
    return 0;
}

API int bb_dev_getlist(bb_host_t *phost, bb_dev_list_t **plist) {
    (void)phost;
    if (!plist || devfd < 0 || gone()) return -1;
    bb_dev_list_t *list = calloc(2, sizeof *list);
    if (!list) return -1;
    list[0] = &the_dev;                          // one baseband per goggle
    *plist = list;
    return 1;
}

API int bb_dev_freelist(bb_dev_list_t *plist) {
    free(plist);
    return 0;
}

API bb_dev_handle_t *bb_dev_open(bb_dev_t *dev) {
    return dev == &the_dev && devfd >= 0 && !gone() ? &the_handle : NULL;
}

API int bb_dev_close(bb_dev_handle_t *handle) {
    (void)handle;
    return 0;
}

API int bb_ioctl_ex(bb_dev_handle_t *dev, uint32_t request_code, const void *in, void *out,
                    int timeout_ms) {
    (void)dev;
    if (request_code == REQ_EVENT_SUBSCRIBE) {   // a client callback: kept here
        const bb_set_event_callback_t *e = in;
        if (!e || e->event < 0 || e->event >= MAX_EVENTS) return -1;
        pthread_mutex_lock(&st_lock);
        events[e->event].cb = e->callback;
        events[e->event].cb_len = NULL;
        events[e->event].user = e->user;
        pthread_mutex_unlock(&st_lock);
        return 0;
    }
    int outsz;
    int ipt = request_sizes(request_code, &outsz);
    if (ipt < 0) {
        LOG("request %08x: not in the size table\n", request_code);
        return -2;
    }
    uint8_t zeros[2048];
    if (ipt > 0 && !in) {                        // a request with input but none given
        if ((size_t)ipt > sizeof zeros) return -1;
        memset(zeros, 0, ipt);
        in = zeros;
    }
    int32_t sta = 0;
    int r = request(request_code, in, (uint32_t)ipt, out, out ? (outsz > 0 ? (uint32_t)outsz : 4096) : 0,
                    timeout_ms, 0, 0, &sta, NULL);
    return r ? r : sta;
}

API int bb_ioctl(bb_dev_handle_t *dev, uint32_t request_code, const void *in, void *out) {
    return bb_ioctl_ex(dev, request_code, in, out, DEFAULT_TIMEOUT_MS);
}

API int bb_init(bb_dev_handle_t *handle) { return bb_ioctl_ex(handle, REQ_INIT, NULL, NULL, 0); }
API int bb_start(bb_dev_handle_t *handle) { return bb_ioctl_ex(handle, REQ_START, NULL, NULL, 0); }

API int bb_socket_open(bb_dev_handle_t *dev, int slot, uint32_t port, uint32_t flag, bb_sock_opt_t *opt) {
    (void)dev;
    int fd = -1;
    for (int i = 0; i < MAX_SOCKETS; i++)
        if (!socks[i].used) { fd = i; break; }
    if (fd < 0) return -1;
    uint32_t tx = opt ? opt->tx_buf_size : 0x800, rx = opt ? opt->rx_buf_size : 0x800;
    struct sock *s = &socks[fd];
    *s = (struct sock){ .slot = slot, .port = port, .sfd = -1 };
    pthread_mutex_init(&s->m, NULL);
    pthread_cond_init(&s->c, NULL);
    pthread_mutex_init(&s->rd, NULL);
    // The socket's own file on arlink.ko, attached before the open so that
    // none of its data goes the other way. Anything else: the ring.
    if (!nofast) {
        struct arlink_sock_attach a = { .slot = (uint8_t)slot, .port = (uint8_t)port };
        int sfd = ioctl(devfd, ARLINK_IOC_SOCK_ATTACH, &a);
        // Only arlink.ko answers this with a socket file. Artosyn's SDIO
        // driver returns 0 to an ioctl it does not know, which is not one.
        if (sfd >= 0) {
            char link[64], self[32];
            snprintf(self, sizeof self, "/proc/self/fd/%d", sfd);
            ssize_t n = readlink(self, link, sizeof link - 1);
            if (n <= 0 || (link[n] = 0, strcmp(link, "anon_inode:[arlink-socket]") != 0))
                sfd = -1;
        }
        uint8_t *stage = sfd >= 0 ? malloc(STAGE_SIZE) : NULL;
        if (stage) {
            fcntl(sfd, F_SETFL, O_NONBLOCK);     // waits are poll()s
            s->sfd = sfd;
            s->stage = stage;
        } else if (sfd >= 0) {
            close(sfd);
        }
    }
    uint32_t cap = 0;
    uint8_t *ring = NULL;
    if (s->sfd < 0) {
        // Room for bursts: the client reads in its own time, the chip does not wait.
        cap = rx * 4 > (1u << 16) ? rx * 4 : (1u << 16);
        ring = malloc(cap);
        if (!ring) return -1;
        s->ring = ring;
        s->cap = cap;
    }
    s->used = 1;                                 // before the open: data can follow at once
    uint8_t p[12];
    memcpy(p, &flag, 4); memcpy(p + 4, &tx, 4); memcpy(p + 8, &rx, 4);
    int32_t sta = 0;
    uint32_t key = sock_key(SO_OPEN, slot, port);
    int r = request((uint32_t)DOM_SOCKET << 24 | key, p, sizeof p, NULL, 0, DEFAULT_TIMEOUT_MS,
                    key, 0, &sta, NULL);
    // Refused: most likely still open from a program that died without
    // closing it - a killed stock daemon, or anything on a driver that does
    // not clean up (Artosyn's). Close it and try once more.
    if (r == 0 && sta != 0) {
        uint32_t ckey = sock_key(SO_CLOSE, slot, port);
        int32_t csta = 0;
        LOG("socket open slot %d port %u refused (%d): closing it and retrying\n", slot, port, sta);
        request((uint32_t)DOM_SOCKET << 24 | ckey, NULL, 0, NULL, 0, 1000, ckey, 0, &csta, NULL);
        sta = 0;
        r = request((uint32_t)DOM_SOCKET << 24 | key, p, sizeof p, NULL, 0, DEFAULT_TIMEOUT_MS,
                    key, 0, &sta, NULL);
    }
    if (r || sta != 0) {
        LOG("socket open slot %d port %u: %s sta=%d\n", slot, port, r ? strerror(-r) : "refused", sta);
        pthread_mutex_lock(&s->m);
        s->used = 0;
        s->ring = NULL;
        pthread_mutex_unlock(&s->m);
        free(ring);
        if (s->sfd >= 0) {
            close(s->sfd);
            s->sfd = -1;
            free(s->stage);
            s->stage = NULL;
        }
        return r ? r : (sta < 0 ? sta : -1);
    }
    if (s->sfd >= 0)
        LOG("socket %d: slot %d port %u, its own file from arlink.ko\n", fd, slot, port);
    else
        LOG("socket %d: slot %d port %u, rx ring %u KB\n", fd, slot, port, cap / 1024);
    return fd;
}

static uint64_t mono_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

// One frame from the socket's own file into the stage. 1 if there was one,
// 0 if none is queued, -1 on an error.
static int sock_next_frame(struct sock *s) {
    ssize_t n = read(s->sfd, s->stage, STAGE_SIZE);
    if (n < 0) {
        if (errno == EAGAIN) return 0;
        if (errno == ENODEV || errno == EIO) device_gone(strerror(errno));
        return -1;
    }
    if (n <= (ssize_t)sizeof(struct arlink_sock_rec)) return -1;
    struct arlink_sock_rec rec;
    memcpy(&rec, s->stage, sizeof rec);
    s->stage_rx_ns = rec.rx_ns;
    s->stage_off = sizeof rec;
    s->stage_len = (uint32_t)n - sizeof rec;
    return 1;
}

// From the socket's own file: everything queued, as much as fits, like the
// ring (and Artosyn's library) - a caller that does other work between reads
// would fall behind a frame at a time. rx_ns is when the last of it arrived.
static int sock_read_file(struct sock *s, void *buf, uint32_t len, int timeout_ms) {
    pthread_mutex_lock(&s->rd);
    if (!__atomic_load_n(&s->used, __ATOMIC_ACQUIRE) || s->sfd < 0) {
        pthread_mutex_unlock(&s->rd);
        return -1;
    }
    uint32_t total = 0;
    uint64_t t0 = 0;
    while (total < len) {
        if (s->stage_len == 0) {
            int r = sock_next_frame(s);
            if (r < 0) break;
            if (r == 0) {
                if (total) break;               // all that was queued
                // Nothing yet: wait, in short slices so that a close (which
                // takes rd) is not held up.
                if (!t0) t0 = mono_ms();
                int slice = 100;
                if (timeout_ms > 0) {
                    int64_t left = (int64_t)timeout_ms - (int64_t)(mono_ms() - t0);
                    if (left <= 0) break;
                    if (left < slice) slice = (int)left;
                }
                struct pollfd p = { .fd = s->sfd, .events = POLLIN };
                int pr = poll(&p, 1, slice);
                if (pr > 0 && !(p.revents & POLLIN) && (p.revents & (POLLHUP | POLLERR)))
                    device_gone("hangup");
                if ((pr > 0 && !(p.revents & POLLIN)) || (pr < 0 && errno != EINTR) ||
                    !__atomic_load_n(&s->used, __ATOMIC_ACQUIRE) || gone())
                    break;
                continue;
            }
        }
        uint32_t m = s->stage_len < len - total ? s->stage_len : len - total;
        memcpy((uint8_t *)buf + total, s->stage + s->stage_off, m);
        s->stage_off += m;
        s->stage_len -= m;
        total += m;
        s->rx_ns = s->stage_rx_ns;
    }
    pthread_mutex_unlock(&s->rd);
    return total ? (int)total : -1;
}

API int bb_socket_read(int sockfd, void *buf, uint32_t len, int timeout_ms) {
    if (sockfd < 0 || sockfd >= MAX_SOCKETS || !socks[sockfd].used || gone()) return -1;
    struct sock *s = &socks[sockfd];
    if (s->sfd >= 0) return sock_read_file(s, buf, len, timeout_ms);
    pthread_mutex_lock(&s->m);
    if (s->count == 0) {
        if (timeout_ms <= 0) {
            while (s->count == 0 && s->used && !gone()) pthread_cond_wait(&s->c, &s->m);
        } else {
            struct timespec dl;
            clock_gettime(CLOCK_REALTIME, &dl);
            dl.tv_sec += timeout_ms / 1000;
            dl.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
            if (dl.tv_nsec >= 1000000000L) { dl.tv_sec++; dl.tv_nsec -= 1000000000L; }
            while (s->count == 0 && s->used && !gone())
                if (pthread_cond_timedwait(&s->c, &s->m, &dl) == ETIMEDOUT) break;
        }
    }
    uint32_t n = s->used && s->ring ? (s->count < len ? s->count : len) : 0;
    if (n) {
        uint32_t first = n < s->cap - s->head ? n : s->cap - s->head;
        memcpy(buf, s->ring + s->head, first);
        memcpy((uint8_t *)buf + first, s->ring, n - first);
        s->head = (s->head + n) % s->cap;
        s->count -= n;
    }
    pthread_mutex_unlock(&s->m);
    return n ? (int)n : -1;                      // -1: nothing came in time
}

// The largest frame the stock air daemon sends over SDIO (twenty blocks), and
// so the largest we send anywhere: the goggle's are far smaller anyway, and
// on the air unit it applies through arlink.ko as much as through Artosyn's
// driver.
#define SDIO_FRAME_MAX 10240

API int bb_socket_write(int sockfd, const void *buf, uint32_t len, int timeout_ms) {
    if (sockfd < 0 || sockfd >= MAX_SOCKETS || !socks[sockfd].used) return -1;
    struct sock *s = &socks[sockfd];
    // One write is one frame of at most SDIO_FRAME_MAX, as the stock air
    // daemon sends them; the rest is the caller's to write again, as with any
    // write the chip takes only part of.
    if (len > SDIO_FRAME_MAX - 19 - 8) len = SDIO_FRAME_MAX - 19 - 8;
    uint8_t *p = malloc(8 + len);
    if (!p) return -1;
    uint32_t off = s->written;                   // the header: data bytes before this write
    memcpy(p, &off, 4);
    memset(p + 4, 0, 4);
    memcpy(p + 8, buf, len);
    int32_t sta = 0;
    uint32_t key = sock_key(SO_WRITE, s->slot, s->port);
    int r = request((uint32_t)DOM_SOCKET << 24 | key, p, 8 + len, NULL, 0, timeout_ms, key, 1, &sta, NULL);
    free(p);
    if (r) return -1;
    if (sta > 0) s->written = off + (uint32_t)sta;   // what the chip took, not what was offered
    return sta;                                  // the bytes the chip took
}

API int bb_socket_close(int sockfd) {
    if (sockfd < 0 || sockfd >= MAX_SOCKETS || !socks[sockfd].used) return -1;
    struct sock *s = &socks[sockfd];
    uint32_t key = sock_key(SO_CLOSE, s->slot, s->port);
    int32_t sta = 0;
    request((uint32_t)DOM_SOCKET << 24 | key, NULL, 0, NULL, 0, 1000, key, 0, &sta, NULL);
    pthread_mutex_lock(&s->m);
    __atomic_store_n(&s->used, 0, __ATOMIC_RELEASE);
    uint8_t *ring = s->ring;
    s->ring = NULL;
    pthread_cond_broadcast(&s->c);               // a reader waiting forever gives up
    pthread_mutex_unlock(&s->m);
    if (s->dropped) LOG("socket %d: %llu bytes dropped (reader behind)\n", sockfd, s->dropped);
    free(ring);
    if (s->sfd >= 0) {
        pthread_mutex_lock(&s->rd);              // the reader sees used == 0 within a slice
        close(s->sfd);
        s->sfd = -1;
        free(s->stage);
        s->stage = NULL;
        s->stage_len = 0;
        pthread_mutex_unlock(&s->rd);
    }
    return 0;
}

// A program that exits without disconnecting (most do) would leave its
// sockets open on the chip. arlink.ko closes them itself; on Artosyn's drivers
// nothing does, and the next owner's open is refused. So disconnect at exit.
__attribute__((destructor)) static void disconnect_at_exit(void) {
    if (devfd >= 0) bb_host_disconnect(&the_host);
}

// ---- ar_libre's own -----------------------------------------------------------
API const char *arlink_version(void) {
    return "ar_libre " ARLINK_VERSION;
}

API int arlink_request(uint32_t request_code, const void *in, uint32_t inlen, void *out,
                       uint32_t outcap, uint32_t *got, int timeout_ms) {
    int32_t sta = 0;
    uint32_t n = 0;
    int r = request(request_code, in, inlen, out, out ? outcap : 0, timeout_ms, 0, 0, &sta, &n);
    if (got) *got = n;
    return r ? r : sta;
}

API int arlink_subscribe(int event, arlink_event_cb cb, void *user) {
    if (event < 0 || event >= MAX_EVENTS) return -1;
    pthread_mutex_lock(&st_lock);
    events[event].cb = NULL;
    events[event].cb_len = cb;
    events[event].user = user;
    pthread_mutex_unlock(&st_lock);
    return 0;
}

API uint64_t arlink_socket_rx_ns(int sockfd) {
    if (sockfd < 0 || sockfd >= MAX_SOCKETS || !socks[sockfd].used || socks[sockfd].sfd < 0)
        return 0;
    return socks[sockfd].rx_ns;
}
