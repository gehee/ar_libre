// SPDX-License-Identifier: GPL-2.0-or-later
//
// libarlink: the AR8030 host client API, without Artosyn's daemon.
//
// The functions and types below are the subset of Artosyn's client API
// (libar8030_client.so) that applications use, with the same names and
// calling conventions, so a program built against the vendor library runs on
// this one unchanged. The handles are opaque here: callers only pass them back.
#ifndef ARLINK_H
#define ARLINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bb_host_t bb_host_t;
typedef struct bb_dev_t bb_dev_t;
typedef struct bb_dev_t *bb_dev_list_t;
typedef struct bb_dev_handle_t bb_dev_handle_t;

typedef struct bb_sock_opt_t {
    uint32_t tx_buf_size;
    uint32_t rx_buf_size;
} bb_sock_opt_t;

typedef void (*bb_event_callback)(void *arg, void *user);

// The input of the event-subscribe request: which event, and the callback to
// run with the event's payload.
typedef struct {
    int event;
    bb_event_callback callback;
    void *user;
} bb_set_event_callback_t;

int bb_host_connect(bb_host_t **phost, const char *addr, int port);
int bb_host_disconnect(bb_host_t *phost);
int bb_dev_getlist(bb_host_t *phost, bb_dev_list_t **plist);
int bb_dev_freelist(bb_dev_list_t *plist);
bb_dev_handle_t *bb_dev_open(bb_dev_t *dev);
int bb_dev_close(bb_dev_handle_t *handle);
int bb_init(bb_dev_handle_t *handle);
int bb_start(bb_dev_handle_t *handle);
int bb_ioctl(bb_dev_handle_t *dev, uint32_t request, const void *in, void *out);
int bb_ioctl_ex(bb_dev_handle_t *dev, uint32_t request, const void *in, void *out, int timeout_ms);
int bb_socket_open(bb_dev_handle_t *dev, int slot, uint32_t port, uint32_t flag, bb_sock_opt_t *opt);
int bb_socket_read(int sockfd, void *buf, uint32_t len, int timeout_ms);
int bb_socket_write(int sockfd, const void *buf, uint32_t len, int timeout_ms);
int bb_socket_close(int sockfd);

// ar_libre's own, not in Artosyn's library. A program that may also run on
// that library should declare them weak and check for NULL.
//
// "ar_libre <version>".
const char *arlink_version(void);
// CLOCK_MONOTONIC, in ns, when the last of the data the last bb_socket_read()
// on this socket returned came off USB (the completion of its transfer, in
// the kernel). 0 when not known: Artosyn's driver, or ARLINK_NOFAST.
uint64_t arlink_socket_rx_ns(int sockfd);
// Any request, sizes given rather than looked up: request_code is the wire
// reqid (domain << 24 | sub-command), in/inlen its payload, the reply's
// payload copied to out (up to outcap) and its length to *got. Returns the
// reply's status (>= 0 is the chip's), or a negative errno: -ETIMEDOUT when
// nothing came back.
int arlink_request(uint32_t request_code, const void *in, uint32_t inlen, void *out,
                   uint32_t outcap, uint32_t *got, int timeout_ms);
// An event's payload with its length (bb_set_event_callback_t's callback
// gets only the payload). Replaces any callback set for that event.
typedef void (*arlink_event_cb)(const uint8_t *data, uint32_t len, void *user);
int arlink_subscribe(int event, arlink_event_cb cb, void *user);

#ifdef __cplusplus
}
#endif

#endif
