// SPDX-License-Identifier: GPL-2.0-or-later
//
// arlink-test: exercise an AR8030 through libarlink, with no daemon and no
// application: connect, read the chip's status, follow its link and MCS
// events, open the video socket and write to it, counting what the chip takes.
// Runs on either side (goggle: USB, air unit: SDIO); the transport is the
// library's business.
//
//   arlink-test [-t SECONDS] [-w BYTES] [-i MS] [-s] [-p PORT]
//
//   -t  how long to run (default 10)
//   -w  bytes per socket write, 0 for none (default 1000)
//   -i  ms between writes (default 20)
//   -s  send BB_INIT and BB_START first (the goggle needs them; the air unit
//       is already running when the stock app has had it)
//   -p  socket port (default 3, the video)
#include "../lib/arlink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define REQ_GET_STATUS      0x01000000u
#define REQ_EVENT_SUBSCRIBE 0x02000000u

static double now_s(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static double t0;

static void on_link(void *arg, void *user) {
    const unsigned char *p = arg;
    (void)user;
    printf("%7.3f event: link slot %u state %u -> %u\n", now_s() - t0, p[0], p[2], p[1]);
}

static void on_mcs(void *arg, void *user) {
    const unsigned char *p = arg;
    (void)user;
    printf("%7.3f event: mcs slot %u %s %u -> %u\n", now_s() - t0, p[0], p[1] ? "rx" : "tx", p[3], p[2]);
}

int main(int argc, char **argv) {
    int secs = 10, wbytes = 1000, gap_ms = 20, start = 0, port = 3, c;
    while ((c = getopt(argc, argv, "t:w:i:sp:")) != -1) {
        switch (c) {
        case 't': secs = atoi(optarg); break;
        case 'w': wbytes = atoi(optarg); break;
        case 'i': gap_ms = atoi(optarg); break;
        case 's': start = 1; break;
        case 'p': port = atoi(optarg); break;
        default:
            fprintf(stderr, "usage: %s [-t SECONDS] [-w BYTES] [-i MS] [-s] [-p PORT]\n", argv[0]);
            return 2;
        }
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    t0 = now_s();

    bb_host_t *host;
    if (bb_host_connect(&host, NULL, 0)) {
        fprintf(stderr, "connect failed\n");
        return 1;
    }
    bb_dev_list_t *list;
    if (bb_dev_getlist(host, &list) < 1) {
        fprintf(stderr, "no device\n");
        return 1;
    }
    bb_dev_handle_t *dev = bb_dev_open(list[0]);
    bb_dev_freelist(list);
    printf("%7.3f connected (%s)\n", now_s() - t0, arlink_version());

    if (start) {
        printf("%7.3f bb_init -> %d\n", now_s() - t0, bb_init(dev));
        printf("%7.3f bb_start -> %d\n", now_s() - t0, bb_start(dev));
    }

    unsigned char q[2] = { 0xff, 0x03 }, st[300];
    memset(st, 0, sizeof st);
    int r = bb_ioctl_ex(dev, REQ_GET_STATUS, q, st, 1000);
    printf("%7.3f status -> %d: role %s, mac %02X:%02X:%02X:%02X, link state %u\n", now_s() - t0, r,
           st[0] == 0 ? "AP (air)" : st[0] == 1 ? "DEV (ground)" : "?", st[6], st[7], st[8], st[9], st[0xfc]);

    bb_set_event_callback_t ev = { 0, on_link, NULL };
    bb_ioctl_ex(dev, REQ_EVENT_SUBSCRIBE, &ev, NULL, 1000);
    ev = (bb_set_event_callback_t){ 1, on_mcs, NULL };
    bb_ioctl_ex(dev, REQ_EVENT_SUBSCRIBE, &ev, NULL, 1000);

    int fd = -1;
    if (wbytes > 0) {
        bb_sock_opt_t opt = { 5, 0x800 };            // the stock application's
        fd = bb_socket_open(dev, 0, (uint32_t)port, 0x23, &opt);
        printf("%7.3f socket slot 0 port %d -> %d\n", now_s() - t0, port, fd);
    }

    unsigned char *data = wbytes > 0 ? malloc((size_t)wbytes) : NULL;
    long writes = 0, ok = 0, bytes = 0, fails = 0;
    double worst = 0, sum = 0;
    while (now_s() - t0 < secs) {
        if (fd < 0) {
            usleep(100000);
            continue;
        }
        for (int i = 0; i < wbytes; i++) data[i] = (unsigned char)(writes + i);
        double a = now_s();
        int w = bb_socket_write(fd, data, (uint32_t)wbytes, 1000);
        double d = now_s() - a;
        writes++;
        if (w > 0) {
            ok++;
            bytes += w;
            sum += d;
            if (d > worst) worst = d;
        } else if (++fails <= 5) {
            printf("%7.3f write %ld -> %d after %.1f ms\n", now_s() - t0, writes, w, d * 1000);
        }
        usleep((useconds_t)gap_ms * 1000);
    }
    if (writes)
        printf("%7.3f writes %ld, taken %ld (%ld bytes), failed %ld; ack mean %.2f ms, worst %.2f ms\n",
               now_s() - t0, writes, ok, bytes, fails, ok ? sum / ok * 1000 : 0, worst * 1000);

    if (fd >= 0) bb_socket_close(fd);
    bb_dev_close(dev);
    bb_host_disconnect(host);
    free(data);
    return 0;
}
