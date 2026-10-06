// SPDX-License-Identifier: GPL-2.0-or-later
// usbcap: capture one USB bus through the kernel's binary usbmon interface
// into a pcap file (LINKTYPE_USB_LINUX_MMAPPED) that Wireshark reads as-is.
//
//   usbcap -b 2 -o capture.pcap [-s 512] [-d DEVNUM] [-t SECONDS]
//
// -s truncates each transfer's captured payload to that many bytes when it is
// longer (the header and the true length are always kept): the AR8030 carries
// video in large bulk transfers whose contents are not protocol, and keeping
// them whole would make every capture hundreds of megabytes.
//
// Stops on SIGINT/SIGTERM or after -t seconds, and flushes after every packet
// so a capture cut short is still readable.
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

// From the kernel's drivers/usb/mon/mon_bin.c: the 64-byte event header,
// which is also exactly the pcap record header of LINKTYPE 220.
struct mon_bin_hdr {
    uint64_t id;
    unsigned char type, xfer_type, epnum, devnum;
    uint16_t busnum;
    char flag_setup, flag_data;
    int64_t ts_sec;
    int32_t ts_usec;
    int32_t status;
    uint32_t len_urb, len_cap;
    union { unsigned char setup[8]; struct { int32_t error_count, numdesc; } iso; } s;
    int32_t interval, start_frame;
    uint32_t xfer_flags, ndesc;
};
struct mon_get_arg { struct mon_bin_hdr *hdr; void *data; size_t alloc; };
#define MON_IOC_MAGIC      0x92
#define MON_IOCT_RING_SIZE _IO(MON_IOC_MAGIC, 4)
#define MON_IOCX_GETX      _IOW(MON_IOC_MAGIC, 10, struct mon_get_arg)

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

int main(int argc, char **argv) {
    int bus = -1, dev = -1, snap = 512, secs = 0, opt;
    const char *out = NULL;
    while ((opt = getopt(argc, argv, "b:d:o:s:t:")) != -1) {
        switch (opt) {
        case 'b': bus = atoi(optarg); break;
        case 'd': dev = atoi(optarg); break;
        case 'o': out = optarg; break;
        case 's': snap = atoi(optarg); break;
        case 't': secs = atoi(optarg); break;
        default: goto usage;
        }
    }
    if (bus < 0 || !out) {
usage:
        fprintf(stderr, "usage: usbcap -b BUS -o FILE.pcap [-s SNAPLEN] [-d DEVNUM] [-t SECONDS]\n");
        return 1;
    }

    char path[32];
    snprintf(path, sizeof path, "/dev/usbmon%d", bus);
    int fd = open(path, O_RDONLY);
    if (fd < 0) { perror(path); return 1; }
    // The largest ring the kernel allows, so bursts of video don't overrun it
    // between reads.
    ioctl(fd, MON_IOCT_RING_SIZE, 1200 * 4096);

    FILE *f = fopen(out, "wb");
    if (!f) { perror(out); return 1; }
    const uint32_t ghdr[6] = { 0xa1b2c3d4, 0x00040002, 0, 0, 65536 + 64, 220 };
    fwrite(ghdr, sizeof ghdr, 1, f);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (secs > 0) { signal(SIGALRM, on_signal); alarm(secs); }

    static unsigned char data[65536];
    struct mon_bin_hdr h;
    unsigned long long n = 0, bytes = 0;
    while (!stop) {
        struct mon_get_arg a = { &h, data, sizeof data };
        if (ioctl(fd, MON_IOCX_GETX, &a) < 0) {
            if (errno == EINTR) continue;
            perror("MON_IOCX_GETX");
            break;
        }
        if (dev >= 0 && h.devnum != dev) continue;
        uint32_t cap = h.len_cap > (uint32_t)snap ? (uint32_t)snap : h.len_cap;
        h.len_cap = cap;
        uint32_t rec[4] = { (uint32_t)h.ts_sec, (uint32_t)h.ts_usec, 64 + cap, 64 + h.len_urb };
        fwrite(rec, sizeof rec, 1, f);
        fwrite(&h, 64, 1, f);
        fwrite(data, cap, 1, f);
        fflush(f);
        n++; bytes += h.len_urb;
    }
    fclose(f);
    fprintf(stderr, "usbcap: %llu events, %llu bytes on the wire\n", n, bytes);
    return 0;
}
