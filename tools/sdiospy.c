// SPDX-License-Identifier: GPL-2.0-or-later
//
// sdiospy: LD_PRELOAD shim that logs what a program sends to and receives
// from an AR8030 device file (/dev/artosyn_sdio, /dev/ar_mdev0, /dev/arlink0):
// every read, write and ioctl with a timestamp, its length and its bytes.
// The SDIO transport has no usbmon; this is how to see it.
//
//   LD_PRELOAD=./sdiospy.so SDIOSPY_LOG=/tmp/sdio.log SDIOSPY_BYTES=64 prog...
//
// SDIOSPY_BYTES: how many bytes of each transfer to print (default 64).
// SDIOSPY_MAX: stop logging after this many lines (default 200000).
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NFD 1024
static char watched[NFD];
static FILE *lf;
static int nbytes = 64;
static long lines, maxlines = 200000;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

static void setup(void) {
    if (lf) return;
    const char *p = getenv("SDIOSPY_LOG");
    lf = fopen(p ? p : "/tmp/sdiospy.log", "a");
    if (getenv("SDIOSPY_BYTES")) nbytes = atoi(getenv("SDIOSPY_BYTES"));
    if (getenv("SDIOSPY_MAX")) maxlines = atol(getenv("SDIOSPY_MAX"));
    if (lf) setvbuf(lf, NULL, _IOLBF, 0);
}

static int is_dev(const char *p) {
    return p && (!strcmp(p, "/dev/artosyn_sdio") || !strcmp(p, "/dev/ar_mdev0") ||
                 !strcmp(p, "/dev/arlink0"));
}

static void logx(const char *op, int fd, long arg, long r, const void *buf, long len) {
    pthread_mutex_lock(&mtx);
    setup();
    if (lf && lines++ < maxlines) {
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        fprintf(lf, "%ld.%06ld %s fd=%d arg=%#lx ret=%ld", (long)t.tv_sec, t.tv_nsec / 1000, op, fd, arg, r);
        if (buf && len > 0) {
            fputs(" |", lf);
            for (long i = 0; i < len && i < nbytes; i++) fprintf(lf, " %02x", ((const unsigned char *)buf)[i]);
            if (len > nbytes) fputs(" ...", lf);
        }
        fputc('\n', lf);
    }
    pthread_mutex_unlock(&mtx);
}

int open(const char *path, int flags, ...) {
    static int (*real)(const char *, int, ...);
    if (!real) real = dlsym(RTLD_NEXT, "open");
    mode_t mode = 0;
    if (flags & O_CREAT) {
        va_list ap;
        va_start(ap, flags);
        mode = va_arg(ap, mode_t);
        va_end(ap);
    }
    int fd = real(path, flags, mode);
    if (fd >= 0 && fd < NFD) {
        watched[fd] = (char)is_dev(path);
        if (watched[fd]) logx("open", fd, flags, fd, path, (long)strlen(path));
    }
    return fd;
}

int close(int fd) {
    static int (*real)(int);
    if (!real) real = dlsym(RTLD_NEXT, "close");
    if (fd >= 0 && fd < NFD && watched[fd]) {
        watched[fd] = 0;
        logx("close", fd, 0, 0, NULL, 0);
    }
    return real(fd);
}

ssize_t read(int fd, void *buf, size_t n) {
    static ssize_t (*real)(int, void *, size_t);
    if (!real) real = dlsym(RTLD_NEXT, "read");
    ssize_t r = real(fd, buf, n);
    if (fd >= 0 && fd < NFD && watched[fd]) logx("read", fd, (long)n, (long)r, buf, r);
    return r;
}

ssize_t write(int fd, const void *buf, size_t n) {
    static ssize_t (*real)(int, const void *, size_t);
    if (!real) real = dlsym(RTLD_NEXT, "write");
    if (fd >= 0 && fd < NFD && watched[fd]) logx("write>", fd, (long)n, 0, buf, (long)n);
    ssize_t r = real(fd, buf, n);
    if (fd >= 0 && fd < NFD && watched[fd]) logx("write<", fd, (long)n, (long)r, NULL, 0);
    return r;
}

// musl: int ioctl(int, int, ...)
int ioctl(int fd, int req, ...) {
    static int (*real)(int, int, ...);
    if (!real) real = dlsym(RTLD_NEXT, "ioctl");
    va_list ap;
    va_start(ap, req);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    int w = fd >= 0 && fd < NFD && watched[fd];
    unsigned size = ((unsigned)req >> 16) & 0x3fff;
    if (w) logx("ioctl>", fd, (long)(unsigned)req, 0, arg, size);
    int r = real(fd, req, arg);
    if (w) logx("ioctl<", fd, (long)(unsigned)req, r, arg, size);
    return r;
}
