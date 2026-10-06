# arlink.ko

One module for both sides: USB (a ground unit) and SDIO (an air unit). `core.c` is the frame pipe and the firmware upload; `usb.c` and
`sdio.c` are the buses, each built when the kernel has it.

The USB side follows USB captures of a stock ground unit and
[protocol.md](protocol.md); the SDIO side is below.

## Boot ROM upload (`4152:8030`)

At power-on the chip enumerates as `4152:8030` ("AR8030 usb gadget"): one
HID-class interface with no endpoints. Everything goes over endpoint 0 as
vendor control writes: `bmRequestType 0x40`, `bRequest 0x0c`, `wValue 0`,
`wIndex 0`. Each write is one block:

| bytes | field |
|---|---|
| 0–1 | `'U' 'S'` |
| 2–3 | data length, LE16, at most 4084 (so a request is at most 4096) |
| 4–7 | load address, LE32 |
| 8–11 | 0 |
| 12– | data |

The image file (`bb_demo_cx485_2PA.img`) starts with a 64-byte header:

| offset | field |
|---|---|
| 0x00 | magic `30 80 52 41` (LE32 `0x41528030`) |
| 0x04 | LE16 1 |
| 0x06 | LE16 64, the header size |
| 0x18 | firmware: load address, length (LE32 each) |
| 0x20 | SPL: load address, length |
| 0x28 | the image's baseband config: load address, length |
| others | not needed for the upload; sent as they are |

The sections follow in the file in the order SPL, config, firmware, each
starting on a 512-byte boundary (0x200, 0x600, 0x4600 in the stock image).

The stock driver's upload, as captured, 122 requests in ~100 ms:

1. the 64-byte header to `0x002f0040`;
2. the SPL to its address (`0x0a098a00`, 944 bytes);
3. the image's config to its address (`0x00200000`, 16 KB);
4. the firmware to its address (`0x00204000`, 439 KB);
5. the separate JSON config (`bb_config_gnd_pro.json`) to the config's
   address again, over the image's own, not padded;
6. a block of length 0 at address 0: run.

No reads and no status between blocks. The chip then drops off the bus and
comes back as `1d6b:8030`. arlink.ko repeats this sequence exactly. The run
request can fail because the chip is already leaving, so its error is logged
but does not fail the probe; whether the chip started shows in it coming back.

A config the firmware cannot run sends the chip back to the boot ROM, over
and over, and so can a firmware that crashes, however slowly. The driver
counts uploads in a row: after 4 with no running chip in between that lasted
30 s, it stops uploading (logged) until the module is reloaded. A running
chip that lasted 30 s starts the count again. Only uploads that reached the
chip count: a missing firmware or config file, or one that does not fit, is
logged and costs nothing (fix it, then replug or rebind).

## Running firmware (`1d6b:8030`)

"Artosyn in HS Mode": one HID-class interface with a bulk endpoint each way
(`0x81` IN, `0x01` OUT, 512-byte packets) and no interrupt endpoint, so
usbhid declines it. Nothing beyond standard enumeration: the host protocol
([protocol.md](protocol.md)) starts with the host's first frame.

The chip sends each frame as one bulk transfer of at most 4096 bytes, and ends
a transfer that is a whole number of packets with a zero-length packet. The
driver sends one frame per transfer too, with a zero-length packet after a
whole number of packets (the captures never show the host sending one of that
length, so this is a guess that does no harm).

## The device file

`/dev/arlink0` (misc device, mode 0600, one per chip):

- The number is given back as soon as the chip goes, so a chip that comes
  back (unplugged, reset) is `/dev/arlink0` again even while a program still
  has the old one open.
- `read()` returns exactly one frame, `AA` to `BB`. `-EMSGSIZE` if the buffer
  is too small; the frame stays queued. Blocks unless `O_NONBLOCK`.
- `write()` takes exactly one frame (checked: `AA`, the length field, `BB`)
  and returns once the chip has taken it, or `-ETIMEDOUT` after 1 s.
- `poll()`: readable when a frame is queued; `POLLERR | POLLHUP` once the
  chip is gone.
- Once the chip is gone (or given up on, see Receive below), `read()` returns
  what was still queued, then `-ENODEV`; `write()` and new opens fail with
  `-ENODEV`. The same goes for attached sockets.
- One open at a time (`-EBUSY`). Frames that arrive while it is closed are
  dropped, so a program never reads replies meant for the one before it.
- On close, the driver finishes what the program left running, the way a
  program that exits cleanly does, waiting for each reply (up to 1 s):
  every socket it opened (domain 4, op 0) and did not close (op 3) is closed,
  then the baseband is stopped (`0x0b000001`) if it was started and
  deinitialised (`0x0b000003`) if it was initialised. Without the last two,
  the next program's `INIT`/`START` on a chip still started wedged its OUT
  endpoint on every second restart after a `kill -9` (writes time out until a
  power cycle). With them, four kills in a row each recovered in 1.7 s.
  The first request the chip does not take or answer ends it (logged once,
  the rest skipped): a chip that is not answering would otherwise hold the
  device, new opens getting `-EBUSY`, for up to 2 s a request. A chip that is
  gone is not asked at all.
  What counts as opened, initialised or started is what the chip answered
  with status 0, not what was asked, so the cleanup does not undo what the
  chip refused; closes, stops and deinits count as they are asked. Up to 16
  sockets are kept track of (one more is logged, once). The cleanup's
  requests go with msgid 0; the reply to one that timed out is dropped should
  it still come (within 10 s), rather than reach the next program as one of
  its own.

### Attached sockets

`ioctl(fd, ARLINK_IOC_SOCK_ATTACH, {slot, port})` (`driver/arlink_uapi.h`)
returns a new file descriptor that from then on gets that socket's data
frames (domain 4, op 2, status `0x12345678`) instead of `/dev/arlink0`.
Attach before opening the socket on the chip so none goes the other way.
`read()` on it returns one frame at a time: a `struct arlink_sock_rec`
(`rx_ns`, `CLOCK_MONOTONIC` when the USB transfer that completed the frame
finished, taken in the completion handler) followed by the payload. Its queue
is `sock_queue_kb` (default 512 KB); counters per socket in `stats`.

libarlink attaches every socket it opens (`ARLINK_NOFAST=1` turns that off),
so `bb_socket_read()` reads straight from the kernel, everything queued up to
the buffer size, and `arlink_socket_rx_ns()` gives when the last of it
arrived. The video then skips the library's reader thread and ring.

Receive: 8 bulk IN transfers of 8 KB are kept queued from probe on, so the
chip is always drained. Each completed transfer goes through a framer (header
XOR, length, `BB`; bytes that cannot start a frame are skipped and counted)
into a queue of whole frames (`rx_queue_kb`, default 1 MB). The queues are
`kvmalloc`ed: a megabyte of physically contiguous memory may not be there
when the chip re-enumerates hours into a session.

After 50 receive errors in a row (`-EPROTO` and the like), or a stalled IN
endpoint (`-EPIPE`), the driver gives up on the chip: the device and its
attached sockets behave as if it were gone, and the chip is reset
(`usb_queue_reset_device()`, which also clears a halt; clearing one directly
sleeps, which a completion handler cannot). It re-enumerates and is probed
afresh, as `/dev/arlink0` again (or back in its boot ROM, and uploaded to).
A program sees its reads fail and has to reopen the device.

Counters: `/sys/class/misc/arlinkN/stats` (the bus, frames and bytes each way,
errors, uploads, and each attached socket).

Module parameters: `fw_name` and `cfg_name` (from `/lib/firmware`, defaults
the goggle's stock pair; `cfg_name=` for none), `rx_queue_kb` and
`sock_queue_kb` (rounded down to a power of two).

## The chip's log

The firmware writes its own log as domain-5 frames, ~300 bytes/s once
running (MCS changes and the like: `Slot 0 RX MCS 8 -> 10`). libarlink appends
it to the file `ARLINK_CHIPLOG` names, rotating to `<file>.1` at 1 MB;
`ARLINK_DEBUG=2` prints it too.

## SDIO (the air unit)

- **IDs:** SDIO `4152:8030` is the boot ROM, `4152:8031` the running
  firmware, both on function 1.
- **Upload:** the same image and the same sequence as on USB. Blocks are
  `'S' 'D'` | length LE16 | address LE32 | 0, written to the function-1 FIFO
  (address 0). Each block is one transfer: at most one 512-byte SDIO block, or
  a whole number of them (4096-byte blocks, the tail cut to fit).
- **After the upload:** the host is set to poll for the card
  (`MMC_CAP_NEEDS_POLL` on, `MMC_CAP_NONREMOVABLE` off, the host's
  `card_event`, a detect), as stock does. The chip leaves and comes back as
  `8031`.
- **Registers (function 1):**
  - `0x13`: non-zero while something is pending; bit 4 means events.
  - `0x68`: the events. Bit 0 is mailbox 0 (`0x54`), bit 1 mailbox 1
    (`0x58`), bit 2 data to read (`0x5c`, in 512-byte blocks), bit 3 room
    to write (`0x60`, in blocks).
  - Probe writes 1 to `0x14` and `0xf0` to `0x68`.
- **Receive:** read the blocks from the FIFO. Each run of blocks is one
  frame, starting on a block boundary; the rest of its last block is stale.
- **Transmit:** one frame per transfer, padded to whole blocks when longer
  than one, once the chip reports room for all of it. A write uses the room
  up, and the chip reports more after it. A frame split across two
  transfers is dropped. The stock daemon's frames are at most 10240 bytes,
  and the library keeps to that.
- **Module swaps:** replacing the stock module on a running chip leaves the
  chip silent (the function is disabled and re-enabled). Load arlink.ko at
  boot instead, with the chip in its boot ROM.
