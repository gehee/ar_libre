# AR8030 host link protocol (USB)

Working notes on the host protocol, from USB captures of the stock daemon
talking to the chip (`tools/usbcap`; the `captures/` named below are not
published).

Confidence: **confirmed** = seen in captures and borne out by arlink.ko and
libarlink on the chip;
**observed** = consistent in captures, meaning inferred; **guess** = a
hypothesis to test.

## Transport

The AR8030 enumerates twice:

1. **Boot ROM**, `4152:8030`. The kernel driver uploads the baseband firmware
   and its config with vendor control requests (`bmRequestType 0x40`,
   `bRequest 0x0C`; ~122 requests), then the chip re-enumerates. *Confirmed*
   (command header magic `'U' 'S'`).
2. **Running firmware**, `1d6b:8030` ("Artosyn in HS Mode"), USB 2.0 high
   speed. One interface, two bulk endpoints: `0x01` OUT and `0x81` IN, 512-byte
   packets. The whole host protocol runs over these two pipes. *Confirmed.*

The stock daemon (`-i 0`) talks to the running firmware through libusb.

## Frame

Every message, both directions, one or more per bulk transfer. *Confirmed.*

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `0xAA` |
| 1 | 4 | payload length, little-endian |
| 5 | 4 | `reqid`, big-endian: `domain` (bits 31..24) and sub-command (23..0) |
| 9 | 4 | `msgid`, big-endian: request/reply match (the host increments it) |
| 13 | 4 | `sta`, big-endian signed: status in replies |
| 17 | 1 | check: `0xFF` XOR bytes 0..16 |
| 18 | n | payload |
| 18+n | 1 | `0xBB` |

## Domains

Names from the client API (`BB_REQ_*`), which agree with the captures: a
request code is the wire `reqid` (domain << 24 | sub).
[`lib/ioctl_sizes.inc`](../lib/ioctl_sizes.inc) lists every request code with
its input and output sizes, as read from the stock binary.

| Domain | Name |
|---|---|
| 0 | CFG |
| 1 | GET |
| 2 | SET |
| 3 | CB (events from the chip) |
| 4 | SOCKET |
| 5 | DBG (the firmware log) |
| 6 | REMOTE |
| 10 | RPC |
| 11 | RPC_IOCTL |
| 12 | PLAT_CTL |
| 0xff | link management (not a client request) |

| Domain | Meaning | Status |
|---|---|---|
| `0xff` | Link management. Sub 1: reset/sync burst at start. Sub 0: hello, reply 8 bytes (`00 10 0e 00` + 4-byte id also seen in the status reply). Sub 2: heartbeat, host → chip about every 120 ms, echoed | observed |
| `0x01` | Request/reply queries (`msgid` matched). Sub `0x00` with payload `ff 03` → 300-byte reply, polled ~3/s: link status (guess). Also `0x05`, `0x06`, `0x0a` (494-byte reply), `0x0c`, `0x6b` | observed |
| `0x02` | Configuration writes at session start (sub `0x03`, `0x04` with a 402-byte blob, `0x05`, `0x08`, `0x09`, `0x0c`, `0x0d`, `0x10`, `0x1c`, `0x20`, `0x24` ×7 with 16-byte entries: an MCS table, guess). Replies are empty, `sta` 0 | observed |
| `0x03` | **Events.** Sub `0x0100NN` subscribes to event `NN` (the stock daemon subscribes 13..0 at start; the chip acks each, event 4 with `sta -2`, unsupported). Sub `0x0200NN` is event `NN` arriving from the chip - see Events below | observed, and used by `arlink-probe` |
| `0x04` | **Data sockets.** Sub = `op << 16 \| slot << 8 \| port`, op 0 open, 1 write, 2 read, 3 close; `0x90..0x92` length and tx-limit queries. Data frames carry `sta = 0x12345678`. Open payload: flags u32 (bit 0 RX, bit 1 TX, bit 2 reset-on-connect, bit 3 datagram), tx buffer size u32, rx buffer size u32 | **confirmed** |
| `0x05` | The chip's firmware log, ASCII text lines (`[ms][task][INF] ...`), chip → host (sub 1). Host sends sub 0 with 4 zero bytes once at start (enable, guess) | observed |
| `0x0b` | Session setup just before configuration: sub 2, then sub 0 | observed |

## Sockets in use on the ground unit studied

| Port | Opened as | Carries |
|---|---|---|
| 3 | flags 3 (RX+TX), tx 2 KB, rx 56 KB | the video stream (reads of up to 4077 bytes) |
| 2 | flags 3, tx 2 KB, rx 2 KB | telemetry / MSP from the air unit; payloads begin `fe a5` |

## Settings, one change at a time

From `captures/settings.pcap` (menu changes, each reverted; markers in
`captures/settings.marks`). Everything else in those windows was periodic.

| Setting | Frames | Status |
|---|---|---|
| TX power | `0x02` sub `0x09` payload `00 PP PP`, then sub `0x08` payload `00 PP`. `PP` = `0x11` at 500 mW (also what bring-up sends), `0x0b` at 200 mW. Plus a message to the air unit on port 2 | observed |
| Channel hop | `0x02` sub `0x05`, payload `01` on, `00` off (bring-up sends `01`) | observed |
| Auto standby, EV, video mode | no chip command: application messages to the air unit (`fe a5 ...`), written to socket port 2. The daemon only carries them; the application speaks that protocol itself | observed |

A socket write (`0x04`, op 1) payload is an 8-byte header - the running count
of data bytes written to that socket so far (u32 LE), then four zero bytes -
followed by the data. The chip answers
twice: a 16-byte frame with `sta = -262`, then an empty-payload frame whose
`sta` is the number of data bytes accepted. *Observed.*

## Periodic traffic in steady state

| Frames | Period |
|---|---|
| `0xff` sub 2 heartbeat | ~120 ms |
| `0x01` sub `0x00` (`ff 03` → 300 bytes) | ~330 ms |
| `0x01` subs `0x05` (1 byte → 32), `0x6b` (4 → 216), `0x06` (2 → 8), `0x0c` (0 → 4, a counter) | 1 s |
| `0x01` sub `0x0a` (0 → 1006 bytes) | 2 s |
| `0x03` sub `0x020001` from the chip, 4 bytes (`00 01 0a 0c`, `00 01 0c 0a`, ...) | irregular: an event, guess |

## Status queries (domain `0x01`)

Named by matching each request to the call an application makes through the
stock client library (its call order and arguments are observable behaviour),
and checked against the link statistics the application reads once a second
(MCS, SNR, rate).

| Sub | Client call | Request payload | Reply |
|---|---|---|---|
| `0x00` | `BB_GET_STATUS` | u16 user bitmap (`0x03ff`: users 0..9) | 300 bytes, below |
| `0x06` | `BB_GET_MCS` | u8 direction (1 = RX), u8 slot | 8 bytes: [0] MCS; [4..7] u32 PHY rate in kbps (MCS 12 → 31415, 10 → 23681, 8 → 15659) |
| `0x0c` | `BB_GET_AP_TIME` | none | 4 bytes: u32 time counter |
| `0x6b` | `BB_GET_1V1_INFO` | u32 frame count (e.g. 64) | 216 bytes: [0..1] u16 SNR, linear: dB = 10·log10(raw / 36) (6086 → 22.3 dB, 3115 → 19.4 dB; r = 0.99 against the application's figure). Rest TBD |
| `0x05` | TBD | `01` | 32 bytes, 1 Hz |
| `0x0a` | TBD (channel scan: 42 channels, polled every 2 s) | none | 1006 bytes |

### `BB_GET_STATUS` reply (300 bytes, request `ff 03`)

Several fields are one byte followed by a byte the chip leaves
uninitialised: across replies that second byte holds zero or text-like junk
(`MC`, `][`, `b_`). Read the low byte only. *Observed.*

| Offset | Field | Status |
|---|---|---|
| 0..5 | `01 00 00 01 01 01`, constant | TBD |
| 6..9 | chip id, 4 bytes (also in the `0xff` hello reply) | observed |
| 10 | u8 + junk; varies 11..77 | TBD |
| 16..251 | 12-byte records, one per user/slot: [2] u8 + junk varying, [4..7] u32 frequency in kHz (first record: 5839000, the live link; the rest 2100000, a placeholder) | observed |
| 253 | MCS (only the values seen: 8, 10, 12; the rate follows it) | observed |
| 254..257 | the connected air unit's id, 4 bytes (changes to the new unit's with a bind) | observed |
| 266 | TX power code: `0x11` at 500 mW, `0x0b` at 200 mW; follows the `0x02` sub `0x09` writes | observed |
| 268..271 | u32 frequency in kHz (5839000) | observed |

Unsolicited `0x03` sub `0x020001` frames (`00 01 NN MM`) arrive when the MCS
changes: `0a 0c` = 10 → 12, `0c 0a` = 12 → 10, `08 0a`, `0a 08`. *Observed.*

## Requests by name

Every request seen so far, by its client-API name
(`BB_REQUEST(domain, sub)`); sizes from `lib/ioctl_sizes.inc` agree with the
captures.

| Code | Name | Payload in → out | Notes |
|---|---|---|---|
| GET 0 | `BB_GET_STATUS` | 2 → 300 | u16 user bitmap. Reply below |
| GET 1 | `BB_GET_PAIR_RESULT` | 0 → 164 | [0] slot bitmap of slots that found a peer, [1..4] the peer id for slot 0; the rest is uninitialised |
| GET 5 | `BB_GET_DISTC_RESULT` | 1 → 32 | u8 slot bitmap → eight i32 LE distances, one per slot; only the requested slot is written (the rest is stale buffer). Below |
| GET 6 | `BB_GET_MCS` | 2 → 8 | below |
| GET 10 | `BB_GET_CHAN_INFO` | 0 → 1006 | channel scan, every 2 s |
| GET 12 | `BB_GET_AP_TIME` | 0 → 4 | u32 counter |
| GET 107 | `BB_GET_1V1_INFO` | 4 → 216 | below |
| SET 2 | `BB_SET_PAIR_MODE` | 14 → 0 | [0] start (1) / stop (0), [1] slot bitmap, rest zero |
| SET 3 | `BB_SET_AP_MAC` | 4 → 0 | the air unit id to associate with (a DEV's one AP). Sent at session start with the first remembered id, and after a bind with the new one |
| SET 4 | `BB_SET_CANDIDATES` | 402 → 0 | [0] slot, [1] count, then 4-byte ids: the remembered air units (e.g. `00 03` and three ids, the list in `/factory/user_cfg.json`) |
| SET 5 | `BB_SET_CHAN_MODE` | 1 → 0 | channel hop: 1 auto, 0 fixed |
| SET 8 | `BB_SET_POWER` | 2 → 0 | [0] slot?, [1] power code (`0x11` = 500 mW, `0x0b` = 200 mW) |
| SET 9 | `BB_SET_POWER_AUTO` | 3 → 0 | [0] ?, [1] [2] power code twice |
| SET 12 | `BB_SET_MCS_MODE` | 2 → 0 | sent at start |
| SET 13 | `BB_SET_MCS` | 2 → 0 | sent at start |
| SET 28 | `BB_SET_LNA_MODE` | 1 → 0 | sent at start (`ff`) |

### Distance (`BB_GET_DISTC_RESULT`)

Slot 0 reads 72..76 with both units side by side, and `-1` while there is no
link. The raw value is a radio time-of-flight measurement configured by
`dist_calc` in `bb_config_gnd_pro.json` (averaging window 3, timeout 63,
offset 20), so the side-by-side reading is a fixed offset, not a distance,
for the application to subtract. *Observed.*

**The unit is still unknown.** A walk to 1, 3, 5 and 10 m indoors
(`captures/distance.pcap`, `distance.marks`) showed no trend: the reading
wanders by several units at rest (sd ~8 on the bench), and multipath indoors
adds more. It needs tens to hundreds of metres: an outdoor walk at 50/100/200 m,
or better one flight logging the raw value next to the flight controller's GPS
distance to home. Log it from the application at 1 Hz, not from `usbcap -s 64`:
the chip packs several frames into one IN transfer, so a short snap length
drops any distance reply that sits behind video data.

## Events (domain 3, chip → host)

| Sub | Payload | Meaning | Status |
|---|---|---|---|
| `0x020000` | 3 bytes, e.g. `00 01 00`, `00 02 01` | link state change (slot, new, old - guess) | observed at pair start and link-up |
| `0x020001` | 4 bytes `00 01 NEW OLD` | MCS change. After a link comes up: 2 → 5 → 7 → 8 → 10 → 12 | observed |
| `0x020002` | 4 bytes `00 01 29 0f` | TBD | observed at pair start / link-up |
| `0x020009` | 12 bytes, ends `ff 56 34 12` | TBD | observed at pair start / link-up |
| `0x02000c` | 4 bytes `00 01 NEW OLD` (`03 02`, `04 03`) | TBD, stepping like MCS | observed after link-up |

## Pairing (a DEV, the goggle)

From `captures/pair.pcap`: the application's bind, the air unit put in
pairing mode by its button.

1. `BB_SET_PAIR_MODE` start, slot bitmap `0x01`.
2. The chip reports link state and MCS events as the current link drops.
3. `BB_GET_PAIR_RESULT` every 0.5 s; the reply carries the id of the air unit
   in pairing mode once one is heard (wait for it to read the same id several
   times).
4. `BB_SET_PAIR_MODE` stop, then `BB_SET_AP_MAC` with that id.
5. Link-up: link state events, then MCS events climbing 2 → 12.

The remembered list (`BB_SET_CANDIDATES`) is not touched by the bind; the host
keeps it (the stock UI in `/factory/user_cfg.json`, a ring of ids) and sends it
at session start.
