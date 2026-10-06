# ar_libre

An open host stack for the Artosyn AR8030 baseband, for any product built on
it, on either end of the link: a ground unit that attaches the chip over USB,
or an air unit that attaches it over SDIO. It replaces Artosyn's kernel
drivers, host daemon and client library.

| | replaces | what it is |
|---|---|---|
| [`driver/`](driver) `arlink.ko` | `artosyn_drv.ko` (goggle, USB), `artosyn_sdio.ko` (air unit, SDIO) | uploads the firmware at power-on, then a frame pipe `/dev/arlink0` ([docs/driver.md](docs/driver.md)) |
| [`lib/`](lib) `libar8030_client.so` | `daemon` + `libar8030_client.so` | the client API, in-process, no daemon |
| [`docs/protocol.md`](docs/protocol.md) | | the host protocol, from USB captures |
| [`tools/`](tools) | | `usbcap` (usbmon → pcap), `arframe.py` / `pcapusb.py` (decoders), `sdiospy` (SDIO daemon traces), `arlink-test` |

The library is a drop-in for the vendor library under the same name: a program
built against the vendor library runs on it unchanged. It opens the lowest `/dev/arlinkN`
that opens, or Artosyn's `/dev/ar_mdev0` when that is the driver loaded
(`ARLINK_DEV` overrides; `ARLINK_DEBUG=1` logs, `=2` adds the chip's own log,
`ARLINK_CHIPLOG=<file>` keeps that log in a file). On `arlink.ko` each socket's
data comes straight from the kernel, stamped with its USB arrival time
(`arlink_socket_rx_ns()`). When the device goes (unplugged, reset, given up on
by the driver), every call fails from then on, as with no device, until the
program disconnects and connects again, which finds the chip as it came back.

## Build

```sh
# driver, against the host's configured kernel tree (5.10 and 6.12 tested);
# usb.c and sdio.c are built when the kernel has USB and SDIO
make -C driver KERNEL_DIR=<kernel tree> ARCH=<arch> CROSS_COMPILE=<toolchain prefix>
# library
make -C lib CROSS_COMPILE=<toolchain prefix>
```

[fpvOS](https://github.com/gehee/fpvOS) is one user: it builds both into its
images.

## Run

```sh
# with the chip powered and in its boot ROM (on SDIO: load it at boot)
insmod arlink.ko fw_name=<firmware image> cfg_name=<baseband config .json>
# /dev/arlink0 appears once the chip is running (~1 s); no daemon
```

The firmware image and the baseband config are the product's own files,
loaded through the kernel's firmware path
(`/sys/module/firmware_class/parameters/path` changes it).

## License

GPL-2.0-or-later ([LICENSE](LICENSE)).
