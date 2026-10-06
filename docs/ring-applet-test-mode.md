# Ring applet test mode

A build of this Aliro reader for people developing an Aliro credential that is
not a phone, such as a Java Card applet in a ring. It lets you:

- load reader keys and the public keys of your test credentials over USB
  serial, with no phone, no Apple Home and no Matter commissioning;
- run transactions that take tens of seconds, which a slow card doing
  AES-GCM in software needs;
- see every APDU of a transaction with its timing, optionally in full hex,
  and a summary that says where a failed transaction stopped.

Firmware variant: `c6-st25r300-wifi-ringtest`. Patch:
`firmware/patches/0027-ring-applet-test-mode.patch`. NFC driver:
`firmware/components/st25r300_spi`.

**Everything loaded in this mode is a TEST key.** The serial console has no
authentication: anyone with a USB cable can load keys. Never use this build on
a reader that guards a real door.

## Status

Compiles. **Not yet run on hardware.** The ST25R300 register sequences are the
ones John's ESPHome driver uses and has proven for ISO 14443A and ISO-DEP; their
port to this firmware, the slow-card handling and the serial provisioning are
new and untested on the bench. Expect the first session to find something.
Logs from it are the most useful thing you can send back.

## Hardware

| Item | Notes |
|---|---|
| ESP32-C6-DevKitC-1 | Any C6 devkit with the USB Serial/JTAG port broken out |
| ST25R300 reader board | Lauri's 40 x 40 mm board, 9-pin PicoBlade |
| USB-C cable | To the devkit port marked USB (not UART) |

Wiring, from `firmware/board_config/c6_st25r300.h`:

| Reader board pin | ESP32-C6 devkit |
|---|---|
| SCLK | GPIO 19 |
| MISO | GPIO 20 |
| MOSI | GPIO 21 |
| CS | GPIO 22 |
| IRQ | GPIO 23 |
| RST | GPIO 18 |
| VTX (5 V) | 5V |
| VIO (3V3) | 3V3 |
| GND | GND |

The devkit's RGB LED shows the result of each tap: green success, red failure,
blue a tag that is not an Aliro credential. GPIO 2 goes high while the lock is
"unlocked"; nothing needs to be attached to it.

## Flashing

With the prebuilt merged image:

```sh
pip install esptool
esptool.py --chip esp32c6 --port /dev/ttyACM0 --baud 460800 \
  write_flash 0x0 aliro-c6-st25r300-ringtest-factory.bin
```

On macOS the port is `/dev/cu.usbmodem*`; on Windows a `COM` port.

From source, with ESP-IDF 5.4 and the pinned esp-matter (see
`firmware/RELEASE.md`):

```sh
export ESP_MATTER_SRC=/path/to/esp-matter   # clean, at the pinned commit
. /path/to/esp-idf/export.sh
scripts/build_release.sh --variant c6-st25r300-wifi-ringtest
```

To wipe every key and setting the test mode stored, erase the NVS partition
only:

```sh
esptool.py --chip esp32c6 --port /dev/ttyACM0 erase_region 0x10000 0xC000
```

## First boot

Open the serial console (`idf.py monitor`, `screen /dev/ttyACM0 115200`,
`tio`, PuTTY; the baud rate is ignored by the USB port). You should see, among
the Matter start-up lines:

```
st25r300: ST25R300 identified over SPI: IC_IDENTITY=0xb1 (revision 1)
st25r300: RF field on: OPERATION=0x78 SENSE_RF=...
st25r300: NFC-A poller ready: 106 kbit/s, FSDI=8, SPI 1000000 Hz, APDU budget 30000 ms, ...
ring_test: RING TEST MODE ACTIVE: keys loaded over serial are TEST keys. ...
```

`no ST25R300 on SPI` means wiring or power. `SPI register read-back failed`
means a noisy link: the build defaults to a 1 MHz SPI clock, and
`CONFIG_ST25R300_SPI_FREQ_HZ=200000` (what the ESPHome driver uses) is the
fallback.

Matter also starts and prints a pairing code. Ignore it; nothing here needs it.

## Keys

`tools/ring_test_keys.py` generates a matching set: a reader key pair, a reader
group identifier, and one or more credential (endpoint) key pairs, all P-256.

```sh
pip install cryptography
tools/ring_test_keys.py --endpoints 1 --out ring-test-keys.json
```

It prints two blocks:

1. **Reader**: lines to paste into the serial console, for example

   ```
   ring reader <reader private key, 64 hex> <group id, 32 hex>
   ring endpoint add <endpoint public key, 130 hex, starting 04>
   ring trust list
   ring show
   ```

2. **Credential**: what your applet needs: the reader public key (65 bytes,
   uncompressed) and group identifier (16 bytes), and each endpoint's private
   and public key. `ring show` also prints the reader group sub-identifier,
   which the reader generates itself on first configuration and keeps.

`ring reader` goes through the same `SetAliroReaderConfig` path a Matter
administrator uses, so the reader under test is configured exactly as a phone
would configure it. Everything persists across reboots.

## Serial commands

One per line on the USB serial console. Responses start with `RING`.

| Command | Effect |
|---|---|
| `ring help` | List the commands |
| `ring show` | Reader keys, group and sub-identifier, endpoint keys and key slots, all settings |
| `ring reader <priv64> <group32>` | Load reader keys (replaces any previous ones) |
| `ring reader clear` | Remove the reader configuration |
| `ring endpoint add <pub130>` | Trust a credential public key (up to 8); prints its key slot |
| `ring endpoint del <index>` | Remove one |
| `ring endpoint clear` | Remove all |
| `ring trust list` | Only endpoint keys loaded here are accepted (default) |
| `ring trust any` | No key-slot lookup is registered with esp-aliro (see limits) |
| `ring budget <ms>` | Wall-clock budget for one APDU, S(WTX) waits included. Default 30000, max 120000 |
| `ring fwt <ms>` | Minimum frame waiting time, whatever the card's ATS says. Default 100, max 19800 |
| `ring wtxcap <ms>` | Longest single wait after granting S(WTX). Default 5000 |
| `ring attempts <n>` | Attempts per APDU. Default 1 in this build |
| `ring trace apdu on\|off` | Full APDU hex, both directions |
| `ring trace frames on\|off` | Every ISO-DEP frame in hex with timestamps, and every S(WTX) grant |
| `ring reload` | Re-create the reader so endpoint or trust changes apply (done automatically after each change) |
| `ring fastclear` | Forget fast-transaction keys, so the next tap is a standard transaction |

Settings persist in NVS.

## Slow cards

A card that needs time asks for it with S(WTX). The stock reader logic, built
for phones, gave up after 8 consecutive requests. This build grants S(WTX)
for as long as the APDU's wall-clock budget allows (`ring budget`), each wait
being FWT x WTXM capped at `ring wtxcap`. A card that computes without sending
S(WTX) needs a longer frame waiting time instead: `ring fwt`.

While a card is working, a progress line appears every 2 seconds:

```
st25r300: card still working on INS 81: 6012 ms so far, 78 WTX granted
```

esp-aliro has no timeouts of its own: it waits on the reader's APDU exchange,
so the budget above is the only limit.

## Reading the log

Each tap prints a block like this (values invented for illustration):

```
RING ---- session start ----
RING t=0 ms > SELECT CLA=00 INS=A4 len=15
RING t=41 ms < SELECT 41 ms, rsp len=..., SW=9000, WTX=0
RING t=41 ms > AUTH0 CLA=80 INS=80 len=...
RING t=3150 ms < AUTH0 3109 ms, rsp len=..., SW=9000, WTX=40
RING t=3151 ms > AUTH1 CLA=80 INS=81 len=...
RING key slot lookup: 0011223344556677
RING key slot lookup: matched endpoint 0
RING t=11800 ms < AUTH1 8649 ms, rsp len=..., SW=9000, WTX=112
...
RING ==== RESULT OK in 14210 ms (4 APDUs, STANDARD transaction)
RING    0 SELECT             41 ms  SW=9000  cmd=15 rsp=...  WTX=0
RING    1 AUTH0            3109 ms  SW=9000  ...
```

- The `>` line is printed **before** the card is asked, so a transaction that
  hangs still shows which command it hung on.
- Step names come from the INS byte (A4 SELECT, 80 AUTH0, 81 AUTH1, D1
  LOAD_CERT, C9 EXCHANGE, 3C CONTROL_FLOW, C3 ENVELOPE, C0 GET_RESPONSE).
  CLA and INS are always printed too; trust those if a name looks wrong.
- `WTX` is how many S(WTX) requests the card made during that APDU.
- On failure, a `diagnosis:` line says what the transport saw: budget
  exceeded, no answer and no S(WTX), the card going silent after asking for
  time, an RF error, a non-9000 status word, or an APDU that completed but
  whose content esp-aliro rejected. In that last case esp-aliro prints its
  own `E (...)` error line just above (signature, key slot, decryption).
- `RESULT ... FAST transaction` means the credential and reader used keys kept
  from a previous standard transaction. `ring fastclear` forces the next tap
  back to a standard one.

After a successful tap the reader ignores the field until it has been empty
for one second: lift the ring away between taps.

## Known limits

- **Untested on hardware**, see Status.
- 106 kbit/s only; no PPS to higher bit rates.
- NFC expedited phase plus EXCHANGE only, as the main firmware runs it. No
  step-up (ENVELOPE), no BLE or UWB.
- `ring trust any` registers no key-slot lookup. The esp-aliro API offers no
  other way to tell the library which credentials to trust, so in that mode the
  library presumably accepts any credential whose AUTH1 signature verifies
  against the public key the credential itself presents. This is inferred from
  the API, not observed.
- Key slots are derived by esp-aliro from the public key; `ring endpoint add`
  prints what the library computed rather than this tool guessing the formula.
- The response buffer esp-aliro offers for each APDU is the library's choice;
  a response larger than it fails the exchange (the diagnosis line says so).
- Multi-tag anticollision with a split byte follows the ST25R300 datasheet and
  has never been exercised. A single ring on the antenna never needs it.
- Only one credential is processed per field: hold one ring at a time.
