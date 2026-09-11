# SwitchFace

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Latest release](https://img.shields.io/github/v/release/peha68/SwitchFace)](https://github.com/peha68/SwitchFace/releases/latest)
[![Platform: ESP32-S3](https://img.shields.io/badge/platform-ESP32--S3-blue.svg)](platformio.ini)
[![Framework: Arduino](https://img.shields.io/badge/framework-Arduino-00979D.svg)](platformio.ini)

An AMOLED touchscreen clock that doubles as a swipe-through remote control
for multiple Home Assistant entities, on a [Waveshare
ESP32-S3-Touch-AMOLED-1.64](https://www.waveshare.com/esp32-s3-touch-amoled-1.64.htm).
Wall-mountable, battery- or mains-powered, no app required - configuration
happens entirely on-device or through a small web page it serves itself.

## Features

- **Live clock & date** - NTP-synced, configurable UTC offset (no
  hardcoded/recompiled timezone).
- **Multi-entity Home Assistant carousel** - swipe left/right on the clock
  screen to cycle through up to 6 configured switches/lights. Each entity
  has its own name and an owner-assigned **identity color**, shown on the
  button and name regardless of on/off state - a low-vision accessibility
  aid, so an entity can be told apart by color alone, not just by reading
  its (comparatively small) name.
- **Tap to toggle** - large touch target, tuned for this panel's real-world
  touch jitter; color brightens when the entity is on, dims (same hue) when
  off.
- **On-device WiFi setup** - first boot (or on request) starts the device's
  own hotspot with a captive config page; no credentials baked into
  firmware, no recompiling to switch networks.
- **Remote setup access** - once on your WiFi, swipe down from the clock to
  see the device's LAN address/mDNS hostname, and reach that same config
  page without re-entering AP mode.
- **OTA firmware updates** - password-protected, over WiFi, once the device
  is on your network (no cable needed after the first flash).
- **Battery-aware power management** - voltage monitoring with a charge
  detector, automatic dim → blank on inactivity (AMOLED pixels draw ~zero
  current when off, so this is the single biggest power lever on this
  panel).
- **Polish-language support** - a custom-generated LVGL font supplies the
  diacritics (Ą Ć Ę Ł Ń Ó Ś Ź Ż and lowercase) that none of LVGL's stock
  fonts include, so entity names aren't limited to ASCII.

## Hardware

- [Waveshare ESP32-S3-Touch-AMOLED-1.64](https://www.waveshare.com/esp32-s3-touch-amoled-1.64.htm)
  (V1/CO5300 display-driver revision - see the note in `src/main.cpp` if
  you have the SH8601-based "-v2" board instead, the CS pin differs).
- ESP32-S3R8 (8MB octal PSRAM) + W25Q128 (16MB flash), on-board.
- FT3168 capacitive touch controller (I2C).
- A companion 230V → 5V power supply board, sized to fit inside a standard
  wall junction box, is currently in production - it'll let the device run
  permanently mains-powered behind a wall plate instead of only on battery/
  USB. Details and design files to follow once it's built and tested.

## Getting started

### Build & flash

Built with [PlatformIO](https://platformio.org/).

```bash
git clone https://github.com/peha68/SwitchFace.git
cd SwitchFace
pio run -t upload
```

The first flash has to be over USB (`upload_port` in `platformio.ini` is a
wildcard that picks up the board automatically on macOS/Linux - set it
explicitly if that doesn't work for you). After that, updates can go over
WiFi - see "OTA updates" below.

### First boot / WiFi setup

With no saved network, the device starts its own hotspot
(`SwitchFace-XXXX`, or your chosen device name). Connect a phone or laptop
to it and open `http://192.168.4.1` - from there you can:

- Scan for and join a WiFi network (saving restarts the device).
- Set the UTC offset (independent of WiFi, doesn't need a network to save).
- Pick a device name (used for the hotspot SSID and `<name>.local` mDNS
  address).
- Configure Home Assistant (see below).
- Set an OTA password.

### Home Assistant integration

1. In Home Assistant: your profile → **Security** → **Long-Lived Access
   Tokens** → **Create Token**. Copy it immediately - it's only shown once.
2. On the device's setup page, under **Home Assistant**: enter your HA base
   URL (e.g. `http://192.168.1.50:8123`) and the token.
3. Add up to 6 entities: a name, the `entity_id` (find it in HA under
   **Settings → Devices & services → Entities**), and optionally a custom
   color (defaults to a distinct built-in color per slot if left alone).
4. Save. On the clock screen, swipe left/right to cycle entities, tap the
   button to toggle.

### Remote setup access

Once connected to WiFi, swipe down from the clock screen to reach **REMOTE
SETUP** - it shows the device's current LAN IP and `<name>.local` address,
so you can reach the same config page from your normal network without
putting the device back into hotspot mode.

### OTA updates

With the device on WiFi (and an OTA password set on the setup page - use
the dedicated environment below, which reads it from an environment
variable so it's never written to a file or committed):

```bash
OTA_PASSWORD=<your password> pio run -e esp32-s3-devkitc-1-ota -t upload --upload-port <device-ip-or-hostname.local>
```

Confirmed working end-to-end on real hardware (~75s for a full upload).
The screen will visibly stutter during the actual firmware write - that's
inherent to `ArduinoOTA` (its progress callback runs synchronously), not a
hang; don't power-cycle mid-update.

### Automatic updates (GitHub Releases)

The device can also find, verify, and install new versions entirely on its
own, with no computer involved:

1. **Publishing a release is one command:**
   ```bash
   git tag v1.2.3 && git push --tags
   ```
   `.github/workflows/release.yml` builds the firmware and publishes a
   GitHub Release with `firmware.bin` attached and its SHA256 in the
   release notes - nothing to build or upload by hand.
2. The device checks `api.github.com/repos/peha68/SwitchFace/releases/
   latest` once per WiFi connection. When the published tag differs from
   its own version (embedded from `git describe` at build time - see
   `scripts/get_version.py`), a small download icon lights up on the
   CLOCK screen's top row.
3. Tap it to reach the **FIRMWARE UPDATE** screen (current/latest version,
   an "Update now" button) - or use the **Firmware** card on the setup web
   page instead, with a "Check now" and "Update now" button there too.
   Either one downloads, verifies, and flashes the same way.

**Security note:** the release-check API call is TLS-certificate-pinned
(GitHub's current root CA is embedded in `src/update_check.cpp`). The
firmware *binary* download is not - it redirects through GitHub's asset
CDN, whose hostname/CA has changed more than once over the years, and
pinning it would mean updates silently stop working whenever that
infrastructure changes again, with no fix short of a USB reflash. Instead,
every downloaded byte is SHA256-hashed and checked against the hash from
the cert-validated API response *before* the new image is ever finalized -
a corrupted or spoofed download fails that check and is aborted, leaving
the currently-running firmware untouched (the existing `ota_0`/`ota_1`
dual-partition scheme makes this safe: a failed update just never becomes
the boot partition).

## Screens & gestures

| Screen | Swipe LEFT / RIGHT | Swipe UP | Swipe DOWN |
|---|---|---|---|
| **CLOCK** | Previous / next entity (wraps at both ends) | WIFI SETUP | REMOTE SETUP |
| **WIFI SETUP** | Back to CLOCK | toggles to REMOTE SETUP | toggles to REMOTE SETUP |
| **REMOTE SETUP** | Back to CLOCK | toggles to WIFI SETUP | toggles to WIFI SETUP |

WIFI SETUP always starts the device's own hotspot (disconnecting WiFi while
active) - it's the on-device provisioning screen. REMOTE SETUP serves the
same config page directly over an existing WiFi connection when there is
one, falling back to its own hotspot only if there isn't.

A fifth screen, **FIRMWARE UPDATE**, isn't part of either rotation above -
it only appears by tapping the download icon on CLOCK (visible only when
an update is actually available), and any swipe from it returns to CLOCK.

## Project layout

```
SwitchFace/
├── .github/workflows/
│   └── release.yml          # Builds + publishes a GitHub Release on every `git tag v*` push
├── scripts/
│   └── get_version.py       # PlatformIO pre-build step: embeds `git describe` as FIRMWARE_VERSION
├── src/
│   ├── main.cpp             # Display/touch/power bring-up, screens, gestures, entity carousel
│   ├── wifi_portal.cpp      # WiFi provisioning portal, remote config access, HA/OTA settings storage (NVS)
│   ├── ha_light.cpp         # Minimal Home Assistant REST client (toggle/poll one entity at a time)
│   ├── ota.cpp               # ArduinoOTA (push-style, from a computer) setup and progress tracking
│   ├── update_check.cpp      # Self-update (pull-style, from GitHub Releases) - check/download/verify/flash
│   └── font_pl_34.c          # Generated LVGL font (Montserrat + Polish diacritics) - see below
├── include/
│   ├── wifi_portal.h, ha_light.h, ota.h, update_check.h   # Public API for the above
│   └── lv_conf.h              # LVGL build configuration
├── lib/
│   ├── esp_lcd_sh8601/        # Display driver (Espressif, vendored - see Credits)
│   └── FT3168/                # Touch controller driver (Waveshare example, vendored - see Credits)
└── partitions.csv             # Flash layout - includes an OTA (ota_0/ota_1) partition scheme
```

### Regenerating the Polish-language font

`src/font_pl_34.c` was generated from LVGL's own bundled Montserrat source
TTF (so it matches the stock `lv_font_montserrat_*` fonts visually) using
[`lv_font_conv`](https://github.com/lvgl/lv_font_conv):

```bash
npx lv_font_conv \
  --font .pio/libdeps/esp32-s3-devkitc-1/lvgl/scripts/built_in_font/Montserrat-Medium.ttf \
  -r "0x20-0x7F,0xD3,0xF3,0x104,0x105,0x106,0x107,0x118,0x119,0x141,0x142,0x143,0x144,0x15A,0x15B,0x179,0x17A,0x17B,0x17C" \
  --size 34 --bpp 4 --format lvgl --no-compress \
  --lv-font-name font_pl_34 \
  -o src/font_pl_34.c
```

`--no-compress` is required: this project's `lv_conf.h` doesn't enable
`LV_USE_FONT_COMPRESSED`, so a compressed-bitmap font (the tool's default)
would compile fine but render blank glyphs at runtime. `-DLV_LVGL_H_
INCLUDE_SIMPLE=1` in `platformio.ini`'s `build_flags` is also required, so
the generated file's `#include "lvgl.h"` resolves the same way the rest of
the project already includes it.

## Known limitations

- No on-device QR code for the setup hotspot yet - SSID/address are shown
  as plain text.
- The bundled `WebServer` library serves one client connection at a time;
  mitigated (explicit `Connection: close`, an instant `/favicon.ico`
  handler, keeping the server running persistently rather than only during
  a screen visit) but it's a real constraint of the library, not something
  fully worked around.
- No automated test suite - this is a single-user embedded project, not a
  library; everything above was verified manually on real hardware.

## Credits & licenses

This project's own code is MIT-licensed - see [LICENSE](LICENSE). Two
vendored driver files under `lib/` carry their own terms:

- **`lib/esp_lcd_sh8601/`** - © Espressif Systems, **Apache License 2.0**.
  Fetched verbatim from Waveshare's official `06_LVGL_Test` example for
  this exact board.
- **`lib/FT3168/`** - adapted from Waveshare's example code for this board.
  Provided by the vendor without a formal license attached; kept as
  provided, with attribution here.
- Built on [LVGL](https://lvgl.io/) (MIT) and the
  [Arduino ESP32](https://github.com/espressif/arduino-esp32) core.
