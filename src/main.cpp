// SwitchFace - AMOLED touch clock + multi-entity Home Assistant light
// switch, on a Waveshare ESP32-S3-Touch-AMOLED-1.64(-v2) (V1/CO5300
// revision - see the display driver notes below). See README.md for the
// full feature list, setup instructions, and screen/gesture map.
//
// Grew out of a TiltDash (vehicle leveling) bring-up board for this same
// display, hence some of the lower-level driver/touch/power-management
// code below still reads like display-driver bring-up notes - all of
// that is display/touch/power hardware knowledge, not tilt-sensing logic,
// so it stayed. The tilt-sensing screens/IMU code did not - this project
// has none of that.
//
// STATUS: all four screens (CLOCK entity carousel, WIFI SETUP, REMOTE
// SETUP, and OTA) are ported and verified end-to-end on real hardware. The
// on-device SETUP screen shows the hotspot name/address as text only (no
// QR code widget yet - a real follow-up, not skipped for a good reason,
// just extra scope this pass didn't need).
//
// The display driver is Waveshare/Espressif's own esp_lcd_sh8601
// component (lib/esp_lcd_sh8601/, fetched verbatim from Waveshare's
// official 06_LVGL_Test example for this exact board), used through the
// standard ESP-IDF esp_lcd_panel_* API - NOT a hand-rolled QSPI driver
// like the main tiltdash project's rm67162.cpp. That approach was tried
// first (same command set, retargeted pins/resolution/clock) and produced
// a persistent black screen with no error anywhere - three rounds of
// comparing our init sequence against the official one found no
// remaining difference, which means the bug was most likely in the raw
// SPI transaction framing itself, not the command bytes. Rather than
// keep guessing at that framing byte-by-byte, this uses Espressif's own
// tested implementation instead.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include "wifi_portal.h"
#include "ha_light.h"
#include "ota.h"
#include "update_check.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh8601.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_system.h"

#define LV_CONF_INCLUDE_SIMPLE 1
#include "lv_conf.h"
#include <lvgl.h>

#include "esp_timer.h"
#include "FT3168.h"

// Custom font with Polish diacritics - see font_pl_34.c and its use on
// clock_entity_name_lbl below for why.
LV_FONT_DECLARE(font_pl_34);

static constexpr int LCD_HOR_RES = 280;
static constexpr int LCD_VER_RES = 456;
static constexpr int I2C_SDA_PIN = 47;
static constexpr int I2C_SCL_PIN = 48;

static FT3168 tp(I2C_SDA_PIN, I2C_SCL_PIN, -1, -1);

// ====== Battery voltage ======
// GPIO4 = ADC1_CH3, with an onboard 3:1 divider (VBAT -> pin = VBAT/3) -
// pin and multiplier confirmed from Waveshare's own official 01_ADC_Test
// example for this exact board (adc_bsp.cpp: ADC_CHANNEL_3, "* 3").
static constexpr int BATTERY_ADC_PIN = 4;

static float readBatteryVoltage()
{
  static bool attenSet = false;
  if (!attenSet) {
    // analogSetPinAttenuation() only accepts a pin already claimed as an
    // ADC channel by the peripheral manager - a throwaway analogRead()
    // (default attenuation) does that claiming first; only then can the
    // attenuation actually be changed for this specific pin. Learned the
    // hard way on the tiltdash-sensors battery-kind child - see that
    // project's readBatteryVoltage() for the "Pin is not configured as
    // analog channel" error this avoids.
    analogRead(BATTERY_ADC_PIN);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    attenSet = true;
  }

  constexpr int SAMPLES = 16; // averaged to smooth out sampling noise
  uint32_t sumMv = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sumMv += analogReadMilliVolts(BATTERY_ADC_PIN);
    delayMicroseconds(200);
  }
  return ((sumMv / (float)SAMPLES) / 1000.0f) * 3.0f;
}

// A single LiPo cell never rests above ~4.2V - a reading above this can
// only mean the charge IC is actively pushing current in (USB present),
// which raises the sensed node well above the cell's true state of
// charge. Confirmed on real hardware: ~3.9V on battery alone, 4.71-4.81V
// with USB present - REGARDLESS of whether a battery is also connected.
// Those two USB-present cases turned out too close together (4.71 vs
// 4.81V, well within measurement noise) to reliably tell apart from
// voltage alone, so this only distinguishes "on battery" vs "USB power
// present" - not whether a battery happens to also be plugged in during
// that second case.
static constexpr float BATTERY_CHARGING_THRESHOLD_V = 4.3f;
static inline bool isCharging(float vbat) { return vbat > BATTERY_CHARGING_THRESHOLD_V; }

// Below this, the battery label turns orange as a heads-up; below the
// second one it also blinks red - both purely visual, no forced action.
static constexpr float BATTERY_LOW_WARN_V  = 3.5f;
static constexpr float BATTERY_CRITICAL_V  = 3.3f;

// ====== Power management ======
// AMOLED pixels that are OFF draw ~zero current (unlike a backlit LCD), so
// dimming/blanking the panel on inactivity is the single biggest lever
// here. CPU throttling on top of that is a smaller, easy win.
static constexpr uint32_t DIM_TIMEOUT_MS   = 15000; // idle -> dim
static constexpr uint32_t BLANK_TIMEOUT_MS = 60000; // idle -> panel off
// AMOLED current draw scales with how brightly pixels are actually
// driven (unlike an LCD backlight, which is roughly all-or-nothing) - so
// unlike DIM (already only used right before going dark), "full"
// brightness itself is worth lowering on battery too, since that's the
// state the panel spends most of its awake time in.
static constexpr uint8_t  BRIGHTNESS_FULL_USB     = 0x33; // ~20% of WRDISBV's 0x00-0xFF range
static constexpr uint8_t  BRIGHTNESS_FULL_BATTERY = 0x33; // same ~20% - user wants both USB and battery full-brightness this low
static constexpr uint8_t  BRIGHTNESS_DIM          = 0x20;
static constexpr uint32_t CPU_MHZ_USB      = 240;
static constexpr uint32_t CPU_MHZ_BATTERY  = 160; // LVGL still needs to stay responsive - 80MHz visibly lags

enum class DisplayPowerState { FULL, DIM, BLANK };
static DisplayPowerState disp_power_state = DisplayPowerState::FULL;
static uint32_t last_activity_ms = 0;
static esp_lcd_panel_io_handle_t g_panel_io = nullptr;
static esp_lcd_panel_handle_t g_panel = nullptr;
static bool g_usbPresent = false; // updated each battery-tick from isCharging(vbat)

// esp_lcd_sh8601's own tx_param() (used for every lcd_init_cmds entry, and
// by esp_lcd_panel_disp_on_off()) never sends a raw DCS command byte - in
// QSPI mode it wraps it into a 32-bit frame first: (0x02 << 24) | (cmd <<
// 8), 0x02 being this panel's "write command" opcode (see
// lib/esp_lcd_sh8601/esp_lcd_sh8601.c's LCD_OPCODE_WRITE_CMD and its
// static tx_param()). That wrapper is private to the driver, so it can't
// be called from here - but esp_lcd_panel_io_tx_param() itself is public,
// and skipping the wrapper (passing 0x53/0x51 as the raw "cmd" argument)
// sends a completely different, meaningless command instead of the
// intended DCS one. The panel just silently ignores it (ESP_OK, no
// error, no visible effect) - which is exactly the "brightness never
// changes" symptom seen on real hardware. Replicate the same encoding
// here so these two writes actually reach WRCTRLD/WRDISBV.
static inline uint32_t sh8601Cmd(uint8_t dcsCmd) { return (0x02UL << 24) | ((uint32_t)dcsCmd << 8); }

// 0x51 = MIPI DCS "Write Display Brightness" - same register lcd_init_cmds
// sets once at boot; this just lets it be changed again at runtime.
static void setLcdBrightness(uint8_t level)
{
  if (!g_panel_io) {
    Serial.println("[POWER] setLcdBrightness: g_panel_io is NULL");
    return;
  }
  // lcd_init_cmds itself proves BCTRL alone (ctrl=0x20, no DD/BL) is enough
  // for 0x51 to take effect: it goes from brightness 0 (screen still dark)
  // to 0xD0 (full) with that same ctrl value and no re-send in between.
  uint8_t ctrl = 0x20; // BCTRL only
  esp_err_t ctrlErr = esp_lcd_panel_io_tx_param(g_panel_io, sh8601Cmd(0x53), &ctrl, 1);
  delay(2); // lcd_init_cmds waits 1ms after this same command - give the panel time to latch CTRL before WRDISBV
  esp_err_t err = esp_lcd_panel_io_tx_param(g_panel_io, sh8601Cmd(0x51), &level, 1);
  Serial.printf("[POWER] setLcdBrightness(0x%02X): ctrl53=%s wrbv51=%s\n",
                level, esp_err_to_name(ctrlErr), esp_err_to_name(err));
}

static void wakeDisplay()
{
  last_activity_ms = millis();
  if (disp_power_state == DisplayPowerState::FULL) return;
  if (disp_power_state == DisplayPowerState::BLANK && g_panel) {
    esp_lcd_panel_disp_on_off(g_panel, true);
  }
  setLcdBrightness(g_usbPresent ? BRIGHTNESS_FULL_USB : BRIGHTNESS_FULL_BATTERY);
  Serial.println("[POWER] display -> FULL (wake)");
  disp_power_state = DisplayPowerState::FULL;
}

// Deep sleep (ext0 wake on the touch controller's TP_INT line) and light
// sleep (GPIO+timer wakeup) were both tried and abandoned: real-hardware
// testing showed TP_INT never actually moves on a genuine touch (so a
// level-triggered RTC wake on it can't work regardless of pull tuning),
// and esp_light_sleep_start() simply hangs with the native USB-CDC
// connection attached. BLANK below is therefore just the panel physically
// switched off - the MCU keeps running completely normally, so the
// already-existing touch_read_cb()/wakeDisplay() path (same one DIM uses)
// is what brings it back on the next touch; no sleep API involved.
static void updateDisplayPower()
{
  uint32_t idleMs = millis() - last_activity_ms;
  if (disp_power_state == DisplayPowerState::FULL && idleMs >= DIM_TIMEOUT_MS) {
    setLcdBrightness(BRIGHTNESS_DIM);
    Serial.println("[POWER] display -> DIM");
    disp_power_state = DisplayPowerState::DIM;
  } else if (disp_power_state == DisplayPowerState::DIM && !g_usbPresent && idleMs >= BLANK_TIMEOUT_MS) {
    // Only blank the panel fully when running off the battery - on USB
    // power there's no battery to save, so just stay dimmed.
    if (g_panel) esp_lcd_panel_disp_on_off(g_panel, false);
    Serial.println("[POWER] display -> BLANK");
    disp_power_state = DisplayPowerState::BLANK;
  } else if (disp_power_state == DisplayPowerState::BLANK && g_usbPresent) {
    // USB got plugged in while blanked - bring the panel back to dim
    // rather than leaving it fully off.
    if (g_panel) esp_lcd_panel_disp_on_off(g_panel, true);
    setLcdBrightness(BRIGHTNESS_DIM);
    Serial.println("[POWER] display -> DIM (USB plugged while blanked)");
    disp_power_state = DisplayPowerState::DIM;
  }
}

static inline lv_color_t ral7037() { return lv_color_make(123, 125, 125); } // muted gray, used for secondary text across screens

// ====== Display pins ======
// CS=9 is the V1/CO5300 revision's pin (confirmed via Waveshare's V1 demo
// zip's lcd_config.h and the V1 schematic's GPIO table: OLED_CS=IO9). The
// "-v2"/SH8601 GitHub repo this was first ported from uses CS=46 instead -
// everything else (CLK/D0-D3/RESET, the whole init command sequence, even
// the esp_lcd_new_panel_sh8601() driver call) is identical between
// revisions, which is why nothing else needed to change once this was
// found: CO5300 is apparently command-compatible enough with the SH8601
// driver that Waveshare just reuses it for both chips.
static constexpr int PIN_LCD_CS   = 9;
static constexpr int PIN_LCD_SCK  = 10;
static constexpr int PIN_LCD_D0   = 11;
static constexpr int PIN_LCD_D1   = 12;
static constexpr int PIN_LCD_D2   = 13;
static constexpr int PIN_LCD_D3   = 14;
static constexpr int PIN_LCD_RST  = 21;
static constexpr spi_host_device_t LCD_SPI_HOST = SPI2_HOST;

// Same command sequence as Waveshare's official 06_LVGL_Test example for
// this board - see lcd_bsp.c there.
static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
  {0x11, (uint8_t[]){0x00}, 0, 80},
  {0xC4, (uint8_t[]){0x80}, 1, 0},
  {0x35, (uint8_t[]){0x00}, 1, 0},
  {0x53, (uint8_t[]){0x20}, 1, 1},
  {0x63, (uint8_t[]){0xFF}, 1, 1},
  {0x51, (uint8_t[]){0x00}, 1, 1},
  {0x29, (uint8_t[]){0x00}, 0, 10},
  {0x51, (uint8_t[]){0xD0}, 1, 0},
};


// ====== LVGL tick ======
static void lv_tick_cb(void*) { lv_tick_inc(1); }

static void lvgl_init_tick()
{
  const esp_timer_create_args_t args = {
    .callback = &lv_tick_cb, .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK, .name = "lv_tick"
  };
  esp_timer_handle_t timer;
  esp_timer_create(&args, &timer);
  esp_timer_start_periodic(timer, 1000);
}

// esp_lcd's color transfer is asynchronous (DMA, queued) - this fires from
// an ISR context once the transfer actually completes, which is the
// correct place to tell LVGL its buffer is free again. Never call
// lv_disp_flush_ready() right after esp_lcd_panel_draw_bitmap() returns -
// that only means the transfer was queued, not finished.
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void* user_ctx)
{
  lv_disp_drv_t* disp_drv = (lv_disp_drv_t*)user_ctx;
  lv_disp_flush_ready(disp_drv);
  return false;
}

// This panel's visible area starts at column 20 within the controller's
// addressable RAM, not column 0 - without this offset, everything is
// drawn 20 columns too far left (wrapping the rightmost 20 columns of
// each row into garbage), which is exactly the vertical stripe seen on
// screen. Value and requirement straight from Waveshare's own official
// example (lcd_bsp.c's example_lvgl_flush_cb) - not documented anywhere
// else, no way to have derived it analytically.
static constexpr int PANEL_X_OFFSET = 0x14;

static void my_flush_cb(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p)
{
  esp_lcd_panel_draw_bitmap(g_panel,
                             area->x1 + PANEL_X_OFFSET, area->y1,
                             area->x2 + PANEL_X_OFFSET + 1, area->y2 + 1,
                             color_p);
  // lv_disp_flush_ready() is called from notify_lvgl_flush_ready() above once the DMA transfer actually completes.
}

// This panel's CASET/RASET window must land on 2-pixel boundaries - an
// unaligned flush area shifts color data across the odd/even pixel
// boundary, which reads back as scrambled R/G/B (the pink/white text
// seen on screen). Also straight from the official example.
static void my_rounder_cb(lv_disp_drv_t*, lv_area_t* area)
{
  area->x1 = (area->x1 >> 1) << 1;
  area->y1 = (area->y1 >> 1) << 1;
  area->x2 = ((area->x2 >> 1) << 1) + 1;
  area->y2 = ((area->y2 >> 1) << 1) + 1;
}

static bool display_init()
{
  const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
      PIN_LCD_SCK, PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3,
      LCD_HOR_RES * LCD_VER_RES * 2 + 8);
  if (spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
    Serial.println("[DISPLAY] spi_bus_initialize failed");
    return false;
  }

  static lv_disp_drv_t disp_drv; // must outlive this function - referenced by the io callback's user_ctx
  esp_lcd_panel_io_handle_t io_handle = nullptr;
  const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(
      PIN_LCD_CS, notify_lvgl_flush_ready, &disp_drv);
  if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &io_handle) != ESP_OK) {
    Serial.println("[DISPLAY] esp_lcd_new_panel_io_spi failed");
    return false;
  }
  g_panel_io = io_handle;

  static sh8601_vendor_config_t vendor_config = {
    .init_cmds = lcd_init_cmds,
    .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
    .flags = { .use_qspi_interface = 1 },
  };
  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = PIN_LCD_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = 16,
    .vendor_config = &vendor_config,
  };
  if (esp_lcd_new_panel_sh8601(io_handle, &panel_config, &g_panel) != ESP_OK) {
    Serial.println("[DISPLAY] esp_lcd_new_panel_sh8601 failed");
    return false;
  }

  esp_err_t rst_err  = esp_lcd_panel_reset(g_panel);
  esp_err_t init_err = esp_lcd_panel_init(g_panel);
  esp_err_t on_err   = esp_lcd_panel_disp_on_off(g_panel, true);
  Serial.printf("[DISPLAY] reset=%s init=%s disp_on=%s\n",
                esp_err_to_name(rst_err), esp_err_to_name(init_err), esp_err_to_name(on_err));

  lv_init();
  lvgl_init_tick();

  const size_t drawBufPixels = (size_t)LCD_HOR_RES * 40;
  lv_color_t* buf1 = (lv_color_t*)heap_caps_malloc(drawBufPixels * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  lv_color_t* buf2 = (lv_color_t*)heap_caps_malloc(drawBufPixels * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  if (!buf1 || !buf2) {
    Serial.println("[BOOT] FATAL: PSRAM alloc for LVGL draw buffers failed");
    return false;
  }
  static lv_disp_draw_buf_t draw_buf;
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, drawBufPixels);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = LCD_HOR_RES;
  disp_drv.ver_res  = LCD_VER_RES;
  disp_drv.flush_cb   = my_flush_cb;
  disp_drv.rounder_cb = my_rounder_cb;
  disp_drv.draw_buf   = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  return true;
}

// ====== WiFi + NTP ======
// Credentials/timezone now come from wifi_portal.cpp's NVS storage (set
// via the on-device SETUP screen), not a hardcoded wifi_credentials.h -
// that was only ever a bring-up stopgap until SETUP itself was ported.
static bool     g_wifiOk         = false;
static bool     g_ntpConfigured  = false;
static bool     g_otaInitialized = false; // see wifi_ntp_update_state() - ota_init() must wait until WiFi.mode() has actually run once, or ArduinoOTA.begin()'s UDP/mDNS setup crashes (xQueueSemaphoreTake assert on an uninitialized network stack) - confirmed on real hardware
static bool     g_updateChecked  = false; // one release-check per WiFi connection, not every loop() tick - see wifi_ntp_update_state()
static uint32_t g_lastWifiTryMs  = 0;
static constexpr uint32_t WIFI_RETRY_MS = 15000;

static void wifi_begin_nonblocking()
{
  // Force STA fully down and back up on every (re)connect attempt rather
  // than layering WiFi.begin() on top of a still-negotiating previous one -
  // see the main project's own wifi_begin_nonblocking() for the full
  // reasoning (a stuck STA otherwise poisons later scanNetworks() calls).
  WiFi.disconnect(true /* wifioff */, false);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(wifi_portal_get_ssid(), wifi_portal_get_pass());
  g_lastWifiTryMs = millis();
}

// WiFi.status() == WL_CONNECTED only means "associated/authenticated with
// the AP" - it does NOT guarantee DHCP has actually handed out a usable
// address. Confirmed on real hardware via the new [NETDIAG] logging: status
// read WL_CONNECTED while WiFi.localIP() was still 0.0.0.0, which meant
// ota_init()/wifi_monitor_start() below were firing (and reporting success)
// against an interface with no real address - the device believed it was
// online and reachable while actually being unreachable from anywhere else
// on the network. This is likely the root cause behind most of tonight's
// "the setup page just won't load" reports, not just today's more obvious
// bugs (crash on SETUP entry, favicon, etc).
static inline bool wifiHasValidIp() { return WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0); }

static void wifi_ntp_update_state()
{
  uint32_t now = millis();
  if (wifiHasValidIp()) {
    if (!g_wifiOk) {
      g_wifiOk = true;
      Serial.printf("[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
    }
    if (!g_ntpConfigured) {
      g_ntpConfigured = true;
      configTime((long)wifi_portal_get_tz_offset_min() * 60, 0, "pool.ntp.org");
    }
    if (!g_otaInitialized) {
      g_otaInitialized = true;
      ota_init();
    }
    // Start serving the setup/config web page as soon as STA connects,
    // regardless of which screen is showing - wifi_monitor_start() is
    // idempotent (see its own comment), so this is safe alongside the
    // still-existing monitor_screen_on_enter() call for the AP-fallback
    // case (no STA yet).
    wifi_monitor_start();
    if (!g_updateChecked) {
      g_updateChecked = true;
      update_check_now(); // blocking HTTPS call - one-shot per connection, see g_updateChecked
    }
  } else {
    if (g_wifiOk) {
      g_wifiOk = false;
      g_ntpConfigured = false;
      g_updateChecked = false; // re-check on the next reconnect, not never again
      Serial.println("[WiFi] Disconnected.");
      // Stop the STA-bound monitor server/mDNS now rather than leaving it
      // stale and bound to a connection that's gone (or stuck at 0.0.0.0) -
      // wifi_monitor_start() will cleanly re-launch it once wifiHasValidIp()
      // is true again. Leave the own-AP fallback alone if that's what's
      // active - unrelated to this STA-side transition.
      if (!wifi_monitor_using_own_ap()) wifi_monitor_stop();
    }
    if (now - g_lastWifiTryMs >= WIFI_RETRY_MS) {
      Serial.println("[WiFi] Retry connect...");
      wifi_begin_nonblocking();
    }
  }
}

// ====== CLOCK screen ======
static lv_obj_t* clock_wifi    = nullptr; // white = connected, red = not - see the 1s tick below
static lv_obj_t* clock_battery = nullptr;
// Top-center status icons: setup/config web server reachable, and last
// Home Assistant poll result - dim gray when off/unknown, lit when good.
// Updated once a second alongside clock_wifi below.
static lv_obj_t* clock_srv_icon = nullptr;
static lv_obj_t* clock_ha_icon  = nullptr;
static bool      g_haLastPollOk = false; // meaningful only alongside wifi_portal_has_ha_config()
// Firmware update available (update_is_available(), see update_check.h) -
// hidden unless there's actually an update, tap jumps to SCR_UPDATE.
static lv_obj_t* clock_update_icon = nullptr;
// Same visibility/tap-target as clock_update_icon, just a much bigger,
// harder-to-miss text version in the middle of the screen - the icon
// alone is small/easy to overlook.
static lv_obj_t* clock_update_text = nullptr;
static lv_obj_t* clock_time    = nullptr;
static lv_obj_t* clock_date    = nullptr;

// Currently-selected entity in the carousel (see update_entity_ui() and
// gesture_event_cb()'s SCR_CLOCK case) - name shown here, above the button.
static lv_obj_t* clock_entity_name_lbl = nullptr;
static int       g_currentEntityIdx    = 0;

// Home Assistant light-switch button - tap to toggle the currently-selected
// carousel entity, color reflects its last known state polled from HA
// (gray = off/unknown, yellow = on).
static lv_obj_t* clock_light_btn  = nullptr;
static lv_obj_t* clock_light_icon = nullptr;
static bool g_haLightOn = false;

// Small persistent dev overlay at the bottom of CLOCK only - not part of
// the real screen, kept as a quick sanity check that touch is still
// alive while iterating on the rest of the port.
static lv_obj_t* lblTouch = nullptr;

// ====== Screens ======
// SCR_UPDATE is deliberately NOT part of either swipe rotation (entity
// carousel or wifi/setup) - it's a tap-to-drill-down from the small update
// icon on CLOCK (see clock_update_icon), not a swipe destination, so it
// doesn't complicate the existing two-axis gesture model. Any swipe
// direction from it just returns to CLOCK.
enum Screen { SCR_CLOCK, SCR_SETUP, SCR_MONITOR, SCR_UPDATE };
static Screen     current_screen = SCR_CLOCK;
static lv_obj_t*  scr_clock   = nullptr;
static lv_obj_t*  scr_setup   = nullptr;
static lv_obj_t*  scr_monitor = nullptr;
static lv_obj_t*  scr_update  = nullptr;

// ====== SETUP screen ======
static lv_obj_t* setup_ssid_lbl = nullptr;
static lv_obj_t* setup_ip_lbl   = nullptr;

// ====== MONITOR screen ======
static lv_obj_t* monitor_status_lbl  = nullptr;
static lv_obj_t* monitor_address_lbl = nullptr;

// ====== UPDATE screen ======
static lv_obj_t* update_version_lbl = nullptr;
static lv_obj_t* update_status_lbl  = nullptr;
static lv_obj_t* update_btn         = nullptr;


// Forward declarations - switch_screen() calls these, but they're defined
// further down (near the rest of the SETUP/MONITOR logic).
static void setup_screen_on_enter();
static void setup_screen_on_leave();
static void monitor_screen_on_enter();
static void monitor_screen_on_leave();
static void clock_screen_on_enter();
static void update_screen_on_enter();

static lv_obj_t* screen_obj(Screen s)
{
  switch (s) {
    case SCR_CLOCK:   return scr_clock;
    case SCR_SETUP:   return scr_setup;
    case SCR_MONITOR: return scr_monitor;
    case SCR_UPDATE:  return scr_update;
  }
  return scr_clock;
}

static void switch_screen(Screen s)
{
  if (current_screen == SCR_SETUP && s != SCR_SETUP) setup_screen_on_leave();
  if (current_screen == SCR_MONITOR && s != SCR_MONITOR) monitor_screen_on_leave();

  current_screen = s;
  lv_scr_load(screen_obj(s));

  if (s == SCR_CLOCK) clock_screen_on_enter();
  if (s == SCR_SETUP) setup_screen_on_enter();
  if (s == SCR_MONITOR) monitor_screen_on_enter();
  if (s == SCR_UPDATE) update_screen_on_enter();
}

// Button (and entity name label, see update_entity_ui()) use the current
// entity's own identity color regardless of on/off state - a low-vision
// aid the user asked for: entities can be told apart by color alone, not
// just by reading the (comparatively small) name. On/off is conveyed by
// brightness of that same hue (full color vs darkened), not by switching
// to an unrelated color, so the identity stays recognizable either way.
static void update_light_btn_style()
{
  if (!clock_light_btn) return;
  lv_color_t entityColor = lv_color_hex(wifi_portal_get_entity_color(g_currentEntityIdx));

  // The button background follows wifi_portal_get_color_button() - the
  // name label (below) always shows the identity color regardless, that
  // toggle only affects the (much bigger, so higher-contrast) button.
  if (wifi_portal_get_color_button()) {
    lv_color_t bg = g_haLightOn ? entityColor : lv_color_darken(entityColor, LV_OPA_70);
    lv_obj_set_style_bg_color(clock_light_btn, bg, 0);
    lv_obj_set_style_text_color(clock_light_icon, g_haLightOn ? lv_color_black() : lv_color_white(), 0);
  } else {
    lv_obj_set_style_bg_color(clock_light_btn, g_haLightOn ? lv_color_hex(0xFFC107) : lv_color_hex(0x333333), 0);
    lv_obj_set_style_text_color(clock_light_icon, g_haLightOn ? lv_color_black() : lv_color_hex(0x888888), 0);
  }

  if (clock_entity_name_lbl) lv_obj_set_style_text_color(clock_entity_name_lbl, entityColor, 0);
}

// Called whenever the selected carousel entry changes (swipe left/right, or
// landing back on CLOCK - see clock_screen_on_enter()): updates the name
// label and immediately polls the newly-selected entity's real state,
// rather than showing the previous entity's color until the next periodic
// poll (up to 5s later, see loop()) catches up.
static void update_entity_ui()
{
  if (!clock_entity_name_lbl) return;

  int count = wifi_portal_get_entity_count();
  if (count <= 0) {
    lv_label_set_text(clock_entity_name_lbl, "No entity configured");
    g_haLightOn = false;
    g_haLastPollOk = false;
    update_light_btn_style();
    return;
  }

  lv_label_set_text(clock_entity_name_lbl, wifi_portal_get_entity_name(g_currentEntityIdx));

  bool isOn = false;
  if (ha_light_poll_state(wifi_portal_get_entity_id(g_currentEntityIdx), &isOn)) {
    g_haLightOn = isOn;
    g_haLastPollOk = true;
  } else {
    // Unknown/offline - default to the "off" visual rather than carrying
    // over whatever the previously-selected entity's state happened to be.
    g_haLightOn = false;
    g_haLastPollOk = false;
  }
  update_light_btn_style();
}

// Blocking HTTP round-trip (see ha_light.cpp) - runs synchronously inside
// this tap handler, so the UI briefly freezes for the call's duration
// (typically well under a second on a local network). Acceptable for an
// occasional tap; revisit with a background task if it ever feels janky.
static void do_ha_light_toggle()
{
  int count = wifi_portal_get_entity_count();
  if (count <= 0) return;

  bool ok = ha_light_toggle(wifi_portal_get_entity_id(g_currentEntityIdx));
  if (ok) {
    // Optimistic flip for instant feedback - the next periodic poll (see
    // loop()) will correct this within a few seconds if the toggle
    // actually failed server-side or something else changed the state
    // concurrently.
    g_haLightOn = !g_haLightOn;
    update_light_btn_style();
  }
}

static void ha_light_btn_cb(lv_event_t* e)
{
  (void)e;
  do_ha_light_toggle();
}

// NOTE: an earlier version of this handler had SETUP/MONITOR react to "any
// swipe direction" instead of a specific one, because real-hardware testing
// at the time found LEFT/RIGHT swipes always coming out classified as
// TOP/BOTTOM by LVGL's gesture detector (see the [GESTURE] log line below -
// it's the tool to check this with). This version requires the two axes to
// actually be distinguishable (LEFT/RIGHT = entity carousel, UP/DOWN =
// wifi/setup carousel) - verify on real hardware; if direction still comes
// out wrong, this navigation won't behave as intended.
static void gesture_event_cb(lv_event_t* e)
{
  lv_indev_t* indev = lv_indev_get_act();
  if (!indev) return;
  lv_dir_t dir = lv_indev_get_gesture_dir(indev);
  Serial.printf("[GESTURE] screen=%d dir=%d\n", (int)current_screen, (int)dir);

  switch (current_screen) {
    case SCR_CLOCK: {
      int count = wifi_portal_get_entity_count();
      if (dir == LV_DIR_LEFT && count > 1) {
        g_currentEntityIdx = (g_currentEntityIdx + 1) % count;
        update_entity_ui();
      } else if (dir == LV_DIR_RIGHT && count > 1) {
        g_currentEntityIdx = (g_currentEntityIdx - 1 + count) % count;
        update_entity_ui();
      } else if (dir == LV_DIR_TOP) {
        switch_screen(SCR_SETUP);
      } else if (dir == LV_DIR_BOTTOM) {
        // Deliberately straight to MONITOR, not via SCR_SETUP first: SETUP
        // always tears down the STA connection to start its own AP (see
        // setup_screen_on_enter()), so routing both directions through it
        // would mean MONITOR could never find an existing WiFi connection
        // to serve over - it'd always hit its own-AP fallback too, even
        // when perfectly good STA connectivity already existed (confirmed
        // on real hardware: this was exactly why the "remote setup" page
        // kept being unreachable at its real LAN IP).
        switch_screen(SCR_MONITOR);
      }
      break;
    }
    case SCR_SETUP:
    case SCR_MONITOR:
      if (dir == LV_DIR_TOP || dir == LV_DIR_BOTTOM) {
        // Only two screens in this group - either direction just toggles
        // to the other one (a real "rotation" once/if a third one joins).
        switch_screen(current_screen == SCR_SETUP ? SCR_MONITOR : SCR_SETUP);
      } else if (dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) {
        switch_screen(SCR_CLOCK);
      }
      break;
    case SCR_UPDATE:
      // Not part of either rotation group - any swipe just leaves it.
      if (dir != LV_DIR_NONE) switch_screen(SCR_CLOCK);
      break;
  }
}

// ====== CLOCK screen hook ======
// Guards against g_currentEntityIdx having gone stale while away from
// CLOCK - e.g. the owner removed entities on the setup page, shrinking the
// list below the previously-selected index. Also covers the very first
// entry into CLOCK from setup(), where update_entity_ui() hasn't run yet.
static void clock_screen_on_enter()
{
  int count = wifi_portal_get_entity_count();
  if (count <= 0) g_currentEntityIdx = 0;
  else if (g_currentEntityIdx >= count) g_currentEntityIdx = count - 1;
  else if (g_currentEntityIdx < 0) g_currentEntityIdx = 0;
  update_entity_ui();
}

// ====== SETUP / MONITOR screen hooks ======
// Entering SETUP starts the AP+portal (see wifi_portal_start_ap()) and
// shows how to reach it; leaving stops the AP and resumes as a normal
// WiFi client if credentials are saved. No on-device QR code widget yet
// (see file header) - the hotspot name/address are shown as plain text.
static void setup_screen_on_enter()
{
  wifi_portal_start_ap();

  if (setup_ssid_lbl) lv_label_set_text(setup_ssid_lbl, wifi_portal_ap_ssid());
  if (setup_ip_lbl) {
    char addr[48];
    snprintf(addr, sizeof(addr), "http://%s", wifi_portal_ap_ip().toString().c_str());
    lv_label_set_text(setup_ip_lbl, addr);
  }
}

static void setup_screen_on_leave()
{
  wifi_portal_stop_ap();
  if (wifi_portal_has_credentials()) wifi_begin_nonblocking();
}

// Entering MONITOR starts serving live pitch/roll over WiFi - piggybacking
// on an existing connection if there is one, otherwise falling back to
// this device's own hotspot the same way SETUP does.
static void monitor_screen_on_enter()
{
  wifi_monitor_start();

  if (wifi_monitor_using_own_ap()) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Connect phone to:\n%s", wifi_portal_ap_ssid());
    if (monitor_status_lbl) lv_label_set_text(monitor_status_lbl, buf);

    char addr[48];
    snprintf(addr, sizeof(addr), "http://%s", wifi_monitor_ip().toString().c_str());
    if (monitor_address_lbl) lv_label_set_text(monitor_address_lbl, addr);
  } else {
    if (monitor_status_lbl) lv_label_set_text(monitor_status_lbl, "Connect phone to the same WiFi");

    char addr[96];
    snprintf(addr, sizeof(addr), "http://%s\nhttp://%s.local",
             wifi_monitor_ip().toString().c_str(), wifi_monitor_hostname());
    if (monitor_address_lbl) lv_label_set_text(monitor_address_lbl, addr);
  }
}

static void monitor_screen_on_leave()
{
  if (wifi_monitor_using_own_ap()) {
    // Was serving via this device's own fallback AP (no STA connection
    // available) - tear that down and try to reconnect to the real
    // network now that the owner has left the screen.
    wifi_monitor_stop();
    if (wifi_portal_has_credentials()) wifi_begin_nonblocking();
  }
  // else: serving directly over an existing STA connection - leave it
  // running in the background (see wifi_ntp_update_state()) so the setup
  // page stays reachable from any screen, not just while physically on
  // REMOTE SETUP.
}

// ====== UPDATE screen hook ======
static void update_screen_on_enter()
{
  if (update_version_lbl) {
    char buf[96];
    snprintf(buf, sizeof(buf), "Current: %s\nLatest: %s",
             update_current_version(),
             update_is_available() ? update_latest_version() : "(up to date)");
    lv_label_set_text(update_version_lbl, buf);
  }
  if (update_status_lbl) lv_label_set_text(update_status_lbl, "");
  if (update_btn) {
    // Dim the button rather than removing it when there's nothing to
    // install - tapping it while !update_is_available() is a silent no-op
    // (see its event callback), this just makes that visually obvious.
    lv_obj_set_style_bg_opa(update_btn, update_is_available() ? LV_OPA_COVER : LV_OPA_40, 0);
  }
}

// ====== Touch (FT3168) ======
// tp itself is declared earlier.
static bool     g_touchActive     = false; // debounced "is a touch session ongoing" - see note below
static uint32_t g_lastPressSeenMs = 0;
static lv_coord_t g_touchFx = 0, g_touchFy = 0;


static void touch_read_cb(lv_indev_drv_t*, lv_indev_data_t* data)
{
  uint16_t x = 0, y = 0;
  uint8_t g = 0;
  bool pressed = tp.getTouch(&x, &y, &g);

  // This touch controller's raw press/release reporting itself flickers
  // during a single continuous physical contact - getTouch() intermittently
  // reports "not pressed" for one read even mid-hold (confirmed via the
  // [TOUCH] log: consecutive reads a few ms apart alternating touch found/
  // not found while a finger stayed down). Treated naively, every single
  // one of those blips looked like a fresh press-then-release to LVGL,
  // which reset the position filter below on every read and also could
  // fire spurious short clicks/releases. Debounce: a raw "not pressed"
  // only ends the session once it's persisted for TOUCH_RELEASE_DEBOUNCE_MS
  // - anything shorter is treated as still the same ongoing touch.
  constexpr uint32_t TOUCH_RELEASE_DEBOUNCE_MS = 100;
  const uint32_t nowMs = millis();

  if (pressed) {
    bool wasAsleep = disp_power_state != DisplayPowerState::FULL;
    wakeDisplay();
    if (wasAsleep) {
      // First touch after dim/blank only wakes the screen - don't also let
      // it land on whatever button happens to be under the finger.
      data->state = LV_INDEV_STATE_REL;
      return;
    }
    if (x >= LCD_HOR_RES) x = LCD_HOR_RES - 1;
    if (y >= LCD_VER_RES) y = LCD_VER_RES - 1;

    // Position dead-zone: raw coordinates also wander 30-50+ px on their
    // own during what's physically a stationary tap - past LVGL's scroll
    // threshold, so a plain tap was being misclassified as a drag and
    // never firing LV_EVENT_CLICKED. Only move the reported point once a
    // reading has actually wandered past a real threshold; a brand new
    // session always reports its first raw position exactly (no lag on
    // touch-down, for the wake-swallow logic above and small targets).
    constexpr lv_coord_t TOUCH_DEADZONE_PX = 18;
    bool freshSession = !g_touchActive;
    if (freshSession) {
      g_touchFx = (lv_coord_t)x;
      g_touchFy = (lv_coord_t)y;
    } else if (abs((int)x - (int)g_touchFx) > TOUCH_DEADZONE_PX || abs((int)y - (int)g_touchFy) > TOUCH_DEADZONE_PX) {
      g_touchFx = (lv_coord_t)x;
      g_touchFy = (lv_coord_t)y;
    }
    g_touchActive = true;
    g_lastPressSeenMs = nowMs;

    data->point.x = g_touchFx;
    data->point.y = g_touchFy;
    data->state = LV_INDEV_STATE_PR;
    if (lblTouch) {
      char buf[32];
      snprintf(buf, sizeof(buf), "touch: %u,%u", x, y);
      lv_label_set_text(lblTouch, buf);
    }
    Serial.printf("[TOUCH] screen=%d x=%u y=%u filt=%d,%d\n", (int)current_screen, x, y, g_touchFx, g_touchFy);
  } else if (g_touchActive && (nowMs - g_lastPressSeenMs) < TOUCH_RELEASE_DEBOUNCE_MS) {
    // Raw says released, but within the debounce window - keep reporting
    // the held position as still pressed; a real lift will show up as a
    // longer gap and fall through to the branch below.
    data->point.x = g_touchFx;
    data->point.y = g_touchFy;
    data->state = LV_INDEV_STATE_PR;
  } else {
    g_touchActive = false;
    data->state = LV_INDEV_STATE_REL;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(150);

  Serial.printf("[BOOT] PSRAM: found=%d size=%u free=%u | heap free=%u\n",
                psramFound(), (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreePsram(),
                (unsigned)ESP.getFreeHeap());

  // Diagnostic: print the actual reset reason by name - a crash
  // (PANIC/WDT) or brownout looks very different from a normal power-on.
  esp_reset_reason_t resetReason = esp_reset_reason();
  const char* resetReasonStr = "UNKNOWN";
  switch (resetReason) {
    case ESP_RST_POWERON:   resetReasonStr = "POWERON"; break;
    case ESP_RST_EXT:       resetReasonStr = "EXT (reset pin/button)"; break;
    case ESP_RST_SW:        resetReasonStr = "SW (esp_restart)"; break;
    case ESP_RST_PANIC:     resetReasonStr = "PANIC (crash)"; break;
    case ESP_RST_INT_WDT:   resetReasonStr = "INT_WDT"; break;
    case ESP_RST_TASK_WDT:  resetReasonStr = "TASK_WDT"; break;
    case ESP_RST_WDT:       resetReasonStr = "WDT (other)"; break;
    case ESP_RST_DEEPSLEEP: resetReasonStr = "DEEPSLEEP"; break;
    case ESP_RST_BROWNOUT:  resetReasonStr = "BROWNOUT (power sag!)"; break;
    case ESP_RST_SDIO:      resetReasonStr = "SDIO"; break;
    default: break;
  }
  Serial.printf("[BOOT] reset_reason=%s\n", resetReasonStr);

  if (!display_init()) {
    Serial.println("[BOOT] FATAL: display_init() failed - halting");
    while (true) delay(1000);
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  tp.begin();

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type    = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_read_cb;
  // This touch controller's raw readings wander noticeably even during a
  // "stationary" tap (observed 50+ px of jitter) - well past LVGL's
  // default 10px scroll_limit, which meant a plain tap on the new HA
  // light button was getting classified as a drag/scroll and never fired
  // LV_EVENT_CLICKED. Raised so ordinary tap jitter doesn't get
  // misclassified, while a real swipe (100+ px) still comfortably clears
  // it.
  indev_drv.scroll_limit = 30;
  lv_indev_drv_register(&indev_drv);

  scr_clock   = lv_obj_create(NULL);
  scr_setup   = lv_obj_create(NULL);
  scr_monitor = lv_obj_create(NULL);
  scr_update  = lv_obj_create(NULL);
  lv_obj_t* screens[] = {scr_clock, scr_setup, scr_monitor, scr_update};
  for (auto* scr : screens) {
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_add_event_cb(scr, gesture_event_cb, LV_EVENT_GESTURE, nullptr);
    // lv_obj_create() is scrollable by default in both axes - a drag is
    // only promoted to a bubbled LV_EVENT_GESTURE once LVGL decides the
    // pressed object can't itself scroll further in that direction.
    // Confirmed on real hardware: horizontal swipes were producing zero
    // [GESTURE] events at all (not even misclassified - just absorbed),
    // while vertical ones worked, which matches this being treated as a
    // no-op horizontal scroll of the screen itself rather than a gesture.
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  }

  // ===== CLOCK screen =====
  clock_wifi = lv_label_create(scr_clock);
  lv_label_set_text(clock_wifi, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_color(clock_wifi, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_wifi, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_wifi, LV_ALIGN_TOP_LEFT, 8, 8);

  clock_battery = lv_label_create(scr_clock);
  lv_label_set_text(clock_battery, LV_SYMBOL_BATTERY_EMPTY " --.--V");
  lv_obj_set_style_text_color(clock_battery, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_battery, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_battery, LV_ALIGN_TOP_RIGHT, -8, 8);

  // Setup/config web page reachable (wifi_monitor_is_active()) - gear icon,
  // since that's literally the setup page's own connectivity, not the
  // general WiFi-association icon (clock_wifi) above.
  clock_srv_icon = lv_label_create(scr_clock);
  lv_label_set_text(clock_srv_icon, LV_SYMBOL_SETTINGS);
  lv_obj_set_style_text_font(clock_srv_icon, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_srv_icon, LV_ALIGN_TOP_MID, -30, 8);

  // Last Home Assistant poll result (g_haLastPollOk, see update_entity_ui()
  // and the periodic poll in loop()) - house icon for "Home" Assistant.
  clock_ha_icon = lv_label_create(scr_clock);
  lv_label_set_text(clock_ha_icon, LV_SYMBOL_HOME);
  lv_obj_set_style_text_font(clock_ha_icon, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_ha_icon, LV_ALIGN_TOP_MID, 0, 8);

  // Firmware update available (update_is_available()) - hidden by default,
  // shown/lit only when true (see the 1s tick below); tap jumps straight
  // to SCR_UPDATE (not a swipe target - see the Screen enum comment).
  clock_update_icon = lv_label_create(scr_clock);
  lv_label_set_text(clock_update_icon, LV_SYMBOL_DOWNLOAD);
  lv_obj_set_style_text_font(clock_update_icon, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_update_icon, LV_ALIGN_TOP_MID, 30, 8);
  lv_obj_add_flag(clock_update_icon, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(clock_update_icon, [](lv_event_t*) { switch_screen(SCR_UPDATE); }, LV_EVENT_CLICKED, nullptr);

  clock_time = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(clock_time, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_time, &lv_font_montserrat_48, 0);
  lv_label_set_text(clock_time, "--:--");
  lv_obj_align(clock_time, LV_ALIGN_CENTER, 0, -130);

  clock_date = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(clock_date, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_date, &lv_font_montserrat_22, 0);
  lv_label_set_text(clock_date, "----------");
  lv_obj_align(clock_date, LV_ALIGN_CENTER, 0, -85);

  // Currently-selected carousel entity's name - between the date and the
  // button, text set for real by update_entity_ui()/clock_screen_on_enter()
  // further down, not here. font_pl_34 (src/font_pl_34.c, generated from
  // LVGL's own bundled Montserrat-Medium.ttf via lv_font_conv - same source
  // the stock lv_font_montserrat_* built-ins use, so it matches visually)
  // adds the Polish diacritics (Ą Ć Ę Ł Ń Ó Ś Ź Ż and lowercase) that none
  // of LVGL's built-in fonts include (ASCII + Latin-1 only) - needed since
  // entity names are free-text and this project's whole audience is Polish.
  // Between clock_date (-85) and clock_entity_name_lbl (-5) below - hidden
  // (opacity toggled in the 1s tick) unless update_is_available().
  clock_update_text = lv_label_create(scr_clock);
  lv_label_set_text(clock_update_text, "UPDATE AVAILABLE");
  lv_obj_set_style_text_color(clock_update_text, lv_color_hex(0xFFA500), 0);
  lv_obj_set_style_text_font(clock_update_text, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_align(clock_update_text, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(clock_update_text, LV_ALIGN_CENTER, 0, -45);
  lv_obj_add_flag(clock_update_text, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(clock_update_text, [](lv_event_t*) { switch_screen(SCR_UPDATE); }, LV_EVENT_CLICKED, nullptr);

  clock_entity_name_lbl = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(clock_entity_name_lbl, ral7037(), 0);
  lv_obj_set_style_text_font(clock_entity_name_lbl, &font_pl_34, 0);
  lv_obj_set_style_text_align(clock_entity_name_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(clock_entity_name_lbl, "");
  lv_obj_align(clock_entity_name_lbl, LV_ALIGN_CENTER, 0, -5);

  // Home Assistant light-switch button - large rounded bar (not a small
  // circle) so it's forgiving of this touch panel's real-world jitter/
  // drift (see touch_read_cb()'s comment) - a small target made it very
  // easy to drift off the hit area mid-tap and get classified as a
  // drag/gesture instead of a click. Uses lv_btn_create(), same as the
  // CAL screen's btnCal (which the user confirmed DOES register clicks
  // reliably on this touch panel) rather than a plain lv_obj_create() -
  // whatever default behavior differs between the two widget classes,
  // matching the proven-working one beats guessing at LVGL internals.
  // Gesture bubble stays on so a swipe starting here still navigates
  // screens normally; only a plain tap (no drag) fires the toggle.
  clock_light_btn = lv_btn_create(scr_clock);
  lv_obj_set_size(clock_light_btn, 250, 150);
  lv_obj_set_style_radius(clock_light_btn, 24, 0);
  lv_obj_set_style_border_width(clock_light_btn, 0, 0);
  lv_obj_align(clock_light_btn, LV_ALIGN_CENTER, 0, 100);
  // lv_btn_create() is scrollable by default too - without clearing this,
  // a swipe starting on the button (a large, central target most swipes
  // will cross) could get absorbed as the button's own no-op scroll
  // instead of ever reaching GESTURE_BUBBLE below. Same fix as the
  // screens themselves, see that comment for the real-hardware symptom.
  lv_obj_clear_flag(clock_light_btn, LV_OBJ_FLAG_SCROLLABLE);
  // GESTURE_BUBBLE alone is enough - it re-fires LV_EVENT_GESTURE on
  // scr_clock, which already has gesture_event_cb registered (see the
  // shared screens loop above). Also registering gesture_event_cb directly
  // on the button double-fired every swipe that started on it (button's own
  // callback, then again via the bubble to the screen) - harmless for the
  // old switch_screen() calls (re-entering the same screen is a no-op) but
  // silently cancelled out entity-carousel rotation: idx+1 immediately
  // followed by idx-1 nets to no visible change, which is exactly the
  // "left/right does nothing on CLOCK" symptom seen on real hardware.
  lv_obj_add_flag(clock_light_btn, LV_OBJ_FLAG_GESTURE_BUBBLE);
  lv_obj_add_event_cb(clock_light_btn, ha_light_btn_cb, LV_EVENT_CLICKED, nullptr);

  clock_light_icon = lv_label_create(clock_light_btn);
  lv_label_set_text(clock_light_icon, LV_SYMBOL_POWER);
  lv_obj_set_style_text_font(clock_light_icon, &lv_font_montserrat_48, 0);
  lv_obj_center(clock_light_icon);

  update_light_btn_style();

  lblTouch = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(lblTouch, lv_color_make(90, 90, 90), 0);
  lv_obj_set_style_text_font(lblTouch, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblTouch, "touch: --");
  lv_obj_align(lblTouch, LV_ALIGN_BOTTOM_MID, 0, -28);

  // ===== SETUP screen port =====
  lv_obj_t* setupTitle = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupTitle, lv_color_white(), 0);
  lv_label_set_text(setupTitle, "WIFI SETUP");
  lv_obj_align(setupTitle, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t* setupHint1 = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupHint1, ral7037(), 0);
  lv_obj_set_style_text_align(setupHint1, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(setupHint1, "Connect phone to:");
  lv_obj_align(setupHint1, LV_ALIGN_CENTER, 0, -60);

  setup_ssid_lbl = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setup_ssid_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_align(setup_ssid_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(setup_ssid_lbl, "---");
  lv_obj_align(setup_ssid_lbl, LV_ALIGN_CENTER, 0, -30);

  lv_obj_t* setupHint2 = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupHint2, ral7037(), 0);
  lv_obj_set_style_text_align(setupHint2, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(setupHint2, "or in a browser:");
  lv_obj_align(setupHint2, LV_ALIGN_CENTER, 0, 10);

  setup_ip_lbl = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setup_ip_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_align(setup_ip_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(setup_ip_lbl, "---");
  lv_obj_align(setup_ip_lbl, LV_ALIGN_CENTER, 0, 40);

  lv_obj_t* setupHint3 = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupHint3, ral7037(), 0);
  lv_obj_set_style_text_align(setupHint3, LV_TEXT_ALIGN_CENTER, 0);
  // Plain ASCII on purpose - lv_font_montserrat_* only ships common Latin
  // glyphs by default, and an unsupported character (e.g. a Unicode bullet)
  // would just render as a blank tofu box.
  lv_label_set_text(setupHint3, "Swipe left/right: clock | up/down: remote setup");
  lv_obj_align(setupHint3, LV_ALIGN_BOTTOM_MID, 0, -20);

  // ===== MONITOR screen port =====
  // Renamed from "REMOTE MONITOR" - this screen has only ever been remote
  // access to the same setup page (see wifi_monitor_start() in
  // wifi_portal.cpp), not a status/telemetry monitor; the old name was a
  // leftover from this project's TiltDash origins (see main.cpp's header
  // comment) and didn't match what the user now calls it either.
  lv_obj_t* monitorTitle = lv_label_create(scr_monitor);
  lv_obj_set_style_text_color(monitorTitle, lv_color_white(), 0);
  lv_label_set_text(monitorTitle, "REMOTE SETUP");
  lv_obj_align(monitorTitle, LV_ALIGN_TOP_MID, 0, 20);

  monitor_status_lbl = lv_label_create(scr_monitor);
  lv_obj_set_style_text_color(monitor_status_lbl, ral7037(), 0);
  lv_obj_set_style_text_align(monitor_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(monitor_status_lbl, "Starting...");
  lv_obj_align(monitor_status_lbl, LV_ALIGN_CENTER, 0, -30);

  monitor_address_lbl = lv_label_create(scr_monitor);
  lv_obj_set_style_text_color(monitor_address_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_align(monitor_address_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(monitor_address_lbl, "---");
  lv_obj_align(monitor_address_lbl, LV_ALIGN_CENTER, 0, 20);

  lv_obj_t* monitorHint = lv_label_create(scr_monitor);
  lv_obj_set_style_text_color(monitorHint, ral7037(), 0);
  lv_obj_set_style_text_align(monitorHint, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(monitorHint, "Swipe left/right: clock | up/down: wifi setup");
  lv_obj_align(monitorHint, LV_ALIGN_BOTTOM_MID, 0, -20);

  // ===== UPDATE screen =====
  // Reached only by tapping clock_update_icon on CLOCK, not via swipe -
  // see the Screen enum comment. update_version_lbl/update_status_lbl get
  // their real text from update_screen_on_enter() and the button's tap
  // handler, not here.
  lv_obj_t* updateTitle = lv_label_create(scr_update);
  lv_obj_set_style_text_color(updateTitle, lv_color_white(), 0);
  lv_label_set_text(updateTitle, "FIRMWARE UPDATE");
  lv_obj_align(updateTitle, LV_ALIGN_TOP_MID, 0, 20);

  update_version_lbl = lv_label_create(scr_update);
  lv_obj_set_style_text_color(update_version_lbl, lv_color_white(), 0);
  lv_obj_set_style_text_align(update_version_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(update_version_lbl, "---");
  lv_obj_align(update_version_lbl, LV_ALIGN_CENTER, 0, -80);

  update_btn = lv_btn_create(scr_update);
  lv_obj_set_size(update_btn, 200, 70);
  lv_obj_set_style_radius(update_btn, 16, 0);
  lv_obj_align(update_btn, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_flag(update_btn, LV_OBJ_FLAG_GESTURE_BUBBLE);
  lv_obj_clear_flag(update_btn, LV_OBJ_FLAG_SCROLLABLE); // see the screens-loop comment above for why
  lv_obj_t* updateBtnLbl = lv_label_create(update_btn);
  lv_label_set_text(updateBtnLbl, "Update now");
  lv_obj_center(updateBtnLbl);
  lv_obj_add_event_cb(update_btn, [](lv_event_t*) {
    if (!update_is_available()) return;
    if (update_status_lbl) lv_label_set_text(update_status_lbl, "Downloading...");
    lv_timer_handler(); // paint the line above before the blocking call below freezes the UI
    bool ok = update_perform(); // blocks; on success this restarts the device and never returns
    if (update_status_lbl) {
      lv_label_set_text(update_status_lbl, ok ? "Done." : "Update failed - see serial log.");
    }
  }, LV_EVENT_CLICKED, nullptr);

  update_status_lbl = lv_label_create(scr_update);
  lv_obj_set_style_text_color(update_status_lbl, ral7037(), 0);
  lv_obj_set_style_text_align(update_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(update_status_lbl, "");
  lv_obj_align(update_status_lbl, LV_ALIGN_CENTER, 0, 60);

  lv_obj_t* updateHint = lv_label_create(scr_update);
  lv_obj_set_style_text_color(updateHint, ral7037(), 0);
  lv_obj_set_style_text_align(updateHint, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(updateHint, "Swipe: back to clock");
  lv_obj_align(updateHint, LV_ALIGN_BOTTOM_MID, 0, -20);

  lv_scr_load(scr_clock);

  wifi_portal_init();
  update_check_init();
  Serial.printf("[BOOT] Applying saved tz offset: %d min\n", wifi_portal_get_tz_offset_min());
  Serial.printf("[BOOT] Firmware version: %s\n", update_current_version());

  // scr_clock was lv_scr_load()'ed directly above (not via switch_screen()),
  // so its on-enter hook never ran - do it once explicitly now that
  // wifi_portal_init() has populated the entity list, to set the initial
  // name label/button state instead of leaving them at their placeholder
  // creation-time values until the first swipe.
  clock_screen_on_enter();

  if (!wifi_portal_has_credentials()) {
    // No saved network - go straight into setup mode instead of waiting
    // on WiFi/NTP (there's nothing to connect to anyway).
    switch_screen(SCR_SETUP);
  } else {
    wifi_begin_nonblocking();
  }

  Serial.println("[BOOT] Ready.");
}

void loop()
{
  lv_timer_handler();
  updateDisplayPower();


  if (wifi_portal_is_active()) {
    // WiFi configuration mode (AP + captive portal) - the normal WiFi/NTP
    // state machine is irrelevant here (and would conflict with AP mode).
    wifi_portal_loop();
  } else {
    wifi_ntp_update_state();
    // Remote monitor serving directly over an existing STA connection (no
    // AP involved, so it isn't covered by wifi_portal_loop() above).
    if (wifi_monitor_is_active()) wifi_monitor_service();
    ota_loop();
  }

  uint32_t now = millis();

  // Diagnostic: while on SETUP/MONITOR, log the actual live network state
  // every 500ms - added specifically to see what's happening moment-to-
  // moment during real-hardware debugging of the setup web page's
  // reachability, instead of only finding out via a failed browser/curl
  // request after the fact. Safe to remove once that's no longer needed.
  static uint32_t lastNetDiag = 0;
  if ((current_screen == SCR_SETUP || current_screen == SCR_MONITOR) && now - lastNetDiag >= 500) {
    lastNetDiag = now;
    Serial.printf(
      "[NETDIAG] screen=%d wifiStatus=%d ip=%s apActive=%d monitorActive=%d monitorOwnAp=%d heapFree=%u\n",
      (int)current_screen, (int)WiFi.status(), WiFi.localIP().toString().c_str(),
      (int)wifi_portal_is_active(), (int)wifi_monitor_is_active(), (int)wifi_monitor_using_own_ap(),
      (unsigned)ESP.getFreeHeap());
  }

  static uint32_t lastClock = 0;
  if (now - lastClock >= 1000) {
    lastClock = now;
    if (clock_wifi) {
      lv_obj_set_style_text_color(clock_wifi, g_wifiOk ? lv_color_white() : lv_color_hex(0xFF4040), 0);
    }

    if (clock_srv_icon) {
      lv_obj_set_style_text_color(clock_srv_icon,
        wifi_monitor_is_active() ? lv_color_hex(0x40C0FF) : lv_color_hex(0x444444), 0);
    }
    if (clock_ha_icon) {
      bool haOk = wifi_portal_has_ha_config() && g_haLastPollOk;
      lv_obj_set_style_text_color(clock_ha_icon,
        haOk ? lv_color_hex(0x40FF80) : lv_color_hex(0x444444), 0);
    }
    // Both hidden (opacity) AND made non-clickable when there's no update -
    // otherwise an invisible label would still silently swallow a tap in
    // the middle of CLOCK, which would be a confusing dead zone even
    // though switch_screen(SCR_UPDATE) itself is harmless with nothing to
    // install.
    bool updAvail = update_is_available();
    if (clock_update_icon) {
      lv_obj_set_style_opa(clock_update_icon, updAvail ? LV_OPA_COVER : LV_OPA_0, 0);
      if (updAvail) lv_obj_add_flag(clock_update_icon, LV_OBJ_FLAG_CLICKABLE);
      else lv_obj_clear_flag(clock_update_icon, LV_OBJ_FLAG_CLICKABLE);
    }
    if (clock_update_text) {
      lv_obj_set_style_opa(clock_update_text, updAvail ? LV_OPA_COVER : LV_OPA_0, 0);
      if (updAvail) lv_obj_add_flag(clock_update_text, LV_OBJ_FLAG_CLICKABLE);
      else lv_obj_clear_flag(clock_update_text, LV_OBJ_FLAG_CLICKABLE);
    }

    if (clock_battery) {
      float vbat = readBatteryVoltage();
      bool charging = isCharging(vbat);
      g_usbPresent = charging;
      const char* icon;
      if (charging) {
        // Charging: the voltage itself is elevated by charge current
        // right now, not a true state-of-charge reading, so a battery-
        // level icon would be actively misleading here - a charge symbol
        // says the right thing instead ("plugged in", not "how full").
        icon = LV_SYMBOL_CHARGE;
      } else {
        // Rough LiPo single-cell thresholds - not a real fuel gauge (no
        // current-integration/temperature compensation), just enough to
        // pick a sensible icon alongside the actual voltage number.
        icon = LV_SYMBOL_BATTERY_EMPTY;
        if      (vbat >= 4.05f) icon = LV_SYMBOL_BATTERY_FULL;
        else if (vbat >= 3.85f) icon = LV_SYMBOL_BATTERY_3;
        else if (vbat >= 3.65f) icon = LV_SYMBOL_BATTERY_2;
        else if (vbat >= 3.45f) icon = LV_SYMBOL_BATTERY_1;
      }
      char buf[24];
      snprintf(buf, sizeof(buf), "%s %.2fV", icon, vbat);
      lv_label_set_text(clock_battery, buf);

      Serial.printf("[POWER] vbat=%.2f usb=%d dispState=%d idleMs=%lu\n",
                    vbat, (int)g_usbPresent, (int)disp_power_state,
                    (unsigned long)(millis() - last_activity_ms));

      lv_color_t color = lv_color_white();
      if (!charging && vbat < BATTERY_CRITICAL_V) {
        // Blink red - same cadence as the offline WiFi icon above.
        color = ((now / 500) % 2) ? lv_color_hex(0xFF3030) : lv_color_white();
      } else if (!charging && vbat < BATTERY_LOW_WARN_V) {
        color = lv_color_hex(0xFFA030);
      }
      lv_obj_set_style_text_color(clock_battery, color, 0);

      // CPU throttling: full speed whenever USB is present (charging or
      // not - it's plugged into external power either way), throttled
      // down only when actually running off the battery alone.
      static bool cpuModeInitialized = false;
      static bool cpuOnBattery = false;
      if (!cpuModeInitialized || cpuOnBattery != !charging) {
        cpuOnBattery = !charging;
        cpuModeInitialized = true;
        // TEMP DISABLED for debugging the black-screen-on-USB-connect
        // report - this call is the prime suspect (it fires at exactly
        // the moment charging state flips), so it's logged but not
        // actually applied until that's ruled in or out.
        // setCpuFrequencyMhz(cpuOnBattery ? CPU_MHZ_BATTERY : CPU_MHZ_USB);
        Serial.printf("[POWER] (would) set CPU to %luMHz (%s) - disabled for debug\n",
                      (unsigned long)(cpuOnBattery ? CPU_MHZ_BATTERY : CPU_MHZ_USB),
                      cpuOnBattery ? "battery" : "USB");
      }
    }

    // Home Assistant light state poll - only while CLOCK is the visible
    // screen (that's the only place the button shows), and only every 5s:
    // it's a blocking HTTP call (see ha_light.cpp), no need to pay that
    // cost once a second or while the button isn't even on screen.
    static uint32_t lastHaPoll = 0;
    if (current_screen == SCR_CLOCK && wifi_portal_has_ha_config() && (now - lastHaPoll) >= 5000) {
      lastHaPoll = now;
      bool isOn = g_haLightOn;
      bool pollOk = ha_light_poll_state(wifi_portal_get_entity_id(g_currentEntityIdx), &isOn);
      g_haLastPollOk = pollOk;
      if (pollOk && isOn != g_haLightOn) {
        g_haLightOn = isOn;
        update_light_btn_style();
      }
    }

    if (g_wifiOk) {
      time_t epoch = time(nullptr);
      if (epoch > 8 * 3600 * 365) { // sanity check - SNTP has actually set the clock (not still 1970)
        struct tm t;
        localtime_r(&epoch, &t);
        char time_str[6], date_str[11];
        snprintf(time_str, sizeof(time_str), "%02d:%02d", t.tm_hour, t.tm_min);
        snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
        if (clock_time) lv_label_set_text(clock_time, time_str);
        if (clock_date) lv_label_set_text(clock_date, date_str);
      }
    }
  }

  delay(5);
}
