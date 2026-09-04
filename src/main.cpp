// SwitchFace - AMOLED touch clock + Home Assistant light switch, on a
// Waveshare ESP32-S3-Touch-AMOLED-1.64(-v2) (V1/CO5300 revision - see the
// display driver notes below).
//
// Grew out of a TiltDash (vehicle leveling) bring-up board for this same
// display, hence some of the lower-level driver/touch/power-management
// code below still reads like display-driver bring-up notes - all of
// that is display/touch/power hardware knowledge, not tilt-sensing logic,
// so it stayed. The tilt-sensing screens/IMU code did not - this project
// has none of that.
//
// STATUS: CLOCK and SETUP are ported and tested on real hardware. The
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

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh8601.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#define LV_CONF_INCLUDE_SIMPLE 1
#include "lv_conf.h"
#include <lvgl.h>

#include "esp_timer.h"
#include "FT3168.h"

static constexpr int LCD_HOR_RES = 280;
static constexpr int LCD_VER_RES = 456;
static constexpr int I2C_SDA_PIN = 47;
static constexpr int I2C_SCL_PIN = 48;

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
static constexpr uint8_t  BRIGHTNESS_FULL  = 0xD0;  // matches lcd_init_cmds' boot value
static constexpr uint8_t  BRIGHTNESS_DIM   = 0x20;
static constexpr uint32_t CPU_MHZ_USB      = 240;
static constexpr uint32_t CPU_MHZ_BATTERY  = 160; // LVGL still needs to stay responsive - 80MHz visibly lags

enum class DisplayPowerState { FULL, DIM, BLANK };
static DisplayPowerState disp_power_state = DisplayPowerState::FULL;
static uint32_t last_activity_ms = 0;
static esp_lcd_panel_io_handle_t g_panel_io = nullptr;
static esp_lcd_panel_handle_t g_panel = nullptr;
static bool g_usbPresent = false; // updated each battery-tick from isCharging(vbat)

// 0x51 = MIPI DCS "Write Display Brightness" - same register lcd_init_cmds
// sets once at boot; this just lets it be changed again at runtime.
static void setLcdBrightness(uint8_t level)
{
  if (!g_panel_io) {
    Serial.println("[POWER] setLcdBrightness: g_panel_io is NULL");
    return;
  }
  // lcd_init_cmds only ever turns on BCTRL (bit5) in the 0x53 Write_CTRL_
  // Display register - the DD (dimming, bit3) and BL (backlight block,
  // bit2) bits stay off. 0x51 alone reported ESP_OK on real hardware but
  // produced no visible change, which matches BL being required for the
  // brightness block to actually drive the panel, not just accept writes.
  uint8_t ctrl = 0x2C; // BCTRL | DD | BL
  esp_err_t ctrlErr = esp_lcd_panel_io_tx_param(g_panel_io, 0x53, &ctrl, 1);
  esp_err_t err = esp_lcd_panel_io_tx_param(g_panel_io, 0x51, &level, 1);
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
  setLcdBrightness(BRIGHTNESS_FULL);
  Serial.println("[POWER] display -> FULL (wake)");
  disp_power_state = DisplayPowerState::FULL;
}

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

static void wifi_ntp_update_state()
{
  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (!g_wifiOk) {
      g_wifiOk = true;
      Serial.printf("[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
    }
    if (!g_ntpConfigured) {
      g_ntpConfigured = true;
      configTime((long)wifi_portal_get_tz_offset_min() * 60, 0, "pool.ntp.org");
    }
  } else {
    if (g_wifiOk) {
      g_wifiOk = false;
      g_ntpConfigured = false;
      Serial.println("[WiFi] Disconnected.");
    }
    if (now - g_lastWifiTryMs >= WIFI_RETRY_MS) {
      Serial.println("[WiFi] Retry connect...");
      wifi_begin_nonblocking();
    }
  }
}

// ====== CLOCK screen ======
static lv_obj_t* clock_wifi    = nullptr;
static lv_obj_t* clock_offline = nullptr;
static lv_obj_t* clock_battery = nullptr;
static lv_obj_t* clock_time    = nullptr;
static lv_obj_t* clock_date    = nullptr;

// Home Assistant light-switch button - tap to toggle, color reflects the
// last known state polled from HA (gray = off/unknown, yellow = on).
static lv_obj_t* clock_light_btn  = nullptr;
static lv_obj_t* clock_light_icon = nullptr;
static bool g_haLightOn = false;

// Small persistent dev overlay at the bottom of CLOCK only - not part of
// the real screen, kept as a quick sanity check that touch is still
// alive while iterating on the rest of the port.
static lv_obj_t* lblTouch = nullptr;

// ====== Screens ======
enum Screen { SCR_CLOCK, SCR_SETUP, SCR_MONITOR };
static Screen     current_screen = SCR_CLOCK;
static lv_obj_t*  scr_clock   = nullptr;
static lv_obj_t*  scr_setup   = nullptr;
static lv_obj_t*  scr_monitor = nullptr;

// ====== SETUP screen ======
static lv_obj_t* setup_ssid_lbl = nullptr;
static lv_obj_t* setup_ip_lbl   = nullptr;

// ====== MONITOR screen ======
static lv_obj_t* monitor_status_lbl  = nullptr;
static lv_obj_t* monitor_address_lbl = nullptr;


// Forward declarations - switch_screen() calls these, but they're defined
// further down (near the rest of the SETUP/MONITOR logic).
static void setup_screen_on_enter();
static void setup_screen_on_leave();
static void monitor_screen_on_enter();
static void monitor_screen_on_leave();

static lv_obj_t* screen_obj(Screen s)
{
  switch (s) {
    case SCR_CLOCK:   return scr_clock;
    case SCR_SETUP:   return scr_setup;
    case SCR_MONITOR: return scr_monitor;
  }
  return scr_clock;
}

static void switch_screen(Screen s)
{
  if (current_screen == SCR_SETUP && s != SCR_SETUP) setup_screen_on_leave();
  if (current_screen == SCR_MONITOR && s != SCR_MONITOR) monitor_screen_on_leave();

  current_screen = s;
  lv_scr_load(screen_obj(s));

  if (s == SCR_SETUP) setup_screen_on_enter();
  if (s == SCR_MONITOR) monitor_screen_on_enter();
}

static void update_light_btn_style()
{
  if (!clock_light_btn) return;
  lv_obj_set_style_bg_color(clock_light_btn, g_haLightOn ? lv_color_hex(0xFFC107) : lv_color_hex(0x333333), 0);
  lv_obj_set_style_text_color(clock_light_icon, g_haLightOn ? lv_color_black() : lv_color_hex(0x888888), 0);
}

// Blocking HTTP round-trip (see ha_light.cpp) - runs synchronously inside
// this tap handler, so the UI briefly freezes for the call's duration
// (typically well under a second on a local network). Acceptable for an
// occasional tap; revisit with a background task if it ever feels janky.
static void do_ha_light_toggle()
{
  bool ok = ha_light_toggle();
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

static void gesture_event_cb(lv_event_t* e)
{
  lv_indev_t* indev = lv_indev_get_act();
  if (!indev) return;
  lv_dir_t dir = lv_indev_get_gesture_dir(indev);
  Serial.printf("[GESTURE] screen=%d dir=%d\n", (int)current_screen, (int)dir);

  switch (current_screen) {
    case SCR_CLOCK:
      if (dir == LV_DIR_LEFT) switch_screen(SCR_SETUP);
      else if (dir == LV_DIR_RIGHT) switch_screen(SCR_MONITOR);
      break;
    case SCR_SETUP:
    case SCR_MONITOR:
      // Any direction goes back to CLOCK - LEFT/RIGHT swipes keep coming
      // out classified as TOP/BOTTOM on this touch panel (see the
      // [GESTURE] log from testing), so requiring a specific direction
      // never worked reliably.
      if (dir != LV_DIR_NONE) switch_screen(SCR_CLOCK);
      break;
  }
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
  bool wasOwnAp = wifi_monitor_using_own_ap();
  wifi_monitor_stop();

  if (wasOwnAp && wifi_portal_has_credentials()) wifi_begin_nonblocking();
}

// ====== Touch (FT3168) ======
static FT3168 tp(I2C_SDA_PIN, I2C_SCL_PIN, -1, -1);

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
  lv_obj_t* screens[] = {scr_clock, scr_setup, scr_monitor};
  for (auto* scr : screens) {
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_add_event_cb(scr, gesture_event_cb, LV_EVENT_GESTURE, nullptr);
  }

  // ===== CLOCK screen =====
  clock_wifi = lv_label_create(scr_clock);
  lv_label_set_text(clock_wifi, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_color(clock_wifi, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_wifi, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_wifi, LV_ALIGN_TOP_LEFT, 8, 8);

  clock_offline = lv_label_create(scr_clock);
  lv_label_set_text(clock_offline, "OFFLINE");
  lv_obj_set_style_text_color(clock_offline, lv_color_hex(0xFF4040), 0);
  lv_obj_set_style_text_font(clock_offline, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_offline, LV_ALIGN_TOP_LEFT, 46, 8);

  clock_battery = lv_label_create(scr_clock);
  lv_label_set_text(clock_battery, LV_SYMBOL_BATTERY_EMPTY " --.--V");
  lv_obj_set_style_text_color(clock_battery, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_battery, &lv_font_montserrat_20, 0);
  lv_obj_align(clock_battery, LV_ALIGN_TOP_RIGHT, -8, 8);

  clock_time = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(clock_time, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_time, &lv_font_montserrat_48, 0);
  lv_label_set_text(clock_time, "--:--");
  lv_obj_align(clock_time, LV_ALIGN_CENTER, 0, -90);

  clock_date = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(clock_date, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_date, &lv_font_montserrat_22, 0);
  lv_label_set_text(clock_date, "----------");
  lv_obj_align(clock_date, LV_ALIGN_CENTER, 0, -30);

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
  lv_obj_add_flag(clock_light_btn, LV_OBJ_FLAG_GESTURE_BUBBLE);
  lv_obj_add_event_cb(clock_light_btn, ha_light_btn_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(clock_light_btn, gesture_event_cb, LV_EVENT_GESTURE, nullptr);

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
  lv_label_set_text(setupHint3, "Swipe: back to clock");
  lv_obj_align(setupHint3, LV_ALIGN_BOTTOM_MID, 0, -20);

  // ===== MONITOR screen port =====
  lv_obj_t* monitorTitle = lv_label_create(scr_monitor);
  lv_obj_set_style_text_color(monitorTitle, lv_color_white(), 0);
  lv_label_set_text(monitorTitle, "REMOTE MONITOR");
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
  lv_label_set_text(monitorHint, "Swipe: back to clock");
  lv_obj_align(monitorHint, LV_ALIGN_BOTTOM_MID, 0, -20);

  lv_scr_load(scr_clock);

  wifi_portal_init();
  Serial.printf("[BOOT] Applying saved tz offset: %d min\n", wifi_portal_get_tz_offset_min());

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
  }

  uint32_t now = millis();

  static uint32_t lastClock = 0;
  if (now - lastClock >= 1000) {
    lastClock = now;
    if (clock_wifi)    lv_obj_set_style_opa(clock_wifi, g_wifiOk ? LV_OPA_COVER : (((now / 500) % 2) ? LV_OPA_COVER : LV_OPA_0), 0);
    if (clock_offline) lv_obj_set_style_opa(clock_offline, g_wifiOk ? LV_OPA_0 : LV_OPA_COVER, 0);

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
      if (ha_light_poll_state(&isOn) && isOn != g_haLightOn) {
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
