#pragma once

#include <stdint.h>

// Over-the-air firmware updates (ArduinoOTA - ships with the arduino-esp32
// core, no extra lib_deps). partitions.csv already has otadata/app0(ota_0)/
// app1(ota_1), so no partition changes were needed to support this.

// Call once from setup(), after wifi_portal_init() (needs
// wifi_portal_get_device_name()/wifi_portal_get_ota_password()). Registers
// ArduinoOTA's start/progress/end/error callbacks; safe to call even before
// WiFi is connected - ArduinoOTA itself only actually listens once STA is up.
void ota_init();

// Call every loop() iteration whenever wifi_portal_is_active() is false (no
// point servicing OTA while the device is its own isolated AP with no route
// out - and ArduinoOTA.handle() is a harmless no-op without a WiFi
// connection anyway, so this is a minor optimization, not a correctness
// requirement).
void ota_loop();

// True while a firmware transfer is actively being written - the main loop
// can use this to show a "don't unplug" message, since the screen will
// visibly stutter during the write (ArduinoOTA's progress callback runs
// synchronously in a blocking loop - inherent to the library, not a bug).
bool ota_in_progress();

// Valid only while ota_in_progress() is true.
uint8_t ota_progress_percent();
