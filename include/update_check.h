#pragma once

#include <stdint.h>

// Self-update: checks GitHub Releases for a newer firmware.bin than the
// one currently running, and can download+flash it. FIRMWARE_VERSION
// comes from git (see scripts/get_version.py) - never hardcode it here.
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "unknown"
#endif

// Call once from setup(). No network call yet.
void update_check_init();

// Blocking HTTPS call to the GitHub Releases API (cert-validated - see
// update_check.cpp for the embedded root CA and why only this call, not
// the firmware download itself, is pinned). Updates the state read by
// update_is_available()/update_latest_version(). Safe to call repeatedly
// (e.g. from both the on-device UPDATE screen and the web page's "Check
// now" button) - each call just re-does the check. Returns false on any
// network/parse failure (state is left as whatever it was before).
bool update_check_now();

// True once update_check_now() has found a release tag different from
// FIRMWARE_VERSION. Doesn't distinguish "newer" from "older/different" -
// deliberately simple (see update_check.cpp) - a human decides whether to
// actually apply it.
bool update_is_available();

// Valid only after a successful update_check_now().
const char* update_latest_version();
// Always valid - this build's own version (FIRMWARE_VERSION).
const char* update_current_version();

// Blocking: downloads the asset found by the last successful
// update_check_now(), verifies its SHA256 against the one published in
// the release notes, and only then flashes + calls ESP.restart(). Returns
// false (current firmware keeps running untouched) on any failure -
// network error, size mismatch, or hash mismatch. Call
// update_check_now() again first if you want to be sure you're not
// re-flashing a stale/already-known release.
bool update_perform();

// True while update_perform() is actively downloading/writing - for the
// UI to show progress instead of looking hung (the screen will stutter
// during the actual flash write, same caveat as the ArduinoOTA path in
// ota.h).
bool update_in_progress();
uint8_t update_progress_percent();
