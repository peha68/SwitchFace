#pragma once

// Minimal Home Assistant REST client, shared across whichever entity is
// currently selected in the CLOCK screen's carousel - backs its
// tap-to-toggle button. Base URL/token come from wifi_portal's NVS-backed
// setup page (one shared HA instance, see wifi_portal_get_ha_url()/
// wifi_portal_get_ha_token()); entityId is passed in by the caller (main.cpp)
// since it changes with the currently-selected carousel entry - see
// wifi_portal_get_entity_id() in wifi_portal.h.

// Fire-and-forget: calls <url>/api/services/<domain>/toggle for entityId,
// where <domain> is the part of entityId before the '.' (e.g. "switch" for
// "switch.lazienka_lampa"). Blocking HTTP call (typically well under a
// second on a local network) - call it from the button's tap handler, not
// from a tight loop. No-op (returns false) if wifi_portal_has_ha_config()
// is false, entityId is empty, or WiFi isn't connected.
bool ha_light_toggle(const char* entityId);

// Polls entityId's current state. Returns true and sets *outIsOn if the
// request succeeded, false otherwise (leaves *outIsOn unchanged on
// failure, so callers can just keep showing the last known state).
// Blocking HTTP call - call it periodically (e.g. every few seconds), not
// every frame.
bool ha_light_poll_state(const char* entityId, bool* outIsOn);

// ====== Weather entities (read-only - no toggle/button) ======
// A "weather.*" entity (e.g. Home Assistant's built-in Weather
// integration) has a completely different shape than a switch/light: its
// `state` is a condition string ("cloudy", "rainy", ...), and the actual
// numbers live in `attributes` (temperature, wind_speed, etc.) rather than
// state itself. Kept as a separate, parallel API rather than folding into
// ha_light_poll_state() - the parsing and the data returned are unrelated
// to on/off.

// True if entityId's domain (the part before '.') is "weather" - used by
// main.cpp to decide whether to show the toggle button or a read-only
// display for the current carousel entry.
bool ha_light_is_weather(const char* entityId);

struct HaWeather {
    float temperature;         // degrees, in whatever unit HA is configured for
    char  temperatureUnit[8];  // e.g. "\xc2\xb0C" (UTF-8 for U+00B0 C) - HA's own attribute, not assumed
    char  condition[24];       // HA's raw condition string (e.g. "cloudy") - shown as-is, not translated/iconified
    float windSpeed;
    char  windSpeedUnit[12];   // e.g. "km/h"
};

// Blocking HTTP call, same pattern/caveats as ha_light_poll_state(). Only
// touches *out on success.
bool ha_light_poll_weather(const char* entityId, HaWeather* out);
