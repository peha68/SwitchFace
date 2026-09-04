#pragma once

// Minimal Home Assistant REST client for a single switch/light entity -
// backs the tap-to-toggle button on the CLOCK screen. Config (URL, token,
// entity_id) comes from wifi_portal's NVS-backed setup page, not from
// here - see wifi_portal_get_ha_*() in wifi_portal.h.

// Fire-and-forget: calls <url>/api/services/<domain>/toggle for the
// configured entity_id, where <domain> is the part of entity_id before
// the '.' (e.g. "switch" for "switch.lazienka_lampa"). Blocking HTTP
// call (typically well under a second on a local network) - call it from
// the button's tap handler, not from a tight loop. No-op (returns false)
// if wifi_portal_has_ha_config() is false or WiFi isn't connected.
bool ha_light_toggle();

// Polls the entity's current state. Returns true and sets *outIsOn if the
// request succeeded, false otherwise (leaves *outIsOn unchanged on
// failure, so callers can just keep showing the last known state).
// Blocking HTTP call - call it periodically (e.g. every few seconds), not
// every frame.
bool ha_light_poll_state(bool* outIsOn);
