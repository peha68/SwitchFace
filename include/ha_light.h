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
