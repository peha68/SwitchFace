#pragma once

#include <Arduino.h>
#include <IPAddress.h>

// ====== WiFi configuration portal ======
// Instead of baking SSID/password into the firmware (wifi_credentials.h,
// requiring a recompile for every network change), credentials are stored
// in NVS (Preferences). When they're missing - or on the user's request -
// the device starts its own hotspot (AP) with a captive portal: a phone
// connects to that hotspot, gets a page listing nearby networks, and once
// saved the device restarts and connects normally as a client (STA).

// Loads saved WiFi credentials from NVS (if any). Call once in setup().
void wifi_portal_init();

// Whether an SSID is saved in NVS (i.e. there's something to try to
// connect to).
bool wifi_portal_has_credentials();

// Saved credentials - valid only after wifi_portal_init(), when
// wifi_portal_has_credentials() returns true.
const char* wifi_portal_get_ssid();
const char* wifi_portal_get_pass();

// Saved UTC offset in minutes (e.g. 120 for UTC+2 / CEST) - configurable
// from the same setup page as the WiFi credentials, since a fixed offset
// baked into firmware means a recompile every DST transition. Valid
// after wifi_portal_init(); defaults to +60 (UTC+1) if never set.
int wifi_portal_get_tz_offset_min();

// Owner-chosen device name (empty string if never set) - used for the AP
// SSID and mDNS hostname (see buildApSsid()/buildMdnsHostname() in
// wifi_portal.cpp) instead of the default MAC-suffixed "SwitchFace-XXXX" /
// "switchface-xxxx.local", and shown as the page title. Configurable from
// its own form on the setup page, independent of WiFi/timezone.
const char* wifi_portal_get_device_name();

// Home Assistant config (base URL, long-lived access token - one shared HA
// instance) plus a list of entities (name + entity_id), configurable from
// its own card on the setup page. wifi_portal_has_ha_config() is true only
// once url+token are non-empty AND at least one entity is configured.
bool wifi_portal_has_ha_config();
const char* wifi_portal_get_ha_url();
const char* wifi_portal_get_ha_token();

// Number of configured entities (0..MAX_HA_ENTITIES, see wifi_portal.cpp).
int wifi_portal_get_entity_count();
// Friendly name / entity_id for entity `index` (0-based, must be <
// wifi_portal_get_entity_count()). Name falls back to the entity_id itself
// if the owner left it blank on the setup page.
const char* wifi_portal_get_entity_name(int index);
const char* wifi_portal_get_entity_id(int index);

// Owner-assigned identity color for entity `index`, as a packed 0xRRGGBB
// (top byte 0). Applied to the CLOCK screen's button/name for that entity
// regardless of on/off state - added as a low-vision aid so which entity is
// on screen can be told apart by color alone, not just by reading the name.
// Defaults to a distinct color per slot (see DEFAULT_ENTITY_COLORS in
// wifi_portal.cpp) until the owner picks their own on the setup page.
uint32_t wifi_portal_get_entity_color(int index);

// Whether the entity's identity color is ALSO applied to the CLOCK
// button's background (the name label always shows it regardless of this
// setting). One global toggle, not per-entity - configurable from a
// checkbox in the Home Assistant card on the setup page. Defaults to true
// (the original behavior).
bool wifi_portal_get_color_button();

// OTA (firmware-over-WiFi) password - empty means unprotected. Configurable
// from its own card on the setup page; takes effect on next boot (saving it
// restarts the device, same as WiFi/timezone saves).
const char* wifi_portal_get_ota_password();

// Starts setup mode: AP + DNS (captive portal) + web server with the
// config form. Blocking for a couple of seconds (WiFi scan) - see the
// comment on scanNetworksIntoHtml() in wifi_portal.cpp. Safe to call
// repeatedly (no-op if the AP is already active).
void wifi_portal_start_ap();

// Stops the AP/portal (web server, DNS, hotspot). Call when leaving the
// setup screen if the user didn't save anything.
void wifi_portal_stop_ap();

// Whether AP/portal mode is currently active.
bool wifi_portal_is_active();

// Call on every loop() iteration while wifi_portal_is_active() == true.
void wifi_portal_loop();

// Hotspot name and IP address - for showing on the setup screen.
const char* wifi_portal_ap_ssid();
IPAddress wifi_portal_ap_ip();

// ====== Remote access ======
// Lets a phone reach the setup page (WiFi/timezone/name/Home Assistant
// config) without needing to physically re-enter AP mode - useful once
// the device is already on the home network and you just want to tweak a
// setting. Reuses the same WebServer/AP machinery as the setup portal
// (same tested mode-transition/memory handling): if already connected to
// a real network (STA), serves straight from that connection (no AP
// needed) and advertises mDNS; if not connected to anything, falls back
// to starting the device's own hotspot like the setup screen does.
// Deliberately only runs while the caller keeps it active (call
// wifi_monitor_stop() when leaving the screen) rather than continuously,
// to keep steady-state memory usage low - see the notes in
// wifi_portal_start_ap() about this board's tight RAM.
void wifi_monitor_start();
void wifi_monitor_stop();
bool wifi_monitor_is_active();

// Call on every loop() iteration while wifi_monitor_is_active() is true
// AND wifi_portal_is_active() is false (the AP case is already serviced
// by wifi_portal_loop()).
void wifi_monitor_service();

// True once wifi_monitor_start() had to fall back to the device's own
// hotspot (no STA connection available) rather than using an existing
// network.
bool wifi_monitor_using_own_ap();

// Reachable address for the phone to open, and hostname for the mDNS
// case (only meaningful/reliable when NOT using the fallback AP - mDNS
// resolution across a phone's own captive-AP connection is inconsistent
// across OSes, so the IP is always shown too regardless).
IPAddress wifi_monitor_ip();
const char* wifi_monitor_hostname();
