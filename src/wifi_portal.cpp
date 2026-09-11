// WiFi setup portal + remote config access + Home Assistant light-switch
// config storage. No pin/resolution dependencies at all (pure WiFi/HTTP/
// NVS logic).

#include "wifi_portal.h"

#include <ctype.h>
#include <math.h>
#include <string.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

// ====== NVS ======
static Preferences prefsWifi;
static constexpr const char* NVS_NAMESPACE = "wifinet";

static String g_ssid;
static String g_pass;
static int    g_tzOffsetMin = 60; // default UTC+1, overridden by NVS if set
static String g_deviceName;       // owner-chosen name, empty = use the MAC-suffixed default

// Home Assistant integration (CLOCK screen tap-to-toggle, now a carousel of
// entities - see ha_light.cpp for the actual REST calls). URL/token are one
// shared HA instance; entities are a small fixed-size list. Stored here
// alongside the other setup-page fields so it goes through the same
// NVS/web-form machinery instead of being hardcoded (the token especially
// should never end up committed to the repo).
static String g_haUrl;
static String g_haToken;

// Deliberately a flat NVS string ("name|entity_id" per line, '\n'-joined),
// not per-index keys and not JSON - matches this project's explicit
// no-ArduinoJson stance (see ha_light.cpp's header comment). Capped at a
// small fixed count: this is a swipe carousel on a small touchscreen, not a
// dashboard - MAX_HA_ENTITIES can be bumped later if that turns out too low.
static constexpr int MAX_HA_ENTITIES = 6;
// colorHex is always 6 hex digits, no '#' (the '#' is added/stripped at the
// HTML <input type=color> boundary only - see handleRoot()/handleSaveHa()).
struct HaEntity { String name; String entityId; String colorHex; };
static HaEntity g_haEntities[MAX_HA_ENTITIES];
static int      g_haEntityCount = 0;

// Distinct default identity color per slot (see wifi_portal_get_entity_
// color() in wifi_portal.h) - picked so a freshly-configured entity is
// already visually distinguishable before the owner ever touches the color
// picker, added as a low-vision aid (told apart by color, not by reading
// the name). Cycles if there are ever more entities than colors here.
static const char* const DEFAULT_ENTITY_COLORS[] = {
    "FFC107", // amber
    "40C4FF", // cyan
    "FF4081", // pink
    "69F0AE", // green
    "FF6E40", // deep orange
    "B388FF", // purple
};
static constexpr int DEFAULT_ENTITY_COLORS_COUNT = sizeof(DEFAULT_ENTITY_COLORS) / sizeof(DEFAULT_ENTITY_COLORS[0]);

// OTA (firmware-over-WiFi) password - see ota.cpp. Empty = unprotected.
static String g_otaPassword;

// Curated list of real-world UTC offsets (minutes) for the setup page's
// timezone <select> - deliberately not every 15-minute step from -12:00
// to +14:00, since most of those don't correspond to an actual timezone.
struct TzOption { int16_t minutes; const char* label; };
static const TzOption TZ_OPTIONS[] = {
    { -720, "UTC-12:00" },
    { -660, "UTC-11:00" },
    { -600, "UTC-10:00 (Hawaii)" },
    { -540, "UTC-9:00 (Alaska)" },
    { -480, "UTC-8:00 (US Pacific)" },
    { -420, "UTC-7:00 (US Mountain)" },
    { -360, "UTC-6:00 (US Central)" },
    { -300, "UTC-5:00 (US Eastern)" },
    { -240, "UTC-4:00 (Atlantic)" },
    { -210, "UTC-3:30 (Newfoundland)" },
    { -180, "UTC-3:00 (Brazil/Argentina)" },
    { -120, "UTC-2:00" },
    {  -60, "UTC-1:00" },
    {    0, "UTC+0:00 (London winter)" },
    {   60, "UTC+1:00 (CET - e.g. Poland winter)" },
    {  120, "UTC+2:00 (CEST - e.g. Poland summer, EET)" },
    {  180, "UTC+3:00 (Moscow)" },
    {  210, "UTC+3:30 (Iran)" },
    {  240, "UTC+4:00" },
    {  270, "UTC+4:30 (Afghanistan)" },
    {  300, "UTC+5:00" },
    {  330, "UTC+5:30 (India)" },
    {  345, "UTC+5:45 (Nepal)" },
    {  360, "UTC+6:00" },
    {  390, "UTC+6:30 (Myanmar)" },
    {  420, "UTC+7:00" },
    {  480, "UTC+8:00 (China)" },
    {  540, "UTC+9:00 (Japan/Korea)" },
    {  570, "UTC+9:30 (Australia Central)" },
    {  600, "UTC+10:00 (Australia East)" },
    {  630, "UTC+10:30 (Lord Howe)" },
    {  660, "UTC+11:00" },
    {  720, "UTC+12:00 (New Zealand)" },
    {  765, "UTC+12:45 (Chatham)" },
    {  780, "UTC+13:00" },
    {  840, "UTC+14:00 (Kiribati)" },
};
static constexpr size_t TZ_OPTIONS_COUNT = sizeof(TZ_OPTIONS) / sizeof(TZ_OPTIONS[0]);

// ====== AP / portal ======
static WebServer   server(80);
static DNSServer   dnsServer;
static bool        g_apActive = false;
static String      g_apSsid;
static IPAddress   g_apIp;
static const uint32_t DNS_PORT = 53;
static bool        g_httpHandlersRegistered = false;

// ====== Remote access ======
static bool   g_monitorServing   = false; // server.begin() called directly on an existing STA connection (no AP)
static bool   g_monitorUsingOwnAp = false;
static String g_mdnsHostname;
static bool   g_mdnsRunning = false;

// ====== Helpers ======

static String defaultDeviceTag()
{
    // MAC-suffixed fallback identity, used for both the AP SSID and mDNS
    // hostname whenever the owner hasn't picked a name of their own -
    // guarantees multiple units nearby don't collide by default.
    uint64_t mac = ESP.getEfuseMac();
    char buf[16];
    snprintf(buf, sizeof(buf), "%04X", (unsigned)(mac & 0xFFFF));
    return String(buf);
}

// DNS labels (mDNS hostnames) only allow letters/digits/hyphens, no
// leading/trailing/duplicate hyphens - anything else in the owner's
// chosen name gets folded into a single hyphen so "My Camper!" becomes
// "my-camper" instead of silently breaking <name>.local resolution.
static String sanitizeForHostname(const String& raw)
{
    String out;
    out.reserve(raw.length());
    bool lastWasHyphen = true; // true so we never start with a hyphen
    for (size_t i = 0; i < raw.length(); i++) {
        char c = raw[i];
        if (isalnum((unsigned char)c)) {
            out += (char)tolower((unsigned char)c);
            lastWasHyphen = false;
        } else if (!lastWasHyphen) {
            out += '-';
            lastWasHyphen = true;
        }
    }
    while (out.length() && out[out.length() - 1] == '-') out.remove(out.length() - 1);
    return out;
}

static String buildApSsid()
{
    // WiFi SSIDs tolerate spaces/punctuation fine, so the owner's chosen
    // name is used as-is (just length-capped - SSIDs are limited to 32
    // bytes).
    if (g_deviceName.length() > 0) {
        String ssid = g_deviceName;
        if (ssid.length() > 32) ssid = ssid.substring(0, 32);
        return ssid;
    }
    return "SwitchFace-" + defaultDeviceTag();
}

static String buildMdnsHostname()
{
    if (g_deviceName.length() > 0) {
        String host = sanitizeForHostname(g_deviceName);
        if (host.length() > 0) return host; // e.g. all-punctuation input falls through to the default below
    }
    String tag = defaultDeviceTag();
    tag.toLowerCase(); // toLowerCase() mutates in place (returns void), can't be chained
    return "switchface-" + tag;
}

// Used wherever the owner-chosen device name (arbitrary user input) gets
// embedded into HTML - the setup page title and the name
// input's value attribute.
static String htmlEscape(const String& s)
{
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        switch (c) {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&#39;";  break;
            default:   out += c;        break;
        }
    }
    return out;
}

static String jsonEscape(const String& s)
{
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if ((uint8_t)c >= 0x20) out += c; // drop control chars
    }
    return out;
}

static String defaultColorForIndex(int index)
{
    return DEFAULT_ENTITY_COLORS[index % DEFAULT_ENTITY_COLORS_COUNT];
}

static bool isValidHexColor(const String& s)
{
    if (s.length() != 6) return false;
    for (size_t i = 0; i < 6; i++) {
        if (!isxdigit((unsigned char)s[i])) return false;
    }
    return true;
}

// Serializes g_haEntities[0..g_haEntityCount) into the flat NVS string
// format described above g_haEntities' declaration.
static String serializeEntities()
{
    String out;
    for (int i = 0; i < g_haEntityCount; i++) {
        if (i > 0) out += '\n';
        out += g_haEntities[i].name;
        out += '|';
        out += g_haEntities[i].entityId;
        out += '|';
        out += g_haEntities[i].colorHex;
    }
    return out;
}

// Inverse of serializeEntities() - repopulates g_haEntities/g_haEntityCount
// from a saved NVS string. Tolerant of a missing 2nd '|' (colorHex just
// falls back to this slot's default, e.g. entities saved before the color
// feature existed) and of a missing 1st '|' (whole line treated as the
// entity_id, name falls back to it) - a corrupt/hand-edited/older-format
// NVS value shouldn't crash parsing either way.
static void deserializeEntities(const String& raw)
{
    g_haEntityCount = 0;
    int lineStart = 0;
    while (lineStart <= (int)raw.length() && g_haEntityCount < MAX_HA_ENTITIES) {
        int lineEnd = raw.indexOf('\n', lineStart);
        if (lineEnd < 0) lineEnd = raw.length();
        if (lineEnd > lineStart) {
            String line = raw.substring(lineStart, lineEnd);
            int sep1 = line.indexOf('|');
            int sep2 = sep1 >= 0 ? line.indexOf('|', sep1 + 1) : -1;
            HaEntity& e = g_haEntities[g_haEntityCount];
            if (sep1 >= 0) {
                e.name = line.substring(0, sep1);
                e.entityId = sep2 >= 0 ? line.substring(sep1 + 1, sep2) : line.substring(sep1 + 1);
            } else {
                e.name = line;
                e.entityId = line;
            }
            String color = sep2 >= 0 ? line.substring(sep2 + 1) : String();
            e.colorHex = isValidHexColor(color) ? color : defaultColorForIndex(g_haEntityCount);
            if (e.entityId.length() > 0) g_haEntityCount++;
        }
        if (lineEnd >= (int)raw.length()) break;
        lineStart = lineEnd + 1;
    }
}

// Handles GET /scan: runs a fresh scan and returns it as JSON. Called
// on demand from the page's own JS (see handleRoot()) instead of once
// up front, so it can be retried without restarting the device.
//
// The radio sits in AP-only mode at rest (see wifi_portal_start_ap()) to
// keep steady-state memory usage as low as possible - a full-time
// WIFI_AP_STA was enough to starve lwIP on this board and
// make the portal page itself unreachable; kept the same way here even
// though this board has PSRAM headroom, since it's a proven-safe pattern.
// STA is added back just for the duration of this one scan and dropped
// again right after; the AP bit is never touched, so its beacon/netif
// isn't disturbed.
static void handleScan()
{
    Serial.printf("[WIFI-PORTAL] /scan requested (heap free=%u, mode=%d, status=%d)\n",
                  (unsigned)ESP.getFreeHeap(), (int)WiFi.getMode(), (int)WiFi.status());

    WiFi.mode(WIFI_AP_STA);
    delay(300); // give the STA side (freshly added on top of the AP) time to settle
    WiFi.scanDelete();

    int n = WiFi.scanNetworks(false /* async */, true /* show_hidden */,
                               false /* passive */, 500 /* max_ms_per_chan */,
                               0 /* all channels */);
    Serial.printf("[WIFI-PORTAL] scanNetworks() returned %d, mode=%d, status=%d\n",
                  n, (int)WiFi.getMode(), (int)WiFi.status());

    for (int attempt = 0; n <= 0 && attempt < 2; attempt++) {
        delay(500);
        n = WiFi.scanNetworks(false, true, false, 500, 0);
        Serial.printf("[WIFI-PORTAL] scanNetworks() retry %d returned %d\n", attempt + 1, n);
    }
    if (n < 0) n = 0; // report "no networks" rather than emit malformed JSON

    String json;
    json.reserve(64 + (size_t)n * 40);
    json += "[";
    bool first = true;
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue; // skip hidden SSIDs in the list
        if (!first) json += ",";
        first = false;
        json += "{\"ssid\":\"" + jsonEscape(ssid) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    json += "]";
    Serial.printf("[WIFI-PORTAL] /scan responding with %d entries\n", n);

    WiFi.scanDelete();
    WiFi.mode(WIFI_AP); // back to the low-memory steady state

    // "Connection: close" on every response, here and below - this
    // WebServer implementation doesn't robustly juggle a kept-alive
    // connection across requests on this board, and without this, POST
    // form submissions (timezone/HA config/etc) started silently hanging
    // client-side after the page had been open and used for a while
    // (confirmed on real hardware: worked right after boot, degraded with
    // use) - not tied to any specific handler, since even an untouched one
    // (timezone save) showed the same symptom, pointing at a shared
    // connection-handling issue rather than a per-handler bug. Forcing a
    // fresh TCP connection per request avoids whatever that stale-
    // connection state was.
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", json);
}

static void handleRoot()
{
    Serial.printf("[WIFI-PORTAL] GET %s from %s (heap free=%u)\n",
                  server.uri().c_str(), server.client().remoteIP().toString().c_str(),
                  (unsigned)ESP.getFreeHeap());

    String displayName = g_deviceName.length() > 0 ? htmlEscape(g_deviceName) : "SwitchFace";

    String page;
    page.reserve(7800); // bumped for the per-entity name/id/color row pairs + OTA card

    page += "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<title>" + displayName + " - Setup</title>"
            "<style>"
            "*{box-sizing:border-box}"
            "body{font-family:-apple-system,'Segoe UI',Roboto,sans-serif;background:#111;color:#eee;"
            "padding:24px 16px;max-width:420px;margin:auto;text-align:center}"
            "header{margin-bottom:24px}"
            "h1{font-size:19px;font-weight:600;margin:0;letter-spacing:.3px}"
            ".sub{margin-top:6px;font-size:12px;color:#7b7d7d;text-transform:uppercase;letter-spacing:1px}"
            ".card{background:#1c1c1c;border:1px solid #2a2a2a;border-radius:14px;"
            "padding:18px 16px;box-shadow:0 4px 14px rgba(0,0,0,.35);margin-bottom:16px;text-align:left}"
            ".card h2{font-size:13px;font-weight:600;color:#7b7d7d;text-transform:uppercase;"
            "letter-spacing:1px;margin:0 0 14px}"
            "label{display:block;margin-top:14px;font-size:13px;color:#999}"
            "label:first-of-type{margin-top:0}"
            "select,input{width:100%;padding:11px;margin-top:5px;"
            "background:#111;color:#eee;border:1px solid #333;border-radius:8px;font-size:15px}"
            "select:focus,input:focus{outline:none;border-color:#7b7d7d}"
            "input[type=color]{width:56px;padding:4px;height:40px}"
            "button{width:100%;padding:13px;margin-top:16px;background:#7b7d7d;color:#111;"
            "border:none;border-radius:8px;font-size:15px;font-weight:600}"
            "button.secondary{background:transparent;color:#ccc;border:1px solid #333;margin-top:8px}"
            "</style></head><body>"
            "<header><h1>" + displayName + "</h1><div class='sub'>Setup</div></header>"
            "<div class='card'>"
            "<h2>WiFi network</h2>"
            "<form action='/save' method='POST'>"
            "<label>Detected networks</label>"
            "<select id='ssid_scan' onchange=\"document.getElementById('ssid').value=this.value\">"
            "<option value=''>-- scanning... --</option>"
            "</select>"
            "<button type='button' class='secondary' onclick='scanNow()'>Rescan</button>"
            "<label>SSID (or type manually, e.g. hidden network)</label>"
            "<input type='text' name='ssid' id='ssid' maxlength='63'>"
            "<label>Password</label>"
            "<input type='password' name='pass' maxlength='63'>"
            "<button type='submit'>Save and connect</button>"
            "</form>"
            "</div>"
            "<div class='card'>"
            "<h2>Timezone</h2>"
            "<form action='/save_tz' method='POST'>"
            "<label>Independent of WiFi - saving this does not require a network</label>"
            "<select name='tz'>";
    for (size_t i = 0; i < TZ_OPTIONS_COUNT; i++) {
        page += "<option value='";
        page += String(TZ_OPTIONS[i].minutes);
        page += "'";
        if (TZ_OPTIONS[i].minutes == g_tzOffsetMin) page += " selected";
        page += ">";
        page += TZ_OPTIONS[i].label;
        page += "</option>";
    }
    page += "</select>"
            "<button type='submit'>Save timezone</button>"
            "</form>"
            "</div>"
            "<div class='card'>"
            "<h2>Device name</h2>"
            "<form action='/save_name' method='POST'>"
            "<label>Used for the WiFi hotspot name and the .local address - "
            "keep it unique if you have more than one SwitchFace</label>"
            "<input type='text' name='name' maxlength='32' placeholder='e.g. Bathroom' value='"
            + htmlEscape(g_deviceName) + "'>"
            "<button type='submit'>Save name</button>"
            "</form>"
            "</div>"
            "<div class='card'>"
            "<h2>Home Assistant</h2>"
            "<form action='/save_ha' method='POST'>"
            "<label>Base URL (e.g. http://192.168.1.50:8123)</label>"
            "<input type='text' name='ha_url' maxlength='64' value='"
            + htmlEscape(g_haUrl) + "'>"
            "<label>Long-Lived Access Token</label>"
            "<input type='password' name='ha_token' maxlength='200' value='"
            + htmlEscape(g_haToken) + "'>";
    for (int i = 0; i < MAX_HA_ENTITIES; i++) {
        String name     = i < g_haEntityCount ? g_haEntities[i].name     : String();
        String entityId = i < g_haEntityCount ? g_haEntities[i].entityId : String();
        String color    = i < g_haEntityCount ? g_haEntities[i].colorHex : defaultColorForIndex(i);
        page += "<label>Entity " + String(i + 1) + " name (e.g. Bathroom light)</label>"
                "<div style='display:flex;gap:8px;align-items:flex-start'>"
                "<input style='flex:1' type='text' name='entity_name_" + String(i) + "' maxlength='32' value='"
                + htmlEscape(name) + "'>"
                // Color picker - a low-vision aid: each entity's CLOCK-screen
                // button/name uses this color regardless of on/off state, so
                // entities can be told apart by color alone, not just by
                // reading the (small) name. Defaults to a distinct built-in
                // color per slot (defaultColorForIndex()) until customized.
                "<input type='color' name='entity_color_" + String(i) + "' value='#" + color + "'>"
                "</div>"
                "<label>Entity " + String(i + 1) + " ID (e.g. switch.lazienka_lampa)</label>"
                "<input type='text' name='entity_id_" + String(i) + "' maxlength='64' value='"
                + htmlEscape(entityId) + "'>";
    }
    page += "<button type='submit'>Save Home Assistant config</button>"
            "</form>"
            "</div>"
            "<div class='card'>"
            "<h2>Firmware update (OTA)</h2>"
            "<form action='/save_ota' method='POST'>"
            "<label>Password (leave blank for no password - only do this on a "
            "trusted network)</label>"
            "<input type='password' name='ota_pass' maxlength='64' value='"
            + htmlEscape(g_otaPassword) + "'>"
            "<button type='submit'>Save OTA password</button>"
            "</form>"
            "</div>"
            "<script>"
            "function scanNow(){"
            "var sel=document.getElementById('ssid_scan');"
            "sel.innerHTML=\"<option value=''>-- scanning... --</option>\";"
            "fetch('/scan').then(function(r){return r.json();}).then(function(list){"
            "sel.innerHTML=\"<option value=''>-- choose (or type below) --</option>\";"
            "list.forEach(function(n){"
            "var o=document.createElement('option');"
            "o.value=n.ssid;o.textContent=n.ssid+' ('+n.rssi+' dBm)';"
            "sel.appendChild(o);"
            "});"
            "if(list.length===0){"
            "var o=document.createElement('option');"
            "o.value='';o.textContent='-- none found, tap Rescan --';"
            "sel.appendChild(o);"
            "}"
            "}).catch(function(){"
            "sel.innerHTML=\"<option value=''>-- scan failed, tap Rescan --</option>\";"
            "});"
            "}"
            "scanNow();"
            "</script>"
            "</body></html>";

    server.sendHeader("Connection", "close");
    server.send(200, "text/html", page);
}

static void handleSave()
{
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");

    if (ssid.length() == 0) {
        server.sendHeader("Connection", "close");
        server.send(400, "text/plain", "Missing SSID.");
        return;
    }

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putString("ssid", ssid);
    prefsWifi.putString("pass", pass);
    prefsWifi.end();

    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Restarting and connecting to the network...</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved SSID='%s'. Restarting...\n", ssid.c_str());
    delay(1200);
    ESP.restart();
}

static void handleSaveTz()
{
    int tzMin = server.arg("tz").toInt();

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putInt("tzmin", tzMin);
    prefsWifi.end();
    g_tzOffsetMin = tzMin;

    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Restarting...</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved tz=%d min (raw arg='%s'). Restarting...\n",
                  tzMin, server.arg("tz").c_str());
    delay(1200);
    ESP.restart();
}

static void handleSaveName()
{
    String name = server.arg("name");
    name.trim();
    if (name.length() > 32) name = name.substring(0, 32);

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putString("devname", name);
    prefsWifi.end();
    g_deviceName = name;

    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Takes effect next time you open the setup or monitor screen.</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved device name='%s'\n", name.c_str());
}

static void handleSaveHa()
{
    Serial.printf("[WIFI-PORTAL] POST /save_ha received from %s (heap free=%u, args=%d)\n",
                  server.client().remoteIP().toString().c_str(),
                  (unsigned)ESP.getFreeHeap(), server.args());
    String url   = server.arg("ha_url");
    String token = server.arg("ha_token");
    url.trim(); token.trim();
    while (url.endsWith("/")) url.remove(url.length() - 1); // normalize away a trailing slash before we start concatenating paths onto it

    HaEntity parsed[MAX_HA_ENTITIES];
    int parsedCount = 0;
    for (int i = 0; i < MAX_HA_ENTITIES; i++) {
        String name = server.arg("entity_name_" + String(i));
        String id   = server.arg("entity_id_" + String(i));
        name.trim(); id.trim();
        if (id.length() == 0) continue; // empty row - skip, not an error

        // '|' and '\n' are the NVS serialization delimiters (see
        // serializeEntities()) - strip them from the name so a saved value
        // can never be misparsed back out. entity_id itself can't contain
        // either (HA entity_ids are alnum/underscore/dot), so it's left as-is.
        String cleanName;
        cleanName.reserve(name.length());
        for (size_t c = 0; c < name.length(); c++) {
            char ch = name[c];
            if (ch != '|' && ch != '\n' && ch != '\r') cleanName += ch;
        }
        if (cleanName.length() == 0) cleanName = id; // default to the entity_id if left blank

        // <input type=color> always sends "#rrggbb" - strip the '#' and
        // fall back to this slot's default if it's ever missing/malformed
        // (e.g. a request that didn't come from this exact form).
        String color = server.arg("entity_color_" + String(i));
        color.trim();
        if (color.startsWith("#")) color.remove(0, 1);
        if (!isValidHexColor(color)) color = defaultColorForIndex(parsedCount);

        parsed[parsedCount].name = cleanName;
        parsed[parsedCount].entityId = id;
        parsed[parsedCount].colorHex = color;
        parsedCount++;
    }

    for (int i = 0; i < parsedCount; i++) g_haEntities[i] = parsed[i];
    g_haEntityCount = parsedCount;
    String entitiesRaw = serializeEntities();

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putString("ha_url", url);
    prefsWifi.putString("ha_token", token);
    prefsWifi.putString("ha_entities", entitiesRaw);
    prefsWifi.end();
    g_haUrl = url;
    g_haToken = token;

    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Used by the entity carousel on the CLOCK screen.</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved HA config: url='%s' entities=%d (token hidden)\n",
                  url.c_str(), g_haEntityCount);
}

static void handleSaveOta()
{
    String pass = server.arg("ota_pass");

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putString("ota_pass", pass);
    prefsWifi.end();
    g_otaPassword = pass;

    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Restarting...</p>"
        "</body></html>");

    Serial.println("[WIFI-PORTAL] Saved OTA password. Restarting...");
    delay(1200);
    ESP.restart();
}

// Browsers silently GET /favicon.ico on every page load - without this,
// that fell through to onNotFound() -> handleRoot(), serving the ENTIRE
// ~7KB setup page a second time for nothing. WebServer (this library) only
// ever services one client connection at a time, with up to 5s to wait for
// data plus 5s to wait for the client to close (HTTP_MAX_DATA_WAIT/
// HTTP_MAX_CLOSE_WAIT in WebServer.h) - so that wasted favicon request
// could occupy the server's only connection slot for several seconds right
// when a real request (e.g. the entity-save POST) was trying to get in.
// Confirmed on real hardware as a major contributor to POST saves silently
// hanging - a tiny, instant 204 here closes that window.
static void handleFavicon()
{
    server.sendHeader("Connection", "close");
    server.send(204);
}

static void ensureHttpHandlersRegistered()
{
    if (g_httpHandlersRegistered) return;
    server.on("/", HTTP_GET, handleRoot);
    server.on("/favicon.ico", HTTP_GET, handleFavicon);
    server.on("/scan", HTTP_GET, handleScan);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/save_tz", HTTP_POST, handleSaveTz);
    server.on("/save_name", HTTP_POST, handleSaveName);
    server.on("/save_ha", HTTP_POST, handleSaveHa);
    server.on("/save_ota", HTTP_POST, handleSaveOta);
    server.onNotFound(handleRoot);
    g_httpHandlersRegistered = true;
}

// ====== API ======

void wifi_portal_init()
{
    prefsWifi.begin(NVS_NAMESPACE, true);
    g_ssid = prefsWifi.getString("ssid", "");
    g_pass = prefsWifi.getString("pass", "");
    g_tzOffsetMin = prefsWifi.getInt("tzmin", 60);
    g_deviceName = prefsWifi.getString("devname", "");
    g_haUrl       = prefsWifi.getString("ha_url", "");
    g_haToken     = prefsWifi.getString("ha_token", "");
    deserializeEntities(prefsWifi.getString("ha_entities", ""));
    g_otaPassword = prefsWifi.getString("ota_pass", "");
    prefsWifi.end();
}

bool wifi_portal_has_ha_config()
{
    return g_haUrl.length() > 0 && g_haToken.length() > 0 && g_haEntityCount > 0;
}
const char* wifi_portal_get_ha_url()    { return g_haUrl.c_str(); }
const char* wifi_portal_get_ha_token()  { return g_haToken.c_str(); }

int wifi_portal_get_entity_count() { return g_haEntityCount; }

const char* wifi_portal_get_entity_name(int index)
{
    if (index < 0 || index >= g_haEntityCount) return "";
    return g_haEntities[index].name.c_str();
}

const char* wifi_portal_get_entity_id(int index)
{
    if (index < 0 || index >= g_haEntityCount) return "";
    return g_haEntities[index].entityId.c_str();
}

uint32_t wifi_portal_get_entity_color(int index)
{
    if (index < 0 || index >= g_haEntityCount) return 0xFFFFFF;
    return (uint32_t)strtoul(g_haEntities[index].colorHex.c_str(), nullptr, 16);
}

const char* wifi_portal_get_ota_password() { return g_otaPassword.c_str(); }

const char* wifi_portal_get_device_name() { return g_deviceName.c_str(); }

int wifi_portal_get_tz_offset_min() { return g_tzOffsetMin; }

bool wifi_portal_has_credentials()
{
    return g_ssid.length() > 0;
}

const char* wifi_portal_get_ssid() { return g_ssid.c_str(); }
const char* wifi_portal_get_pass() { return g_pass.c_str(); }

void wifi_portal_start_ap()
{
    if (g_apActive) return;

    Serial.println("[WIFI-PORTAL] Starting setup AP...");

    // About to fully disconnect STA below - if the monitor/setup web server
    // is currently serving over that STA connection (it now runs
    // persistently in the background, see wifi_monitor_start() being
    // called from wifi_ntp_update_state()), it MUST be cleanly stopped
    // first. Confirmed on real hardware: skipping this crashed hard
    // (igmp_lookup_group assert and, separately, a LoadProhibited panic -
    // different symptoms of the same root cause) - MDNS/the WebServer were
    // still bound to the STA interface's multicast group/socket when
    // WiFi.disconnect() below tore it down under them.
    if (g_monitorServing) wifi_monitor_stop();

    static bool eventHandlerRegistered = false;
    if (!eventHandlerRegistered) {
        WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
            if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
                Serial.println("[WIFI-PORTAL] Phone/station joined the AP");
            } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
                Serial.println("[WIFI-PORTAL] Phone/station left the AP");
            }
        });
        eventHandlerRegistered = true;
    }

    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(true /* wifioff */, true /* eraseap */);
    delay(500);

    g_apSsid = buildApSsid();

    WiFi.mode(WIFI_STA);
    delay(200);

    WiFi.mode(WIFI_AP_STA);
    delay(100);

    Serial.printf("[WIFI-PORTAL] Free heap before softAP(): %u\n", (unsigned)ESP.getFreeHeap());

    bool apOk = WiFi.softAP(g_apSsid.c_str());
    if (!apOk) {
        Serial.println("[WIFI-PORTAL] WiFi.softAP() FAILED - retrying once");
        delay(200);
        apOk = WiFi.softAP(g_apSsid.c_str());
        Serial.printf("[WIFI-PORTAL] retry %s\n", apOk ? "OK" : "FAILED");
    }
    g_apIp = WiFi.softAPIP();

    WiFi.mode(WIFI_AP);

    dnsServer.start(DNS_PORT, "*", g_apIp);

    ensureHttpHandlersRegistered();
    server.begin();

    g_apActive = true;

    Serial.printf("[WIFI-PORTAL] AP '%s' ready at %s\n",
                  g_apSsid.c_str(), g_apIp.toString().c_str());
}

void wifi_portal_stop_ap()
{
    if (!g_apActive) return;

    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);

    g_apActive = false;

    Serial.println("[WIFI-PORTAL] AP stopped.");
}

bool wifi_portal_is_active() { return g_apActive; }

void wifi_portal_loop()
{
    if (!g_apActive) return;
    dnsServer.processNextRequest();
    server.handleClient();
}

const char* wifi_portal_ap_ssid() { return g_apSsid.c_str(); }
IPAddress wifi_portal_ap_ip() { return g_apIp; }

// ====== Remote monitor ======

void wifi_monitor_start()
{
    // Idempotent - safe to call both from wifi_ntp_update_state() (as soon
    // as STA connects, so the setup page stays reachable from any screen,
    // not just while physically on REMOTE SETUP - confirmed on real
    // hardware to be a frequent source of "the page won't load" reports
    // that were actually just the device sitting on a different screen)
    // and from monitor_screen_on_enter(). Re-running server.begin()/
    // MDNS.begin() while already serving would be redundant at best.
    if (g_monitorServing || (g_monitorUsingOwnAp && g_apActive)) return;

    // Reconnecting to a saved network after leaving SETUP is asynchronous
    // (wifi_begin_nonblocking() just kicks off WiFi.begin() and returns) -
    // landing here right after leaving SETUP (e.g. swiping straight back to
    // MONITOR) could otherwise catch WiFi mid-negotiation and wrongly
    // conclude there's "no STA connection", falling back to this device's
    // own AP even though it was about to reconnect just fine. Give it a
    // bounded moment to finish first - confirmed on real hardware that
    // without this, SETUP -> MONITOR reliably landed on the own-AP fallback
    // instead of the real network.
    // WiFi.status() == WL_CONNECTED only means associated/authenticated -
    // it does NOT guarantee DHCP has actually handed out a usable address.
    // Confirmed on real hardware via [NETDIAG] logging in main.cpp: status
    // read WL_CONNECTED while WiFi.localIP() was still 0.0.0.0, which would
    // have made this start serving on an unreachable interface while
    // believing it had succeeded. Both checks below require a real IP too.
    auto hasValidIp = []() { return WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0); };

    if (!hasValidIp() && wifi_portal_has_credentials()) {
        Serial.println("[MONITOR] Not connected yet - giving reconnect up to 4s before falling back to own AP...");
        uint32_t waitStart = millis();
        while (!hasValidIp() && millis() - waitStart < 4000) {
            delay(100);
        }
        Serial.printf("[MONITOR] After wait: status=%d ip=%s (%s)\n",
                      (int)WiFi.status(), WiFi.localIP().toString().c_str(), hasValidIp() ? "connected" : "still not connected");
    }

    if (hasValidIp()) {
        g_monitorUsingOwnAp = false;
        ensureHttpHandlersRegistered();
        server.begin();
        g_monitorServing = true;

        g_mdnsHostname = buildMdnsHostname();
        if (MDNS.begin(g_mdnsHostname.c_str())) {
            MDNS.addService("http", "tcp", 80);
            g_mdnsRunning = true;
            Serial.printf("[MONITOR] Serving on existing WiFi at %s / %s.local\n",
                          WiFi.localIP().toString().c_str(), g_mdnsHostname.c_str());
        } else {
            Serial.println("[MONITOR] mDNS start failed - IP address still works");
        }
    } else {
        g_monitorUsingOwnAp = true;
        wifi_portal_start_ap();
        Serial.printf("[MONITOR] No STA connection - serving from own AP '%s' at %s\n",
                      g_apSsid.c_str(), g_apIp.toString().c_str());
    }
}

void wifi_monitor_stop()
{

    if (g_monitorUsingOwnAp) {
        wifi_portal_stop_ap();
    } else if (g_monitorServing) {
        server.stop();
        g_monitorServing = false;
    }
    if (g_mdnsRunning) {
        MDNS.end();
        g_mdnsRunning = false;
    }
    g_monitorUsingOwnAp = false;

    Serial.println("[MONITOR] Stopped.");
}

bool wifi_monitor_is_active()
{
    return g_monitorServing || (g_monitorUsingOwnAp && g_apActive);
}

void wifi_monitor_service()
{
    if (g_monitorServing) {
        server.handleClient();
    }
}

bool wifi_monitor_using_own_ap() { return g_monitorUsingOwnAp; }

IPAddress wifi_monitor_ip()
{
    return g_monitorUsingOwnAp ? g_apIp : WiFi.localIP();
}

const char* wifi_monitor_hostname() { return g_mdnsHostname.c_str(); }
