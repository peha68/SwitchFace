#include "ota.h"
#include "wifi_portal.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ctype.h>

static bool    g_otaInProgress = false;
static uint8_t g_otaProgressPct = 0;

// Same reasoning as wifi_portal.cpp's sanitizeForHostname() (not exported
// from there, so duplicated here rather than adding a new public API for
// this one small use) - ArduinoOTA's hostname ends up in mDNS, which only
// tolerates letters/digits/hyphens.
static String sanitizeHostname(const String& raw)
{
    String out;
    out.reserve(raw.length());
    bool lastWasHyphen = true;
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

void ota_init()
{
    String hostname = sanitizeHostname(wifi_portal_get_device_name());
    if (hostname.length() == 0) hostname = "switchface";
    ArduinoOTA.setHostname(hostname.c_str());

    // wifi_portal.cpp already owns the single global MDNS instance
    // (MDNS.begin()/addService()/end() in wifi_monitor_start()/_stop()) -
    // letting ArduinoOTA register its own mDNS service on top of that would
    // fight over the same singleton (e.g. wifi_monitor_stop()'s MDNS.end()
    // would also kill ArduinoOTA's registration). OTA still works fine by
    // IP (`--upload-port <device-ip>`), it just won't show up by hostname
    // in an IDE's network-port picker.
    ArduinoOTA.setMdnsEnabled(false);

    const char* pass = wifi_portal_get_ota_password();
    if (pass && pass[0]) ArduinoOTA.setPassword(pass);

    ArduinoOTA.onStart([]() {
        g_otaInProgress = true;
        g_otaProgressPct = 0;
        const char* type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        Serial.printf("[OTA] Start updating %s\n", type);
    });
    ArduinoOTA.onEnd([]() {
        g_otaInProgress = false;
        Serial.println("[OTA] End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        g_otaProgressPct = total ? (uint8_t)((progress * 100u) / total) : 0;
        Serial.printf("[OTA] Progress: %u%%\n", g_otaProgressPct);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        g_otaInProgress = false;
        const char* reason = "Unknown";
        switch (error) {
            case OTA_AUTH_ERROR:    reason = "Auth failed";    break;
            case OTA_BEGIN_ERROR:   reason = "Begin failed";   break;
            case OTA_CONNECT_ERROR: reason = "Connect failed"; break;
            case OTA_RECEIVE_ERROR: reason = "Receive failed"; break;
            case OTA_END_ERROR:     reason = "End failed";     break;
            default: break;
        }
        Serial.printf("[OTA] Error[%u]: %s\n", (unsigned)error, reason);
    });

    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready, hostname=%s, password=%s\n",
                  hostname.c_str(), (pass && pass[0]) ? "set" : "none");
}

void ota_loop()
{
    ArduinoOTA.handle();
}

bool ota_in_progress() { return g_otaInProgress; }
uint8_t ota_progress_percent() { return g_otaProgressPct; }
