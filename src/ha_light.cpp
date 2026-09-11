#include "ha_light.h"
#include "wifi_portal.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <ctype.h>
#include <string.h>

// Deliberately no ArduinoJson here - the only thing we ever need out of a
// response body is a single "on"/"off" state, and the WiFi credentials
// portal's page save confirmations are already plain strings. Pulling in
// a JSON library for one substring check isn't worth the flash/RAM on
// this board (see main.cpp's header comment on why ArduinoJson was
// removed from this project entirely).

static String haDomainFromEntity(const String& entity)
{
    int dot = entity.indexOf('.');
    if (dot < 0) return String();
    return entity.substring(0, dot);
}

bool ha_light_toggle(const char* entityId)
{
    if (!wifi_portal_has_ha_config() || WiFi.status() != WL_CONNECTED) return false;
    if (!entityId || !entityId[0]) return false;

    String entity = entityId;
    String domain = haDomainFromEntity(entity);
    if (domain.length() == 0) return false;

    String url = String(wifi_portal_get_ha_url()) + "/api/services/" + domain + "/toggle";

    HTTPClient http;
    http.setTimeout(4000);
    if (!http.begin(url)) return false;
    http.addHeader("Authorization", String("Bearer ") + wifi_portal_get_ha_token());
    http.addHeader("Content-Type", "application/json");

    String body = String("{\"entity_id\":\"") + entity + "\"}";
    int code = http.POST(body);
    http.end();

    Serial.printf("[HA] toggle %s -> HTTP %d\n", entity.c_str(), code);
    return code >= 200 && code < 300;
}

bool ha_light_poll_state(const char* entityId, bool* outIsOn)
{
    if (!wifi_portal_has_ha_config() || WiFi.status() != WL_CONNECTED) return false;
    if (!entityId || !entityId[0]) return false;

    String entity = entityId;
    String url = String(wifi_portal_get_ha_url()) + "/api/states/" + entity;

    HTTPClient http;
    http.setTimeout(4000);
    if (!http.begin(url)) return false;
    http.addHeader("Authorization", String("Bearer ") + wifi_portal_get_ha_token());

    int code = http.GET();
    if (code != 200) {
        Serial.printf("[HA] poll %s -> HTTP %d\n", entity.c_str(), code);
        http.end();
        return false;
    }

    String payload = http.getString();
    http.end();

    // Looking for "state":"on" / "state":"off" in the JSON body - a plain
    // substring search is enough for a fixed two-value field like this.
    if (payload.indexOf("\"state\":\"on\"") >= 0) {
        *outIsOn = true;
        return true;
    }
    if (payload.indexOf("\"state\":\"off\"") >= 0) {
        *outIsOn = false;
        return true;
    }
    return false; // unexpected/unavailable state - leave *outIsOn as-is
}

bool ha_light_is_weather(const char* entityId)
{
    if (!entityId) return false;
    return strncmp(entityId, "weather.", 8) == 0;
}

// Finds `key` (e.g. "\"temperature_unit\":\"") and copies the text up to
// the next '"' into out (capped at outSize-1). Same no-ArduinoJson
// targeted substring approach as ha_light_poll_state() above and
// update_check.cpp's extractJsonString() - this project deliberately
// doesn't pull in a JSON library for pulling a couple of known fields out
// of an otherwise-ignored response body.
static bool extractJsonString(const String& body, const char* key, char* out, size_t outSize)
{
    int idx = body.indexOf(key);
    if (idx < 0) return false;
    int start = idx + strlen(key);
    int end = body.indexOf('"', start);
    if (end < 0) return false;
    size_t len = (size_t)(end - start);
    if (len >= outSize) len = outSize - 1;
    memcpy(out, body.c_str() + start, len);
    out[len] = '\0';
    return true;
}

// Finds `key` (e.g. "\"temperature\":") and parses the number right after
// it (HA emits attribute numbers unquoted - optional '-', digits, optional
// '.', digits).
static bool extractJsonNumber(const String& body, const char* key, float* out)
{
    int idx = body.indexOf(key);
    if (idx < 0) return false;
    int start = idx + strlen(key);
    int end = start;
    int len = body.length();
    if (end < len && body[end] == '-') end++;
    while (end < len && (isdigit((unsigned char)body[end]) || body[end] == '.')) end++;
    if (end == start || (end == start + 1 && body[start] == '-')) return false;
    *out = body.substring(start, end).toFloat();
    return true;
}

bool ha_light_poll_weather(const char* entityId, HaWeather* out)
{
    if (!wifi_portal_has_ha_config() || WiFi.status() != WL_CONNECTED) return false;
    if (!entityId || !entityId[0] || !out) return false;

    String url = String(wifi_portal_get_ha_url()) + "/api/states/" + entityId;

    HTTPClient http;
    http.setTimeout(4000);
    if (!http.begin(url)) return false;
    http.addHeader("Authorization", String("Bearer ") + wifi_portal_get_ha_token());

    int code = http.GET();
    if (code != 200) {
        Serial.printf("[HA] weather poll %s -> HTTP %d\n", entityId, code);
        http.end();
        return false;
    }

    String payload = http.getString();
    http.end();

    HaWeather parsed = {};
    bool ok = extractJsonNumber(payload, "\"temperature\":", &parsed.temperature)
           && extractJsonString(payload, "\"state\":\"", parsed.condition, sizeof(parsed.condition));
    if (!ok) {
        Serial.printf("[HA] weather poll %s - missing temperature/state in response\n", entityId);
        return false;
    }
    // Unit/wind fields are nice-to-have, not required - a missing one
    // just leaves that field blank/zero rather than failing the whole poll
    // (some weather integrations may not expose wind_speed, for example).
    extractJsonString(payload, "\"temperature_unit\":\"", parsed.temperatureUnit, sizeof(parsed.temperatureUnit));
    extractJsonNumber(payload, "\"wind_speed\":", &parsed.windSpeed);
    extractJsonString(payload, "\"wind_speed_unit\":\"", parsed.windSpeedUnit, sizeof(parsed.windSpeedUnit));

    *out = parsed;
    return true;
}
