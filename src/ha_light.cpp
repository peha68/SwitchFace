#include "ha_light.h"
#include "wifi_portal.h"

#include <WiFi.h>
#include <HTTPClient.h>

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
