#include "update_check.h"

#include <Arduino.h>
#include <ctype.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include "mbedtls/sha256.h"

// Hardcoded to this specific repo - update if this project is ever forked
// under a different owner/name.
static const char* GITHUB_API_URL = "https://api.github.com/repos/peha68/SwitchFace/releases/latest";

// api.github.com's current root CA (Sectigo "Public Server Authentication
// Root E46", chaining to USERTrust) - fetched via `openssl s_client
// -connect api.github.com:443 -showcerts` and used to validate the
// release-check API call. NOT used for the firmware binary download
// itself - see update_perform()'s comment for why (GitHub's asset CDN
// uses a different, less stable CA/hostname; integrity there is checked
// via SHA256 instead of TLS pinning). Root certs are long-lived (this one
// is valid to 2038) but if GitHub ever migrates away from Sectigo, this
// will need refreshing the same way - the failure mode is just "update
// checks stop working until a USB reflash", not anything unsafe.
static const char GITHUB_API_ROOT_CA[] PROGMEM = R"CERT(
-----BEGIN CERTIFICATE-----
MIIDRjCCAsugAwIBAgIQGp6v7G3o4ZtcGTFBto2Q3TAKBggqhkjOPQQDAzCBiDEL
MAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNl
eSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMT
JVVTRVJUcnVzdCBFQ0MgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMjEwMzIy
MDAwMDAwWhcNMzgwMTE4MjM1OTU5WjBfMQswCQYDVQQGEwJHQjEYMBYGA1UEChMP
U2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1TZWN0aWdvIFB1YmxpYyBTZXJ2ZXIg
QXV0aGVudGljYXRpb24gUm9vdCBFNDYwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAR2
+pmpbiDt+dd34wc7qNs9Xzjoq1WmVk/WSOrsfy2qw7LFeeyZYX8QeccCWvkEN/U0
NSt3zn8gj1KjAIns1aeibVvjS5KToID1AZTc8GgHHs3u/iVStSBDHBv+6xnOQ6Oj
ggEgMIIBHDAfBgNVHSMEGDAWgBQ64QmG1M8ZwpZ2dEl23OA1xmNjmjAdBgNVHQ4E
FgQU0SLaTFnxS18mOKqd1u7rDcP7qWEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB
/wQFMAMBAf8wHQYDVR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMBEGA1UdIAQK
MAgwBgYEVR0gADBQBgNVHR8ESTBHMEWgQ6BBhj9odHRwOi8vY3JsLnVzZXJ0cnVz
dC5jb20vVVNFUlRydXN0RUNDQ2VydGlmaWNhdGlvbkF1dGhvcml0eS5jcmwwNQYI
KwYBBQUHAQEEKTAnMCUGCCsGAQUFBzABhhlodHRwOi8vb2NzcC51c2VydHJ1c3Qu
Y29tMAoGCCqGSM49BAMDA2kAMGYCMQCMCyBit99vX2ba6xEkDe+YO7vC0twjbkv9
PKpqGGuZ61JZryjFsp+DFpEclCVy4noCMQCwvZDXD/m2Ko1HA5Bkmz7YQOFAiNDD
49IWa2wdT7R3DtODaSXH/BiXv8fwB9su4tU=
-----END CERTIFICATE-----
)CERT";

static String g_latestVersion;
static String g_assetUrl;
static String g_expectedSha256; // lowercase hex, 64 chars
static bool   g_available   = false;
static bool   g_inProgress  = false;
static uint8_t g_progressPct = 0;

void update_check_init()
{
    // Nothing to do yet - kept as a symmetrical init() alongside
    // update_check_now(), matching ota_init()/ota_loop()'s split in
    // ota.h, in case this ever needs one-time setup later.
}

// Finds `key` starting at/after `fromIndex`, and returns the text between
// it and the next '"' as `out`. Deliberately no ArduinoJson - matches
// this project's existing stance (see ha_light.cpp's header comment) for
// pulling one or two known fields out of an otherwise-ignored JSON blob.
static bool extractJsonString(const String& body, const String& key, int fromIndex, String& out)
{
    int idx = body.indexOf(key, fromIndex);
    if (idx < 0) return false;
    int start = idx + key.length();
    int end = body.indexOf('"', start);
    if (end < 0) return false;
    out = body.substring(start, end);
    return true;
}

bool update_check_now()
{
    if (WiFi.status() != WL_CONNECTED) return false;

    WiFiClientSecure client;
    client.setCACert(GITHUB_API_ROOT_CA);

    HTTPClient http;
    http.setTimeout(8000);
    if (!http.begin(client, GITHUB_API_URL)) {
        Serial.println("[UPDATE] http.begin() failed (release check)");
        return false;
    }
    http.addHeader("User-Agent", "SwitchFace-ESP32"); // GitHub's API 403s without one
    http.addHeader("Accept", "application/vnd.github+json");

    int code = http.GET();
    if (code != 200) {
        Serial.printf("[UPDATE] GitHub API GET -> HTTP %d\n", code);
        http.end();
        return false;
    }

    String body = http.getString();
    http.end();

    String tag;
    if (!extractJsonString(body, "\"tag_name\":\"", 0, tag)) {
        Serial.println("[UPDATE] Could not find tag_name in the API response");
        return false;
    }

    // Search for the browser_download_url belonging to the firmware.bin
    // asset specifically (a release could in principle have other
    // assets), by anchoring the search to just after that asset's "name".
    int nameIdx = body.indexOf("\"name\":\"firmware.bin\"");
    if (nameIdx < 0) {
        Serial.println("[UPDATE] Latest release has no firmware.bin asset");
        return false;
    }
    String assetUrl;
    if (!extractJsonString(body, "\"browser_download_url\":\"", nameIdx, assetUrl)) {
        Serial.println("[UPDATE] Could not find firmware.bin's download URL");
        return false;
    }

    // The expected SHA256 is published as plain text ("SHA256: <hex>") in
    // the release body by .github/workflows/release.yml - searched for
    // directly rather than JSON-unescaping the whole "body" field, since
    // hex digits are never escaped either way.
    int shaIdx = body.indexOf("SHA256: ");
    String sha256;
    if (shaIdx >= 0) {
        sha256 = body.substring(shaIdx + 8, shaIdx + 8 + 64);
        sha256.toLowerCase();
    }
    bool shaValid = sha256.length() == 64;
    for (size_t i = 0; shaValid && i < sha256.length(); i++) {
        if (!isxdigit((unsigned char)sha256[i])) shaValid = false;
    }
    if (!shaValid) {
        Serial.println("[UPDATE] Could not find a valid SHA256 in the release notes");
        return false;
    }

    g_latestVersion = tag;
    g_assetUrl = assetUrl;
    g_expectedSha256 = sha256;
    g_available = !tag.equals(FIRMWARE_VERSION);

    Serial.printf("[UPDATE] Latest release: %s (current: %s) - %s\n",
                  tag.c_str(), FIRMWARE_VERSION, g_available ? "update available" : "up to date");
    return true;
}

bool update_is_available() { return g_available; }
const char* update_latest_version() { return g_latestVersion.c_str(); }
const char* update_current_version() { return FIRMWARE_VERSION; }
bool update_in_progress() { return g_inProgress; }
uint8_t update_progress_percent() { return g_progressPct; }

bool update_perform()
{
    if (g_assetUrl.length() == 0 || g_expectedSha256.length() != 64) {
        Serial.println("[UPDATE] No release info to install - call update_check_now() first");
        return false;
    }
    if (WiFi.status() != WL_CONNECTED) return false;

    g_inProgress = true;
    g_progressPct = 0;

    // Deliberately setInsecure() here, unlike the CA-pinned release-check
    // call above: release assets redirect through GitHub's CDN
    // (objects.githubusercontent.com at the time this was written),
    // confirmed via openssl s_client to use a completely different CA
    // (Let's Encrypt, not Sectigo) than api.github.com - and that
    // hostname/CA has changed more than once in GitHub's history. Pinning
    // it would mean updates silently stop working whenever GitHub
    // reshuffles that infrastructure, with no fix short of a USB reflash.
    // Instead, every downloaded byte is hashed below and checked against
    // the SHA256 obtained over the CA-pinned API call before the image is
    // ever finalized - a compromised/spoofed CDN response fails that
    // check and is aborted, never flashed.
    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setTimeout(15000);
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    if (!http.begin(client, g_assetUrl)) {
        Serial.println("[UPDATE] http.begin() failed (asset download)");
        g_inProgress = false;
        return false;
    }
    http.addHeader("User-Agent", "SwitchFace-ESP32");

    int code = http.GET();
    if (code != 200) {
        Serial.printf("[UPDATE] Asset GET -> HTTP %d\n", code);
        http.end();
        g_inProgress = false;
        return false;
    }

    int contentLength = http.getSize();
    if (contentLength <= 0) {
        Serial.println("[UPDATE] Asset response has no usable Content-Length - refusing");
        http.end();
        g_inProgress = false;
        return false;
    }

    if (!Update.begin(contentLength, U_FLASH)) {
        Serial.printf("[UPDATE] Update.begin() failed: %s\n", Update.errorString());
        http.end();
        g_inProgress = false;
        return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0); // 0 = SHA-256 (not the SHA-224 variant)

    WiFiClient* stream = http.getStreamPtr();
    uint8_t buf[1024];
    int written = 0;
    uint32_t lastLog = millis();

    while (http.connected() && written < contentLength) {
        size_t avail = stream->available();
        if (avail == 0) { delay(5); continue; }
        size_t chunk = avail < sizeof(buf) ? avail : sizeof(buf);
        int n = stream->readBytes(buf, chunk);
        if (n <= 0) break;
        Update.write(buf, n);
        mbedtls_sha256_update(&sha, buf, n);
        written += n;
        g_progressPct = (uint8_t)(((long)written * 100) / contentLength);
        if (millis() - lastLog > 1000) {
            lastLog = millis();
            Serial.printf("[UPDATE] %u%% (%d/%d bytes)\n", g_progressPct, written, contentLength);
        }
    }
    http.end();

    uint8_t hash[32];
    mbedtls_sha256_finish(&sha, hash);
    mbedtls_sha256_free(&sha);

    char hashHex[65];
    for (int i = 0; i < 32; i++) sprintf(&hashHex[i * 2], "%02x", hash[i]);
    hashHex[64] = '\0';

    bool sizeOk = (written == contentLength);
    bool hashOk = g_expectedSha256.equalsIgnoreCase(hashHex);

    Serial.printf("[UPDATE] Downloaded %d/%d bytes, hash=%s expected=%s (%s)\n",
                  written, contentLength, hashHex, g_expectedSha256.c_str(),
                  (sizeOk && hashOk) ? "OK" : "MISMATCH");

    g_inProgress = false;

    if (!sizeOk || !hashOk) {
        Update.abort();
        Serial.println("[UPDATE] Aborted - size or hash mismatch, current firmware left untouched");
        return false;
    }

    if (!Update.end(true)) {
        Serial.printf("[UPDATE] Update.end() failed: %s\n", Update.errorString());
        return false;
    }

    Serial.println("[UPDATE] Flashed OK, restarting...");
    Serial.flush();
    delay(200);
    ESP.restart();
    return true; // unreachable - ESP.restart() doesn't return
}
