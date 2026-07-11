/*
 * Mobile Tag (ESP32-C3) — ESP-NOW broadcast only.
 *
 * Flash:  pio run -e tag -t upload
 *
 * Channel must match the Wi-Fi AP channel used by the anchors
 * (anchors stay on the AP channel so UDP works).
 */
#ifdef NODE_ROLE_TAG

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "config.h"

typedef struct __attribute__((packed)) {
    uint8_t  tag_id;
    uint32_t counter;
    uint32_t uptime_ms;
} tag_broadcast_t;

static const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint32_t g_counter  = 0;
static uint32_t g_lastSend = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TAG starting ===");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    // Tag does not join the AP; radio must still sit on the same channel as anchors/AP.
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.printf("  MAC : %s\n", WiFi.macAddress().c_str());
    Serial.printf("  CH  : %d  (must match AP / anchors)\n", ESPNOW_CHANNEL);
    Serial.printf("  TX  : every %d ms\n", TAG_TX_INTERVAL_MS);

    if (esp_now_init() != ESP_OK) {
        Serial.println("FATAL: esp_now_init failed. Restarting.");
        ESP.restart();
    }
    // Send callback is optional; skip it to avoid Arduino 2.x/3.x ABI differences.

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("FATAL: esp_now_add_peer failed. Restarting.");
        ESP.restart();
    }

    Serial.println("[TAG] Ready.\n");
}

void loop() {
    uint32_t now = millis();
    if (now - g_lastSend < (uint32_t)TAG_TX_INTERVAL_MS) return;

    g_lastSend = now;
    g_counter++;

    // Re-assert channel occasionally (not every packet — reduces radio churn)
    if ((g_counter % 25u) == 0u) {
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    }

    tag_broadcast_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.tag_id    = TAG_ID;
    pkt.counter   = g_counter;
    pkt.uptime_ms = now;

    esp_err_t err = esp_now_send(BROADCAST_ADDR,
                                 (const uint8_t *)&pkt, sizeof(pkt));
    if (err != ESP_OK)
        Serial.printf("[TAG] send error 0x%x\n", err);

    if (g_counter % 25 == 0)
        Serial.printf("[TAG] #%u  uptime=%u ms\n", g_counter, now);
}

#endif // NODE_ROLE_TAG
