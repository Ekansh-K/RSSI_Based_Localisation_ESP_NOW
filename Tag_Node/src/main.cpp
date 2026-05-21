/*
 * tag_main.cpp  -  Mobile Tag (ESP32-C3)
 * Broadcasts ESP-NOW packet every TAG_TX_INTERVAL_MS ms.
 * No Wi-Fi AP connection needed — radio only for ESP-NOW.
 *
 * Flash:  pio run -e tag -t upload
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

static const uint8_t BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint32_t g_counter  = 0;
static uint32_t g_lastSend = 0;

void IRAM_ATTR onTagSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac; (void)status;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TAG starting ===");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.printf("  MAC : %s\n", WiFi.macAddress().c_str());
    Serial.printf("  TX  : every %d ms\n", TAG_TX_INTERVAL_MS);

    if (esp_now_init() != ESP_OK) {
        Serial.println("FATAL: esp_now_init failed. Restarting.");
        ESP.restart();
    }
    esp_now_register_send_cb(onTagSent);

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
    if (now - g_lastSend >= TAG_TX_INTERVAL_MS) {
        g_lastSend = now;
        g_counter++;

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
}

#endif // NODE_ROLE_TAG