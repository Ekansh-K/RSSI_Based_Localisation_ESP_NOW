/*
 * anchor_main.cpp  -  Fixed Anchor (ESP32-C3)
 * =============================================
 * - Connects to Wi-Fi (so UDP to laptop works)
 * - Enables promiscuous mode to capture RSSI of every received frame
 * - Receives ESP-NOW broadcasts from the tag
 * - Averages RSSI over a window and sends UDP report to laptop
 *
 * In CALIBRATION_MODE=1: sends one report per packet (no averaging)
 *
 * Flash normal  :  pio run -e anchor1     -t upload
 * Flash cal mode:  pio run -e anchor1_cal -t upload
 */
#ifdef NODE_ROLE_ANCHOR

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFiUdp.h>
#include "config.h"

#ifndef ANCHOR_ID
  #error "ANCHOR_ID not defined. Add -D ANCHOR_ID=<1|2|3|4> in platformio.ini."
#endif
#ifndef CALIBRATION_MODE
  #define CALIBRATION_MODE 0
#endif

// UDP report — MUST match '<BfBIBHB' (14 bytes) in server.py + calibration.py
typedef struct __attribute__((packed)) {
    uint8_t  anchor_id;
    float    avg_rssi;
    uint8_t  channel;
    uint32_t timestamp_ms;
    uint8_t  tag_id;
    uint16_t sample_count;
    uint8_t  calibration_mode;
} anchor_report_t;

// Tag broadcast — must match tag_main.cpp
typedef struct __attribute__((packed)) {
    uint8_t  tag_id;
    uint32_t counter;
    uint32_t uptime_ms;
} tag_broadcast_t;

WiFiUDP         g_udp;
volatile int8_t g_latest_rssi = -127;
static int8_t   g_rssi_window[RSSI_WINDOW_SIZE];
static uint8_t  g_win_index  = 0;
static uint8_t  g_win_filled = 0;

// ISR — fires on every received Wi-Fi frame including ESP-NOW
void IRAM_ATTR promiscuousRxCB(void *buf, wifi_promiscuous_pkt_type_t type) {
    (void)type;
    g_latest_rssi = ((wifi_promiscuous_pkt_t *)buf)->rx_ctrl.rssi;
}

static float windowMean() {
    int n = (g_win_filled < RSSI_WINDOW_SIZE) ? g_win_filled : RSSI_WINDOW_SIZE;
    if (n == 0) return (float)g_latest_rssi;
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += (float)g_rssi_window[i];
    return s / (float)n;
}

static void sendReport(float rssi_val, uint8_t tag_id, uint16_t samples) {
    anchor_report_t rep;
    memset(&rep, 0, sizeof(rep));
    rep.anchor_id        = (uint8_t)ANCHOR_ID;
    rep.avg_rssi         = rssi_val;
    rep.channel          = (uint8_t)ESPNOW_CHANNEL;
    rep.timestamp_ms     = (uint32_t)millis();
    rep.tag_id           = tag_id;
    rep.sample_count     = samples;
    rep.calibration_mode = (uint8_t)CALIBRATION_MODE;

    g_udp.beginPacket(LAPTOP_IP, LAPTOP_UDP_PORT);
    g_udp.write((const uint8_t *)&rep, sizeof(rep));
    g_udp.endPacket();

    Serial.printf("[A%d] RSSI=%5.1f dBm  n=%d%s\n",
                  ANCHOR_ID, rssi_val, samples,
                  CALIBRATION_MODE ? "  [CAL]" : "");
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    (void)mac_addr;
    if (len < (int)sizeof(tag_broadcast_t)) return;

    tag_broadcast_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    int8_t rssi_snap = g_latest_rssi;

#if CALIBRATION_MODE
    sendReport((float)rssi_snap, pkt.tag_id, 1);
#else
    g_rssi_window[g_win_index % RSSI_WINDOW_SIZE] = rssi_snap;
    g_win_index++;
    if (g_win_filled < RSSI_WINDOW_SIZE) g_win_filled++;
    if ((g_win_index % RSSI_WINDOW_SIZE) == 0)
        sendReport(windowMean(), pkt.tag_id, RSSI_WINDOW_SIZE);
#endif
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("\n=== ANCHOR %d  [%s] ===\n",
                  ANCHOR_ID, CALIBRATION_MODE ? "CALIBRATION" : "NORMAL");
    Serial.printf("  Target SSID : \"%s\"\n", WIFI_SSID);
    Serial.printf("  Target pass : \"%s\"\n", WIFI_PASSWORD);

    // ── Wi-Fi: scan → find BSSID → connect with pinned BSSID+channel ──
    Serial.setDebugOutput(true);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    // Arduino ESP32 3.x defaults _minSecurity to WIFI_AUTH_WPA2_PSK which causes
    // AUTH_EXPIRE on WPA/WPA2-Personal mixed-mode routers (TKIP cipher rejected).
    // Set to WPA_PSK so both WPA1-TKIP and WPA2-CCMP are accepted.
    WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
    WiFi.mode(WIFI_STA);
    delay(100);

    Serial.println("  Scanning...");
    int n = WiFi.scanNetworks(false, true);  // blocking, show hidden
    uint8_t  tgt_bssid[6] = {0};
    int      tgt_channel   = ESPNOW_CHANNEL;
    bool     tgt_found     = false;

    for (int i = 0; i < n; i++) {
        int enc = (int)WiFi.encryptionType(i);
        bool match = (WiFi.SSID(i) == String(WIFI_SSID));
        Serial.printf("  [SCAN] %-20s ch=%-2d RSSI=%-4d enc=%d %s\n",
                      WiFi.SSID(i).c_str(), WiFi.channel(i),
                      WiFi.RSSI(i), enc,
                      match ? "<-- TARGET" : "");
        if (match && !tgt_found) {
            memcpy(tgt_bssid, WiFi.BSSID(i), 6);
            tgt_channel = WiFi.channel(i);
            tgt_found   = true;
            Serial.printf("  BSSID: %s  enc=%d (3=WPA2 4=WPA/WPA2 6=WPA3)\n",
                          WiFi.BSSIDstr(i).c_str(), enc);
        }
    }
    WiFi.scanDelete();

    if (!tgt_found) {
        Serial.printf("  SSID \"%s\" not found in scan!\n", WIFI_SSID);
    }

    // Connect — with BSSID pinned if found (avoids AP-selection ambiguity)
    if (tgt_found) {
        Serial.printf("  Connecting via BSSID on ch%d", tgt_channel);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD, tgt_channel, tgt_bssid, true);
    } else {
        Serial.printf("  Connecting (no BSSID)");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    uint32_t wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 30000) {
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("\nFATAL: Wi-Fi failed (status=%d) after 30s\n", WiFi.status());
        delay(5000);
        ESP.restart();
    }
    Serial.printf("\n  IP  : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  MAC : %s\n", WiFi.macAddress().c_str());
    Serial.printf("  Ch  : %d\n", WiFi.channel());
    Serial.setDebugOutput(false);

    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuousRxCB);

    if (esp_now_init() != ESP_OK) {
        Serial.println("FATAL: esp_now_init failed. Restarting.");
        ESP.restart();
    }
    esp_now_register_recv_cb(onDataRecv);
    g_udp.begin(LAPTOP_UDP_PORT);

    Serial.printf("[ANCHOR %d] Ready.\n\n", ANCHOR_ID);
}

void loop() {
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat > 5000) {
        lastBeat = millis();
        Serial.printf("[A%d] alive  latest_rssi=%d  wifi=%s\n",
                      ANCHOR_ID, g_latest_rssi,
                      WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
    }
}

#endif // NODE_ROLE_ANCHOR