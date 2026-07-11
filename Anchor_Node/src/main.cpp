/*
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
#include <string.h>
#include "config.h"

#ifndef ANCHOR_ID
  #error "ANCHOR_ID not defined. Add -D ANCHOR_ID=<1..254> in platformio.ini."
#endif
#if (ANCHOR_ID < 1) || (ANCHOR_ID > 254)
  #error "ANCHOR_ID must be in 1..254 (UDP report uses uint8_t)."
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

// Tag broadcast — must match Tag_Node/src/main.cpp
typedef struct __attribute__((packed)) {
    uint8_t  tag_id;
    uint32_t counter;
    uint32_t uptime_ms;
} tag_broadcast_t;

WiFiUDP         g_udp;
static int8_t    g_rssi_window[RSSI_WINDOW_SIZE];
static uint16_t  g_win_index   = 0;   // uint16 avoids wrap issues if WINDOW is large
static uint8_t   g_win_filled  = 0;
static uint8_t  g_radio_channel = ESPNOW_CHANNEL;
static int8_t   g_last_rssi   = -127;

// Promiscuous fallback only when ESP-NOW rx_ctrl RSSI is unavailable (Arduino 2.x).
// Filtered by known tag MAC after first ESP-NOW packet is seen.
static volatile int8_t  g_promisc_rssi = -127;
static uint8_t          g_tag_mac[6]   = {0};
static volatile bool    g_tag_mac_known = false;

static void IRAM_ATTR promiscuousRxCB(void *buf, wifi_promiscuous_pkt_type_t type) {
    (void)type;
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    if (!pkt) return;

    // 802.11 header: addr2 (transmitter) starts at offset 10
    if (pkt->rx_ctrl.sig_len < 16) return;
    const uint8_t *hdr = pkt->payload;
    const uint8_t *src = hdr + 10;

    if (g_tag_mac_known) {
        if (memcmp(src, g_tag_mac, 6) != 0) return;
        g_promisc_rssi = pkt->rx_ctrl.rssi;
    }
}

// Median of the sliding window — more robust to single-packet multipath spikes
// than a plain mean (RSSI indoors is often left-skewed / spike-heavy).
static float windowMedian() {
    int n = (g_win_filled < RSSI_WINDOW_SIZE) ? g_win_filled : RSSI_WINDOW_SIZE;
    if (n <= 0) return (float)g_last_rssi;
    int8_t tmp[RSSI_WINDOW_SIZE];
    for (int i = 0; i < n; i++) tmp[i] = g_rssi_window[i];
    // Insertion sort (N is small: RSSI_WINDOW_SIZE ~ 10)
    for (int i = 1; i < n; i++) {
        int8_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    if (n & 1) {
        return (float)tmp[n / 2];
    }
    // Even count: average the two central samples (in dBm domain — standard for RSSI)
    return 0.5f * ((float)tmp[n / 2 - 1] + (float)tmp[n / 2]);
}

static void sendReport(float rssi_val, uint8_t tag_id, uint16_t samples) {
    // Skip clearly invalid RSSI (no packet / unset)
    if (rssi_val < -120.0f || rssi_val > 0.0f) return;

    anchor_report_t rep;
    memset(&rep, 0, sizeof(rep));
    rep.anchor_id        = (uint8_t)ANCHOR_ID;
    rep.avg_rssi         = rssi_val;
    rep.channel          = g_radio_channel;
    rep.timestamp_ms     = (uint32_t)millis();
    rep.tag_id           = tag_id;
    rep.sample_count     = samples;
    rep.calibration_mode = (uint8_t)CALIBRATION_MODE;

    if (WiFi.status() != WL_CONNECTED) return;

    if (!g_udp.beginPacket(LAPTOP_IP, LAPTOP_UDP_PORT)) {
#if DEBUG_VERBOSE
        Serial.printf("[A%d] UDP beginPacket failed\n", ANCHOR_ID);
#endif
        return;
    }
    g_udp.write((const uint8_t *)&rep, sizeof(rep));
    if (!g_udp.endPacket()) {
#if DEBUG_VERBOSE
        Serial.printf("[A%d] UDP endPacket failed\n", ANCHOR_ID);
#endif
    }

    Serial.printf("[A%d] RSSI=%5.1f dBm  n=%d  ch=%u%s\n",
                  ANCHOR_ID, rssi_val, samples, (unsigned)g_radio_channel,
                  CALIBRATION_MODE ? "  [CAL]" : "");
}

// Process one tag packet with a measured RSSI for that packet.
static void handleTagPacket(const uint8_t *mac_addr, const uint8_t *data, int len, int8_t rssi) {
    if (len < (int)sizeof(tag_broadcast_t)) return;

    // Learn tag MAC *before* RSSI validation. On Arduino-ESP32 2.x the only RSSI
    // source is MAC-filtered promiscuous mode, which cannot arm until the MAC is
    // known — validating RSSI first created a permanent chicken-and-egg deadlock
    // (rssi stays -127 → MAC never learned → promisc never filters).
    if (mac_addr) {
        memcpy(g_tag_mac, mac_addr, 6);
        g_tag_mac_known = true;
    }

    if (rssi < -120 || rssi > 0) return;

    tag_broadcast_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    g_last_rssi = rssi;

#if CALIBRATION_MODE
    sendReport((float)rssi, pkt.tag_id, 1);
#else
    // Sliding window: once full, report every packet (smooth + ~TX rate updates)
    g_rssi_window[g_win_index % RSSI_WINDOW_SIZE] = rssi;
    g_win_index++;
    if (g_win_filled < RSSI_WINDOW_SIZE) g_win_filled++;

    if (g_win_filled >= RSSI_WINDOW_SIZE) {
        sendReport(windowMedian(), pkt.tag_id, RSSI_WINDOW_SIZE);
    }
#endif
}

// Arduino-ESP32 3.x: recv info carries per-packet RSSI (no promiscuous pollution).
// Arduino-ESP32 2.x: fall back to MAC-filtered promiscuous RSSI.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data) return;
    int8_t rssi = -127;
    if (info->rx_ctrl) {
        rssi = info->rx_ctrl->rssi;
    } else if (g_tag_mac_known) {
        rssi = g_promisc_rssi;
    }
    handleTagPacket(info->src_addr, data, len, rssi);
}
#else
static void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    int8_t rssi = g_promisc_rssi;
    handleTagPacket(mac_addr, data, len, rssi);
}
#endif

static bool connectWifi() {
    Serial.println("  Scanning...");
    int n = WiFi.scanNetworks(false, true);
    uint8_t tgt_bssid[6] = {0};
    int     tgt_channel  = 0;
    bool    tgt_found    = false;

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
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    } else {
        Serial.printf("  Connecting via BSSID on ch%d", tgt_channel);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD, tgt_channel, tgt_bssid, true);
    }

    uint32_t wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 30000) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();
    return WiFi.status() == WL_CONNECTED;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("\n=== ANCHOR %d  [%s] ===\n",
                  ANCHOR_ID, CALIBRATION_MODE ? "CALIBRATION" : "NORMAL");
    Serial.printf("  Target SSID : \"%s\"\n", WIFI_SSID);

    Serial.setDebugOutput(true);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    // Arduino ESP32 3.x defaults _minSecurity to WIFI_AUTH_WPA2_PSK which causes
    // AUTH_EXPIRE on WPA/WPA2-Personal mixed-mode routers (TKIP cipher rejected).
    WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
    WiFi.mode(WIFI_STA);
    delay(100);

    if (!connectWifi()) {
        Serial.printf("FATAL: Wi-Fi failed (status=%d) after 30s\n", WiFi.status());
        delay(5000);
        ESP.restart();
    }

    // Stay on the AP channel. Forcing a different ESPNOW_CHANNEL breaks STA/UDP.
    g_radio_channel = (uint8_t)WiFi.channel();
    Serial.printf("  IP  : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  MAC : %s\n", WiFi.macAddress().c_str());
    Serial.printf("  Ch  : %u (AP)\n", (unsigned)g_radio_channel);
    if (g_radio_channel != (uint8_t)ESPNOW_CHANNEL) {
        Serial.printf("  WARNING: ESPNOW_CHANNEL=%d but AP is on ch %u.\n",
                      ESPNOW_CHANNEL, (unsigned)g_radio_channel);
        Serial.println("  Staying on AP channel so Wi-Fi/UDP keep working.");
        Serial.println("  Set ESPNOW_CHANNEL in config.h to the AP channel and reflash TAG.");
    }
    // Re-assert AP channel (do not hop away for ESP-NOW)
    esp_wifi_set_channel(g_radio_channel, WIFI_SECOND_CHAN_NONE);
    Serial.setDebugOutput(false);

    // Promiscuous used as RSSI fallback / Arduino 2.x path (MAC-filtered after first pkt)
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuousRxCB);

    if (esp_now_init() != ESP_OK) {
        Serial.println("FATAL: esp_now_init failed. Restarting.");
        ESP.restart();
    }
    esp_now_register_recv_cb(onDataRecv);
    // Local bind required by WiFiUDP on ESP32 before send; port is per-device so
    // LAPTOP_UDP_PORT is fine (each anchor has its own IP).
    g_udp.begin(LAPTOP_UDP_PORT);

    Serial.printf("[ANCHOR %d] Ready.\n\n", ANCHOR_ID);
}

void loop() {
    static uint32_t lastBeat = 0;
    static uint32_t downSince = 0;

    // Recover Wi-Fi if association drops (auto-reconnect + explicit retry)
    if (WiFi.status() != WL_CONNECTED) {
        if (downSince == 0) downSince = millis();
        if (millis() - downSince > 15000) {
            Serial.println("[WiFi] reconnecting...");
            WiFi.disconnect(false, false);
            delay(100);
            if (!connectWifi()) {
                Serial.println("[WiFi] reconnect failed — will retry");
            } else {
                g_radio_channel = (uint8_t)WiFi.channel();
                esp_wifi_set_channel(g_radio_channel, WIFI_SECOND_CHAN_NONE);
                esp_wifi_set_promiscuous(true);
                Serial.printf("[WiFi] reconnected IP=%s ch=%u\n",
                              WiFi.localIP().toString().c_str(),
                              (unsigned)g_radio_channel);
            }
            downSince = millis();  // backoff window
        }
    } else {
        downSince = 0;
        // Keep radio on AP channel if stack drifted
        uint8_t ch = (uint8_t)WiFi.channel();
        if (ch != 0 && ch != g_radio_channel) {
            g_radio_channel = ch;
            esp_wifi_set_channel(g_radio_channel, WIFI_SECOND_CHAN_NONE);
        }
    }

    if (millis() - lastBeat > 5000) {
        lastBeat = millis();
        Serial.printf("[A%d] alive  last_rssi=%d  wifi=%s  ch=%u\n",
                      ANCHOR_ID, g_last_rssi,
                      WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                      (unsigned)g_radio_channel);
    }
}

#endif // NODE_ROLE_ANCHOR
