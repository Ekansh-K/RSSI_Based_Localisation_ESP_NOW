#pragma once
// ================================================================
//  config.h  — Edit ONLY this file to configure your deployment
// ================================================================

// Wi-Fi (all ESP32s connect to the same AP)
#define WIFI_SSID      "Deepakk"
#define WIFI_PASSWORD  "Radiant@426"

// Laptop IP + UDP port  (find laptop IP: ipconfig / ip addr)
#define LAPTOP_IP       "192.168.1.14"   // laptop IP on Deepakk network
#define LAPTOP_UDP_PORT 5005

// Tag settings
#define TAG_ID              1
#define TAG_TX_INTERVAL_MS  200   // broadcast every 200 ms (5 Hz)

// Anchor settings
// Window size: anchor averages this many packets before sending one report
// At 5 Hz, RSSI_WINDOW_SIZE=10 means one report every 2 seconds
#define RSSI_WINDOW_SIZE    10

// Path-loss model defaults  RSSI = A - 10*N*log10(d)
// Update A and N per anchor AFTER running calibration.py
#define DEFAULT_A   -60.0f    // RSSI at 1 m (dBm)
#define DEFAULT_N     2.7f    // path-loss exponent

// Channel: must match your Wi-Fi AP's channel (usually 1, 6, or 11)
#define ESPNOW_CHANNEL 8    // Deepakk broadcasts on ch=8

// Verbose debug output on Serial (0 = off, 1 = on)
#define DEBUG_VERBOSE 0