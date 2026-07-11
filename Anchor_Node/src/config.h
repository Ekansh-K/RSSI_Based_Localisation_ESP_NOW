#pragma once
// ================================================================
//  config.h  — Edit ONLY this file to configure your deployment
// ================================================================

// Wi-Fi (all ESP32s connect to the same AP)
#define WIFI_SSID      "Any Wifi of ur choice to connect the anchor node to the laptop"
#define WIFI_PASSWORD  "Password of the wifi"

// Laptop IP + UDP port  (find laptop IP: ipconfig / ip addr)
#define LAPTOP_IP       "IP address of your laptop"  
#define LAPTOP_UDP_PORT 5005

// Tag settings
#define TAG_ID              1
#define TAG_TX_INTERVAL_MS  200   // broadcast every 200 ms (5 Hz)

// Anchor settings
// Sliding-window length for median RSSI. Once full, a report is sent on every
// new packet (~TAG_TX_INTERVAL_MS) using the last RSSI_WINDOW_SIZE samples.
#define RSSI_WINDOW_SIZE    10

// Path-loss model defaults  RSSI = A - 10*N*log10(d)
// Update A and N per anchor AFTER running calibration.py
#define DEFAULT_A   -60.0f    // RSSI at 1 m (dBm)
#define DEFAULT_N     2.7f    // path-loss exponent

// Channel: must match your Wi-Fi AP's channel 
#define ESPNOW_CHANNEL 8   

// Verbose debug output on Serial (0 = off, 1 = on)
#define DEBUG_VERBOSE 0