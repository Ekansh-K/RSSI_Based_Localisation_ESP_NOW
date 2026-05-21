# ESP32-C3 Multi-Mode RSSI Indoor Localization — Complete Project Reference

> **Status:** Implementation phase  
> **Team:** 1 person  
> **Hardware:** 5× ESP32-C3 DevKitM-1, 1× Raspberry Pi (coordinator), 1× Laptop (dev/calibration)  
> **Goal:** Compare BLE, Wi-Fi and ESP-NOW as RSSI sources for indoor position estimation, implement Kalman-filtered trilateration, and serve a live UI from the Raspberry Pi.

---

## 1. What We Are Trying to Achieve

The core research question is:

> **Which of the three ESP32-C3 radio modes — ESP-NOW, BLE advertising, or Wi-Fi — produces more stable and usable RSSI for indoor room-scale (≤ 10 m) localization, and how accurate is trilateration under each mode when obstacles are present?**

To answer this we build a complete working localization system using ESP-NOW first (because it is the simplest to control on two ESP32 boards and has the cleanest lab setup), then replicate the same experiment with BLE and Wi-Fi, and compare all three.

### What the system does

1. A **mobile tag** ESP32-C3 broadcasts packets periodically.
2. Four **fixed anchor** ESP32-C3 nodes receive the broadcast and each independently measure the RSSI of the received signal.
3. Each anchor converts the raw RSSI to an estimated distance using a pre-calibrated **path-loss model** (log-distance model).
4. A **coordinator** (Raspberry Pi in the final deployment, laptop during development) receives all four distance estimates via UDP and runs **trilateration** to compute a 2D (x, y) position.
5. The position estimate is fed into a **Kalman filter** to smooth out noise and tracking lag.
6. A **live web UI** hosted by the coordinator shows the room map, estimated position, trail, RSSI per anchor, and distance rings.

---

## 2. IoT Level Classification

This system is a **Level 5** IoT architecture.

| Level | Description | This Project |
|---|---|---|
| Level 1 | Single device, local storage/compute | ✗ |
| Level 2 | Single node, cloud storage | ✗ |
| Level 3 | Single node, cloud compute + storage | ✗ |
| Level 4 | Multiple nodes, cloud-based compute | ✗ |
| **Level 5** | **Multiple nodes + a physical coordinator device** | **✓** |
| Level 6 | Multiple nodes, cloud-based centralized controller | ✗ |

**Why Level 5:** There are 5 ESP32-C3 nodes (4 anchors + 1 tag). A dedicated physical coordinator (Raspberry Pi) manages all nodes, aggregates all RSSI data, runs trilateration and filtering, and serves the web UI — all locally with no cloud.

---

## 3. Hardware Roles

| Device | Quantity | Role | Powered by |
|---|---|---|---|
| ESP32-C3 DevKitM-1 | 4 | Fixed anchors at room corners | USB wall adapter or power bank |
| ESP32-C3 DevKitM-1 | 1 | Mobile tag (carried by person or object) | USB power bank |
| Raspberry Pi (3B/4/5) | 1 | Coordinator: UDP receiver, trilateration, Kalman filter, web UI | USB-C/microUSB |
| Laptop | 1 | Development, flashing code, calibration only | — |

> During development and calibration, the laptop acts as the coordinator. Once everything is working, you move `server.py` to the Raspberry Pi.

---

## 4. Technologies and Algorithms Implemented

### ESP-NOW (Phase 1 — Current)
- Tag broadcasts a compact struct every 200 ms.
- Anchors use **Wi-Fi promiscuous mode** to capture the RSSI (`rx_ctrl.rssi`) of every received frame, including ESP-NOW frames.
- Promiscuous mode is the only reliable way to read ESP-NOW frame RSSI on Arduino ESP-IDF.

### BLE Advertising (Phase 2 — Planned)
- Tag advertises a BLE beacon (using `esp_ble_gap_start_advertising`).
- Anchors scan continuously and extract `scan_rst.rssi` from `esp_ble_gap_cb_param_t`.
- No pairing or connection needed — BLE advertising is unidirectional broadcast.

### Wi-Fi Beacon / Scan (Phase 3 — Planned)
- Tag operates as a soft-AP broadcasting beacon frames.
- Anchors use `esp_wifi_scan_start` and `esp_wifi_scan_get_ap_records` to read the AP's RSSI.
- This replicates how commercial Wi-Fi fingerprinting systems work.

### Path-Loss Model (all modes)
The relationship between RSSI and distance follows the log-distance path-loss model:

```
RSSI (dBm) = A − 10 × n × log₁₀(distance_metres)
```

- **A** = RSSI measured at exactly 1 metre (calibrated per anchor, typically −55 to −70 dBm indoors)
- **n** = path-loss exponent (free space = 2.0; typical indoors = 2.5–3.5; cluttered = 3.0–4.5)
- Both A and n **must be calibrated independently for each anchor** because hardware variation, mounting position, reflections, and nearby objects all affect these values.

Inverting the model gives distance from RSSI:

```
distance = 10 ^ ((A − RSSI) / (10 × n))
```

### Trilateration
Given 3 or more anchors at known (x, y) positions and estimated distances d₁, d₂, d₃, the tag position is found by solving:

```
(x − x₁)² + (y − y₁)² = d₁²
(x − x₂)² + (y − y₂)² = d₂²
(x − x₃)² + (y − y₃)² = d₃²
```

The coordinator uses `scipy.optimize.least_squares` (Levenberg–Marquardt method) because the equations are nonlinear and noisy, and least-squares gives a robust best-fit solution rather than an exact (over-determined) algebraic solution.

### Kalman Filter (2D)
Raw trilateration output is very noisy (RSSI variance is typically ±3–8 dBm). A 2D constant-velocity Kalman filter smooths the position estimate:

- **State vector:** `[x, y, vx, vy]` (position + velocity)
- **Measurement:** `[x, y]` (raw trilateration result)
- **Process noise (Q):** How much the true position can change between updates
- **Measurement noise (R):** How noisy trilateration measurements are

The Kalman filter is implemented in `server.py` as a Python class `Kalman2D` using NumPy matrix operations.

### Outlier Rejection
Before trilateration, any anchor whose estimated distance deviates more than 40% from the median of all anchor distances is discarded. This removes clearly incorrect RSSI readings (hardware spikes, interfering packets).

---

## 5. Complete File Structure

```
├── Anchor_Node/
│   ├── src/
│   │   ├── main.cpp          
│   │   └── config.h          ← shared config goes here
│   ├── include/              ← leave empty
│   ├── lib/                  ← leave empty
│   ├── platformio.ini        ← anchor platformio.ini goes here
│   └── .gitignore
│
├── Tag_Node/
│   ├── src/
│   │   ├── main.cpp          
│   │   └── config.h          ← same config.h as above (copy it here too)
│   ├── include/              ← leave empty
│   ├── platformio.ini        ← tag platformio.ini goes here
│   └── .gitignore
│
└── coordinator/
    ├── calibration.py        ← already there ✓
    ├── server.py             ← already there ✓
    └── requirements.txt      ← already there ✓
```
---

## 6. What Each File Actually Does (Detailed)

### `Tag_Node OR Anchor_Node/src/config.h`
A single shared header included by both tag and anchor firmware. You edit this once and it applies when you flash any device. Contains:
- `WIFI_SSID` / `WIFI_PASSWORD` — for anchors to join the network (tag does not use these)
- `LAPTOP_IP` / `LAPTOP_UDP_PORT` — where anchors send UDP reports
- `TAG_ID` — identifies which tag is broadcasting (useful if you later test multiple tags)
- `TAG_TX_INTERVAL_MS` — how often the tag broadcasts (200 ms = 5 Hz)
- `RSSI_WINDOW_SIZE` — how many packets the anchor averages before sending one report
- `ESPNOW_CHANNEL` — must match your router's Wi-Fi channel (1, 6, or 11 typically)
- `DEFAULT_A` / `DEFAULT_N` — fallback path-loss parameters if calibration_results.json is missing

### `Tag_Node/src/main.cpp`
- Sets Wi-Fi mode to STA but does NOT connect to any AP — just needs the radio active for ESP-NOW
- Sets the channel to `ESPNOW_CHANNEL` so it matches anchors
- Registers broadcast peer (FF:FF:FF:FF:FF:FF)
- In the main loop: sends `tag_broadcast_t` struct every `TAG_TX_INTERVAL_MS` milliseconds
- Prints packet counter and uptime on Serial every 25 packets

### `Anchor_Node/src/main.cpp`
- Connects to Wi-Fi (so UDP to laptop works)
- Forces the Wi-Fi radio to stay on `ESPNOW_CHANNEL` after connecting
- Enables promiscuous mode with `esp_wifi_set_promiscuous(true)` — this fires an ISR on every received radio frame and stores the RSSI in `g_latest_rssi`
- Initialises ESP-NOW and registers the receive callback
- When a tag packet arrives: snapshots the latest RSSI, adds to circular window buffer
- In normal mode: when the window is full, computes mean and sends UDP report
- In calibration mode: sends a UDP report for every single packet immediately
- UDP report struct (14 bytes): `{anchor_id, avg_rssi, channel, timestamp_ms, tag_id, sample_count, calibration_mode}`

### `coordinator/calibration.py`
Run this **once per anchor**, with that anchor flashed in calibration mode.
- Opens a UDP socket listening on `LAPTOP_UDP_PORT`
- Shows a checklist (tag on, anchor in cal mode, tape measure ready, LOS clear)
- Walks you through placing the tag at each distance (1.0, 1.5, 2.0, 3.0, 4.0, 5.0 m)
- Collects 80 samples per distance point
- Fits path-loss model using both linear regression and `scipy.optimize.curve_fit`
- Prints RMSE (target < 3 dB, acceptable < 5 dB)
- Saves per-anchor `A` and `n` to `calibration_results.json`

### `coordinator/server.py`
Main runtime coordinator. Run this during actual localization.
- Loads `calibration_results.json` at startup (uses defaults if missing)
- UDP thread listens on `LAPTOP_UDP_PORT` and stores latest RSSI per anchor in `rssi_store`
- Localization loop runs at 5 Hz:
  1. For each anchor: check if data is fresh (< 1.5 s old)
  2. Convert RSSI to distance using calibrated A, n
  3. Run outlier rejection
  4. Run trilateration via `least_squares`
  5. Pass result through `Kalman2D` filter
  6. Emit `position_update` event over WebSocket to browser
- Flask app serves the HTML/JS web UI at port 8080
- Web UI: canvas room map, anchor circles, tag dot (blue = Kalman, yellow = raw), trail, RSSI cards

---

## 7. `platformio.ini` Environment Table

| Environment | Node | Mode | When to use |
|---|---|---|---|
| `tag` | Tag ESP32 | Normal | Flash tag for all experiments |
| `anchor1` | Anchor 1 | Normal | Flash after calibration, for real use |
| `anchor1_cal` | Anchor 1 | **Calibration** | Flash only during calibration of Anchor 1 |
| `anchor2` | Anchor 2 | Normal | Flash after calibration |
| `anchor2_cal` | Anchor 2 | **Calibration** | Flash only during calibration of Anchor 2 |
| `anchor3` | Anchor 3 | Normal | Flash after calibration |
| `anchor3_cal` | Anchor 3 | **Calibration** | Flash only during calibration of Anchor 3 |
| `anchor4` | Anchor 4 | Normal | Flash after calibration |
| `anchor4_cal` | Anchor 4 | **Calibration** | Flash only during calibration of Anchor 4 |

---

## 8. Complete Order of Execution

### PHASE 0 — Software Installation (one time)

**On your laptop:**

```bash
# Install VS Code from https://code.visualstudio.com
# Install PlatformIO IDE extension inside VS Code

# Install Python packages
pip install flask flask-socketio eventlet numpy scipy
```

**On Raspberry Pi (when you move to Pi later):**
```bash
sudo apt update && sudo apt install python3-pip -y
pip3 install flask flask-socketio eventlet numpy scipy
```

---

### PHASE 1 — Configuration (before touching any ESP32)

**1.1 Find your laptop's IP address on the local network:**

- Windows: open Command Prompt → `ipconfig` → look for `IPv4 Address`
- Mac/Linux: `ip addr` or `ifconfig` → look for `inet` under your Wi-Fi adapter

**1.2 Find your router's Wi-Fi channel:**

- Windows: `netsh wlan show interfaces` → look for `Channel`
- Mac: hold Option and click Wi-Fi icon → note the channel number

**1.3 Edit `src/config.h`:**
```cpp
#define WIFI_SSID        "YourNetworkName"
#define WIFI_PASSWORD    "YourPassword"
#define LAPTOP_IP        "192.168.1.XXX"   // your laptop's IP from step 1.1
#define LAPTOP_UDP_PORT  5005
#define TAG_ID           1
#define TAG_TX_INTERVAL_MS  200
#define RSSI_WINDOW_SIZE    10
#define ESPNOW_CHANNEL      6              // your router channel from step 1.2
```

**1.4 Measure your room and edit `coordinator/server.py`:**
```python
ANCHOR_POSITIONS = {
    1: (0.0, 0.0),    # Anchor 1 = origin corner
    2: (W,   0.0),    # Anchor 2 = same wall, other end
    3: (W,   H  ),    # Anchor 3 = opposite corner
    4: (0.0, H  ),    # Anchor 4 = remaining corner
}
ROOM_W = W   # room width in metres (e.g. 5.0)
ROOM_H = H   # room height in metres (e.g. 4.0)
```

---

### PHASE 2 — Anchor Physical Placement

- Place anchors near the corners of your room.
- Height: 1.5–2.0 m (above table height, below ceiling).
- Distance from wall: at least 10–20 cm — do not press against walls.
- Avoid: metal shelves, large screens, refrigerators, pillars directly next to anchor.
- Antenna pointing straight up for consistent radiation pattern.
- Measure and write down the exact (x, y) coordinates of each anchor relative to Anchor 1 (the origin).

---

### PHASE 3 — Flash the Tag

Connect Tag ESP32-C3 to laptop via USB. In PlatformIO terminal:

```bash
pio run -e tag -t upload
```

Open Serial Monitor (`pio device monitor -b 115200`). You should see:

```
=== TAG starting ===
  MAC : AA:BB:CC:DD:EE:FF
  TX  : every 200 ms
[TAG] Ready.
[TAG] #25  uptime=5000 ms
```

The tag is now broadcasting every 200 ms. You can leave it running.

---

### PHASE 4 — Calibration (Repeat for Each Anchor: 1, 2, 3, 4)

> Do this **one anchor at a time**. The anchor must be at its **final mounted position** before you calibrate it. Do not move the anchor after calibrating.

#### Step 4.1 — Flash anchor in CALIBRATION mode

For Anchor 1:
```bash
pio run -e anchor1_cal -t upload
```

Serial Monitor should show:
```
=== ANCHOR 1  [CALIBRATION] ===
  IP  : 192.168.1.XXX
  MAC : AA:BB:CC:DD:EE:FF
[ANCHOR 1] Ready.
```

#### Step 4.2 — Run calibration script on laptop

```bash
python coordinator/calibration.py --anchor 1
```

The script prints a checklist. Verify each item:
- [ ] Anchor 1 is powered and shows Wi-Fi connected on Serial
- [ ] Tag is powered and broadcasting (see tag Serial Monitor)
- [ ] Tape measure is on the floor from anchor position outward
- [ ] Floor marks at 1.0, 1.5, 2.0, 3.0, 4.0, 5.0 m
- [ ] Clear line-of-sight — no furniture or people between tag and anchor
- [ ] Tag held at the same height as the anchor, antenna vertical

Press ENTER to begin. For each distance it prompts you, place the tag at that mark and press ENTER. The script collects 80 samples (about 20 seconds at 5 Hz with window averaging off).

#### Step 4.3 — Interpret the output

```
  Dist    Measured    Predicted   Err
  1.0m     -62.40      -62.40    0.00 dB
  1.5m     -66.23      -65.90    0.33 dB
  2.0m     -69.35      -68.91    0.44 dB
  3.0m     -74.11      -73.36    0.75 dB
  4.0m     -77.82      -77.28    0.54 dB
  5.0m     -80.95      -80.56    0.39 dB

  Curve fit : A = -62.4 dBm,  n = 2.81
  RMSE: 1.43 dB  ← GOOD (target < 3 dB)
```

| RMSE range | Meaning | Action |
|---|---|---|
| < 3 dB | Excellent | Proceed |
| 3–5 dB | Acceptable | Proceed or re-collect at noisy distances |
| > 5 dB | Poor | Clear obstacles, redo measurements |

#### Step 4.4 — Reflash anchor in NORMAL mode

```bash
pio run -e anchor1 -t upload
```

Serial Monitor confirms normal mode is running.

#### Step 4.5 — Repeat for anchors 2, 3, 4

```bash
# Anchor 2
pio run -e anchor2_cal -t upload
python coordinator/calibration.py --anchor 2
pio run -e anchor2 -t upload

# Anchor 3
pio run -e anchor3_cal -t upload
python coordinator/calibration.py --anchor 3
pio run -e anchor3 -t upload

# Anchor 4
pio run -e anchor4_cal -t upload
python coordinator/calibration.py --anchor 4
pio run -e anchor4 -t upload
```

`calibration_results.json` is updated after each run and keeps all 4 anchors' data.

---

### PHASE 5 — Run the System (Laptop as Coordinator)

All anchors flashed in normal mode. All calibrated. Tag broadcasting. Then on laptop:

```bash
python coordinator/server.py
```

Output:
```
[CAL] Anchor 1: A=-62.4  n=2.81
[CAL] Anchor 2: A=-61.9  n=2.76
[CAL] Anchor 3: A=-63.1  n=2.90
[CAL] Anchor 4: A=-62.7  n=2.85
[UDP] Listening on :5005
[Server] http://localhost:8080
```

Open browser → `http://localhost:8080`

---

### PHASE 6 — Move Coordinator to Raspberry Pi

Once the laptop system is fully working:

1. Copy the `coordinator/` folder to the Pi (via USB drive, scp, or git).
2. On the Pi: `pip3 install flask flask-socketio eventlet numpy scipy`
3. Change `LAPTOP_IP` in `config.h` to the Pi's IP address.
4. Reflash all anchors with the new `LAPTOP_IP`.
5. On Pi: `python3 coordinator/server.py`
6. Access the UI from any device on the same network: `http://[PI_IP]:8080`

The Raspberry Pi is now the permanent Level-5 coordinator.

---

### PHASE 7 — Multi-Mode Comparison Experiments

#### ESP-NOW (current — already done in Phase 5)
Record: RSSI stability (σ), localization error vs ground truth, power draw.

#### BLE Advertising (Phase 2)
Change tag firmware to use BLE `esp_ble_gap_start_advertising`. Change anchor firmware to use `esp_ble_gap_start_scanning` and record `scan_rst.rssi` in the scan callback. Re-run calibration, re-run localization, record same metrics.

#### Wi-Fi Beacon (Phase 3)
Change tag to soft-AP mode. Change anchor to run `esp_wifi_scan_start` periodically and extract RSSI from `esp_wifi_scan_get_ap_records`. Repeat.

#### What to compare across modes

| Metric | How to measure |
|---|---|
| RSSI variance (σ) | Collect 200 samples at fixed distance, compute std dev |
| Calibration RMSE | From calibration.py output |
| Mean localization error | Stand at known positions, compare estimated vs true |
| Effect of obstacles | Repeat with a person standing between tag and one anchor |
| Power consumption | Measure tag current with USB meter or INA219 |

---

## 9. Antenna Orientation Effect

Research into ESP32-C3 antenna directivity has shown up to **10 dB RSSI variation** depending on whether the tag antenna is pointing toward or away from the anchor, and this varies with distance.

**To mitigate this:**
- During calibration: tag antenna always vertical (same as normal use).
- During deployment: tag should be worn or held consistently (e.g. in a pocket with antenna up).
- Future improvement: average RSSI from multiple orientations or use antenna diversity.

---

## 10. Anchor Placement Guidelines

| Rule | Reason |
|---|---|
| Near room corners (not in the middle of walls) | Maximises geometric diversity for trilateration accuracy |
| 10–30 cm away from the wall surface | Reduces reflections from the wall directly behind the antenna |
| 1.5–2.0 m above floor | Above furniture level, below ceiling — avoids ground reflections and ceiling bounce |
| Avoid metal objects within 0.5 m | Metal causes severe multipath and RSSI distortion |
| All anchors at same height | Makes the 2D planar model valid — mixing heights introduces 3D distance error into the 2D model |
| Antenna pointing straight up | Most symmetric radiation pattern horizontally |

---

## 11. Calibration Rules Summary

1. Anchor must be at its **final mounted position** before calibrating — do not move it after.
2. **Tag must be in direct line-of-sight** at the same height as the anchor.
3. **Clear the path** — no person, furniture, or object between tag and anchor during a distance burst.
4. **Tag antenna vertical**, same as operational orientation.
5. Use at least **6 distance points** spread from 1 m to the full expected room range.
6. Collect at least **80 samples** per distance.
7. **Mark floor positions with tape** for repeatable placement.
8. Target calibration **RMSE < 3 dB** per anchor.
9. Run calibration in **the same environment** the system will operate in — don't calibrate in a hallway and deploy in a room.

---

## 12. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Anchor Serial: `Wi-Fi failed` | Wrong SSID/password | Re-check `config.h` |
| No UDP received on laptop | Wrong `LAPTOP_IP` | Run `ipconfig`/`ip addr`, update `config.h`, reflash |
| RSSI stuck at −127 | Promiscuous mode not receiving | Ensure `ESPNOW_CHANNEL` matches router; try reboot |
| Position jumps wildly | Fewer than 3 anchors active | Check all 4 are powered, Wi-Fi connected, sending UDP |
| Calibration RMSE > 5 dB | Multipath or obstacles during collection | Clear path, ensure LOS, redo |
| `calibration.py` times out | Anchor not in CAL mode | Flash with `anchor1_cal` env |
| Position always at room corner | Wrong anchor coordinates in `server.py` | Re-measure and update `ANCHOR_POSITIONS` |
| UI loads but no dot appears | Browser WebSocket blocked | Open `http://localhost:8080` not `https://` |
| Tag not detected | Tag on wrong channel | Check `ESPNOW_CHANNEL` matches in `config.h` |

---

## 13. What Is NOT Yet Implemented (Planned)

- [ ] BLE advertising mode on tag + BLE scan RSSI on anchors
- [ ] Wi-Fi beacon mode on tag + Wi-Fi scan on anchors
- [ ] Multi-tag support (distinguish multiple tags by tag_id)
- [ ] Z-axis (height) estimation
- [ ] RSSI fingerprinting as an alternative to path-loss model
- [ ] Data logging to CSV for offline analysis
- [ ] Automatic channel detection / matching

---

## 14. Key Terms Reference

| Term | Meaning |
|---|---|
| RSSI | Received Signal Strength Indicator — signal power at receiver in dBm |
| dBm | Decibel-milliwatts. More negative = weaker signal (−30 dBm excellent, −90 dBm very weak) |
| Path-loss exponent (n) | How quickly signal weakens with distance. Free space = 2.0; indoors cluttered = 3–4 |
| A (path-loss intercept) | RSSI expected at exactly 1 m from the transmitter |
| Trilateration | Using 3+ known distances to calculate 2D position (like GPS) |
| Kalman filter | Recursive estimator that smooths noisy position updates using a motion model |
| ESP-NOW | Espressif proprietary connectionless 2.4 GHz protocol — no AP needed |
| Promiscuous mode | Wi-Fi mode where the radio delivers every received frame to the CPU, not just addressed ones |
| Coordinator | The device that collects all sensor data and runs the position algorithm |
| Level 5 IoT | Architecture with multiple nodes + a physical coordinator — what this project implements |

