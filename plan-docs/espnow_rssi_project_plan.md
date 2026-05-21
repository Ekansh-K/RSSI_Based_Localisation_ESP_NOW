# ESP-NOW RSSI Localization Project Plan and Best-Method Survey

## 1. Chosen Direction for the Project

This project will implement an **indoor localization system using ESP32/ESP32-C3 boards, ESP-NOW, and RSSI**. The focus is on:

- Comparing RSSI-based distance estimation behavior across **ESP-NOW, Wi‑Fi, and BLE** modes.
- Implementing a **Level 5 IoT architecture**: multiple ESP32-C3 anchors + Raspberry Pi coordinator + web UI.
- Using **RSSI-based trilateration** with filtering and calibration, informed by prior work on BLE/Wi‑Fi RSSI localization.[cite:520][cite:530][cite:592]

Core choices:

- **Transport for main experiments:** ESP-NOW (controllable, low overhead, no AP required).[cite:555]
- **Anchors:** 4 fixed ESP32-C3 nodes (battery powered) placed around the area.
- **Tag:** 1 mobile ESP32-C3 node broadcast-only.
- **Coordinator:** Raspberry Pi collecting RSSI, running distance model + trilateration + web UI.

This setup is then used to **compare**:

- ESP-NOW RSSI vs BLE RSSI vs Wi‑Fi RSSI for indoor distance estimation and localization.

---

## 2. Best Methods and Results in Existing Work

Below is a synthesis of the most relevant GitHub projects and papers reviewed, and what they achieved.

### 2.1 GitHub Projects

#### A. Indoor Positioning System Using Bluetooth RSSI and Trilateration (ESP32‑S3)[cite:520]

- Project: https://github.com/avibn/indoor-positioning-trilateration  
- Hardware: 3 ESP32‑S3 Feather boards as BLE receivers; external BLE device as target.  
- Methodology:
  - Collect BLE RSSI from multiple beacons via MQTT.
  - Apply **Kalman filtering** to RSSI to reduce noise.
  - Use **least-squares trilateration** to compute 2D position.
  - Real-time visualization with Matplotlib.
- Reported outcome:
  - Qualitative description: Kalman filter **significantly smooths** RSSI; trilateration shows reasonable path tracking in a small indoor area.[cite:520]
  - Exact numerical error is not clearly emphasized in the README but trajectories indicate **meter-level accuracy** consistent with BLE RSSI literature.[cite:592]
- Why it is strong:
  - Uses a **clean, reproducible pipeline** very similar to what this project plans: RSSI → filter → path-loss/trilateration → visualization.
  - Demonstrates the benefit of **Kalman filtering on raw RSSI**, not just moving averages.

#### B. Indoor WiFi Localization in ESP32 Using Machine Learning[cite:572]

- Project: https://github.com/joaocarvalhoopen/Indoor_WiFi_Localization_in_ESP32_using_Machine_Leaning  
- Methodology:
  - ESP32 collects Wi‑Fi RSSI fingerprints from multiple APs.
  - Use ML (e.g., k‑NN / classifiers) on the PC to map RSSI vectors → (x, y).
- Reported outcome:
  - Fingerprinting method yields **better robustness** than pure trilateration, as widely seen in Wi‑Fi literature.[cite:530][cite:593]
- Relevance:
  - Shows a **fingerprint / ML alternative** to analytical trilateration.
  - More complex than needed for the first version of an ESP‑NOW-focused project.

#### C. ESP32 iBeacon Indoor Positioning Demo[cite:577][cite:579]

- Project: https://github.com/simonbogh/ESP32-iBeacon-indoor-positioning  
- Blog: https://www.beaconzone.co.uk/blog/indoor-positioning-using-ibeacon-and-esp32/  
- Methodology:
  - ESP32 boards receive iBeacon packets, log RSSI.
  - Basic trilateration / zone detection for indoor positioning.
- Reported outcome:
  - Demonstrates **zone-level accuracy** (identify which room / area a tag is in), but not high precision.[cite:579][cite:530]
- Relevance:
  - Confirms that **RSSI trilateration with simple filtering** is sufficient for room/zone‑level localization, not sub‑meter accuracy.

### 2.2 Academic Papers

#### A. BLE Indoor Positioning System Using RSSI‑based Trilateration (JOWUA, 2020)[cite:592]

- Paper: https://jowua.com/article/jowua-v11n3-3/70444/  
- Methodology:
  - BLE beacons; mobile device receives RSSI.
  - Use **log-distance path loss** model.
  - Apply **improvements** such as RSSI averaging and environment‑specific calibration.
- Reported results:
  - Achieves **sub‑2 m average error** in many test conditions with proper calibration and smoothing.[cite:592]
  - Confirms that carefully calibrated RSSI + trilateration can reach **acceptable precision** for many indoor use cases.
- Key ideas to reuse:
  - Environment‑specific calibration of path loss parameters (A, n).
  - Combining multiple beacons (3–4) with smoothing to offset RSSI noise.

#### B. BLE Indoor Localization based on Improved RSSI and Trilateration (2023)[cite:584]

- Info: https://ouci.dntb.gov.ua/en/works/4Ey5eBO4/  
- Methodology:
  - Proposes **improved RSSI-distance modeling** and error compensation.
  - Uses multiple measurements per position and outlier rejection.
- Reported results:
  - Improved error vs. classic log-distance model, especially in cluttered indoor spaces.[cite:584]
- Relevance:
  - Supports using **enhanced path loss model / error compensation** instead of a single simple formula.

#### C. Industry Overview: RSSI-Based Method in Indoor Asset Tracking (Navigine, 2024)[cite:530]

- Article: https://navigine.com/blog/rssi-based-method-in-indoor-asset-tracking/  
- Key points:
  - RSSI is generally limited to **zone-level accuracy (several meters)** unless heavily engineered.[cite:530]
  - Describes **log-distance path loss model** and calibration of parameters A and B across the environment.[cite:530]
  - Suggests combining RSSI with probabilistic methods (particle filters) or more advanced techniques (AoA) to improve accuracy.[cite:530]
- Relevance:
  - Provides a realistic ceiling of what can be expected from pure RSSI.
  - Motivates using **simple but smart filtering and calibration**, not over‑promising sub‑meter precision.

---

## 3. Best Methods to Borrow for Our ESP‑NOW Project

From the survey above, the most effective and realistic techniques for an ESP‑NOW RSSI localization project are:

### 3.1 Must‑Have Elements

1. **Log-distance path loss model with calibration**[cite:530][cite:592]
   - Use: \( d = 10^{(A - RSSI) / (10n)} \) (A = RSSI at 1 m, n = path loss exponent).  
   - Calibrate A and n per environment by measuring RSSI at known distances.

2. **Sliding window averaging of RSSI**[cite:520][cite:592]
   - Average 10–20 recent RSSI samples per anchor before converting to distance.
   - Strongly reduces fast fading and packet‑to‑packet noise.

3. **Overdetermined trilateration with 4 anchors**[cite:533][cite:592]
   - Use 4 anchors in a roughly rectangular layout.
   - Solve position using **least-squares** (not closed‑form intersection of 3 circles), to average out distance errors.

### 3.2 Strong “Nice-to-Have” Improvements

1. **Kalman filtering on distance or position**[cite:520]
   - Similar to the GitHub BLE project, apply a **1D Kalman filter** to each anchor’s estimated distance, or a **2D Kalman filter** to the (x, y) position.
   - This smooths trajectories and handles temporary RSSI spikes.

2. **Multi-channel averaging for ESP‑NOW** (channel hopping)[cite:492]
   - Sequentially send ESP‑NOW packets on channels 1, 6, 11.  
   - Measure RSSI on each, then average to reduce multipath and orientation‑dependent fading.

3. **Orientation‑aware deployment**[cite:530]
   - Fix anchor orientation (ceiling/wall mount).  
   - Keep tag orientation approximately consistent, or average over movement.

4. **BLE/Wi‑Fi comparison mode**[cite:530][cite:592]
   - For the “multi‑mode” goal, repeat experiments with:
     - BLE advertising + RSSI, and
     - Wi‑Fi RSSI from an AP or from packet sniffer.  
   - Use the **same calibration and trilateration pipeline** to compare modes directly.

---

## 4. Recommended Methodology for the ESP‑NOW Project

### 4.1 Hardware Setup

- **Anchors:** 4 ESP32‑C3 nodes, fixed at known (x, y) positions, all battery-powered.
- **Tag:** 1 ESP32‑C3 node, mobile, periodically broadcasting ESP‑NOW frames.
- **Coordinator:** Raspberry Pi, Wi‑Fi connected to all anchors, running data collection, calibration, and localization.

Total: **5 ESP32‑C3 nodes + 1 Raspberry Pi.**

### 4.2 Data Flow

1. Tag broadcasts ESP‑NOW packets (optionally with channel hopping across 1/6/11).
2. Each anchor:
   - Receives packets.
   - Extracts RSSI (via promiscuous callback).
   - Averages RSSI over a small window.
   - Sends (anchor_id, averaged_RSSI, channel, timestamp) to Raspberry Pi via UDP/MQTT.
3. Raspberry Pi:
   - Aggregates per‑anchor RSSI.
   - Converts to distance using calibrated log-distance model.
   - Runs least-squares trilateration with 4 anchors.
   - Applies optional Kalman filter to smooth (x, y) over time.
   - Serves live coordinates through a local web UI.

### 4.3 Calibration Procedure

1. Choose a calibration line or grid in the real room.
2. Place tag at known distances (1 m, 2 m, 3 m, 4 m, 5 m) from one anchor, roughly centered relative to others.
3. For each distance:
   - Collect 100+ RSSI samples from each anchor.
   - Compute mean RSSI per anchor.
4. Fit A and n (per anchor or globally) by minimizing error between model distance and actual distance.[cite:530][cite:592]
5. Store calibrated (A, n) in Raspberry Pi configuration.

### 4.4 Algorithms to Use

1. **RSSI Filtering per Anchor**
   - Sliding window average of RSSI (N ≈ 10–20).
   - Optionally, a simple 1D Kalman filter, as in the BLE GitHub project.[cite:520]

2. **Distance Estimation**
   - Use log-distance model with calibrated A and n.[cite:530][cite:592]

3. **Trilateration**
   - Use 4 anchors and solve via least-squares (e.g., linearization → normal equations or direct library solver).
   - If one anchor’s distance is clearly inconsistent (too large residual), down‑weight or exclude it in that time step.

4. **Position Smoothing**
   - Apply a 2D Kalman filter or exponential moving average on (x, y).

5. **Comparison Mode**
   - Repeat same pipeline for BLE and Wi‑Fi modes (only the RSSI source changes), to compare:
     - Noise/variance of RSSI.
     - Average localization error (measured against ground truth points).

---

## 5. What to Highlight in the Report

Based on the survey of existing projects and papers:

- The **BLE project with Kalman + trilateration** (GitHub) and the **JOWUA BLE trilateration paper** represent the most relevant and practical best methods for RSSI-based indoor localization.[cite:520][cite:592]
- They consistently show that:
  - Raw RSSI → bad;  
  - Filtered RSSI + calibrated path-loss model + trilateration with 3–4 anchors → **meter‑level accuracy** in typical rooms.[cite:520][cite:530][cite:592]
- Your ESP‑NOW system should therefore:
  - Use the **same proven structure** (filter → distance model → trilateration → smoothing).
  - Add ESP‑NOW‑specific advantages (no AP, lower overhead, channel hopping) as an extra contribution.
  - Provide **quantitative comparison** with BLE/Wi‑Fi RSSI on the same setup, which is rarely done in a single student project.

This combination of **well‑known algorithms** (log-distance + least squares + Kalman) with **ESP‑NOW transport and multi‑mode comparison** is both realistic to implement and strong enough for a research‑quality project.
