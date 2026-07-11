#!/usr/bin/env python3
"""
Full automated test suite for multi-anchor coordinator.
Run: python test_full_suite.py
"""
from __future__ import annotations

import json
import math
import os
import struct
import sys
import tempfile
import time
import traceback
from copy import deepcopy

PASS = 0
FAIL = 0
ERRORS = []


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        msg = f"  FAIL  {name}" + (f" — {detail}" if detail else "")
        print(msg)
        ERRORS.append(msg)


def section(title):
    print(f"\n=== {title} ===")


# ---------------------------------------------------------------------------
# 1. Imports & compile
# ---------------------------------------------------------------------------
section("1. Module imports")
try:
    import site_config as site
    import server
    import calibration as cal
    check("import site_config", True)
    check("import server", True)
    check("import calibration", True)
except Exception as e:
    check("imports", False, str(e))
    traceback.print_exc()
    sys.exit(1)

# ---------------------------------------------------------------------------
# 2. site_config unit tests
# ---------------------------------------------------------------------------
section("2. site_config")

def_cfg = site._default_config()
check("default has 4 anchors", len(def_cfg["anchors"]) == 4)
check("default has 1 room 5x4",
      def_cfg["rooms"][0]["width"] == 5.0 and def_cfg["rooms"][0]["height"] == 4.0)
check("default min_anchors 3", def_cfg["min_anchors"] == 3)

ok, msg = site.validate_config(def_cfg)
check("validate default ok", ok, msg)

bad = deepcopy(def_cfg)
bad["anchors"].append({"id": 1, "x": 0, "y": 0})  # duplicate
ok, msg = site.validate_config(bad)
check("reject duplicate id", not ok)

bad2 = deepcopy(def_cfg)
bad2["anchors"][0]["id"] = 0
ok, msg = site.validate_config(bad2)
check("reject id 0", not ok)

bad3 = deepcopy(def_cfg)
bad3["anchors"][0]["id"] = 255
ok, msg = site.validate_config(bad3)
check("reject id 255", not ok)

bad4 = deepcopy(def_cfg)
bad4["rooms"][0]["width"] = 0.1
ok, msg = site.validate_config(bad4)
check("reject tiny room", not ok)

# 1.13 is closer to 1.25 than to 1.00 on a 0.25 grid
check("snap 1.13 @ 0.25", abs(site.snap(1.13, 0.25) - 1.25) < 1e-9)
check("snap 1.20 @ 0.25", abs(site.snap(1.20, 0.25) - 1.25) < 1e-9)
check("snap 1.05 @ 0.25", abs(site.snap(1.05, 0.25) - 1.0) < 1e-9)

aabb = site.floor_aabb(def_cfg)
check("aabb width > 5", aabb["width"] > 5.0)
check("aabb height > 4", aabb["height"] > 4.0)

# multi-room aabb
multi = deepcopy(def_cfg)
multi["rooms"].append({
    "id": "hall", "name": "Hall",
    "origin_x": 5.5, "origin_y": 0, "width": 3, "height": 4,
})
aabb2 = site.floor_aabb(multi)
check("multi-room aabb spans both", aabb2["max_x"] > 8.0)

# save/load roundtrip to temp (don't corrupt real config permanently —
# we'll restore after)
SITE_BAK = None
if os.path.exists(site.SITE_FILE):
    with open(site.SITE_FILE, encoding="utf-8") as f:
        SITE_BAK = f.read()

try:
    cfg = site.get()
    check("get() returns anchors", len(cfg.get("anchors", [])) >= 1)

    # path loss / anchor maps
    am = site.anchor_map(cfg)
    pl = site.path_loss_map(cfg)
    check("anchor_map keys match enabled", len(am) == len(site.enabled_anchors(cfg)))
    check("path_loss has A,n tuples", all(len(v) == 2 for v in pl.values()))

    view = site.public_view()
    check("public_view has aabb+config", "aabb" in view and "config" in view)

    # normalize snaps coords
    raw = deepcopy(def_cfg)
    raw["grid_snap_m"] = 0.5
    raw["anchors"][0]["x"] = 1.1
    norm = site.normalize_config(raw)
    check("normalize snaps x", abs(norm["anchors"][0]["x"] - 1.0) < 1e-9)

finally:
    pass  # restore later at end

# ---------------------------------------------------------------------------
# 3. Localization math
# ---------------------------------------------------------------------------
section("3. Localization math")

true = (2.5, 2.0)
anchors = [(0.0, 0.0), (5.0, 0.0), (5.0, 4.0), (0.0, 4.0)]
dists = [math.hypot(true[0] - a[0], true[1] - a[1]) for a in anchors]
bounds = (-0.5, -0.5, 5.5, 4.5)

pos = server.trilaterate(anchors, dists, bounds=bounds)
check("trilaterate 4 perfect", pos is not None and math.hypot(pos[0] - true[0], pos[1] - true[1]) < 1e-3,
      str(pos))

# N=3
pos3 = server.trilaterate(anchors[:3], dists[:3], bounds=bounds)
check("trilaterate 3 anchors", pos3 is not None and math.hypot(pos3[0] - true[0], pos3[1] - true[1]) < 0.05,
      str(pos3))

# N=2 should fail min
pos2 = server.trilaterate(anchors[:2], dists[:2], bounds=bounds)
check("trilaterate 2 returns None", pos2 is None)

# distance conversion
d = server.rssi_to_dist(-62.0, -61.3, 2.84)
check("rssi_to_dist ~1m", 0.8 < d < 1.3, f"d={d}")

dmax = server.rssi_to_dist(-120.0, -60.0, 2.7)
check("rssi_to_dist clamps max", dmax == server.MAX_DIST_M)

dmin = server.rssi_to_dist(-30.0, -60.0, 2.7)
check("rssi_to_dist clamps min", dmin >= server.MIN_DIST_M)

# noisy + robust
dists_bad = list(dists)
dists_bad[2] += 4.0  # A3 wrong
aids = [1, 2, 3, 4]
raw, ua, up, ud = server.robust_trilaterate(aids, anchors, dists_bad, bounds=bounds)
check("robust handles bad range", raw is not None, str(raw))
if raw:
    err = math.hypot(raw[0] - true[0], raw[1] - true[1])
    check("robust error < 1.5m with one bad", err < 1.5, f"err={err}")

# Kalman
k = server.Kalman2D()
t0 = time.time()
p1 = k.update((1.0, 1.0), t=t0)
p2 = k.update((1.1, 1.0), t=t0 + 0.2)
check("kalman returns 2-vector", len(p2) == 2)

# path-loss fit
fit = server.fit_path_loss([1, 2, 3, 4, 5], [-62, -68, -72, -76, -79])
check("fit_path_loss A finite", math.isfinite(fit["A"]))
check("fit_path_loss n in range", 0.5 < fit["n"] < 6)
check("fit rmse reasonable", fit["rmse_dB"] < 5)

# calibration module fit
fit2 = cal.fit_model([1, 2, 3, 4], [-61, -67, -71, -75])
check("cal.fit_model rmse", "rmse_dB" in fit2)

# ---------------------------------------------------------------------------
# 4. Room-aware selection
# ---------------------------------------------------------------------------
section("4. Room-aware selection")

cfg = site._default_config()
# all 4 in room_lab
coh = []
for aid, pos in [(1, (0, 0)), (2, (5, 0)), (3, (5, 4)), (4, (0, 4))]:
    coh.append({"aid": aid, "pos": pos, "dist": 2.0, "rssi": -55, "recv_ts": 0, "seq": 1})
sel, mode, rid = server.room_aware_select(coh, cfg)
check("4-in-room -> room mode", mode == "room", mode)
check("room_id set", rid in ("room1", "room_lab"), str(rid))
check("selected >= 3", len(sel) >= 3)

# only 2 anchors -> global
sel2, mode2, rid2 = server.room_aware_select(coh[:2], cfg)
check("2 anchors -> global", mode2 == "global", mode2)

# two rooms: room A has 3, room B has 1 — prefer A
cfg2 = deepcopy(cfg)
cfg2["rooms"].append({
    "id": "room_b", "name": "B", "origin_x": 6, "origin_y": 0, "width": 3, "height": 4
})
cfg2["anchors"].append({
    "id": 5, "name": "A5", "x": 7, "y": 1, "room_id": "room_b",
    "enabled": True, "A": -60, "n": 2.7
})
# only anchors 1,2,3 (room_lab) + 5 (room_b) with strong lab RSSI
coh3 = [
    {"aid": 1, "pos": (0, 0), "dist": 1.5, "rssi": -50, "recv_ts": 0, "seq": 1},
    {"aid": 2, "pos": (5, 0), "dist": 2.0, "rssi": -52, "recv_ts": 0, "seq": 1},
    {"aid": 3, "pos": (5, 4), "dist": 2.5, "rssi": -54, "recv_ts": 0, "seq": 1},
    {"aid": 5, "pos": (7, 1), "dist": 3.0, "rssi": -70, "recv_ts": 0, "seq": 1},
]
sel3, mode3, rid3 = server.room_aware_select(coh3, cfg2)
check("prefer room with >=3", mode3 == "room" and rid3 in ("room1", "room_lab"), f"{mode3}/{rid3}")
check("selected only lab anchors", all(c["aid"] != 5 for c in sel3), str([c["aid"] for c in sel3]))

# lab has only 2, room_b has 1 -> global (all 3)
coh4 = coh3[:2] + [coh3[3]]
sel4, mode4, rid4 = server.room_aware_select(coh4, cfg2)
check("no room has 3 -> global", mode4 == "global", mode4)
check("global uses all 3", len(sel4) == 3)

# ---------------------------------------------------------------------------
# 5. UDP struct packing
# ---------------------------------------------------------------------------
section("5. UDP protocol")

check("REPORT_SIZE 14", server.REPORT_SIZE == 14, str(server.REPORT_SIZE))
check("cal REPORT_SIZE 14", cal.REPORT_SIZE == 14)

# pack like C struct little-endian
pkt = struct.pack(server.REPORT_FMT, 3, -65.5, 8, 12345, 1, 10, 1)
aid, rssi, ch, ts, tid, sc, calm = struct.unpack(server.REPORT_FMT, pkt)
check("pack/unpack anchor_id", aid == 3)
check("pack/unpack rssi", abs(rssi + 65.5) < 0.01)
check("pack/unpack cal flag", calm == 1)

# ---------------------------------------------------------------------------
# 6. Flask API integration
# ---------------------------------------------------------------------------
section("6. Flask API integration")

# Isolate site file during API tests using a temp copy logic:
# server uses site.get() which is live — restore config at end

client = server.app.test_client()

pages = ["/", "/live", "/setup", "/calibrate"]
for p in pages:
    r = client.get(p)
    check(f"GET {p}", r.status_code == 200, str(r.status_code))

r = client.get("/api/config")
check("GET /api/config", r.status_code == 200 and r.get_json().get("ok"))
data = r.get_json()["data"]
check("config has rooms", "rooms" in data["config"])
check("config has anchors", "anchors" in data["config"])
check("aabb present", "aabb" in data)

r = client.post("/api/setup/default")
check("POST setup/default", r.status_code == 200 and r.get_json().get("ok"))
cfg = r.get_json()["data"]["config"]
check("default setup_complete", cfg.get("setup_complete") is True)
check("default 4 anchors", len(cfg["anchors"]) == 4)

r = client.post("/api/setup/custom", json={"reset": True})
check("POST setup/custom reset", r.status_code == 200 and r.get_json().get("ok"))
cfg = r.get_json()["data"]["config"]
check("custom reset 0 anchors", len(cfg["anchors"]) == 0)
check("custom has 1 room", len(cfg["rooms"]) == 1)

r = client.post("/api/rooms", json={
    "name": "Hall", "origin_x": 5.5, "origin_y": 0, "width": 3, "height": 4
})
check("POST room", r.status_code == 200 and r.get_json().get("ok"))
rooms = r.get_json()["data"]["config"]["rooms"]
check("now 2 rooms", len(rooms) == 2)
hall_id = [x["id"] for x in rooms if x["name"] == "Hall"][0]

r = client.post("/api/anchors", json={"id": 1, "x": 0, "y": 0, "room_id": rooms[0]["id"]})
check("POST anchor 1", r.get_json().get("ok"))
r = client.post("/api/anchors", json={"id": 2, "x": 5, "y": 0, "room_id": rooms[0]["id"]})
check("POST anchor 2", r.get_json().get("ok"))
r = client.post("/api/anchors", json={"id": 3, "x": 5, "y": 4, "room_id": rooms[0]["id"]})
check("POST anchor 3", r.get_json().get("ok"))
r = client.post("/api/anchors", json={"id": 1, "x": 1, "y": 1})  # duplicate
check("reject duplicate anchor", not r.get_json().get("ok"))

r = client.patch("/api/anchors/2", json={"x": 4.75, "y": 0.25, "enabled": True})
check("PATCH anchor", r.get_json().get("ok"))
a2 = [a for a in r.get_json()["data"]["config"]["anchors"] if a["id"] == 2][0]
# may be snapped to 0.25 grid
check("anchor moved near 4.75", abs(a2["x"] - 4.75) < 0.01 or abs(a2["x"] - 4.75) < 0.26, str(a2["x"]))

r = client.patch(f"/api/rooms/{hall_id}", json={"width": 3.5, "height": 4.0})
check("PATCH room", r.get_json().get("ok"))

r = client.delete("/api/anchors/3")
check("DELETE anchor 3", r.get_json().get("ok"))
check("3 anchors left? wait 2", len(r.get_json()["data"]["config"]["anchors"]) == 2)

# restore useful set for cal tests
client.post("/api/setup/default")

# Calibration API
r = client.post("/api/cal/start", json={"anchor_id": 1, "samples": 20, "accept_normal_mode": True})
check("cal start", r.get_json().get("ok"))
r = client.get("/api/cal/status")
check("cal status active", r.get_json()["data"]["active"] is True)

r = client.post("/api/cal/collect", json={"distance_m": 1.0, "samples": 20})
check("cal collect start", r.get_json().get("ok"))

# Inject samples via cal_session directly (simulates UDP)
with server.cal_lock:
    server.cal_session["samples"] = [-62.0 + 0.1 * i for i in range(20)]
r = client.post("/api/cal/commit_point")
check("cal commit point", r.get_json().get("ok"), str(r.get_json()))

# more points
for d, base in [(2.0, -68.0), (3.0, -72.0), (4.0, -76.0)]:
    client.post("/api/cal/collect", json={"distance_m": d, "samples": 20})
    with server.cal_lock:
        server.cal_session["samples"] = [base + 0.05 * i for i in range(20)]
    r = client.post("/api/cal/commit_point")
    check(f"commit {d}m", r.get_json().get("ok"))

r = client.post("/api/cal/fit")
check("cal fit", r.get_json().get("ok"), str(r.get_json()))
if r.get_json().get("ok"):
    fit = r.get_json()["data"]["fit"]
    check("fit has A,n", "A" in fit and "n" in fit)
    check("fit rmse finite", math.isfinite(fit["rmse_dB"]))

r = client.post("/api/cal/cancel")
check("cal cancel", r.get_json().get("ok"))

# PUT full config
full = site.get()
full["grid_snap_m"] = 0.5
r = client.put("/api/config", json={"config": full})
check("PUT config", r.get_json().get("ok"))

# invalid PUT
r = client.put("/api/config", json={"config": {"rooms": [], "anchors": [{"id": 99}]}})  # missing x,y
check("PUT invalid rejected", not r.get_json().get("ok"))

# fonts route (may 404 if missing file — check dir)
font_dir = server.FONTS_DIR
if os.path.isdir(font_dir):
    fonts = os.listdir(font_dir)
    check("fonts directory non-empty", len(fonts) > 0, str(fonts[:3]))
    if fonts:
        r = client.get("/fonts/" + fonts[0])
        check(f"GET font {fonts[0]}", r.status_code == 200, str(r.status_code))
else:
    check("fonts directory exists", False)

# path traversal blocked
r = client.get("/fonts/../server.py")
check("font path traversal blocked", r.status_code in (403, 404))

# static css/js
r = client.get("/static/css/theme.css")
check("static theme.css", r.status_code == 200)
r = client.get("/static/js/map.js")
check("static map.js", r.status_code == 200)

# ---------------------------------------------------------------------------
# 7. Simulated localization loop inputs
# ---------------------------------------------------------------------------
section("7. Simulated localization pipeline")

client.post("/api/setup/default")
cfg = site.get()
cfg["setup_complete"] = True
site.save(cfg)

# inject rssi_store
true = (2.0, 1.5)
with server.state_lock:
    server.rssi_store.clear()
    for a in site.enabled_anchors():
        ax, ay = a["x"], a["y"]
        d_true = math.hypot(true[0] - ax, true[1] - ay)
        A, n = a.get("A", -60), a.get("n", 2.7)
        # invert: RSSI = A - 10 n log10(d)
        rssi = A - 10 * n * math.log10(max(d_true, 0.1))
        server.rssi_store[a["id"]] = {
            "rssi": rssi, "recv_ts": time.time(), "channel": 8,
            "samples": 10, "tag_id": 1, "seq": a["id"], "cal_mode": 0,
        }

# one-shot of core pipeline (mirror localization_loop)
cfg = site.get()
amap = site.anchor_map(cfg)
plmap = site.path_loss_map(cfg)
aabb = site.floor_aabb(cfg)
bounds = server.bounds_tuple(aabb)
now = time.time()
candidates = []
with server.state_lock:
    snap = dict(server.rssi_store)
for aid, pos in amap.items():
    e = snap[aid]
    A, n = plmap[aid]
    d = server.rssi_to_dist(e["rssi"], A, n)
    candidates.append({
        "aid": aid, "pos": pos, "dist": d, "recv_ts": e["recv_ts"],
        "seq": e["seq"], "rssi": e["rssi"],
    })
check("pipeline candidates >= 3", len(candidates) >= 3, str(len(candidates)))
selected, mode, rid = server.room_aware_select(candidates, cfg)
aid_v = [c["aid"] for c in selected]
anch_v = [c["pos"] for c in selected]
dist_v = [c["dist"] for c in selected]
aid_v, anch_v, dist_v = server.filter_outliers(aid_v, anch_v, dist_v, bounds=bounds)
raw, aid_v, anch_v, dist_v = server.robust_trilaterate(aid_v, anch_v, dist_v, bounds=bounds)
check("pipeline produces fix", raw is not None, str(raw))
if raw:
    err = math.hypot(raw[0] - true[0], raw[1] - true[1])
    check("pipeline error < 0.5m (ideal RSSI)", err < 0.5, f"err={err} pos={raw}")

# stale rejection: old timestamps
with server.state_lock:
    for aid in list(server.rssi_store.keys()):
        server.rssi_store[aid]["recv_ts"] = time.time() - 10.0
candidates_stale = []
now = time.time()
with server.state_lock:
    snap = dict(server.rssi_store)
for aid, pos in amap.items():
    e = snap.get(aid)
    if not e or now - e["recv_ts"] > server.MAX_STALE_SEC:
        continue
    candidates_stale.append(aid)
check("stale data excluded", len(candidates_stale) == 0)

# unregistered id accepted into store via unpack path logic
with server.state_lock:
    server.rssi_store[99] = {
        "rssi": -70, "recv_ts": time.time(), "channel": 8,
        "samples": 1, "tag_id": 1, "seq": 999, "cal_mode": 0,
    }
    server.unregistered_seen[99] = time.time()
r = client.get("/api/config")
unreg = r.get_json()["data"].get("unregistered", {})
check("unregistered visible in API", "99" in unreg)

# ---------------------------------------------------------------------------
# 8. Large-N robust trilaterate smoke
# ---------------------------------------------------------------------------
section("8. Large-N (12 anchors)")

import random
random.seed(0)
true = (3.0, 2.0)
big_a = []
big_d = []
big_ids = []
for i in range(12):
    ax = (i % 4) * 1.5
    ay = (i // 4) * 1.5
    big_a.append((ax, ay))
    big_d.append(math.hypot(true[0] - ax, true[1] - ay) + random.uniform(-0.05, 0.05))
    big_ids.append(i + 1)
# corrupt two
big_d[5] += 5
big_d[8] += 4
b = (-1, -1, 8, 6)
raw, *_ = server.robust_trilaterate(big_ids, big_a, big_d, bounds=b)
check("12-anchor robust runs", raw is not None, str(raw))
if raw:
    err = math.hypot(raw[0] - true[0], raw[1] - true[1])
    check("12-anchor err < 2m", err < 2.0, f"err={err}")

# ---------------------------------------------------------------------------
# Restore site config
# ---------------------------------------------------------------------------
section("9. Restore site_config")
if SITE_BAK is not None:
    with open(site.SITE_FILE, "w", encoding="utf-8") as f:
        f.write(SITE_BAK)
    site.load(force_reload=True)
    check("restored site_config.json", True)
else:
    client.post("/api/setup/default")
    check("re-applied default layout", True)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print("\n" + "=" * 50)
print(f"RESULTS: {PASS} passed, {FAIL} failed, total {PASS+FAIL}")
if ERRORS:
    print("Failures:")
    for e in ERRORS:
        print(e)
print("=" * 50)
sys.exit(0 if FAIL == 0 else 1)
