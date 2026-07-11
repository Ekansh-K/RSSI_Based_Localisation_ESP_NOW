#!/usr/bin/env python3
"""
server.py — Dynamic multi-anchor / multi-room coordinator + web dashboard.

Run:  pip install -r requirements.txt
      python server.py
Open: http://localhost:8080
"""
from __future__ import annotations

import math
import os
import struct
import threading
import time
from collections import defaultdict
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from flask import Flask, jsonify, request, send_file, send_from_directory
from flask_socketio import SocketIO
from scipy.optimize import curve_fit, least_squares

import site_config as site

# ================================================================
UDP_PORT = 5005
WEB_PORT = 8080
MAX_STALE_SEC = 3.0
MAX_COHERENCE_SEC = 1.2
MIN_DIST_M = 0.10
# Soft cap for a single range; raised for multi-room / large floors.
# Also re-derived from layout AABB diagonal each fix (see max_range_m).
MAX_DIST_M = 50.0
OUTLIER_ABS_M = 1.2
MAX_SPEED_MPS = 3.0
LOC_POLL_SEC = 0.1
DEFAULT_DISTANCES = [1.0, 2.0, 3.0, 4.0, 5.0]  # 1 m steps from 1 m
# Real-time multilateration: use strongest K ranges when N is huge (UI still
# shows all coherent distances). 24 is enough for geometry; keeps CPU bounded
# for N up to the protocol limit (~254).
MAX_FIX_ANCHORS = 24
# Leave-one-out outlier search is O(N) NLS solves — only for small N.
OUTLIER_LOO_MAX_N = 12
ROBUST_LOO_MAX_N = 16
# ================================================================

REPORT_FMT = '<BfBIBHB'
REPORT_SIZE = struct.calcsize(REPORT_FMT)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, 'static')
FONTS_DIR = os.path.join(STATIC_DIR, 'fonts')
state_lock = threading.Lock()
rssi_store: Dict[int, dict] = {}
_report_seq = 0
unregistered_seen: Dict[int, float] = {}  # aid -> last_seen

# In-dashboard calibration session
cal_lock = threading.Lock()
cal_session: Dict[str, Any] = {
    'active': False,
    'anchor_id': None,
    'phase': 'idle',  # idle | checklist | collecting | done
    'distance_m': None,
    'target_samples': 80,
    'samples': [],  # current distance buffer
    'points': {},   # distance -> {mean, std, n, samples}
    'message': '',
    'accept_normal_mode': True,  # also accept cal==0 for easier UX
}


# ---------------------------------------------------------------------------
# Path-loss / multilateration / Kalman (dynamic bounds)
# ---------------------------------------------------------------------------

def rssi_to_dist(rssi, A, n):
    if (not math.isfinite(rssi) or not math.isfinite(A) or not math.isfinite(n)
            or n <= 0.0):
        return MAX_DIST_M
    d = 10.0 ** ((A - rssi) / (10.0 * n))
    if not math.isfinite(d) or d <= 0.0:
        return MAX_DIST_M
    return min(max(d, MIN_DIST_M), MAX_DIST_M)


def _range_weights(distances):
    return [1.0 / max(d, MIN_DIST_M) for d in distances]


def _rms_range_residual(xy, anchors, distances):
    if not anchors:
        return float('inf')
    err = [abs(math.hypot(xy[0] - ax, xy[1] - ay) - d)
           for (ax, ay), d in zip(anchors, distances)]
    return math.sqrt(sum(e * e for e in err) / len(err))


def max_range_m(aabb=None):
    """Max usable range: layout diagonal * 1.25, floored/capped reasonably."""
    if aabb and aabb.get('width') and aabb.get('height'):
        diag = math.hypot(float(aabb['width']), float(aabb['height']))
        return min(max(diag * 1.25, 10.0), 200.0)
    return MAX_DIST_M


def trilaterate(anchors, distances, bounds=None, weights=None):
    """
    Weighted NLS multilateration.
    bounds = (min_x, min_y, max_x, max_y) or None.

    Cost gate is *mean* residual based so it does not reject large-N fits
    (scipy cost = 0.5 * sum r_i^2 grows with N).
    """
    anchors = list(anchors)
    distances = list(distances)
    n = len(anchors)
    min_a = site.MIN_ANCHORS
    if n < min_a:
        return None
    # Clamp invalid/zero ranges instead of aborting (tag can sit on an anchor)
    distances = [
        MIN_DIST_M if (not math.isfinite(d) or d <= 0.0) else float(d)
        for d in distances
    ]
    if weights is None:
        weights = _range_weights(distances)

    def residuals(p):
        return [w * (math.hypot(p[0] - ax, p[1] - ay) - d)
                for (ax, ay), d, w in zip(anchors, distances, weights)]

    wsum = sum(weights)
    if wsum <= 0.0:
        x0 = [sum(a[0] for a in anchors) / n,
              sum(a[1] for a in anchors) / n]
    else:
        x0 = [sum(w * a[0] for a, w in zip(anchors, weights)) / wsum,
              sum(w * a[1] for a, w in zip(anchors, weights)) / wsum]

    if bounds is None:
        lo = [-200.0, -200.0]
        hi = [200.0, 200.0]
    else:
        min_x, min_y, max_x, max_y = bounds
        # Degenerate bounds crash TRF; expand slightly
        if not (math.isfinite(min_x) and math.isfinite(max_x)
                and math.isfinite(min_y) and math.isfinite(max_y)):
            lo, hi = [-200.0, -200.0], [200.0, 200.0]
        else:
            if max_x - min_x < 0.5:
                mid = 0.5 * (min_x + max_x)
                min_x, max_x = mid - 0.25, mid + 0.25
            if max_y - min_y < 0.5:
                mid = 0.5 * (min_y + max_y)
                min_y, max_y = mid - 0.25, mid + 0.25
            lo = [min_x, min_y]
            hi = [max_x, max_y]
            x0[0] = min(max(x0[0], min_x), max_x)
            x0[1] = min(max(x0[1], min_y), max_y)

    try:
        res = least_squares(
            residuals, x0, method='trf',
            bounds=(lo, hi),
            max_nfev=300,
        )
        # Mean squared residual (un-normalized by weight for gate): use cost/n
        # cost = 0.5 * sum r^2  =>  mean r^2 ≈ 2*cost/n
        mean_sq = (2.0 * float(res.cost) / n) if n else float('inf')
        if mean_sq > 25.0:  # RMS weighted residual ≳ 5 m — absurd
            return None
        if not (math.isfinite(res.x[0]) and math.isfinite(res.x[1])):
            return None
        return float(res.x[0]), float(res.x[1])
    except Exception as e:
        print(f"[TRILAT] {e}")
        return None


class Kalman2D:
    def __init__(self, q_acc=0.8, r_meas=0.8):
        self.q_acc = float(q_acc)
        self.R = np.eye(2) * float(r_meas)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        self.x = np.zeros(4)
        self.P = np.eye(4) * 10.0
        self.ok = False
        self.last_t = None

    def _F_Q(self, dt):
        dt = float(max(min(dt, 5.0), 1e-3))
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        q = self.q_acc
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = np.zeros((4, 4))
        Q[0, 0] = dt4 / 4 * q
        Q[0, 2] = dt3 / 2 * q
        Q[2, 0] = dt3 / 2 * q
        Q[2, 2] = dt2 * q
        Q[1, 1] = dt4 / 4 * q
        Q[1, 3] = dt3 / 2 * q
        Q[3, 1] = dt3 / 2 * q
        Q[3, 3] = dt2 * q
        return F, Q

    def update(self, xy, t=None):
        z = np.asarray(xy, dtype=float)
        t = time.time() if t is None else float(t)
        if not self.ok:
            self.x[:2] = z
            self.x[2:] = 0.0
            self.ok = True
            self.last_t = t
            return z.copy()
        dt = t - self.last_t if self.last_t is not None else 0.2
        if dt > 2.0:
            self.x[:2] = z
            self.x[2:] = 0.0
            self.P = np.eye(4) * 5.0
            self.last_t = t
            return z.copy()
        self.last_t = t
        F, Q = self._F_Q(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        try:
            K = self.P @ self.H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P @ self.H.T @ np.linalg.pinv(S)
        self.x = self.x + K @ y
        I = np.eye(4)
        IKH = I - K @ self.H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T
        speed = float(math.hypot(self.x[2], self.x[3]))
        if speed > MAX_SPEED_MPS:
            s = MAX_SPEED_MPS / speed
            self.x[2] *= s
            self.x[3] *= s
        return self.x[:2].copy()


kalman = Kalman2D()


def filter_outliers(anchor_ids, anch_positions, distances, bounds=None):
    """
    Drop at most one inconsistent range.
    - Small N (≤ OUTLIER_LOO_MAX_N): leave-one-out consistency test.
    - Large N: single full fit + drop largest geometric residual if big enough.
      (Avoid O(N) NLS solves when N is dozens–hundreds.)
    """
    anchor_ids = list(anchor_ids)
    anch_positions = list(anch_positions)
    distances = list(distances)
    n = len(distances)
    if n < 4:
        return anchor_ids, anch_positions, distances

    if n > OUTLIER_LOO_MAX_N:
        full_pos = trilaterate(anch_positions, distances, bounds=bounds)
        if full_pos is None:
            return anchor_ids, anch_positions, distances
        residuals = [
            abs(math.hypot(full_pos[0] - ax, full_pos[1] - ay) - d)
            for (ax, ay), d in zip(anch_positions, distances)
        ]
        worst = max(range(n), key=lambda i: residuals[i])
        med = float(np.median(residuals))
        thr = max(OUTLIER_ABS_M, 2.5 * med + 0.25)
        if residuals[worst] > thr and n - 1 >= site.MIN_ANCHORS:
            keep = [i for i in range(n) if i != worst]
            return ([anchor_ids[i] for i in keep],
                    [anch_positions[i] for i in keep],
                    [distances[i] for i in keep])
        return anchor_ids, anch_positions, distances

    full_pos = trilaterate(anch_positions, distances, bounds=bounds)
    full_rms = (_rms_range_residual(full_pos, anch_positions, distances)
                if full_pos is not None else float('inf'))

    best_drop = None
    best_rms = full_rms
    best_held = 0.0

    for i in range(n):
        others_pos = [anch_positions[j] for j in range(n) if j != i]
        others_dist = [distances[j] for j in range(n) if j != i]
        trial = trilaterate(others_pos, others_dist, bounds=bounds)
        if trial is None:
            continue
        rms = _rms_range_residual(trial, others_pos, others_dist)
        ax, ay = anch_positions[i]
        held = abs(math.hypot(trial[0] - ax, trial[1] - ay) - distances[i])
        if rms < best_rms - 1e-9:
            best_rms = rms
            best_drop = i
            best_held = held

    if best_drop is None or n - 1 < site.MIN_ANCHORS:
        return anchor_ids, anch_positions, distances

    thr_held = max(OUTLIER_ABS_M, 2.5 * best_rms + 0.25)
    improved = (full_rms - best_rms) >= 0.25 or (
        best_rms < 0.35 and full_rms > 0.8)
    if improved and best_held > thr_held:
        keep = [i for i in range(n) if i != best_drop]
        return ([anchor_ids[i] for i in keep],
                [anch_positions[i] for i in keep],
                [distances[i] for i in keep])
    return anchor_ids, anch_positions, distances


def robust_trilaterate(anchor_ids, anch_positions, distances, bounds=None):
    """
    Small N: full set + leave-one-out score.
    Large N: full fit + iterative residual pruning (max 3 drops) — O(N) not O(N²).
    """
    anchor_ids = list(anchor_ids)
    anch_positions = list(anch_positions)
    distances = list(distances)
    n = len(distances)
    if n < site.MIN_ANCHORS:
        return None, anchor_ids, anch_positions, distances

    def consider(idxs, bag):
        pos = trilaterate([anch_positions[i] for i in idxs],
                          [distances[i] for i in idxs], bounds=bounds)
        if pos is None:
            return
        sub_a = [anch_positions[i] for i in idxs]
        sub_d = [distances[i] for i in idxs]
        rms = _rms_range_residual(pos, sub_a, sub_d)
        score = rms + (0.0 if len(idxs) >= 4 else 0.15)
        bag.append((score, pos, idxs))

    candidates = []
    consider(list(range(n)), candidates)

    if n <= ROBUST_LOO_MAX_N and n >= 4:
        for drop in range(n):
            consider([i for i in range(n) if i != drop], candidates)
    else:
        # Iterative residual prune (cheap for N=200)
        live = list(range(n))
        for _ in range(3):
            if len(live) <= site.MIN_ANCHORS:
                break
            pos = trilaterate([anch_positions[i] for i in live],
                              [distances[i] for i in live], bounds=bounds)
            if pos is None:
                break
            res = [
                abs(math.hypot(pos[0] - anch_positions[i][0],
                               pos[1] - anch_positions[i][1]) - distances[i])
                for i in live
            ]
            worst_local = max(range(len(live)), key=lambda k: res[k])
            med = float(np.median(res))
            thr = max(OUTLIER_ABS_M, 2.5 * med + 0.25)
            if res[worst_local] <= thr:
                consider(live, candidates)
                break
            drop_i = live[worst_local]
            live = [i for i in live if i != drop_i]
            consider(live, candidates)

    if not candidates:
        return None, anchor_ids, anch_positions, distances
    candidates.sort(key=lambda c: c[0])
    _, best_pos, best_idxs = candidates[0]
    return (best_pos,
            [anchor_ids[i] for i in best_idxs],
            [anch_positions[i] for i in best_idxs],
            [distances[i] for i in best_idxs])


def prune_to_strongest(coherent, k=MAX_FIX_ANCHORS):
    """Keep up to k strongest-RSSI ranges for the solver (stable for large N)."""
    if len(coherent) <= k:
        return list(coherent)
    return sorted(coherent, key=lambda c: c.get('rssi', -999.0), reverse=True)[:k]


def room_aware_select(coherent, cfg):
    """
    If the best room has ≥ min_anchors fresh anchors, use only that room.
    Else use global set of all coherent anchors.

    Returns (subset_list, mode_str, room_id_or_None)
    where subset_list is list of coherent dicts.
    """
    min_a = int(cfg.get('min_anchors', site.MIN_ANCHORS))
    # Map id -> room_id from config
    id_to_room = {int(a['id']): a.get('room_id') for a in cfg.get('anchors', [])}

    by_room: Dict[Any, list] = defaultdict(list)
    for c in coherent:
        rid = id_to_room.get(int(c['aid']))
        if rid:
            by_room[rid].append(c)

    best_room = None
    best_score = -1e9
    for rid, items in by_room.items():
        if len(items) < min_a:
            continue
        # Score: count + mean signal strength (less negative RSSI better)
        mean_rssi = sum(c.get('rssi', -90) for c in items) / len(items)
        score = len(items) * 10.0 + mean_rssi  # prefer more anchors, then stronger RSSI
        if score > best_score:
            best_score = score
            best_room = rid

    if best_room is not None:
        return by_room[best_room], 'room', best_room
    return coherent, 'global', None


def bounds_tuple(aabb):
    return (aabb['min_x'], aabb['min_y'], aabb['max_x'], aabb['max_y'])


# ---------------------------------------------------------------------------
# Calibration math (shared with CLI)
# ---------------------------------------------------------------------------

def path_loss_model(d, A, n):
    return A - 10.0 * n * np.log10(np.asarray(d, dtype=float))


def fit_path_loss(distances, mean_rssi):
    d = np.array(distances, dtype=float)
    r = np.array(mean_rssi, dtype=float)
    log_d = np.log10(d)
    c = np.polyfit(log_d, r, 1)
    A_lin = float(c[1])
    n_lin = float(-c[0] / 10.0)
    try:
        popt, pcov = curve_fit(path_loss_model, d, r,
                               p0=[A_lin, n_lin],
                               bounds=([-120, 0.5], [-10, 6.0]))
        A_cf, n_cf = float(popt[0]), float(popt[1])
        perr = np.sqrt(np.diag(pcov))
    except Exception as e:
        print(f"[CAL] curve_fit warning: {e}")
        A_cf, n_cf, perr = A_lin, n_lin, [0.0, 0.0]
    rmse = float(np.sqrt(np.mean((path_loss_model(d, A_cf, n_cf) - r) ** 2)))
    return {
        'A': round(A_cf, 3),
        'n': round(n_cf, 3),
        'A_lin': round(A_lin, 3),
        'n_lin': round(n_lin, 3),
        'rmse_dB': round(rmse, 3),
        'A_stderr': round(float(perr[0]), 3),
        'n_stderr': round(float(perr[1]), 3),
    }


# ---------------------------------------------------------------------------
# UDP + localization threads
# ---------------------------------------------------------------------------

# Cached registered IDs — refreshed when site config is reloaded/saved
_registered_cache: set = set()
_registered_cache_ts: float = 0.0


def _refresh_registered_cache(force: bool = False) -> set:
    global _registered_cache, _registered_cache_ts
    now = time.time()
    if not force and _registered_cache and (now - _registered_cache_ts) < 1.0:
        return _registered_cache
    cfg = site.get()
    _registered_cache = {int(a['id']) for a in cfg.get('anchors', [])}
    _registered_cache_ts = now
    return _registered_cache


def udp_listener():
    global _report_seq
    import socket as _socket
    sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.settimeout(0.5)
    print(f"[UDP] Listening on :{UDP_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(256)
        except _socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] recv {e}")
            continue
        try:
            if len(data) < REPORT_SIZE:
                continue
            aid, avg_rssi, ch, ts, tid, sc, cal = struct.unpack(
                REPORT_FMT, data[:REPORT_SIZE])
            aid = int(aid)
            if aid < 1 or aid > 254:
                continue
            if not math.isfinite(avg_rssi) or avg_rssi < -120.0 or avg_rssi > 0.0:
                continue

            now = time.time()
            registered = _refresh_registered_cache()

            with state_lock:
                _report_seq += 1
                rssi_store[aid] = {
                    'rssi': float(avg_rssi),
                    'recv_ts': now,
                    'channel': ch,
                    'samples': sc,
                    'tag_id': tid,
                    'seq': _report_seq,
                    'cal_mode': int(cal),
                }
                if aid not in registered:
                    unregistered_seen[aid] = now

            with cal_lock:
                if (cal_session['active']
                        and cal_session['phase'] == 'collecting'
                        and cal_session['anchor_id'] == aid):
                    if cal == 1 or cal_session.get('accept_normal_mode'):
                        if len(cal_session['samples']) < cal_session['target_samples']:
                            cal_session['samples'].append(float(avg_rssi))
        except Exception as e:
            print(f"[UDP] {e}")


def localization_loop(sio_ref):
    print(f"[LOC] Polling every {LOC_POLL_SEC}s; room-aware multilateration "
          f"(fix uses ≤{MAX_FIX_ANCHORS} strongest ranges)")
    last_data_sig = None

    while True:
        time.sleep(LOC_POLL_SEC)
        now = time.time()
        cfg = site.get()
        if not cfg.get('setup_complete'):
            continue

        amap = site.anchor_map(cfg)
        plmap = site.path_loss_map(cfg)
        aabb = site.floor_aabb(cfg)
        bounds = bounds_tuple(aabb)
        range_cap = max_range_m(aabb)
        min_a = int(cfg.get('min_anchors', site.MIN_ANCHORS))
        if min_a < 3:
            min_a = 3

        with state_lock:
            snap = {k: dict(v) for k, v in rssi_store.items()}

        candidates = []
        rssi_out = {}
        for aid, pos in amap.items():
            if aid not in snap:
                continue
            e = snap[aid]
            if now - e['recv_ts'] > MAX_STALE_SEC:
                continue
            A, n = plmap.get(aid, (site.DEFAULT_A, site.DEFAULT_N))
            d = rssi_to_dist(e['rssi'], A, n)
            if d >= range_cap:
                continue
            rssi_out[str(aid)] = round(float(e['rssi']), 1)
            candidates.append({
                'aid': aid, 'pos': pos, 'dist': d,
                'recv_ts': e['recv_ts'], 'seq': e.get('seq', 0),
                'rssi': float(e['rssi']),
            })

        if len(candidates) < min_a:
            continue

        newest = max(c['recv_ts'] for c in candidates)
        coherent = [c for c in candidates
                    if newest - c['recv_ts'] <= MAX_COHERENCE_SEC]
        if len(coherent) < min_a:
            continue

        # Room-aware selection, then cap N for solver cost
        selected, mode, room_id = room_aware_select(coherent, cfg)
        if len(selected) < min_a:
            selected, mode, room_id = coherent, 'global', None

        n_before_prune = len(selected)
        selected = prune_to_strongest(selected, MAX_FIX_ANCHORS)
        if len(selected) < min_a:
            continue

        aid_v = [c['aid'] for c in selected]
        anch_v = [c['pos'] for c in selected]
        dist_v = [c['dist'] for c in selected]
        seqs = [c['seq'] for c in selected]
        distances_all = {str(c['aid']): round(float(c['dist']), 2) for c in coherent}

        data_sig = (mode, room_id, tuple(sorted(zip(aid_v, seqs))))
        if data_sig == last_data_sig:
            continue
        last_data_sig = data_sig

        aid_v, anch_v, dist_v = filter_outliers(aid_v, anch_v, dist_v, bounds=bounds)
        if len(anch_v) < min_a:
            continue

        raw, aid_v, anch_v, dist_v = robust_trilaterate(
            aid_v, anch_v, dist_v, bounds=bounds)
        if raw is None:
            continue
        sm = kalman.update(raw, t=now)

        sio_ref.emit('position_update', {
            'x': round(float(sm[0]), 2),
            'y': round(float(sm[1]), 2),
            'raw_x': round(float(raw[0]), 2),
            'raw_y': round(float(raw[1]), 2),
            'distances': distances_all,
            'used_anchors': sorted(int(a) for a in aid_v),
            'rssi': rssi_out,
            'n_anchors': len(aid_v),
            'n_available': len(coherent),
            'n_selected_pool': n_before_prune,
            'n_enabled': len(amap),
            'fix_mode': mode,
            'room_id': room_id,
            'timestamp': now,
            'aabb': aabb,
        })


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------

app = Flask(__name__, static_folder=STATIC_DIR, static_url_path='/static')
sio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')


def _after_config_write():
    """Invalidate caches that depend on layout after any successful save."""
    _refresh_registered_cache(force=True)


def _json_ok(data=None, **extra):
    body = {'ok': True}
    if data is not None:
        body['data'] = data
    body.update(extra)
    return jsonify(body)


def _json_err(msg, code=400):
    return jsonify({'ok': False, 'error': msg}), code


@app.route('/')
def index():
    return send_from_directory(STATIC_DIR, 'index.html')


@app.route('/live')
def live_page():
    return send_from_directory(STATIC_DIR, 'live.html')


@app.route('/setup')
def setup_page():
    return send_from_directory(STATIC_DIR, 'setup.html')


@app.route('/calibrate')
def calibrate_page():
    return send_from_directory(STATIC_DIR, 'calibrate.html')


@app.route('/fonts/<path:filename>')
def serve_font(filename):
    root = os.path.abspath(FONTS_DIR)
    path = os.path.abspath(os.path.join(root, filename))
    if not path.startswith(root + os.sep) and path != root:
        return 'Forbidden', 403
    if not os.path.isfile(path):
        return 'Not found', 404
    lower = filename.lower()
    mime = ('font/woff2' if lower.endswith('.woff2')
            else 'font/ttf' if lower.endswith('.ttf')
            else 'application/octet-stream')
    return send_file(path, mimetype=mime, max_age=86400 * 30)


# ---- Config API ----

@app.route('/api/config', methods=['GET'])
def api_get_config():
    view = site.public_view()
    with state_lock:
        unreg = {str(k): v for k, v in unregistered_seen.items()
                 if time.time() - v < 30.0}
        live_rssi = {str(k): {'rssi': v['rssi'], 'age': time.time() - v['recv_ts'],
                              'cal_mode': v.get('cal_mode', 0)}
                     for k, v in rssi_store.items()}
    view['unregistered'] = unreg
    view['live_rssi'] = live_rssi
    return _json_ok(view)


@app.route('/api/config', methods=['PUT'])
def api_put_config():
    body = request.get_json(force=True, silent=True) or {}
    cfg = body.get('config', body)
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/setup/default', methods=['POST'])
def api_setup_default():
    site.apply_default_layout()
    # re-merge cal into defaults
    site.load(force_reload=True)
    cfg = site.get()
    cfg['setup_complete'] = True
    cfg['profile'] = 'default'
    site.save(cfg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/setup/custom', methods=['POST'])
def api_setup_custom():
    """Start custom profile — keep current or empty shell with one room."""
    body = request.get_json(force=True, silent=True) or {}
    if body.get('reset'):
        cfg = {
            'version': 1,
            'setup_complete': True,
            'profile': 'custom',
            'grid_snap_m': 0.5,
            'min_anchors': 3,
            'defaults': {'A': site.DEFAULT_A, 'n': site.DEFAULT_N},
            'map': {'padding_m': 0.5, 'show_grid': True},
            'rooms': [{
                'id': 'room1', 'name': 'Room1',
                'origin_x': 0, 'origin_y': 0, 'width': 5, 'height': 4,
            }],
            'anchors': [],
        }
    else:
        cfg = site.get()
        cfg['setup_complete'] = True
        cfg['profile'] = 'custom'
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/rooms', methods=['POST'])
def api_add_room():
    body = request.get_json(force=True, silent=True) or {}
    cfg = site.get()
    rid = body.get('id') or ('room_' + __import__('uuid').uuid4().hex[:8])
    room = {
        'id': rid,
        'name': body.get('name') or rid,
        'origin_x': float(body.get('origin_x', 0)),
        'origin_y': float(body.get('origin_y', 0)),
        'width': float(body.get('width', 5)),
        'height': float(body.get('height', 4)),
    }
    cfg.setdefault('rooms', []).append(room)
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/rooms/<rid>', methods=['PATCH'])
def api_patch_room(rid):
    body = request.get_json(force=True, silent=True) or {}
    cfg = site.get()
    move_anchors = bool(body.get('move_anchors', False))
    found = None
    for r in cfg.get('rooms', []):
        if r['id'] == rid:
            found = r
            break
    if not found:
        return _json_err('Room not found', 404)
    old_ox, old_oy = float(found['origin_x']), float(found['origin_y'])
    for k in ('name', 'origin_x', 'origin_y', 'width', 'height'):
        if k in body:
            found[k] = body[k] if k == 'name' else float(body[k])
    if move_anchors:
        dx = float(found['origin_x']) - old_ox
        dy = float(found['origin_y']) - old_oy
        for a in cfg.get('anchors', []):
            if a.get('room_id') == rid:
                a['x'] = float(a['x']) + dx
                a['y'] = float(a['y']) + dy
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/rooms/<rid>', methods=['DELETE'])
def api_del_room(rid):
    cfg = site.get()
    cfg['rooms'] = [r for r in cfg.get('rooms', []) if r['id'] != rid]
    for a in cfg.get('anchors', []):
        if a.get('room_id') == rid:
            a['room_id'] = None
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/anchors', methods=['POST'])
def api_add_anchor():
    body = request.get_json(force=True, silent=True) or {}
    cfg = site.get()
    aid = int(body.get('id', 0))
    if aid < 1 or aid > 254:
        return _json_err('id must be 1–254')
    if any(int(a['id']) == aid for a in cfg.get('anchors', [])):
        return _json_err(f'Anchor {aid} already exists')
    defs = cfg.get('defaults', {})
    anchor = {
        'id': aid,
        'name': body.get('name') or f'A{aid}',
        'x': float(body.get('x', 0)),
        'y': float(body.get('y', 0)),
        'room_id': body.get('room_id'),
        'enabled': bool(body.get('enabled', True)),
        'A': float(body.get('A', defs.get('A', site.DEFAULT_A))),
        'n': float(body.get('n', defs.get('n', site.DEFAULT_N))),
    }
    cfg.setdefault('anchors', []).append(anchor)
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/anchors/<int:aid>', methods=['PATCH'])
def api_patch_anchor(aid):
    body = request.get_json(force=True, silent=True) or {}
    cfg = site.get()
    found = None
    for a in cfg.get('anchors', []):
        if int(a['id']) == aid:
            found = a
            break
    if not found:
        return _json_err('Anchor not found', 404)
    for k in ('name', 'room_id', 'enabled', 'x', 'y', 'A', 'n'):
        if k in body:
            if k in ('x', 'y', 'A', 'n'):
                found[k] = float(body[k])
            elif k == 'enabled':
                found[k] = bool(body[k])
            else:
                found[k] = body[k]
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


@app.route('/api/anchors/<int:aid>', methods=['DELETE'])
def api_del_anchor(aid):
    cfg = site.get()
    cfg['anchors'] = [a for a in cfg.get('anchors', []) if int(a['id']) != aid]
    ok, msg = site.save(cfg)
    if not ok:
        return _json_err(msg)
    sio.emit('config_updated', site.public_view())
    return _json_ok(site.public_view())


# ---- Calibration API (dashboard wizard) ----

@app.route('/api/cal/status', methods=['GET'])
def api_cal_status():
    with cal_lock:
        s = dict(cal_session)
        s['samples'] = list(cal_session['samples'])
        s['n_samples'] = len(cal_session['samples'])
        s['points'] = {str(k): {
            'mean': v['mean'], 'std': v['std'], 'n': v['n']
        } for k, v in cal_session['points'].items()}
    return _json_ok(s)


@app.route('/api/cal/start', methods=['POST'])
def api_cal_start():
    body = request.get_json(force=True, silent=True) or {}
    aid = int(body.get('anchor_id', 0))
    if aid < 1 or aid > 254:
        return _json_err('Invalid anchor_id')
    with cal_lock:
        cal_session.update({
            'active': True,
            'anchor_id': aid,
            'phase': 'checklist',
            'distance_m': None,
            'target_samples': int(body.get('samples', 80)),
            'samples': [],
            'points': {},
            'message': f'Calibration started for anchor {aid}',
            'accept_normal_mode': bool(body.get('accept_normal_mode', True)),
            'distances': body.get('distances') or list(DEFAULT_DISTANCES),
        })
    return _json_ok(cal_session_public())


@app.route('/api/cal/collect', methods=['POST'])
def api_cal_collect():
    body = request.get_json(force=True, silent=True) or {}
    d = float(body.get('distance_m', 0))
    if d <= 0:
        return _json_err('distance_m must be > 0')
    with cal_lock:
        if not cal_session['active']:
            return _json_err('No active calibration session', 400)
        cal_session['phase'] = 'collecting'
        cal_session['distance_m'] = d
        cal_session['samples'] = []
        cal_session['target_samples'] = int(body.get('samples', cal_session.get('target_samples', 80)))
        cal_session['message'] = f'Collecting at {d} m — keep tag still'
    return _json_ok(cal_session_public())


@app.route('/api/cal/commit_point', methods=['POST'])
def api_cal_commit_point():
    with cal_lock:
        if not cal_session['active'] or cal_session['phase'] != 'collecting':
            return _json_err('Not collecting')
        samples = list(cal_session['samples'])
        d = cal_session['distance_m']
        if len(samples) < 10:
            return _json_err(f'Need ≥10 samples, got {len(samples)}')
        mean_v = float(np.mean(samples))
        std_v = float(np.std(samples))
        cal_session['points'][float(d)] = {
            'mean': mean_v, 'std': std_v, 'n': len(samples),
            'samples': samples,
        }
        cal_session['phase'] = 'checklist'
        cal_session['samples'] = []
        cal_session['message'] = f'Saved {d} m: mean={mean_v:.2f} σ={std_v:.2f}'
    return _json_ok(cal_session_public())


@app.route('/api/cal/fit', methods=['POST'])
def api_cal_fit():
    with cal_lock:
        if not cal_session['active']:
            return _json_err('No session')
        pts = cal_session['points']
        if len(pts) < 3:
            return _json_err('Need ≥3 distance points')
        distances = sorted(pts.keys())
        means = [pts[d]['mean'] for d in distances]
        stds = [pts[d]['std'] for d in distances]
        aid = cal_session['anchor_id']

    fit = fit_path_loss(distances, means)
    site.write_cal_results_entry(
        aid, fit['A'], fit['n'], fit['rmse_dB'],
        distances=list(distances), mean_rssi=means, std_rssi=stds)
    site.update_anchor_calibration(aid, fit['A'], fit['n'], fit['rmse_dB'])

    with cal_lock:
        cal_session['phase'] = 'done'
        cal_session['message'] = (
            f"Fit A={fit['A']} n={fit['n']} RMSE={fit['rmse_dB']} dB")
        cal_session['last_fit'] = fit

    sio.emit('config_updated', site.public_view())
    return _json_ok({'fit': fit, 'session': cal_session_public()})


@app.route('/api/cal/cancel', methods=['POST'])
def api_cal_cancel():
    with cal_lock:
        cal_session.update({
            'active': False, 'anchor_id': None, 'phase': 'idle',
            'samples': [], 'points': {}, 'message': 'Cancelled',
        })
    return _json_ok(cal_session_public())


def cal_session_public():
    with cal_lock:
        return {
            'active': cal_session['active'],
            'anchor_id': cal_session['anchor_id'],
            'phase': cal_session['phase'],
            'distance_m': cal_session['distance_m'],
            'target_samples': cal_session['target_samples'],
            'n_samples': len(cal_session['samples']),
            'last_rssi': (cal_session['samples'][-1]
                          if cal_session['samples'] else None),
            'points': {str(k): {
                'mean': round(v['mean'], 2),
                'std': round(v['std'], 2),
                'n': v['n'],
            } for k, v in cal_session['points'].items()},
            'distances': cal_session.get('distances', DEFAULT_DISTANCES),
            'message': cal_session.get('message', ''),
            'last_fit': cal_session.get('last_fit'),
            'accept_normal_mode': cal_session.get('accept_normal_mode', True),
        }


@sio.on('connect')
def on_connect():
    print("[WEB] Client connected")
    sio.emit('config_updated', site.public_view())


@sio.on('disconnect')
def on_disconnect():
    print("[WEB] Client disconnected")


if __name__ == '__main__':
    site.load(force_reload=True)
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=localization_loop, args=(sio,), daemon=True).start()
    print(f"[Server] Aura Tracker (RSSI based localization)")
    print(f"[Server] Dashboard : http://0.0.0.0:{WEB_PORT}")
    print(f"[Server] Live      : http://localhost:{WEB_PORT}/live")
    print(f"[Server] Setup     : http://localhost:{WEB_PORT}/setup")
    print(f"[Server] Calibrate : http://localhost:{WEB_PORT}/calibrate")
    sio.run(app, host='0.0.0.0', port=WEB_PORT, allow_unsafe_werkzeug=True)
