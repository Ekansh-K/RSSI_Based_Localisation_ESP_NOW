"""
site_config.py — Dynamic multi-room / multi-anchor site configuration.

Persists to site_config.json. Thread-safe load/save. Merges calibration A/n.
"""
from __future__ import annotations

import copy
import json
import math
import os
import threading
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
SITE_FILE = os.path.join(CONFIG_DIR, 'site_config.json')
CAL_FILE = os.path.join(CONFIG_DIR, 'calibration_results.json')

DEFAULT_A = -60.0
DEFAULT_N = 2.7
MIN_ANCHORS = 3
GRID_SNAP_DEFAULT = 0.5

_lock = threading.RLock()
_config: Dict[str, Any] = {}


def _default_config() -> Dict[str, Any]:
    """Classic 4-corner layout in a 5 m × 4 m lab room (project default)."""
    return {
        'version': 1,
        'setup_complete': True,
        'profile': 'default',  # 'default' | 'custom'
        'grid_snap_m': GRID_SNAP_DEFAULT,
        'min_anchors': MIN_ANCHORS,
        'defaults': {'A': DEFAULT_A, 'n': DEFAULT_N},
        'map': {'padding_m': 0.5, 'show_grid': True},
        'rooms': [
            {
                'id': 'room1',
                'name': 'Room1',
                'origin_x': 0.0,
                'origin_y': 0.0,
                'width': 5.0,
                'height': 4.0,
            }
        ],
        'anchors': [
            {'id': 1, 'name': 'A1', 'x': 0.0, 'y': 0.0,
             'room_id': 'room1', 'enabled': True, 'A': DEFAULT_A, 'n': DEFAULT_N},
            {'id': 2, 'name': 'A2', 'x': 5.0, 'y': 0.0,
             'room_id': 'room1', 'enabled': True, 'A': DEFAULT_A, 'n': DEFAULT_N},
            {'id': 3, 'name': 'A3', 'x': 5.0, 'y': 4.0,
             'room_id': 'room1', 'enabled': True, 'A': DEFAULT_A, 'n': DEFAULT_N},
            {'id': 4, 'name': 'A4', 'x': 0.0, 'y': 4.0,
             'room_id': 'room1', 'enabled': True, 'A': DEFAULT_A, 'n': DEFAULT_N},
        ],
        'updated_at': None,
    }


def snap(v: float, step: float) -> float:
    if step is None or step <= 0:
        return float(v)
    return round(float(v) / step) * step


def floor_aabb(cfg: Dict[str, Any]) -> Dict[str, float]:
    """Axis-aligned bounding box of rooms ∪ anchors (with padding)."""
    pad = float(cfg.get('map', {}).get('padding_m', 0.5))
    rooms = cfg.get('rooms') or []
    anchors = cfg.get('anchors') or []
    xs: List[float] = []
    ys: List[float] = []
    for r in rooms:
        xs.extend([float(r['origin_x']), float(r['origin_x']) + float(r['width'])])
        ys.extend([float(r['origin_y']), float(r['origin_y']) + float(r['height'])])
    for a in anchors:
        xs.append(float(a['x']))
        ys.append(float(a['y']))
    if not xs:
        return {'min_x': -0.5, 'min_y': -0.5, 'max_x': 5.5, 'max_y': 4.5,
                'width': 6.0, 'height': 5.0}
    min_x = min(xs) - pad
    min_y = min(ys) - pad
    max_x = max(xs) + pad
    max_y = max(ys) + pad
    # Ensure non-zero / non-degenerate extent for solvers
    if max_x - min_x < 1.0:
        mid = 0.5 * (min_x + max_x)
        min_x, max_x = mid - 0.5, mid + 0.5
    if max_y - min_y < 1.0:
        mid = 0.5 * (min_y + max_y)
        min_y, max_y = mid - 0.5, mid + 0.5
    return {
        'min_x': min_x, 'min_y': min_y,
        'max_x': max_x, 'max_y': max_y,
        'width': max_x - min_x,
        'height': max_y - min_y,
    }


def _merge_calibration(cfg: Dict[str, Any]) -> None:
    if not os.path.exists(CAL_FILE):
        return
    try:
        with open(CAL_FILE, encoding='utf-8') as f:
            data = json.load(f)
        if not isinstance(data, list):
            data = [data]
        by_id = {}
        for e in data:
            aid = int(e['anchor_id'])
            rec = e.get('recommended') or {}
            A = float(rec.get('A', DEFAULT_A))
            n = float(rec.get('n', DEFAULT_N))
            rmse = e.get('rmse', e.get('fits', {}).get('rmse_dB'))
            by_id[aid] = {'A': A, 'n': n, 'rmse': rmse}
        for a in cfg.get('anchors', []):
            if a['id'] in by_id:
                a['A'] = by_id[a['id']]['A']
                a['n'] = by_id[a['id']]['n']
                if by_id[a['id']]['rmse'] is not None:
                    a['rmse'] = by_id[a['id']]['rmse']
    except Exception as ex:
        print(f"[SITE] calibration merge failed: {ex}")


def validate_config(cfg: Dict[str, Any]) -> Tuple[bool, str]:
    if not isinstance(cfg, dict):
        return False, 'Config must be an object'
    rooms = cfg.get('rooms')
    if rooms is None:
        rooms = []
    if not isinstance(rooms, list):
        return False, 'rooms must be a list'
    for r in rooms:
        for k in ('id', 'name', 'origin_x', 'origin_y', 'width', 'height'):
            if k not in r:
                return False, f'Room missing {k}'
        if float(r['width']) < 0.5 or float(r['height']) < 0.5:
            return False, f"Room {r.get('name')} size must be ≥ 0.5 m"
    anchors = cfg.get('anchors')
    if anchors is None:
        anchors = []
    if not isinstance(anchors, list):
        return False, 'anchors must be a list'
    seen = set()
    for a in anchors:
        aid = int(a.get('id', -1))
        if aid < 1 or aid > 254:
            return False, f'Anchor id {aid} out of range 1–254'
        if aid in seen:
            return False, f'Duplicate anchor id {aid}'
        seen.add(aid)
        if 'x' not in a or 'y' not in a:
            return False, f'Anchor {aid} needs x,y'
    return True, 'ok'


def normalize_config(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Snap coords, coerce types, fill defaults."""
    out = copy.deepcopy(cfg)
    step = float(out.get('grid_snap_m', GRID_SNAP_DEFAULT) or GRID_SNAP_DEFAULT)
    out['grid_snap_m'] = step
    out.setdefault('min_anchors', MIN_ANCHORS)
    out.setdefault('defaults', {'A': DEFAULT_A, 'n': DEFAULT_N})
    out.setdefault('map', {'padding_m': 0.5, 'show_grid': True})
    out.setdefault('setup_complete', False)
    out.setdefault('profile', 'custom')
    out.setdefault('rooms', [])
    out.setdefault('anchors', [])
    out['version'] = 1

    for r in out['rooms']:
        r['origin_x'] = snap(float(r['origin_x']), step)
        r['origin_y'] = snap(float(r['origin_y']), step)
        r['width'] = max(0.5, snap(float(r['width']), step))
        r['height'] = max(0.5, snap(float(r['height']), step))
        if not r.get('id'):
            r['id'] = 'room_' + uuid.uuid4().hex[:8]
        r['name'] = str(r.get('name') or r['id'])

    defs = out['defaults']
    for a in out['anchors']:
        a['id'] = int(a['id'])
        a['x'] = snap(float(a['x']), step)
        a['y'] = snap(float(a['y']), step)
        a['name'] = str(a.get('name') or f"A{a['id']}")
        a['enabled'] = bool(a.get('enabled', True))
        a['A'] = float(a.get('A', defs.get('A', DEFAULT_A)))
        a['n'] = float(a.get('n', defs.get('n', DEFAULT_N)))
        if a.get('room_id') in ('', None):
            a['room_id'] = None
    return out


def _atomic_write(path: str, data: Dict[str, Any]) -> None:
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def load(force_reload: bool = False) -> Dict[str, Any]:
    global _config
    with _lock:
        if _config and not force_reload:
            return copy.deepcopy(_config)
        if os.path.exists(SITE_FILE):
            try:
                with open(SITE_FILE, encoding='utf-8') as f:
                    raw = json.load(f)
                ok, msg = validate_config(raw)
                if not ok:
                    print(f"[SITE] Invalid site_config.json ({msg}) — using default")
                    raw = _default_config()
                else:
                    raw = normalize_config(raw)
            except Exception as ex:
                print(f"[SITE] Load failed ({ex}) — using default")
                raw = _default_config()
        else:
            raw = _default_config()
            raw['setup_complete'] = False  # force welcome until user picks
            print("[SITE] No site_config.json — starting with default template (setup not complete)")
        _merge_calibration(raw)
        _config = raw
        return copy.deepcopy(_config)


def save(cfg: Dict[str, Any]) -> Tuple[bool, str]:
    global _config
    ok, msg = validate_config(cfg)
    if not ok:
        return False, msg
    cfg = normalize_config(cfg)
    # Cap anchors at protocol limit (uint8 id 1–254)
    if len(cfg.get('anchors') or []) > 254:
        return False, 'Too many anchors (max 254; UDP anchor_id is uint8)'
    cfg['updated_at'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    with _lock:
        try:
            _atomic_write(SITE_FILE, cfg)
            _config = cfg
            return True, 'saved'
        except Exception as ex:
            return False, str(ex)


def get() -> Dict[str, Any]:
    with _lock:
        if not _config:
            return load()
        return copy.deepcopy(_config)


def apply_default_layout() -> Dict[str, Any]:
    cfg = _default_config()
    cfg['setup_complete'] = True
    cfg['profile'] = 'default'
    _merge_calibration(cfg)
    save(cfg)
    return get()


def enabled_anchors(cfg: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
    cfg = cfg or get()
    return [a for a in cfg.get('anchors', []) if a.get('enabled', True)]


def anchor_map(cfg: Optional[Dict[str, Any]] = None) -> Dict[int, Tuple[float, float]]:
    return {int(a['id']): (float(a['x']), float(a['y'])) for a in enabled_anchors(cfg)}


def path_loss_map(cfg: Optional[Dict[str, Any]] = None) -> Dict[int, Tuple[float, float]]:
    cfg = cfg or get()
    dA = float(cfg.get('defaults', {}).get('A', DEFAULT_A))
    dN = float(cfg.get('defaults', {}).get('n', DEFAULT_N))
    out = {}
    for a in cfg.get('anchors', []):
        out[int(a['id'])] = (float(a.get('A', dA)), float(a.get('n', dN)))
    return out


def calibration_status(cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Whether path-loss calibration data is available for tracking."""
    cfg = cfg or get()
    file_exists = os.path.exists(CAL_FILE)
    anchors = cfg.get('anchors') or []
    with_rmse = [a for a in anchors if a.get('rmse') is not None]
    # Consider calibrated if file exists and has at least one entry, or anchors have rmse
    n_file = 0
    if file_exists:
        try:
            with open(CAL_FILE, encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, list):
                n_file = len(data)
            elif data:
                n_file = 1
        except Exception:
            n_file = 0
    ready = file_exists and n_file > 0
    return {
        'file_exists': file_exists,
        'n_file_entries': n_file,
        'n_anchors_with_rmse': len(with_rmse),
        'ready': ready,
        'message': (
            None if ready else
            'No calibration data found. Kindly run Calibration for each anchor before tracking.'
        ),
    }


def public_view(cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    cfg = cfg or get()
    aabb = floor_aabb(cfg)
    return {
        'config': cfg,
        'aabb': aabb,
        'n_enabled': len(enabled_anchors(cfg)),
        'n_total': len(cfg.get('anchors') or []),
        'min_anchors': int(cfg.get('min_anchors', MIN_ANCHORS)),
        'calibration': calibration_status(cfg),
    }


def update_anchor_calibration(anchor_id: int, A: float, n: float,
                              rmse: Optional[float] = None) -> bool:
    """Patch A/n for one anchor and persist."""
    with _lock:
        cfg = get()
        found = False
        for a in cfg.get('anchors', []):
            if int(a['id']) == int(anchor_id):
                a['A'] = float(A)
                a['n'] = float(n)
                if rmse is not None:
                    a['rmse'] = float(rmse)
                found = True
                break
        if not found:
            return False
        ok, _ = save(cfg)
        return ok


def write_cal_results_entry(anchor_id: int, A: float, n: float, rmse: float,
                            distances=None, mean_rssi=None, std_rssi=None) -> None:
    """Merge one anchor into calibration_results.json (CLI-compatible)."""
    existing = {}
    if os.path.exists(CAL_FILE):
        try:
            with open(CAL_FILE, encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, list):
                existing = {int(e['anchor_id']): e for e in data}
        except Exception:
            pass
    existing[int(anchor_id)] = {
        'anchor_id': int(anchor_id),
        'calibrated_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'distances_m': distances or [],
        'mean_rssi_dBm': mean_rssi or [],
        'std_rssi_dB': std_rssi or [],
        'recommended': {'A': float(A), 'n': float(n)},
        'rmse': float(rmse),
        'fits': {
            'curve_fit': {'A': float(A), 'n': float(n)},
            'rmse_dB': float(rmse),
        },
    }
    with open(CAL_FILE, 'w', encoding='utf-8') as f:
        json.dump(list(existing.values()), f, indent=2)


# Load on import
load()
