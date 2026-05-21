#!/usr/bin/env python3
"""
calibration.py — Run on your LAPTOP, one anchor at a time.

Usage:
    pip install numpy scipy
    python calibration.py --anchor 1
    python calibration.py --anchor 2
    python calibration.py --anchor 3
    python calibration.py --anchor 4

Saves results to calibration_results.json which server.py reads.
"""
import socket, struct, time, json, argparse, sys, os
import numpy as np
from scipy.optimize import curve_fit

REPORT_FMT  = '<BfBIBHB'
REPORT_SIZE = struct.calcsize(REPORT_FMT)   # = 14 bytes

DEFAULT_DISTANCES = [1.0, 1.5, 2.0, 3.0, 4.0, 5.0]
CAL_FILE = os.path.join(os.path.dirname(__file__), 'calibration_results.json')


def collect_samples(sock, anchor_id, n_samples, timeout=90.0):
    samples = []
    deadline = time.time() + timeout
    sock.settimeout(0.5)
    print(f"  Collecting {n_samples} samples from Anchor {anchor_id}...")
    while len(samples) < n_samples:
        if time.time() > deadline:
            print(f"\n  TIMEOUT — got {len(samples)} samples.")
            break
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        if len(data) < REPORT_SIZE:
            continue
        aid, avg_rssi, ch, ts, tid, scount, cal = struct.unpack(REPORT_FMT, data[:REPORT_SIZE])
        if aid != anchor_id:
            continue
        samples.append(avg_rssi)
        print(f"  {len(samples):>3}/{n_samples}  RSSI = {avg_rssi:6.1f} dBm", end='\r', flush=True)
    print()
    return samples


def path_loss_model(d, A, n):
    return A - 10.0 * n * np.log10(np.asarray(d, dtype=float))


def fit_model(distances, mean_rssi):
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
        print(f"  curve_fit warning: {e} — using linear result")
        A_cf, n_cf, perr = A_lin, n_lin, [0.0, 0.0]
    rmse = float(np.sqrt(np.mean((path_loss_model(d, A_cf, n_cf) - r)**2)))
    return {
        'linear':    {'A': round(A_lin, 3), 'n': round(n_lin, 3)},
        'curve_fit': {'A': round(A_cf, 3), 'n': round(n_cf, 3),
                      'A_stderr': round(float(perr[0]), 3),
                      'n_stderr': round(float(perr[1]), 3)},
        'rmse_dB': round(rmse, 3),
    }


def load_existing():
    if os.path.exists(CAL_FILE):
        try:
            with open(CAL_FILE) as f:
                data = json.load(f)
            if isinstance(data, list):
                return {e['anchor_id']: e for e in data}
        except Exception:
            pass
    return {}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--anchor',    type=int,   required=True)
    parser.add_argument('--port',      type=int,   default=5005)
    parser.add_argument('--samples',   type=int,   default=80)
    parser.add_argument('--distances', type=float, nargs='+',
                        default=DEFAULT_DISTANCES)
    args = parser.parse_args()
    aid = args.anchor

    print(f"\n{'='*55}")
    print(f"  CALIBRATION — Anchor {aid}")
    print(f"{'='*55}")
    print(f"  Distances : {args.distances} m")
    print(f"  Samples   : {args.samples} per distance")
    print()
    print("  CHECKLIST:")
    print(f"  [ ] Anchor {aid} flashed with CALIBRATION_MODE=1 (env:anchor{aid}_cal)")
    print(f"  [ ] Tag flashed (env:tag) and powered on")
    print(f"  [ ] Anchor {aid} is at its FINAL mounted position — do not move after")
    print(f"  [ ] Tape measure + floor marks ready")
    print(f"  [ ] Clear line-of-sight for each measurement")
    print(f"  [ ] Tag antenna vertical, at same height as anchor\n")
    input("  Press ENTER when ready...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))

    distances_done, mean_rssi_list, std_rssi_list = [], [], []

    for d in args.distances:
        print(f"\n  ── {d:.1f} m ──")
        input(f"  Place tag at {d:.1f} m from Anchor {aid} and press ENTER...")
        samples = collect_samples(sock, aid, args.samples)
        if len(samples) < 10:
            print(f"  Only {len(samples)} samples — skipping.")
            continue
        mean_v = float(np.mean(samples))
        std_v  = float(np.std(samples))
        print(f"  {d:.1f} m → mean={mean_v:.2f} dBm  σ={std_v:.2f} dB  (n={len(samples)})")
        distances_done.append(d)
        mean_rssi_list.append(mean_v)
        std_rssi_list.append(std_v)

    sock.close()
    if len(distances_done) < 3:
        print("\nERROR: Need ≥ 3 distance points. Exiting.")
        sys.exit(1)

    fits = fit_model(distances_done, mean_rssi_list)
    A_best = fits['curve_fit']['A']
    n_best = fits['curve_fit']['n']

    print(f"\n  Linear  : A={fits['linear']['A']}  n={fits['linear']['n']}")
    print(f"  CurveFit: A={A_best}  n={n_best}")
    print(f"  RMSE    : {fits['rmse_dB']} dB")

    print(f"\n  {'Dist':>6}  {'Measured':>10}  {'Predicted':>10}  {'Err':>7}")
    for d, m in zip(distances_done, mean_rssi_list):
        pred = A_best - 10*n_best*np.log10(d)
        print(f"  {d:>6.1f}m  {m:>9.2f}  {pred:>9.2f}  {abs(pred-m):>6.2f} dB")

    result = {
        'anchor_id':     aid,
        'calibrated_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'distances_m':   distances_done,
        'mean_rssi_dBm': mean_rssi_list,
        'std_rssi_dB':   std_rssi_list,
        'fits':          fits,
        'recommended':   {'A': A_best, 'n': n_best},
    }
    all_results = load_existing()
    all_results[aid] = result
    with open(CAL_FILE, 'w') as f:
        json.dump(list(all_results.values()), f, indent=2)

    print(f"\n  Saved to {CAL_FILE}")
    print(f"  *** Reflash Anchor {aid} with env:anchor{aid} (normal mode) ***")
    print(f"  Recommended: DEFAULT_A={A_best}f  DEFAULT_N={n_best}f")
    print(f"{'='*55}\n")


if __name__ == '__main__':
    main()