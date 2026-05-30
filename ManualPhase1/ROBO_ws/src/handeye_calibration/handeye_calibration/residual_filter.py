"""Residual-based outlier filter for hand-eye samples.

For an eye-in-hand calibration the target pose in the base frame,
    T_base_target_i = T_base_gripper_i @ X @ T_camera_target_i
should be constant across samples (the target is fixed in the world).
Per-sample deviation from the median T_base_target reveals which captures
disagree with the rest — those are dropped before the final solve.

Workflow:
  1. Solve calibrateHandEye on the full sample set → seed X.
  2. Compute T_base_target_i for every sample using the seed X.
  3. Rank samples by translation deviation from the median target position.
  4. Drop the top --drop samples and emit a filtered samples YAML.
  5. Re-run compute_calibration on the filtered file (per method) and
     report cross-method spread before vs after.

Pure script, no rclpy.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import yaml


_METHOD_MAP = {
    'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
    'PARK':       cv2.CALIB_HAND_EYE_PARK,
    'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _invert(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def _load_samples(path: str):
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    raw = data.get('samples', [])
    out = []
    for s in raw:
        T_bg = np.array(s['T_base_gripper'], dtype=np.float64)
        T_ct = np.array(s['T_camera_target'], dtype=np.float64)
        out.append((T_bg, T_ct))
    return out


def _solve(samples, method: str) -> np.ndarray:
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for T_bg, T_ct in samples:
        # eye_in_hand: gripper2base = T_bg directly
        R_g2b.append(T_bg[:3, :3])
        t_g2b.append(T_bg[:3, 3].reshape(3, 1))
        T_t2c = _invert(T_ct)
        R_t2c.append(T_t2c[:3, :3])
        t_t2c.append(T_t2c[:3, 3].reshape(3, 1))
    R_X, t_X = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=_METHOD_MAP[method])
    X = np.eye(4)
    X[:3, :3] = R_X
    X[:3, 3] = t_X.flatten()
    return X


def _base_target_per_sample(samples, X):
    """Return list of T_base_target estimates, one per sample."""
    out = []
    for T_bg, T_ct in samples:
        out.append(T_bg @ X @ T_ct)
    return out


def _spread_across_methods(samples):
    """Solve under all methods, return dict method → X, plus spread stats."""
    xs = {}
    for m in _METHOD_MAP:
        try:
            xs[m] = _solve(samples, m)
        except Exception as exc:
            print(f"  [{m}] FAILED: {exc}", file=sys.stderr)
    if not xs:
        return xs, None, None
    ts = np.stack([X[:3, 3] for X in xs.values()])
    t_spread = float(np.linalg.norm(ts.std(axis=0)))
    return xs, ts, t_spread


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', required=True,
                    help='Path to samples YAML (input).')
    ap.add_argument('--out', required=True,
                    help='Path to write filtered samples YAML.')
    ap.add_argument('--drop', type=int, default=5,
                    help='Number of worst-residual samples to drop.')
    ap.add_argument('--seed-method', default='PARK',
                    choices=list(_METHOD_MAP),
                    help='Method used to compute the seed X for ranking.')
    args = ap.parse_args()

    samples = _load_samples(args.samples)
    n = len(samples)
    if n < 4:
        print(f"need ≥4 samples, got {n}", file=sys.stderr)
        return 2
    if args.drop >= n - 3:
        print(f"--drop {args.drop} would leave fewer than 3 samples", file=sys.stderr)
        return 2

    print(f"loaded {n} samples from {args.samples}")

    # --- baseline cross-method spread ----------------------------------------
    print("\n== baseline (all samples) ==")
    xs_before, _, spread_before = _spread_across_methods(samples)
    for m, X in xs_before.items():
        t = X[:3, 3]
        print(f"  {m:11s} t=[{t[0]:+.4f} {t[1]:+.4f} {t[2]:+.4f}]")
    print(f"  translation std-norm across methods: {spread_before:.4f} m")

    # --- seed X for residual ranking -----------------------------------------
    if args.seed_method not in xs_before:
        print(f"seed method {args.seed_method} failed; aborting", file=sys.stderr)
        return 2
    X_seed = xs_before[args.seed_method]

    bts = _base_target_per_sample(samples, X_seed)
    bts_t = np.stack([T[:3, 3] for T in bts])    # (n, 3)
    median_t = np.median(bts_t, axis=0)
    dev = np.linalg.norm(bts_t - median_t, axis=1)   # (n,) metres
    order = np.argsort(dev)[::-1]                    # worst first

    print(f"\n== per-sample residual (seed={args.seed_method}) ==")
    for rank, idx in enumerate(order):
        flag = "DROP" if rank < args.drop else "keep"
        print(f"  [{flag}] sample {idx:2d}  dev={dev[idx]*100:6.2f} cm")

    keep_idx = sorted(order[args.drop:].tolist())
    filtered = [samples[i] for i in keep_idx]

    # --- write filtered YAML --------------------------------------------------
    out_payload = {'samples': [
        {
            'T_base_gripper':  T_bg.tolist(),
            'T_camera_target': T_ct.tolist(),
        }
        for T_bg, T_ct in filtered
    ]}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump(out_payload, f, default_flow_style=False, sort_keys=False)
    print(f"\nwrote {len(filtered)} filtered samples → {args.out}")

    # --- post-filter cross-method spread -------------------------------------
    print("\n== after filter ==")
    xs_after, _, spread_after = _spread_across_methods(filtered)
    for m, X in xs_after.items():
        t = X[:3, 3]
        print(f"  {m:11s} t=[{t[0]:+.4f} {t[1]:+.4f} {t[2]:+.4f}]")
    print(f"  translation std-norm across methods: {spread_after:.4f} m")

    if spread_before:
        ratio = spread_after / spread_before
        print(f"\nspread {spread_before:.4f} → {spread_after:.4f} m  "
              f"({(1-ratio)*100:+.1f}% change)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
