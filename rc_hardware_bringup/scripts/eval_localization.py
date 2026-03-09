#!/usr/bin/env python3
"""
Compute localization error metrics (APE, RPE) by comparing estimated
poses against ground truth, both in TUM format.

Since estimated poses use wall-clock timestamps (from the replayer)
and ground truth uses original bag timestamps, alignment is done by
matching pose indices (nearest-neighbour in time after resampling both
trajectories to the same length).

Usage:
    python3 eval_localization.py \
        --gt /path/to/gt_poses.txt \
        --est /path/to/estimated_poses.txt \
        --output /path/to/localization_results.json
"""

import argparse
import json
import sys

import numpy as np


def load_tum(path):
    """Load TUM-format poses: timestamp tx ty tz qx qy qz qw"""
    data = np.loadtxt(path)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:8]
    return timestamps, positions, quaternions


def quat_to_yaw(q):
    """Extract yaw from quaternion [qx, qy, qz, qw]."""
    qx, qy, qz, qw = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def align_trajectories_by_index(gt_pos, gt_quat, est_pos, est_quat):
    """Resample the shorter trajectory to match the longer one's length,
    then return equal-length arrays."""
    n_gt = len(gt_pos)
    n_est = len(est_pos)
    n = min(n_gt, n_est)

    gt_idx = np.linspace(0, n_gt - 1, n, dtype=int)
    est_idx = np.linspace(0, n_est - 1, n, dtype=int)

    return gt_pos[gt_idx], gt_quat[gt_idx], est_pos[est_idx], est_quat[est_idx]


def umeyama_alignment(gt, est):
    """Compute SE(2) alignment (rotation + translation) using Umeyama.
    Returns aligned estimated positions."""
    gt_2d = gt[:, :2]
    est_2d = est[:, :2]

    mu_gt = gt_2d.mean(axis=0)
    mu_est = est_2d.mean(axis=0)

    gt_c = gt_2d - mu_gt
    est_c = est_2d - mu_est

    H = est_c.T @ gt_c
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    S = np.eye(2)
    if d < 0:
        S[1, 1] = -1
    R = Vt.T @ S @ U.T
    t = mu_gt - R @ mu_est

    aligned = (R @ est[:, :2].T).T + t
    result = est.copy()
    result[:, 0] = aligned[:, 0]
    result[:, 1] = aligned[:, 1]
    return result, R, t


def compute_ape(gt_pos, est_pos):
    """Absolute Pose Error (translation only, 2D)."""
    errors = np.linalg.norm(gt_pos[:, :2] - est_pos[:, :2], axis=1)
    return {
        'ape_rmse': float(np.sqrt(np.mean(errors ** 2))),
        'ape_mean': float(np.mean(errors)),
        'ape_median': float(np.median(errors)),
        'ape_std': float(np.std(errors)),
        'ape_max': float(np.max(errors)),
    }


def compute_rpe(gt_pos, est_pos, gt_quat, est_quat, delta=10):
    """Relative Pose Error over fixed index delta."""
    n = len(gt_pos)
    if n <= delta:
        return {'rpe_trans_rmse': float('nan'), 'rpe_rot_rmse': float('nan')}

    gt_yaw = quat_to_yaw(gt_quat)
    est_yaw = quat_to_yaw(est_quat)

    trans_errors = []
    rot_errors = []

    for i in range(n - delta):
        gt_dt = gt_pos[i + delta, :2] - gt_pos[i, :2]
        est_dt = est_pos[i + delta, :2] - est_pos[i, :2]
        trans_errors.append(np.linalg.norm(gt_dt - est_dt))

        gt_dyaw = gt_yaw[i + delta] - gt_yaw[i]
        est_dyaw = est_yaw[i + delta] - est_yaw[i]
        dyaw_err = abs(gt_dyaw - est_dyaw)
        dyaw_err = min(dyaw_err, 2 * np.pi - dyaw_err)
        rot_errors.append(dyaw_err)

    trans_errors = np.array(trans_errors)
    rot_errors = np.array(rot_errors)

    return {
        'rpe_trans_rmse': float(np.sqrt(np.mean(trans_errors ** 2))),
        'rpe_trans_mean': float(np.mean(trans_errors)),
        'rpe_rot_rmse_deg': float(np.degrees(np.sqrt(np.mean(rot_errors ** 2)))),
        'rpe_rot_mean_deg': float(np.degrees(np.mean(rot_errors))),
    }


def main():
    parser = argparse.ArgumentParser(description='Localization evaluation')
    parser.add_argument('--gt', required=True, help='Ground truth TUM file')
    parser.add_argument('--est', required=True, help='Estimated poses TUM file')
    parser.add_argument('--output', default=None, help='Output JSON path')
    parser.add_argument('--align', action='store_true',
                        help='Apply Umeyama SE(2) alignment before APE')
    args = parser.parse_args()

    gt_ts, gt_pos, gt_quat = load_tum(args.gt)
    est_ts, est_pos, est_quat = load_tum(args.est)

    print(f'Ground truth: {len(gt_ts)} poses, '
          f'{gt_ts[-1] - gt_ts[0]:.1f}s')
    print(f'Estimated:    {len(est_ts)} poses, '
          f'{est_ts[-1] - est_ts[0]:.1f}s')

    gt_p, gt_q, est_p, est_q = align_trajectories_by_index(
        gt_pos, gt_quat, est_pos, est_quat)

    print(f'Aligned:      {len(gt_p)} pose pairs')

    if args.align:
        est_p, R, t = umeyama_alignment(gt_p, est_p)
        print(f'Umeyama alignment applied (t=[{t[0]:.3f}, {t[1]:.3f}])')

    ape = compute_ape(gt_p, est_p)
    rpe = compute_rpe(gt_p, est_p, gt_q, est_q)

    results = {**ape, **rpe, 'n_poses': len(gt_p)}

    print('\n--- Results ---')
    for k, v in results.items():
        if isinstance(v, float):
            print(f'  {k:20s}: {v:.4f}')
        else:
            print(f'  {k:20s}: {v}')

    if args.output:
        with open(args.output, 'w') as f:
            json.dump(results, f, indent=2)
        print(f'\nSaved to {args.output}')

    return results


if __name__ == '__main__':
    main()
