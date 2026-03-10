#!/usr/bin/env python3
"""
Automated experiment runner for depth error injection trials.

Loops over depth profiles x worlds x N seeds, launching
sim_depth_experiment.launch.py for each trial and collecting metrics.

Each trial:
  1. Launches the full stack (Gazebo + injector + Nav2 + goal sender)
  2. Waits for trial completion (metrics_logger exits → launch shuts down)
  3. Kills any stale processes
  4. Sleeps briefly for cleanup

Usage:
  # Quick checkpoint (2 profiles x 1 trial):
  python3 run_depth_error_trials.py --profiles 0 1 --num_seeds 1

  # Full sweep (all 10 profiles x 5 trials):
  python3 run_depth_error_trials.py --num_seeds 5

  # Specific profiles + headless:
  python3 run_depth_error_trials.py --profiles 0 1 7 8 --num_seeds 3 --gui false
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

PROFILE_NAMES = {
    0: 'GT',
    1: 'DA3-Small',
    2: 'V4',
    3: 'V5',
    4: 'V6',
    5: 'V7',
    6: 'V8',
    7: 'V9',
    8: 'sensor-fail',
    9: 'DA3+sensor-fail',
}

WORLD_CONFIGS = {
    'corridor_narrow_furnished': {
        'sdf': 'corridor_narrow_furnished.sdf',
        'spawn_x': -6.5, 'spawn_y': 0.0, 'spawn_yaw': 0.0,
        'goal_x': 6.0, 'goal_y': 0.0,
    },
    'corridor_narrow': {
        'sdf': 'corridor_narrow.sdf',
        'spawn_x': -6.5, 'spawn_y': 0.0, 'spawn_yaw': 0.0,
        'goal_x': 6.0, 'goal_y': 0.0,
    },
}


def kill_stale_processes():
    """Kill any leftover Gazebo/ROS processes between trials."""
    stale = ['ign gazebo', 'gz sim', 'ruby.*ign', 'parameter_bridge',
             'robot_state_publisher', 'rviz2', 'nav2', 'bt_navigator',
             'controller_server', 'planner_server', 'amcl',
             'depth_error_injector', 'da3_to_pointcloud']
    for pattern in stale:
        subprocess.run(
            ['pkill', '-f', pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


def run_single_trial(profile: int, world_name: str, seed: int,
                     output_dir: str, gui: str, timeout_s: float) -> dict:
    """Execute one trial. Returns result dict."""
    wc = WORLD_CONFIGS[world_name]
    trial_id = f'profile{profile}_{world_name}_seed{seed:03d}'
    output_file = os.path.join(output_dir, 'data', f'{trial_id}.csv')
    log_file = os.path.join(output_dir, 'logs', f'{trial_id}.log')

    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    os.makedirs(os.path.dirname(log_file), exist_ok=True)

    launch_cmd = [
        'ros2', 'launch',
        'rc_model_description', 'sim_depth_experiment.launch.py',
        f'world:={wc["sdf"]}',
        f'depth_profile:={profile}',
        f'spawn_x:={wc["spawn_x"]}',
        f'spawn_y:={wc["spawn_y"]}',
        f'spawn_yaw:={wc["spawn_yaw"]}',
        f'goal_x:={wc["goal_x"]}',
        f'goal_y:={wc["goal_y"]}',
        f'timeout_s:={timeout_s}',
        f'output_file:={output_file}',
        f'scenario_name:=depth_profile_{profile}',
        f'controller_name:=mppi_depth_sim',
        f'seed:={seed}',
        f'gui:={gui}',
    ]

    profile_name = PROFILE_NAMES.get(profile, f'unknown_{profile}')
    print(f'\n{"="*70}')
    print(f'  TRIAL: {trial_id}')
    print(f'  Profile {profile} ({profile_name}) | {world_name} | seed={seed}')
    print(f'  Route: ({wc["spawn_x"]}, {wc["spawn_y"]}) -> '
          f'({wc["goal_x"]}, {wc["goal_y"]})')
    print(f'  Timeout: {timeout_s}s | GUI: {gui}')
    print(f'  Output: {output_file}')
    print(f'{"="*70}')

    t0 = time.time()
    with open(log_file, 'w') as lf:
        proc = subprocess.Popen(
            launch_cmd,
            stdout=lf, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    max_wait = timeout_s + 120
    try:
        proc.wait(timeout=max_wait)
    except subprocess.TimeoutExpired:
        print(f'  [WARN] Trial exceeded max wait ({max_wait:.0f}s), killing...')
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    elapsed = time.time() - t0

    success = False
    if os.path.exists(output_file):
        try:
            with open(output_file) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row.get('success') == '1':
                        success = True
        except Exception:
            pass

    status = 'SUCCESS' if success else ('METRICS_WRITTEN' if os.path.exists(output_file) else 'NO_OUTPUT')
    print(f'  [{status}] Completed in {elapsed:.1f}s')

    return {
        'trial_id': trial_id,
        'profile': profile,
        'profile_name': profile_name,
        'world': world_name,
        'seed': seed,
        'elapsed_s': round(elapsed, 1),
        'status': status,
        'success': success,
        'output_file': output_file,
    }


def main():
    parser = argparse.ArgumentParser(
        description='Run depth error injection experiment trials')
    parser.add_argument('--profiles', type=int, nargs='+',
                        default=list(range(10)),
                        help='Depth profiles to test (default: 0-9)')
    parser.add_argument('--worlds', nargs='+',
                        default=['corridor_narrow_furnished'],
                        choices=list(WORLD_CONFIGS.keys()),
                        help='World configs to use')
    parser.add_argument('--num_seeds', type=int, default=5,
                        help='Number of trials per profile-world combo (default: 5)')
    parser.add_argument('--start_seed', type=int, default=0,
                        help='Starting seed number (default: 0)')
    parser.add_argument('--output_dir', default='depth_sim_results',
                        help='Output directory (default: depth_sim_results)')
    parser.add_argument('--gui', default='false',
                        choices=['true', 'false'],
                        help='Launch Gazebo with GUI (default: false)')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='Per-trial timeout in seconds (default: 120)')
    parser.add_argument('--sleep_between', type=float, default=8.0,
                        help='Sleep between trials for cleanup (default: 8)')
    parser.add_argument('--dry_run', action='store_true',
                        help='Print trial plan without executing')
    args = parser.parse_args()

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join(args.output_dir, timestamp)

    total = len(args.profiles) * len(args.worlds) * args.num_seeds
    est_minutes = total * 3
    seeds = list(range(args.start_seed, args.start_seed + args.num_seeds))

    print(f'\n{"#"*70}')
    print(f'  DEPTH ERROR INJECTION EXPERIMENT')
    print(f'  Profiles: {args.profiles}')
    print(f'  Worlds: {args.worlds}')
    print(f'  Seeds: {seeds}')
    print(f'  Total trials: {total}')
    print(f'  Estimated time: ~{est_minutes} min ({est_minutes/60:.1f} h)')
    print(f'  Output: {output_dir}')
    print(f'  GUI: {args.gui}')
    print(f'{"#"*70}\n')

    if args.dry_run:
        print('[DRY RUN] Trial plan:')
        for profile in args.profiles:
            for world in args.worlds:
                for seed in seeds:
                    name = PROFILE_NAMES.get(profile, '?')
                    print(f'  profile={profile} ({name}) | {world} | seed={seed}')
        print(f'\nTotal: {total} trials')
        return

    os.makedirs(output_dir, exist_ok=True)
    summary_file = os.path.join(output_dir, 'summary.csv')
    results = []

    trial_num = 0
    try:
        for profile in args.profiles:
            for world in args.worlds:
                for seed in seeds:
                    trial_num += 1
                    print(f'\n>>> Trial {trial_num}/{total}')

                    kill_stale_processes()

                    result = run_single_trial(
                        profile=profile, world_name=world, seed=seed,
                        output_dir=output_dir, gui=args.gui,
                        timeout_s=args.timeout,
                    )
                    results.append(result)

                    with open(summary_file, 'w', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=results[0].keys())
                        writer.writeheader()
                        writer.writerows(results)

                    if trial_num < total:
                        print(f'  Sleeping {args.sleep_between}s before next trial...')
                        time.sleep(args.sleep_between)

    except KeyboardInterrupt:
        print(f'\n\n[INTERRUPTED] Completed {trial_num - 1}/{total} trials')
    finally:
        kill_stale_processes()

    successes = sum(1 for r in results if r['success'])
    failures = sum(1 for r in results if not r['success'])

    print(f'\n{"#"*70}')
    print(f'  EXPERIMENT COMPLETE')
    print(f'  Trials run: {len(results)}/{total}')
    print(f'  Successes: {successes}')
    print(f'  Failures: {failures}')
    print(f'  Summary: {summary_file}')
    print(f'  Data: {output_dir}/data/')
    print(f'  Logs: {output_dir}/logs/')
    print(f'{"#"*70}\n')


if __name__ == '__main__':
    main()
