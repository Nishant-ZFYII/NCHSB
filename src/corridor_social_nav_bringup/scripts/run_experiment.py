#!/usr/bin/env python3
"""
Per-trial experiment orchestrator for corridor social navigation.

Runs a single experiment trial:
  1. Loads scenario YAML, resolves parameters for the given seed
  2. Spawns pedestrian models into the Gazebo world
  3. Launches the full experiment stack via sim_experiment.launch.py
  4. Waits for trial completion (goal reached, timeout, or error)
  5. Collects metrics CSV output
  6. Kills all processes cleanly

Usage:
  python3 run_experiment.py \
    --scenario scenarios/head_on_single.yaml \
    --controller mppi_vanilla \
    --seed 0 \
    --output_dir results/

Controller names map to config files:
  mppi_vanilla     -> nav2_mppi_vanilla.yaml
  mppi_social      -> nav2_mppi_social_instant.yaml
  mppi_pred        -> nav2_mppi_social_pred.yaml
  full             -> nav2_full.yaml
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from corridor_social_nav_bringup.scenario_loader import ScenarioLoader

CONTROLLER_CONFIG_MAP = {
    'mppi_vanilla': 'nav2_mppi_vanilla.yaml',
    'mppi_social': 'nav2_mppi_social_instant.yaml',
    'mppi_pred': 'nav2_mppi_social_pred.yaml',
    'full': 'nav2_full.yaml',
}

SHIELD_CONTROLLERS = {'full'}


def find_package_share(package_name: str) -> str:
    """Find ROS 2 package share directory without importing ament."""
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', '--share', package_name],
        capture_output=True, text=True, timeout=10
    )
    if result.returncode != 0:
        raise RuntimeError(f'Cannot find package {package_name}: {result.stderr}')
    return result.stdout.strip()


def spawn_pedestrians(trial_config, model_sdf_path: str) -> list:
    """Spawn pedestrian models into Gazebo before starting navigation."""
    procs = []
    for ped in trial_config.pedestrians:
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'default',
            '-file', model_sdf_path,
            '-name', f'pedestrian_{ped.id}',
            '-x', str(ped.spawn_x),
            '-y', str(ped.spawn_y),
            '-z', str(ped.spawn_z),
        ]
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(proc)
    for p in procs:
        p.wait(timeout=30)
    return procs


def run_trial(scenario_path: str, controller: str, seed: int,
              output_dir: str, map_yaml: str = '',
              tracking_noise_std: float = 0.0,
              extra_params: dict = None) -> dict:
    """Execute a single experiment trial. Returns metrics dict."""

    loader = ScenarioLoader(scenario_path)
    trial = loader.resolve(seed)

    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(
        output_dir,
        f'{trial.scenario_name}_{controller}_seed{seed:03d}.csv'
    )

    config_yaml = CONTROLLER_CONFIG_MAP.get(controller)
    if config_yaml is None:
        raise ValueError(f'Unknown controller: {controller}. '
                         f'Options: {list(CONTROLLER_CONFIG_MAP.keys())}')

    rc_share = find_package_share('rc_model_description')
    controller_config = os.path.join(rc_share, 'config', config_yaml)
    model_sdf = os.path.join(rc_share, 'models', 'pedestrian', 'model.sdf')
    enable_shield = 'true' if controller in SHIELD_CONTROLLERS else 'false'

    ped_json = json.dumps([{
        'id': p.id,
        'speed': p.speed,
        'start_delay': p.start_delay,
        'waypoints': p.waypoints,
        'spawn_z': p.spawn_z,
    } for p in trial.pedestrians])

    launch_cmd = [
        'ros2', 'launch',
        'corridor_social_nav_bringup', 'sim_experiment.launch.py',
        f'world:={trial.world_file}',
        f'spawn_x:={trial.robot_start_x}',
        f'spawn_y:={trial.robot_start_y}',
        f'spawn_yaw:={trial.robot_start_yaw}',
        f'goal_x:={trial.goal_x}',
        f'goal_y:={trial.goal_y}',
        f'controller_config:={controller_config}',
        f'enable_shield:={enable_shield}',
        f'scenario_name:={trial.scenario_name}',
        f'controller_name:={controller}',
        f'seed:={seed}',
        f'timeout_s:={trial.timeout}',
        f'output_file:={output_file}',
        f'pedestrians_json:={ped_json}',
        f'tracking_noise_std:={tracking_noise_std}',
    ]

    if map_yaml:
        launch_cmd.append(f'map_yaml:={map_yaml}')

    print(f'\n{"="*60}')
    print(f'TRIAL: {trial.scenario_name} | {controller} | seed={seed}')
    print(f'  World: {trial.world_file}')
    print(f'  Robot: ({trial.robot_start_x}, {trial.robot_start_y}) -> '
          f'({trial.goal_x}, {trial.goal_y})')
    print(f'  Pedestrians: {len(trial.pedestrians)}')
    print(f'  Shield: {enable_shield}')
    print(f'  Output: {output_file}')
    print(f'{"="*60}\n')

    trial_start = time.time()
    launch_proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    max_wait = trial.timeout + 60  # extra margin for startup/shutdown
    try:
        launch_proc.wait(timeout=max_wait)
    except subprocess.TimeoutExpired:
        print(f'[WARN] Trial exceeded max wait ({max_wait}s), killing...')
    finally:
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    elapsed = time.time() - trial_start
    print(f'\nTrial completed in {elapsed:.1f}s')
    print(f'Metrics file: {output_file}')

    return {
        'scenario': trial.scenario_name,
        'controller': controller,
        'seed': seed,
        'elapsed_s': elapsed,
        'output_file': output_file,
        'output_exists': os.path.exists(output_file),
    }


def main():
    parser = argparse.ArgumentParser(
        description='Run a single corridor social navigation experiment trial')
    parser.add_argument('--scenario', required=True,
                        help='Path to scenario YAML file')
    parser.add_argument('--controller', required=True,
                        choices=list(CONTROLLER_CONFIG_MAP.keys()),
                        help='Controller configuration name')
    parser.add_argument('--seed', type=int, required=True,
                        help='Random seed for this trial')
    parser.add_argument('--output_dir', default='results/',
                        help='Output directory for metrics CSV')
    parser.add_argument('--map_yaml', default='',
                        help='Path to map YAML for AMCL localization')
    parser.add_argument('--tracking_noise_std', type=float, default=0.0,
                        help='Noise std for tracking ablation')
    args = parser.parse_args()

    result = run_trial(
        scenario_path=args.scenario,
        controller=args.controller,
        seed=args.seed,
        output_dir=args.output_dir,
        map_yaml=args.map_yaml,
        tracking_noise_std=args.tracking_noise_std,
    )

    if result['output_exists']:
        print(f'\n[OK] Trial completed, metrics at: {result["output_file"]}')
    else:
        print(f'\n[WARN] Trial completed but no metrics file found!')
        sys.exit(1)


if __name__ == '__main__':
    main()
