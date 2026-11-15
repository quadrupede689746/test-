#!/usr/bin/env python3
"""Generate diagnostic plots from a robot_logger.py CSV."""
import argparse
import csv
import re
import sys
from pathlib import Path

# Ubuntu's apt Matplotlib was built against NumPy 1.x. This machine also has a
# user-local NumPy 2.x, so prefer the matching system scientific packages here.
sys.path[:] = [p for p in sys.path if '/.local/lib/python3.10/site-packages' not in p]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LEGS = ('front_left', 'front_right', 'back_left', 'back_right')
JOINTS = [f'motor_{leg}_{joint}' for leg in ('front_left', 'front_right', 'back_left', 'back_right')
          for joint in ('hip', 'upper_leg', 'lower_leg')]


def read_csv(path):
    with Path(path).open(newline='') as f:
        rows = list(csv.DictReader(f))
    data = {}
    for key in rows[0] if rows else []:
        vals = []
        for row in rows:
            try:
                vals.append(float(row[key]))
            except (ValueError, TypeError):
                vals.append(float('nan'))
        data[key] = vals
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', type=Path)
    parser.add_argument('--summary', type=Path)
    parser.add_argument('--params', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    data = read_csv(args.csv)
    if not data or not data.get('sim_time_sec'):
        raise SystemExit('CSV has no samples.')
    t = data['sim_time_sec']
    out = args.output_dir or args.csv.parent / (args.csv.stem + '_plots')
    out.mkdir(parents=True, exist_ok=True)
    summary = args.summary or args.csv.with_name(args.csv.stem + '_summary.txt')
    params = args.params or args.csv.with_name(args.csv.stem + '_params.txt')
    effort_limit = 4.0
    if params.exists():
        match = re.search(r'^effort_limit_nm=([-+0-9.eE]+)', params.read_text(), re.MULTILINE)
        if match:
            effort_limit = float(match.group(1))
    divergence_time = None
    if summary.exists():
        text = summary.read_text()
        match = re.search(r'first_anomaly=[^\n]*?t=([0-9.]+)', text)
        if match:
            divergence_time = float(match.group(1))
    def finish(fig, name):
        if divergence_time is not None:
            for ax in fig.axes:
                ax.axvline(divergence_time, color='red', linestyle='--', linewidth=1, label='first anomaly')
        fig.tight_layout()
        fig.savefig(out / name, dpi=140)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4)); ax.plot(t, data['z_m']); ax.set(xlabel='Simulation time (s)', ylabel='Body z (m)', title='Body height'); ax.grid(True); finish(fig, 'body_height.png')
    fig, ax = plt.subplots(figsize=(10, 4)); ax.plot(t, data['roll_rad'], label='roll'); ax.plot(t, data['pitch_rad'], label='pitch'); ax.set(xlabel='Simulation time (s)', ylabel='Angle (rad)', title='Body attitude'); ax.grid(True); ax.legend(); finish(fig, 'roll_pitch.png')
    fig, ax = plt.subplots(figsize=(12, 6))
    for j in JOINTS:
        ax.plot(t, data.get(f'{j}_error_rad', []), label=j.replace('motor_', ''))
    ax.set(xlabel='Simulation time (s)', ylabel='Command - measured (rad)', title='Joint tracking error'); ax.grid(True); ax.legend(ncol=3, fontsize=7); finish(fig, 'joint_errors.png')
    fig, ax = plt.subplots(figsize=(12, 6))
    for j in JOINTS:
        ax.plot(t, data.get(f'{j}_effort_nm', []), label=j.replace('motor_', ''))
    ax.axhline(effort_limit, color='black', linestyle=':', label=f'+{effort_limit:g} Nm effort limit')
    ax.axhline(-effort_limit, color='black', linestyle=':', label=f'-{effort_limit:g} Nm effort limit')
    ax.set(xlabel='Simulation time (s)', ylabel='Effort (Nm)', title='Joint effort (blank if unavailable in position interface)'); ax.grid(True); ax.legend(ncol=3, fontsize=7); finish(fig, 'efforts.png')
    fig, ax = plt.subplots(figsize=(10, 4))
    for leg in LEGS:
        values = data.get(f'{leg}_contact', [])
        ax.step(t, values, where='post', label=leg.replace('_', ' '))
    ax.set(xlabel='Simulation time (s)', ylabel='Contact (1=yes, 0=no)', title='Foot contacts'); ax.set_ylim(-0.1, 1.1); ax.grid(True); ax.legend(ncol=2); finish(fig, 'foot_contacts.png')
    print(f'Plots saved in {out}')

if __name__ == '__main__':
    main()
