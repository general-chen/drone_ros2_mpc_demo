#!/usr/bin/env python3

import csv
import math
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


LOG_DIR = os.path.expanduser('~/ros2_ws/drone_tracking_logs')
NO_WIND_LOG = os.path.join(LOG_DIR, 'mpc_tracking_log_no_wind.csv')
WIND_LOG = os.path.join(LOG_DIR, 'mpc_tracking_log_wind_strong.csv')
ERROR_PLOT = os.path.join(LOG_DIR, 'wind_metric_error_comparison.png')
TRAJECTORY_PLOT = os.path.join(LOG_DIR, 'wind_metric_trajectory_comparison.png')

MISSING_LOG_INSTRUCTION = (
    'Please copy mpc_tracking_log.csv to mpc_tracking_log_no_wind.csv after a '
    'no-wind run, and to mpc_tracking_log_wind_strong.csv after a strong-wind run.'
)


def load_log(path):
    with open(path, newline='') as csv_file:
        reader = csv.DictReader(csv_file)
        rows = []
        for row in reader:
            rows.append({
                'time': float(row['time']),
                'x_ref': float(row['x_ref']),
                'y_ref': float(row['y_ref']),
                'x': float(row['x']),
                'y': float(row['y']),
                'error_xy': float(row['error_xy']),
            })
    return rows


def mean(values):
    if not values:
        return float('nan')
    return sum(values) / len(values)


def rmse(values):
    if not values:
        return float('nan')
    return math.sqrt(sum(value * value for value in values) / len(values))


def format_metric(value):
    if math.isnan(value):
        return 'n/a'
    return f'{value:.4f}'


def values_in_window(rows, start=None, end=None):
    values = []
    for row in rows:
        if start is not None and row['time'] < start:
            continue
        if end is not None and row['time'] > end:
            continue
        values.append(row['error_xy'])
    return values


def compute_metrics(rows):
    all_errors = [row['error_xy'] for row in rows]
    steady_errors = values_in_window(rows, start=5.0)
    gust_errors = values_in_window(rows, start=12.0, end=18.0)
    recovery_errors = values_in_window(rows, start=18.0, end=25.0)

    return {
        'mean_xy': mean(all_errors),
        'rmse_xy': rmse(all_errors),
        'max_xy': max(all_errors) if all_errors else float('nan'),
        'steady_mean_xy': mean(steady_errors),
        'steady_rmse_xy': rmse(steady_errors),
        'gust_mean_xy': mean(gust_errors),
        'gust_rmse_xy': rmse(gust_errors),
        'recovery_mean_xy': mean(recovery_errors),
        'recovery_rmse_xy': rmse(recovery_errors),
    }


def print_table(metrics_by_case):
    columns = [
        ('case', 'Case'),
        ('mean_xy', 'Mean XY'),
        ('rmse_xy', 'RMSE XY'),
        ('max_xy', 'Max XY'),
        ('steady_mean_xy', 'Mean XY >5s'),
        ('steady_rmse_xy', 'RMSE XY >5s'),
        ('gust_mean_xy', 'Mean XY 12-18s'),
        ('gust_rmse_xy', 'RMSE XY 12-18s'),
        ('recovery_mean_xy', 'Mean XY 18-25s'),
        ('recovery_rmse_xy', 'RMSE XY 18-25s'),
    ]

    table_rows = []
    for case_name, metrics in metrics_by_case.items():
        row = {'case': case_name}
        row.update(metrics)
        table_rows.append(row)

    widths = []
    for key, title in columns:
        cells = [title]
        for row in table_rows:
            value = row[key] if key == 'case' else format_metric(row[key])
            cells.append(value)
        widths.append(max(len(cell) for cell in cells))

    header = ' | '.join(
        title.ljust(widths[index])
        for index, (_, title) in enumerate(columns)
    )
    separator = '-+-'.join('-' * width for width in widths)
    print(header)
    print(separator)

    for row in table_rows:
        cells = []
        for key, _ in columns:
            if key == 'case':
                cells.append(row[key].ljust(widths[len(cells)]))
            else:
                cells.append(format_metric(row[key]).rjust(widths[len(cells)]))
        print(' | '.join(cells))


def plot_error_comparison(no_wind_rows, wind_rows):
    plt.figure(figsize=(10, 5))
    plt.plot(
        [row['time'] for row in no_wind_rows],
        [row['error_xy'] for row in no_wind_rows],
        label='No wind',
    )
    plt.plot(
        [row['time'] for row in wind_rows],
        [row['error_xy'] for row in wind_rows],
        label='Strong wind',
    )
    plt.axvline(12.0, color='black', linestyle='--', linewidth=1.0, label='Gust window')
    plt.axvline(18.0, color='black', linestyle='--', linewidth=1.0)
    plt.xlabel('Time (s)')
    plt.ylabel('XY tracking error (m)')
    plt.title('Wind Disturbance Tracking Error')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(ERROR_PLOT, dpi=150)
    plt.close()


def plot_trajectory_comparison(no_wind_rows, wind_rows):
    reference_rows = no_wind_rows if no_wind_rows else wind_rows

    plt.figure(figsize=(7, 7))
    plt.plot(
        [row['x_ref'] for row in reference_rows],
        [row['y_ref'] for row in reference_rows],
        label='Reference',
        color='black',
        linewidth=2.0,
    )
    plt.plot(
        [row['x'] for row in no_wind_rows],
        [row['y'] for row in no_wind_rows],
        label='Actual no wind',
    )
    plt.plot(
        [row['x'] for row in wind_rows],
        [row['y'] for row in wind_rows],
        label='Actual strong wind',
    )
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('XY Trajectory Comparison')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(TRAJECTORY_PLOT, dpi=150)
    plt.close()


def main():
    missing_logs = [
        path for path in (NO_WIND_LOG, WIND_LOG)
        if not os.path.exists(path)
    ]
    if missing_logs:
        print(MISSING_LOG_INSTRUCTION)
        return 1

    no_wind_rows = load_log(NO_WIND_LOG)
    wind_rows = load_log(WIND_LOG)

    metrics_by_case = {
        'No wind': compute_metrics(no_wind_rows),
        'Strong wind': compute_metrics(wind_rows),
    }
    print_table(metrics_by_case)

    os.makedirs(LOG_DIR, exist_ok=True)
    plot_error_comparison(no_wind_rows, wind_rows)
    plot_trajectory_comparison(no_wind_rows, wind_rows)

    print(f'\nSaved: {ERROR_PLOT}')
    print(f'Saved: {TRAJECTORY_PLOT}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
