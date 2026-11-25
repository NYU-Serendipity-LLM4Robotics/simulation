"""Run the shared sensor baseline or launch benchmarks in their native environments."""
import argparse
from collections import Counter
from datetime import datetime, timezone
from itertools import product
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def suite(output, seeds):
    from workspace import run

    cases = list(product(seeds, [('red apple', 'green tray'), ('blue block', 'yellow tray')]))
    reports = []
    for index, (seed, (target, destination)) in enumerate(cases):
        report = run(seed, target, destination, output / f'episode_{index:03}', mode='team-kinematic')
        reports.append(report)
    errors = [event['center_error_m'] for report in reports for event in report['events']
              if event['stage'] == 'report']
    pregrasp = [event for report in reports for event in report['events']
                if event['stage'] == 'g1_pregrasp_ik' and event['converged']]
    feet = [event['max_error_m'] for report in reports for event in report['events']
            if event['stage'].endswith('_foot_ik') and event['max_error_m'] is not None]
    summary = {'episodes': len(reports), 'localized': len(errors),
               'pregrasp_pose_reached': len(pregrasp),
               'max_pregrasp_position_error_m': max((e['error_m'] for e in pregrasp), default=None),
               'max_pregrasp_orientation_error_rad': max((e['orientation_error_rad'] for e in pregrasp), default=None),
               'max_foot_position_error_m': max(feet, default=None),
               'mean_surface_to_center_m': sum(errors) / len(errors) if errors else None,
               'max_surface_to_center_m': max(errors, default=None)}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return int(len(pregrasp) != len(reports))


def lab_suite(output, seeds, policy):
    from workspace import run_lab

    episodes = []
    tasks = [('red apple', 'green tray'), ('blue block', 'yellow tray'), ('purple cylinder', 'green tray')]
    for seed, (target, destination) in product(seeds, tasks):
        episode = output / f'episode_{len(episodes):03d}'
        report = run_lab(seed, target, destination, episode, policy)
        row = {field: report[field] for field in (
            'seed', 'object', 'destination', 'status', 'failure_stage', 'handoff_status', 'evidence',
            'initial_object_xyz_m', 'tray_xyz_m')}
        row['report'] = f'{episode.name}/report.json'
        episodes.append(row)
    successes = sum(row['status'] == 'placed' for row in episodes)
    summary = {'episodes': len(episodes), 'successes': successes,
               'success_rate': successes / len(episodes),
               'failure_stages': dict(Counter(row['failure_stage'] for row in episodes if row['failure_stage'])),
               'trials': episodes}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, default=HERE / 'benchmarks.json')
    actions = parser.add_subparsers(dest='action', required=True)
    actions.add_parser('list')
    native = actions.add_parser('suite')
    native.add_argument('--output', type=Path, required=True)
    native.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    lab = actions.add_parser('lab-suite')
    lab.add_argument('--output', type=Path, required=True)
    lab.add_argument('--policy', type=Path, required=True)
    lab.add_argument('--seeds', type=int, nargs='+', default=[0, 1])
    external = actions.add_parser('run')
    external.add_argument('benchmark')
    external.add_argument('--python', help='Override the registry Python interpreter')
    external.add_argument('--dry-run', action='store_true')
    external.add_argument('--args', nargs=argparse.REMAINDER, default=[])
    args = parser.parse_args()
    if args.action == 'suite':
        args.output.mkdir(parents=True, exist_ok=False)
        return suite(args.output, args.seeds)
    if args.action == 'lab-suite':
        if min(args.seeds) < 0:
            parser.error('seeds must be nonnegative')
        args.output.mkdir(parents=True, exist_ok=False)
        return lab_suite(args.output, args.seeds, args.policy)
    registry = json.loads(args.registry.read_text())
    if args.action == 'list':
        for name, spec in registry.items():
            print(f'{name:<26} {spec["description"]}')
        return 0
    if args.benchmark not in registry:
        parser.error(f'unknown benchmark {args.benchmark}; see list')
    spec = registry[args.benchmark]
    python = args.python or spec.get('python', sys.executable).format(workspace=HERE)
    if '/' in python:
        python = str(Path(python).expanduser().absolute())
    values = {'python': python, 'workspace': str(HERE)}
    command = [arg.format_map(values) for arg in spec['command']] + args.args
    cwd = Path(spec.get('cwd', '{workspace}').format_map(values)).expanduser()
    print(json.dumps({'benchmark': args.benchmark, 'argv': command, 'cwd': str(cwd)}), flush=True)
    if args.dry_run:
        return 0
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    log_dir = HERE / 'runs' / 'benchmarks'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{timestamp}-{args.benchmark}.log'
    print(f'Benchmark log: {log_path}', flush=True)
    with log_path.open('x', encoding='utf-8') as stream:
        print(json.dumps({'benchmark': args.benchmark, 'argv': command, 'cwd': str(cwd)}),
              file=stream, flush=True)
        try:
            with subprocess.Popen(command, cwd=cwd, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True, errors='replace') as process:
                for line in process.stdout:
                    print(line, end='', flush=True)
                    print(line, end='', file=stream, flush=True)
            status = process.returncode
            print(f'exit_code={status}', file=stream)
            return status if status >= 0 else 128 - status
        except OSError as error:
            print(f'exit_code=2\n{error}', file=stream)
            parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
