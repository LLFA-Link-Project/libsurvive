#!/usr/bin/env python3
"""Paired optical replay audit using immutable builds and disk-backed references.

Short captures are replayed in full, long captures use a declared prefix.
Original captures are read only. Output directories must be new; --resume
requires exactly the same binaries, sources and settings. Requires NumPy.
"""
import argparse
import concurrent.futures
import json
from pathlib import Path
import subprocess
import time

from lighthouse_replay import summarize, file_sha256
import lighthouse_optical_metrics as optical
import lighthouse_compare_disk as comparison


def build_proof(binary):
    files = {binary, *binary.parent.glob('libsurvive.so*'), *binary.parent.glob('plugins/*.so')}
    return {str(p.relative_to(binary.parent)): file_sha256(p) for p in sorted(files) if p.is_file()}


def write_json(path, value):
    temp = path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temp.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--dumps', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--reference-cache', type=Path)
    p.add_argument('--prefix-seconds', type=float, default=180)
    p.add_argument('--full-below-seconds', type=float, default=600)
    p.add_argument('--jobs', type=int, default=3)
    p.add_argument('--timeout', type=float, default=1200)
    p.add_argument('--resume', action='store_true')
    p.add_argument('--session', action='append', default=[])
    p.add_argument('--outlier-ratio', type=float, default=0)
    p.add_argument('--huber', type=float, default=-1)
    args = p.parse_args()
    if args.prefix_seconds <= 0 or args.jobs < 1 or args.timeout <= 0:
        p.error('Prefix, jobs and timeout must be positive')
    args.output.mkdir(parents=True, exist_ok=args.resume)
    builds = {'baseline': args.baseline.resolve(strict=True), 'candidate': args.candidate.resolve(strict=True)}
    settings = dict(builds={k: dict(path=str(v), sha256=build_proof(v)) for k, v in builds.items()},
        prefix_seconds=args.prefix_seconds, full_below_seconds=args.full_below_seconds,
        candidate_options=['--mpfit-light-outlier-ratio', str(args.outlier_ratio), '--mpfit-light-huber', str(args.huber)],
        evaluator_sha256={x.name: file_sha256(x) for x in [Path(__file__), Path(optical.__file__), Path(comparison.__file__)]},
        reference_is_ground_truth=False, evaluation_hz=10,
        reference_window='Full capture for full replays; declared prefix from first clock sample otherwise')
    sources = []
    skipped = []
    for folder in sorted(args.dumps.expanduser().resolve().iterdir()):
        manifest = folder/'manifest.json'
        if not manifest.is_file() or args.session and folder.name not in args.session:
            continue
        if not (folder/'usb.pcap.gz').is_file() or not (folder/'usb.pcap.gz.usbdevs').is_file():
            skipped.append(dict(session=folder.name, reason='No USB replay capture/device list; native handled separately'))
            continue
        m = json.loads(manifest.read_text())
        start, end = m.get('start_realtime_ns'), m.get('end_realtime_ns')
        duration = (end-start)/1e9 if start is not None and end is not None else None
        if duration is None:
            start, end = m.get('start_monotonic_ns'), m.get('end_monotonic_ns')
            duration = (end-start)/1e9 if start is not None and end is not None else None
        complete = duration is not None and 0 < duration <= args.full_below_seconds
        limit = 0 if complete else args.prefix_seconds
        names = ['manifest.json', 'usb.pcap.gz', 'usb.pcap.gz.usbdevs', 'poses.jsonl.gz', 'poses.jsonl',
                 'clock_sync.jsonl.gz', 'clock_sync.jsonl']
        proof = {n: dict(bytes=(folder/n).stat().st_size, sha256=file_sha256(folder/n)) for n in names if (folder/n).is_file()}
        sources.append(dict(session=folder.name, path=str(folder), status=m.get('status'), errors=m.get('errors'),
                            duration_s=duration, run_time_limit_s=limit, scope='full' if complete else 'prefix', source=proof))
    settings['sources'] = sources
    old = args.output/'settings.json'
    if old.exists() and json.loads(old.read_text()) != settings:
        p.error('Resume sources, build or evaluation settings differ')
    write_json(old, settings)
    write_json(args.output/'skipped.json', skipped)

    def run(source):
        session = source['session']
        folder = Path(source['path'])
        result = dict(source=source, runs={})
        for variant, binary in builds.items():
            out = args.output/session/variant
            out.mkdir(parents=True, exist_ok=True)
            target = out/'result.json'
            if args.resume and target.exists():
                result['runs'][variant] = json.loads(target.read_text())
                continue
            config = out/'config.json'
            if config.exists():
                raise ValueError(f'Unfinished run has config {config}; use a new output directory')
            cmd = [str(binary), '--usbmon-playback', str(folder/'usb.pcap.gz'), '--playback-factor', '0',
                   '--configfile', str(config), '--no-threaded-posers', '--no-gss-threaded',
                   '--record-pose-timing', '--show-raw-obs', '--record', str(out/'record.rec.gz'),
                   '--no-record-rawlight', '--no-record-imu', '--no-record-angle']
            if source['run_time_limit_s']:
                cmd += ['--run-time', str(source['run_time_limit_s'])]
            if variant == 'candidate':
                cmd += settings['candidate_options']
            report = dict(command=cmd)
            started = time.monotonic()
            try:
                with (out/'replay.log').open('w') as log:
                    proc = subprocess.run(cmd, stdout=log, stderr=log, timeout=args.timeout)
                report.update(returncode=proc.returncode, wall_s=time.monotonic()-started,
                              pose_summary=summarize(out/'record.rec.gz'))
                if proc.returncode == 0:
                    report['optical'] = optical.evaluate(out/'record.rec.gz', {})
                    cache = args.reference_cache
                    if cache and (cache/(session+'.json')).exists():
                        ref = json.loads((cache/(session+'.json')).read_text())
                        current = source['source'].get('poses.jsonl.gz', source['source'].get('poses.jsonl', {}))
                        if ref.get('source_sha256') != current.get('sha256'):
                            raise ValueError('Reference cache source SHA mismatch')
                        if 'error' in ref:
                            report['reference_unavailable'] = ref['error']
                        else:
                            report['reference'] = comparison.run(folder, out/'record.rec.gz', out/'comparison.sqlite',
                                evaluation_hz=10, allow_truncated=source['status'] != 'complete',
                                reference_database=cache/(session+'.sqlite'), reference_info=ref,
                                duration_s=source['run_time_limit_s'] or None)
            except Exception as error:
                report.update(error=str(error), wall_s=time.monotonic()-started)
            write_json(target, report)
            result['runs'][variant] = report
            print(session, variant, report.get('error',report.get('returncode')), round(report['wall_s'],1), flush=True)
        return result

    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run, source): source['session'] for source in sources}
        for future in concurrent.futures.as_completed(futures):
            results.append(future.result())
            write_json(args.output/'results.json', dict(settings=settings, skipped=skipped, sessions=sorted(results,key=lambda x:x['source']['session']),
                                                       completed=len(results), planned=len(sources)))


if __name__ == '__main__':
    main()
