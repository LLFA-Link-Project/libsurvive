#!/usr/bin/env python3
"""Prepare an explicitly requested, offline optical diagnostic for the viewer.

The HTTP server never runs libsurvive. Use an input-only recording and a copy
of its libsurvive calibration, not a SteamVR calibration. Outputs are local.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cli', type=Path, required=True)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--calibration', type=Path, required=True)
    p.add_argument('--target-id', required=True, help='Viewer source ID of the original recording')
    p.add_argument('--target-file', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True,
                   help='New directory under .lh-evaluation/viewer-optical/')
    args = p.parse_args()
    cli, input_path, calibration, target = [x.expanduser().resolve() for x in
                                           (args.cli, args.input, args.calibration, args.target_file)]
    out = args.output.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(calibration, out/'calibration.initial.json')
    shutil.copyfile(calibration, out/'config.json')
    command = [str(cli), '--playback', str(input_path), '--playback-factor', '0',
               '--configfile', str(out/'config.json'), '--no-threaded-posers',
               '--globalscenesolver', '0', '--report-in-imu', '--record-tracking-state',
               '--record-pose-timing', '--record-cal-imu', '--no-gss-auto-floor-height',
               '--record', str(out/'record.rec.gz'), '--no-record-rawlight',
               '--no-record-imu', '--no-record-angle', '--kalman-minimize-state-space', '0',
               '--show-raw-obs']
    provenance = dict(kind='offline optical replay', target_id=args.target_id,
                      target_sha256=digest(target), input_sha256=digest(input_path),
                      calibration_sha256=digest(calibration), cli_sha256=digest(cli),
                      command=command, time_basis='input recording receive seconds',
                      coordinate_relation='same libsurvive calibration; GSS disabled; raw IMU before floor',
                      limitations='Reconstructed optical candidates, not the original live solver output. '
                      'Solver uses IMU/prior pose for initialization; this is not an independent ground truth.')
    # Freeze the shared libraries too in the proof, without copying/altering them.
    provenance['binary_files'] = {str(x.relative_to(cli.parent)): digest(x) for x in sorted(cli.parent.rglob('*'))
                                  if x.is_file() and (x.name == cli.name or '.so' in x.name)}
    started = time.monotonic()
    with (out/'replay.log').open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=log, timeout=900)
    provenance.update(returncode=result.returncode, wall_s=time.monotonic()-started)
    (out/'run.json').write_text(json.dumps(provenance, indent=2)+'\n')
    if result.returncode:
        raise SystemExit(f'Replay failed ({result.returncode}); see {out}/replay.log')
    provenance['record_sha256'] = digest(out/'record.rec.gz')
    (out/'viewer-source.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print(f'Optical diagnostic ready: {out}', flush=True)


if __name__ == '__main__':
    main()
