"""Summarize stationary transitions in native CLI replays (requires NumPy).

Unlike live captures, CLI replays need not contain INPUT_TIME. A calibrated I
sample is included only when its tick matches the most recent POSE_STATE tick.
Actual hand motion contributes to every pose-variation metric.
"""

import argparse
import collections
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import lighthouse_body_drift as body


def read_native(path, serial):
    rows = collections.defaultdict(list)
    configs, options, previous, worst = {}, {}, {}, {}
    device, last_tick = None, None
    pose_hash = hashlib.sha256()
    device_hashes = collections.defaultdict(hashlib.sha256)
    counts = collections.Counter()
    with gzip.open(path, 'rt') as stream:
        for line in stream:
            if '\0' in line or not line.endswith('\n'):
                raise ValueError('damaged native row')
            f = line.split()
            t, d, kind = float(f[0]), f[1], f[2]
            if d == 'OPTION':
                options[kind] = ' '.join(f[4:])
            if kind == 'CONFIG':
                configs[d] = json.loads(line.split(' CONFIG ', 1)[1])
                if configs[d].get('device_serial_number') == serial:
                    device = d
            if kind == 'POSE':
                pose_hash.update((' '.join(f[1:]) + '\n').encode())
                device_hashes[d].update((' '.join(f[1:]) + '\n').encode())
                counts[d] += 1
            if kind == 'POSE_STATE':
                tick, hz = int(f[3]), int(f[4])
                p = list(map(float, f[8:15]))
                if not all(map(math.isfinite, p)) or sum(x * x for x in p[3:]) < 1e-20:
                    raise ValueError('nonfinite pose or zero quaternion')
                if d in previous:
                    pt, previous_tick, pp = previous[d]
                    dt = (tick - previous_tick) / hz
                    step = math.sqrt(sum((x - y) ** 2 for x, y in zip(p[:3], pp[:3]))) * 1000
                    if 0 < dt <= .02 and step > worst.get(d, {}).get('position_step_mm', -1):
                        worst[d] = dict(receive_s=t, previous_receive_s=pt, sensor_tick=tick,
                                        sensor_dt_ms=dt * 1000, position_step_mm=step)
                previous[d] = (t, tick, p)
            if d != device:
                continue
            if kind == 'POSE_STATE':
                last_tick = int(f[3])
                rows['state'].append([t, *map(float, f[3:15])])
            elif kind == 'POSE_BIAS':
                rows['bias'].append([t, *map(float, f[3:18])])
            elif kind == 'POSE_BIAS_VAR':
                rows['var'].append([t, *map(float, f[3:10])])
            elif kind == 'VELOCITY':
                rows['vel'].append([t, last_tick, *map(float, f[3:9])])
            elif kind == 'I' and last_tick is not None and (last_tick & 0xffffffff) == int(f[4]):
                rows['imu'].append([t, last_tick, *map(float, f[5:11])])
    a, joins = body.join_rows({k: np.asarray(v) for k, v in rows.items()})
    return a, dict(input=str(path.resolve()), input_sha256=body.sha256(path), serial=serial,
                   native_name=device, options=options, joins=joins, pose_sha256=pose_hash.hexdigest(),
                   pose_hashes_by_device={d: h.hexdigest() for d, h in device_hashes.items()},
                   pose_counts=dict(counts), worst_position_steps=worst,
                   serials={d: c.get('device_serial_number') for d, c in configs.items()})


def highpass(a, lo, hi):
    s = a['state']
    s = s[(s[:, 0] >= lo) & (s[:, 0] <= hi)]
    ts = s[:, 1] / s[:, 2]
    grid = np.arange(ts[0], ts[-1], .005)
    if len(grid) < 103:
        return dict(unavailable='window too short for 101-sample centered mean')
    p = np.stack([np.interp(grid, ts, s[:, 6 + i]) for i in range(3)], axis=-1)
    q = s[:, 9:13].copy()
    signs = np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1, 1)
    q[1:] *= np.cumprod(signs)[:, None]
    q = np.stack([np.interp(grid, ts, q[:, i]) for i in range(4)], axis=-1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    angles = body.rotation_vector(body.relative_rotation(q[len(q) // 2], q))

    def residual_rms(v):
        av = np.stack([np.convolve(v[:, i], np.ones(101) / 101, 'valid') for i in range(3)], axis=-1)
        return body.rms(v[50:-50] - av)

    return dict(position_hp_mm=residual_rms(p) * 1000,
                orientation_hp_deg=residual_rms(angles) * 180 / np.pi
                if np.max(np.linalg.norm(angles, axis=1)) < np.pi / 6 else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('native', type=Path)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--window', action='append', required=True, help='label:start:end receive seconds')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    a, report = read_native(args.native, args.serial)
    report['windows'] = {}
    for spec in args.window:
        label, lo, hi = spec.split(':')
        lo, hi = float(lo), float(hi)
        result = body.summarize(a, lo, hi)
        result.update(highpass(a, lo, hi))
        report['windows'][label] = result
    report['method'] = dict(
        imu_join='calibrated I low-32-bit tick must match the latest full-tick POSE_STATE; other I samples excluded',
        highpass='200 Hz sensor-time interpolation; subtract centered 101-sample mean; no extrapolation',
        orientation='local quaternion log; omitted for excursions >=30 degrees from window midpoint',
        note='Pose variation includes actual movement; internal agreement is not physical accuracy or latency.',
    )
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
