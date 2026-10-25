#!/usr/bin/env python3
"""Measure optical candidate continuity without smoothing tracker output.

Requires NumPy. Motion contributes to every trajectory metric: these are
internal diagnostics, not ground-truth accuracy or motion-to-photon latency.
OPTICAL_TIMING permits an exact sensor-clock comparison with calibrated gyro.
"""
import argparse
import collections
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np


def qmul(a, b):
    return np.concatenate([a[..., :1]*b[..., :1]-np.sum(a[..., 1:]*b[..., 1:], axis=-1, keepdims=True),
        a[..., :1]*b[..., 1:]+b[..., :1]*a[..., 1:]+np.cross(a[..., 1:], b[..., 1:])], axis=-1)


def qinv(q):
    return q*np.array([1, -1, -1, -1])


def qangle(a, b):
    q = qmul(qinv(a), b)
    return np.degrees(2*np.arctan2(np.linalg.norm(q[..., 1:], axis=-1), np.abs(q[..., 0])))


def stats(a):
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    return None if not len(a) else dict(count=len(a), rms=float(np.sqrt(np.mean(a*a))),
        median=float(np.median(a)), p95=float(np.percentile(a, 95)), p99=float(np.percentile(a, 99)), maximum=float(np.max(a)))


def read(path):
    objects = collections.defaultdict(lambda: collections.defaultdict(list))
    serials, clocks, state, bias = {}, {}, {}, {}
    options, health = {}, dict(lines=0, incomplete_rows=0, stream_integrity='complete')
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as f:
        for line in f:
            health['lines'] += 1
            if not line.endswith('\n') or '\0' in line:
                health['incomplete_rows'] += 1
                continue
            fields = line.split(maxsplit=3)
            if len(fields) != 4:
                continue
            ts, device, kind, value = fields
            ts = float(ts)
            if device == 'OPTION':
                options[kind] = value.split(maxsplit=1)[-1].strip()
            elif kind == 'CONFIG':
                serials[device] = json.loads(value).get('device_serial_number', device)
            elif kind == 'OPTICAL_TIMING':
                clocks[device] = tuple(map(int, value.split()))
            elif kind == 'EXTERNAL_POSE' and device.endswith('-raw-obs'):
                obj = device.removesuffix('-raw-obs')
                tick, hz = clocks.pop(device, (None, None))
                p = list(map(float, value.split()))
                objects[obj]['optical'].append([ts, tick/hz if tick is not None else float('nan'), *p])
            elif kind == 'POSE_STATE':
                fields = value.split()
                tick, hz = int(fields[0]), int(fields[1])
                state[device] = (tick, hz)
                objects[device]['fused'].append([ts, tick/hz, *map(float, fields[5:12])])
            elif kind == 'POSE_BIAS':
                fields = value.split()
                bias[device] = list(map(float, fields[2:5]))
            elif kind == 'I' and device in state:
                fields = value.split()
                tick, hz = state[device]
                if tick & 0xffffffff == int(fields[1]):
                    gyro = np.array(list(map(float, fields[5:8])))-bias.get(device, [0, 0, 0])
                    objects[device]['gyro'].append([tick/hz, *gyro])
    for obj, data in objects.items():
        for kind in data:
            data[kind] = np.asarray(data[kind], dtype=float)
        for kind in ('optical', 'fused'):
            if kind in data:
                q = data[kind][:, 5:9]
                length = np.linalg.norm(q, axis=1, keepdims=True)
                if not np.all(np.isfinite(data[kind][:, 2:])) or np.any(length < 1e-12):
                    raise ValueError(f'Nonfinite/zero {kind} pose in {path}')
                q /= length
    return {serials.get(obj, obj): data for obj, data in objects.items()}, options, health


def gyro_curve(gyro):
    if gyro is None or len(gyro) < 2:
        return None
    import math
    times = gyro[:, 0].copy()
    if np.any(np.diff(times) <= 0):
        return None
    qs = np.zeros((len(gyro), 4))
    qs[0, 0] = 1
    segments = np.zeros(len(gyro), dtype=int)
    for i in range(1, len(gyro)):
        dt = times[i]-times[i-1]
        segments[i] = segments[i-1]
        if not 0 < dt <= .01+1e-9:
            qs[i] = qs[i-1]
            segments[i] += 1
            continue
        ax, ay, az = (gyro[i, 1:]+gyro[i-1, 1:])*(dt/2)
        angle = math.sqrt(ax*ax+ay*ay+az*az)
        f = math.sin(angle/2)/angle if angle else .5
        v, x, y, z = math.cos(angle/2), ax*f, ay*f, az*f
        w, a, b, c = qs[i-1]
        qs[i] = [w*v-a*x-b*y-c*z, w*x+a*v+b*z-c*y,
                 w*y-a*z+b*v+c*x, w*z+a*y-b*x+c*v]
    qs /= np.linalg.norm(qs, axis=1, keepdims=True)
    return times, qs, segments


def gyro_residual(rows, curve):
    if curve is None or len(rows) < 2 or not np.all(np.isfinite(rows[:, 1])):
        return None
    times, qs, segments = curve
    ticks = rows[:, 1]
    i = np.clip(np.searchsorted(times, ticks, side='right')-1, 0, len(times)-2)
    span = times[i+1]-times[i]
    fraction = np.clip(np.divide(ticks-times[i], span, out=np.zeros_like(ticks), where=span > 0), 0, 1)
    a, b = qs[i], qs[i+1]
    d = np.sum(a*b, axis=1)
    b = b*np.where(d < 0, -1, 1)[:, None]
    theta = np.arccos(np.clip(np.abs(d), 0, 1))
    denom = np.sin(theta)
    sa = np.divide(np.sin((1-fraction)*theta), denom, out=1-fraction, where=denom > 1e-12)
    sb = np.divide(np.sin(fraction*theta), denom, out=fraction.copy(), where=denom > 1e-12)
    sampled = a*sa[:, None]+b*sb[:, None]
    predicted_delta = qmul(qinv(sampled[:-1]), sampled[1:])
    observed_delta = qmul(qinv(rows[:-1, 5:9]), rows[1:, 5:9])
    dt = np.diff(ticks)
    sample_valid = (ticks >= times[0]) & (ticks <= times[-1]) & ((fraction < 1e-9) | (span <= .01+1e-9))
    segment = np.where(fraction < 1e-9, segments[i], segments[i+1])
    valid = (dt > 0) & (dt <= .02+1e-9) & sample_valid[:-1] & sample_valid[1:]
    valid &= (segment[:-1] == segment[1:])
    return stats(qangle(predicted_delta, observed_delta)[valid])


def summarize(rows, lo, hi, gyro=None):
    rows = rows[(rows[:, 0] >= lo) & (rows[:, 0] <= hi)]
    if not len(rows):
        return dict(samples=0)
    exact_clock = np.all(np.isfinite(rows[:, 1]))
    clock = rows[:, 1] if exact_clock else rows[:, 0]
    dt = np.diff(clock)
    good = (dt > 0) & (dt <= .02+1e-9)
    p, q = rows[:, 2:5], rows[:, 5:9]
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)*1000
    angle = qangle(q[:-1], q[1:])
    # The midpoint residual vanishes for constant velocity even with uneven
    # sample times. It includes physical acceleration; it is not pure noise.
    triples = good[:-1] & good[1:]
    fraction = np.divide(dt[:-1], dt[:-1]+dt[1:], out=np.zeros_like(dt[:-1]), where=dt[:-1]+dt[1:] > 0)
    midpoint = np.linalg.norm(p[1:-1]-(p[:-2]+fraction[:, None]*(p[2:]-p[:-2])), axis=1)*1000
    valid_bins = len(np.unique(np.floor((rows[:, 0]-lo)*50)))
    result = dict(samples=len(rows), clock='sensor' if exact_clock else 'receive (optical tick unavailable)',
        first_receive_s=float(rows[0, 0]), last_receive_s=float(rows[-1, 0]),
        bins_20ms=valid_bins, dt_ms=stats(dt[dt > 0]*1000), gaps_over_20ms=int(np.sum(dt > .02+1e-9)),
        position_step_mm=stats(step[good]), rotation_step_deg=stats(angle[good]),
        position_midpoint_residual_mm=stats(midpoint[triples]),
        rotation_steps_over_5deg=int(np.sum(angle[good] > 5)),
        rotation_steps_over_30deg=int(np.sum(angle[good] > 30)),
        position_steps_over_20mm=int(np.sum(step[good] > 20)),
        position_extent_mm=(np.ptp(p, axis=0)*1000).tolist(),
        gyro_rotation_residual_deg=gyro_residual(rows, gyro))
    return result


def evaluate(path, windows):
    data, options, health = read(path)
    with path.open('rb') as source:
        digest = hashlib.file_digest(source, 'sha256').hexdigest()
    result = dict(path=str(path.resolve()), sha256=digest,
                  health=health, options=options, objects={})
    for serial, arrays in data.items():
        result['objects'][serial] = {}
        curve = gyro_curve(arrays.get('gyro'))
        for kind in ('optical', 'fused'):
            rows = arrays.get(kind)
            if rows is None or not len(rows):
                continue
            limits = dict(windows, all=(float(rows[0, 0]), float(rows[-1, 0])))
            reference_curve = curve if kind == 'fused' or options.get('report-in-imu') == '1' else None
            result['objects'][serial][kind] = {label: summarize(rows, lo, hi, reference_curve)
                                               for label, (lo, hi) in limits.items()}
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('recordings', type=Path, nargs='+')
    p.add_argument('--window', action='append', default=[], help='label:start:end in receive seconds')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    windows = {}
    for item in args.window:
        label, lo, hi = item.split(':')
        windows[label] = (float(lo), float(hi))
        if not windows[label][0] < windows[label][1]:
            p.error('Window start must precede end')
    report = dict(method='Adjacent actual samples, <=20ms sensor/receive gaps; no pose smoothing. '
        'All trajectory metrics include physical motion. Gyro residual only with exact optical sensor ticks; '
        'calibrated gyro minus recorded bias, piecewise integration. Not ground truth or physical latency.',
        recordings=[])
    for path in args.recordings:
        report['recordings'].append(evaluate(path, windows))
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(path, 'measured', flush=True)


if __name__ == '__main__':
    main()
