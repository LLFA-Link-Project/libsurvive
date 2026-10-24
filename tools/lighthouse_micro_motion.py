"""Summarize zero-prediction pose variation in short micro-motion recordings.

This is an output-variation metric, not tracking error: real movement contributes.
A centered 101-sample mean on a 200 Hz grid supplies the +/-0.25 s residual.
Inputs must be valid native text/gzip; do not silently repair damaged recordings.
"""
import argparse
import json, math, bisect, statistics, collections, sys, gzip
from pathlib import Path
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('input', type=Path)
parser.add_argument('--window', action='append', required=True, help='start:end seconds; repeatable')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
windows = [tuple(map(float, window.split(':'))) for window in args.window]
if any((len(window) != 2 or window[0] >= window[1] or window[1] - window[0] <= 0.5 for window in windows)):
    parser.error('each window must be start:end and longer than 0.5 seconds')
P = args.input

def mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w * v - x * i - y * j - z * k, w * i + x * v + y * k - z * j, w * j - x * k + y * v + z * i, w * k + x * j - y * i + z * v)

def norm(q):
    d = math.sqrt(sum((x * x for x in q)))
    return tuple((x / d for x in q))

def conj(q):
    return (q[0], -q[1], -q[2], -q[3])

def rv(q):
    q = norm(q)
    if q[0] < 0:
        q = tuple((-x for x in q))
    n = math.sqrt(sum((x * x for x in q[1:])))
    f = 2 * math.atan2(n, q[0]) / n if n > 1e-12 else 2
    return tuple((x * f for x in q[1:]))

def predict(q, w, h):
    a = math.sqrt(sum((x * x for x in w))) * h
    if a < 1e-12:
        return q
    n = math.sqrt(sum((x * x for x in w)))
    r = (math.cos(a / 2), *(x / n * math.sin(a / 2) for x in w))
    return norm(mul(r, q))

def metrics(t, rows, scale=1):
    n = len(rows)
    m = [statistics.mean((r[k] for r in rows)) for k in range(3)]
    tm = statistics.mean(t)
    tv = sum(((x - tm) ** 2 for x in t))
    slope = [sum(((x - tm) * (r[k] - m[k]) for x, r in zip(t, rows))) / tv for k in range(3)]
    radial = [math.sqrt(sum(((r[k] - m[k]) ** 2 for k in range(3)))) * scale for r in rows]
    detr = [sum(((r[k] - m[k] - slope[k] * (x - tm)) ** 2 for k in range(3))) for x, r in zip(t, rows)]
    half = 50
    hp = []
    prefix = [[0.0, 0.0, 0.0]]
    for row in rows:
        prefix.append([prefix[-1][k] + row[k] for k in range(3)])
    for i in range(half, n - half):
        av = [(prefix[i + half + 1][k] - prefix[i - half][k]) / (2 * half + 1) for k in range(3)]
        hp.append(sum(((rows[i][k] - av[k]) ** 2 for k in range(3))))
    return {'rms': math.sqrt(statistics.mean((x * x for x in radial))), 'p95_radius': sorted(radial)[int(0.95 * (n - 1))], 'detrended_rms': math.sqrt(statistics.mean(detr)) * scale, 'highpass_0p5s_rms': math.sqrt(statistics.mean(hp)) * scale, 'drift_per_s': math.sqrt(sum((x * x for x in slope))) * scale, 'axis_std': [statistics.pstdev((r[k] for r in rows)) * scale for k in range(3)]}

def rate_consistency(rows, imu, biases, lo, hi):
    errors = []
    g2 = []
    p2 = []
    for a, b in zip(rows, rows[1:]):
        if not lo <= a[0] < b[0] <= hi:
            continue
        dt = (b[1] - a[1]) / b[2]
        if not 0 < dt <= 0.02:
            continue
        ta, tb = (a[1] & 4294967295, b[1] & 4294967295)
        if ta not in imu or tb not in imu or b[1] not in biases:
            continue
        bias, correction = biases[b[1]]
        if len(correction) != 4 or sum((x * x for x in correction[1:])) > 1e-20 or abs(abs(correction[0]) - 1) > 1e-10:
            return {'unavailable': 'nonidentity IMUCorrection is not supported by this diagnostic'}
        delta = mul(conj(a[3]), b[3])
        rate = [x / dt for x in rv(delta)]
        gyro = [(x + y) / 2 - c for x, y, c in zip(imu[ta], imu[tb], bias)]
        errors.append(sum(((x - y) ** 2 for x, y in zip(rate, gyro))))
        g2.append(sum((x * x for x in gyro)))
        p2.append(sum((x * x for x in rate)))
    if not errors:
        return {'unavailable': 'no matched sensor ticks with calibrated IMU and bias records'}
    return {'pairs': len(errors), 'gyro_rms_rad_s': math.sqrt(statistics.mean(g2)), 'pose_rate_rms_rad_s': math.sqrt(statistics.mean(p2)), 'disagreement_rms_rad_s': math.sqrt(statistics.mean(errors)), 'note': 'Internal agreement, not ground truth or physical latency; identity IMUCorrection required.'}
pose = collections.defaultdict(list)
vel = collections.defaultdict(list)
lh = collections.defaultdict(list)
cfg = {}
events = []
opts = {}
inputs = collections.defaultdict(list)
pt = collections.defaultdict(list)
states = collections.defaultdict(list)
imus = collections.defaultdict(dict)
biases = collections.defaultdict(dict)
for l in gzip.open(P, 'rt') if P.suffix == '.gz' else P.open():
    l = l.replace('\\r\\n', '').strip()
    v = l.split()
    if len(v) < 3:
        continue
    t = float(v[0])
    dev, kind = v[1:3]
    if dev == 'OPTION':
        opts[kind] = ' '.join(v[4:])
        continue
    if kind == 'CONFIG':
        cfg[dev] = json.loads(l.split(' CONFIG ', 1)[1])
        continue
    if kind in ('POSE', 'VELOCITY'):
        values = tuple(map(float, v[3:]))
        (pose if kind == 'POSE' else vel)[dev].append((t, *values))
    if kind == 'LH_POSE':
        lh[v[-1]].append((t, *map(float, v[3:10])))
    if kind == 'INPUT_TIME':
        inputs[dev].append((t, int(v[3]), int(v[4])))
    if kind == 'I':
        imus[dev][int(v[4])] = list(map(float, v[8:11]))
    if kind == 'POSE_BIAS':
        biases[dev][int(v[3])] = (list(map(float, v[5:8])), list(map(float, v[14:18])))
    if kind == 'POSE_STATE':
        states[dev].append((t, int(v[3]), int(v[4]), tuple(map(float, v[11:15]))))
    if kind == 'POSE_TIMING':
        pt[dev].append((t, *map(int, v[3:])))
    if dev == 'INFO' and ('MPFIT success' in l or 'MPFIT status failure' in l or 'Too many failures' in l):
        events.append(l)
report = {'file': str(P), 'duration_s': max((x[-1][0] for x in pose.values())), 'pose_frame': opts.get('report-in-imu'), 'options': opts, 'calibration_events': events, 'devices': {}}
for dev, ps in pose.items():
    vs = vel[dev]
    vt = [v[0] for v in vs]
    ts = [p[0] for p in ps]
    obj = {'model': cfg[dev].get('model_number'), 'serial': cfg[dev].get('device_serial_number'), 'pose_samples': len(ps), 'velocity_samples': len(vs), 'window': {}, 'excluded_windows': {}}
    for lo, hi in windows:
        key = f'{lo:g}-{hi:g}'
        if not vs or lo < ts[0] or hi > ts[-1]:
            obj['excluded_windows'][key] = 'pose/velocity span does not cover this window'
            continue
        selected = [p for p in ps if lo <= p[0] <= hi]
        covering = ps[max(0, bisect.bisect_right(ts, lo) - 1):min(len(ps), bisect.bisect_left(ts, hi) + 1)]
        if len(selected) < 2 or not covering or covering[0][0] > lo or (covering[-1][0] < hi) or any((not 0 < b[0] - a[0] <= 0.02 for a, b in zip(covering, covering[1:]))) or any((not all(map(math.isfinite, p)) or sum((x * x for x in p[4:8])) == 0 for p in covering)):
            obj['excluded_windows'][key] = 'insufficient samples, nonincreasing timestamps, or pose gap >20 ms'
            continue
        qref = norm(selected[len(selected) // 2][4:8])
        times = [lo + i / 200 for i in range(int((hi - lo) * 200) + 1)]
        interp = []
        vinterp = []
        align = []
        for t in times:
            j = max(1, min(len(ps) - 1, bisect.bisect_left(ts, t)))
            a, b = (ps[j - 1], ps[j])
            u = (t - a[0]) / (b[0] - a[0])
            qa, qb = (norm(a[4:8]), norm(b[4:8]))
            if sum((x * y for x, y in zip(qa, qb))) < 0:
                qb = tuple((-x for x in qb))
            q = norm(tuple(((1 - u) * x + u * y for x, y in zip(qa, qb))))
            p = tuple(((1 - u) * x + u * y for x, y in zip(a[1:4], b[1:4])))
            interp.append((p, q))
            k = bisect.bisect_left(vt, t)
            k = min(range(max(0, k - 1), min(len(vs), k + 1)), key=lambda k: abs(vt[k] - t))
            vinterp.append(vs[k][1:])
            align.append(abs(vt[k] - t))
        results = {}
        for h in [0]:
            positions = [tuple((p[k] + v[k] * h for k in range(3))) for (p, q), v in zip(interp, vinterp)]
            rots = [tuple((x * 180 / math.pi for x in rv(mul(predict(q, v[3:], h), conj(qref))))) for (p, q), v in zip(interp, vinterp)]
            results[str(h)] = {'position_mm': metrics(times, positions, 1000), 'orientation_deg': metrics(times, rots)}
        win = {'native_samples': len(selected), 'hz_mean': len(selected) / (hi - lo), 'largest_pose_gap_ms': max((b[0] - a[0] for a, b in zip(selected, selected[1:]))) * 1000, 'vel_alignment_ms_p95': sorted(align)[int(0.95 * (len(align) - 1))] * 1000, 'linear_velocity_std_mm_s': math.sqrt(sum((statistics.pvariance((v[k] for v in vinterp)) for k in range(3)))) * 1000, 'angular_velocity_std_deg_s': math.sqrt(sum((statistics.pvariance((v[k] for v in vinterp)) for k in range(3, 6)))) * 180 / math.pi, 'predictions': results}
        win['gyro_pose_consistency'] = rate_consistency(states[dev], imus[dev], biases[dev], lo, hi)
        obj['window'][key] = win
    report['devices'][dev] = obj
    print(dev, obj['model'], obj['serial'])
    for key, win in obj['window'].items():
        print(key, 'hz', round(win['hz_mean'], 1), 'gap_ms', round(win['largest_pose_gap_ms'], 1), 'vstd_mm_s', round(win['linear_velocity_std_mm_s'], 2), 'wstd_deg_s', round(win['angular_velocity_std_deg_s'], 2))
        for h, r in win['predictions'].items():
            p, o = (r['position_mm'], r['orientation_deg'])
            print(h, 'p_rms/hp/drift', *[round(p[k], 4) for k in ['rms', 'highpass_0p5s_rms', 'drift_per_s']], 'q_rms/hp/drift', *[round(o[k], 4) for k in ['rms', 'highpass_0p5s_rms', 'drift_per_s']])
report['method'] = {'interpolation_hz': 200, 'prediction_s': 0, 'highpass_mean_samples': 101, 'note': 'Output variation includes physical motion. No ground-truth accuracy or latency claim. Short recordings only.'}
args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
