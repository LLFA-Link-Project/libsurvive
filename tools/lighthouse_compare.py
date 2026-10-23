#!/usr/bin/env python3
"""Compare timed libsurvive replay output with native Valve reference estimates.

Uses only the Python standard library. No files in the source session are changed.
Quaternion helpers use wxyz internally; native reference files use xyzw.
"""
import argparse
import bisect
import collections
import gzip
import json
import math
import pathlib
import statistics


def jsonl(path):
    compressed = pathlib.Path(str(path) + ".gz")
    opener = gzip.open if compressed.exists() else open
    with opener(compressed if compressed.exists() else path, "rt") as f:
        for line in f:
            yield json.loads(line)


def qmul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j, w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def qinv(q):
    return (q[0], -q[1], -q[2], -q[3])


def unit(q):
    if len(q) != 4 or not all(v is not None and math.isfinite(v) for v in q):
        raise ValueError("Invalid quaternion")
    norm = math.sqrt(sum(v*v for v in q))
    if abs(norm-1) > .01:
        raise ValueError("Non-unit quaternion")
    return tuple(v/norm for v in q)


def rotate(q, p):
    return qmul(qmul(q, (0, *p)), qinv(q))[1:]


def add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x-y for x, y in zip(a, b))


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def angle(a, b):
    return 2*math.acos(min(1, abs(sum(x*y for x, y in zip(a, b)))))


def compose(a, b):
    p, q = a
    r, s = b
    return add(p, rotate(q, r)), qmul(q, s)


def xyzw(q):
    return unit((q[3], *q[:3]))


class ClockMap:
    """Bracketed clock interpolation, rejecting steps, overlap and extrapolation."""
    def __init__(self, samples):
        intervals = []
        for a, b in zip(samples, samples[1:]):
            dt = b['monotonic_before_ns']-a['monotonic_after_ns']
            change = abs(b['offset_realtime_minus_monotonic_ns']-a['offset_realtime_minus_monotonic_ns'])
            limit = 1_000_000 + max(0, dt)*.0005 + a['uncertainty_ns']+b['uncertainty_ns']
            valid = dt > 0 and b['realtime_ns'] > a['realtime_ns'] and change <= limit
            intervals.append((min(a['realtime_ns'], b['realtime_ns']),
                              max(a['realtime_ns'], b['realtime_ns']), valid, a, b))
        self.intervals = sorted(intervals, key=lambda x: x[0])
        self.starts = [i[0] for i in self.intervals]
        self.maximum_end = []
        for interval in self.intervals:
            self.maximum_end.append(max(interval[1], self.maximum_end[-1] if self.maximum_end else 0))

    def convert(self, realtime_ns):
        i = bisect.bisect_right(self.starts, realtime_ns)-1
        candidates = []
        while i >= 0 and self.maximum_end[i] >= realtime_ns:
            if self.intervals[i][1] >= realtime_ns:
                candidates.append(self.intervals[i])
            i -= 1
        if len(candidates) != 1 or not candidates[0][2]:
            return None
        _, _, _, a, b = candidates[0]
        change = b['offset_realtime_minus_monotonic_ns']-a['offset_realtime_minus_monotonic_ns']
        offset = a['offset_realtime_minus_monotonic_ns'] + round(
            (realtime_ns-a['realtime_ns'])/(b['realtime_ns']-a['realtime_ns'])*change)
        error = max(a['uncertainty_ns'], b['uncertainty_ns'])+abs(change)+1000
        return realtime_ns-offset, error


def largest_eigenvector(matrix):
    # Jacobi diagonalization avoids power-iteration failures with tied magnitudes.
    a = [list(row) for row in matrix]
    v = [[float(i == j) for j in range(4)] for i in range(4)]
    for _ in range(100):
        p, q = max(((i, j) for i in range(4) for j in range(i+1, 4)), key=lambda ij: abs(a[ij[0]][ij[1]]))
        if abs(a[p][q]) < 1e-12:
            break
        theta = .5*math.atan2(2*a[p][q], a[q][q]-a[p][p])
        c, s = math.cos(theta), math.sin(theta)
        for i in range(4):
            a[i][p], a[i][q] = c*a[i][p]-s*a[i][q], s*a[i][p]+c*a[i][q]
            v[i][p], v[i][q] = c*v[i][p]-s*v[i][q], s*v[i][p]+c*v[i][q]
        for j in range(4):
            a[p][j], a[q][j] = c*a[p][j]-s*a[q][j], s*a[p][j]+c*a[q][j]
    k = max(range(4), key=lambda i: a[i][i])
    return unit(tuple(v[i][k] for i in range(4)))


def fit(pairs, orientation_scale=.25):
    """Common rigid fit, balancing positions and three 25cm orientation axes."""
    if len(pairs) < 3:
        raise ValueError("Too few alignment samples")
    # Equal total weight per object, rather than favoring high-rate devices.
    counts = collections.Counter(row[0] for row in pairs)
    weights = [1/counts[row[0]] for row in pairs]
    total = sum(weights)
    ca = tuple(sum(w*row[1][0][i] for w, row in zip(weights, pairs))/total for i in range(3))
    cb = tuple(sum(w*row[2][0][i] for w, row in zip(weights, pairs))/total for i in range(3))
    h = [[0.0]*3 for _ in range(3)]
    for w, (_, (p, q), (r, s), _) in zip(weights, pairs):
        vectors = [(subtract(p, ca), subtract(r, cb), 1)]
        vectors += [(rotate(q, axis), rotate(s, axis), orientation_scale**2)
                    for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
        for u, v, scale in vectors:
            for i in range(3):
                for j in range(3):
                    h[i][j] += w*scale*u[i]*v[j]
    a, b, c = h[0]
    d, e, f = h[1]
    g, i, j = h[2]
    q = largest_eigenvector([[a+e+j, f-i, g-c, b-d], [f-i, a-e-j, b+d, g+c],
                             [g-c, b+d, e-a-j, f+i], [b-d, g+c, f+i, j-a-e]])
    return subtract(cb, rotate(q, ca)), q


def interpolate(samples, times, t, max_gap_ns=20_000_000):
    i = bisect.bisect_left(times, t)
    if i < len(times) and times[i] == t:
        return samples[i][1]
    if i == 0 or i == len(times):
        return None
    ta, a = samples[i-1]
    tb, b = samples[i]
    if a is None or b is None or tb-ta > max_gap_ns:
        return None
    u = (t-ta)/(tb-ta)
    p = tuple(x+(y-x)*u for x, y in zip(a[0], b[0]))
    dot = sum(x*y for x, y in zip(a[1], b[1]))
    q2 = tuple(x*(-1 if dot < 0 else 1) for x in b[1])
    q = tuple(x*(1-u)+y*u for x, y in zip(a[1], q2))
    norm = math.sqrt(sum(x*x for x in q))
    return p, tuple(x/norm for x in q)


def load_replay(path, clock, sample_hz=50):
    data = collections.defaultdict(lambda: {'poses': [], 'inputs': [], 'serial': None})
    with gzip.open(path, 'rt') as f:
        for line in f:
            row = line.split(maxsplit=3)
            if len(row) != 4:
                continue
            obj = data[row[1]]
            if row[2] == 'CONFIG':
                serial = json.loads(row[3]).get('device_serial_number')
                if obj['serial'] is not None and serial != obj['serial']:
                    raise ValueError('Receiver changed tracked object; split replay at config change')
                obj['serial'] = serial
            elif row[2] == 'POSE':
                values = tuple(map(float, row[3].split()))
                if len(values) != 7 or not all(math.isfinite(v) for v in values):
                    raise ValueError('Invalid replay pose components')
                obj['pending_pose'] = (values[:3], unit(values[3:]))
            elif row[2] == 'POSE_TIMING':
                tick, hz = map(int, row[3].split()[:2])
                obj['poses'].append((tick, hz, obj.pop('pending_pose')))
            elif row[2] == 'INPUT_TIME':
                tick, receive = map(int, row[3].split())
                if tick > 0:
                    obj['inputs'].append((tick, receive*1000))
    mapped, counts = {}, collections.Counter()
    input_receives = [ns for obj in data.values() for _, ns in obj['inputs']]
    if not input_receives:
        raise ValueError('No input receive times; enable --record-pose-timing')
    for label, receive in [('capture_start_monotonic_ns', min(input_receives)),
                           ('capture_end_monotonic_ns', max(input_receives))]:
        converted = clock.convert(receive)
        if converted is None:
            raise ValueError('Input time span is not bracketed by clock samples')
        counts[label] = converted[0]
    clock_errors, arrival_spreads = [], []
    for obj in data.values():
        if not obj['poses']:
            continue
        if not obj['serial'] or not obj['inputs']:
            raise ValueError('Missing object config or input timing; enable --record-pose-timing')
        inputs = sorted(obj['inputs'])
        ticks = [row[0] for row in inputs]
        output = []
        previous = None
        for tick, hz, pose in obj['poses']:
            k = bisect.bisect_left(ticks, tick)
            anchors = [(tc, ns) for tc, ns in inputs[max(0, k-8):k+8] if abs(tc-tick)/hz < .05]
            if not anchors:
                counts['unbracketed_sensor_time'] += 1
                continue
            estimates = [ns + round((tick-tc)*1e9/hz) for tc, ns in anchors]
            estimate = int(statistics.median(estimates))
            spread = max(estimates)-min(estimates)
            if spread > 20_000_000:
                counts['ambiguous_sensor_clock'] += 1
                continue
            converted = clock.convert(estimate)
            if converted is None:
                counts['unmapped_clock'] += 1
                continue
            ns, error = converted
            if previous is not None and ns-previous < 1e9/sample_hz:
                continue
            previous = ns
            output.append((ns, pose))
            clock_errors.append(error)
            arrival_spreads.append(spread)
        mapped[obj['serial']] = sorted(output)
    counts['max_clock_mapping_estimate_ns'] = max(clock_errors, default=0)
    counts['max_local_input_arrival_spread_ns'] = max(arrival_spreads, default=0)
    return mapped, dict(counts)


def load_reference(session, time_field):
    data, rejected = collections.defaultdict(list), collections.Counter()
    for row in jsonl(session/'poses.jsonl'):
        if row.get('device_class') == 4:
            continue
        t = row.get(time_field)
        if t is None:
            rejected['missing_time'] += 1
            continue
        pose = None
        if row['pose_is_valid'] and row['device_is_connected'] and row['tracking_result'] == 200:
            try:
                p = row['position_m']
                if not all(v is not None and math.isfinite(v) for v in p):
                    raise ValueError('Nonfinite position')
                pose = compose((row['world_from_driver_translation_m'], xyzw(row['world_from_driver_orientation_xyzw'])),
                               compose((p, xyzw(row['orientation_xyzw'])),
                                       (row['driver_from_head_translation_m'], xyzw(row['driver_from_head_orientation_xyzw']))))
            except (ValueError, TypeError):
                rejected['invalid_components'] += 1
        else:
            rejected['invalid_or_not_running_ok'] += 1
        data[row['device_id']].append((t, pose))
    return {key: sorted(value, key=lambda row: row[0]) for key, value in data.items()}, dict(rejected)


def distribution(values):
    if not values:
        return None
    s = sorted(values)
    return {'n': len(s), 'rmse': math.sqrt(sum(v*v for v in s)/len(s)),
            'median': statistics.median(s), 'p95': s[round((len(s)-1)*.95)], 'max': s[-1]}


def load_reference_events(session):
    states, summary, reacquisitions = {}, collections.defaultdict(collections.Counter), collections.defaultdict(list)
    events = sorted((row for row in jsonl(session/'events.jsonl')
                     if row.get('event') == 'driver_state_changed'), key=lambda row: row['capture_time_ns'])
    ever_tracked = set()
    for row in events:
        serial = row['device_id']
        connected = row['device_is_connected']
        valid = connected and row['pose_is_valid'] and row['tracking_result'] == 200
        previous = states.get(serial)
        if previous is not None:
            summary[serial]['lost_tracking'] += previous[1] and not valid
            summary[serial]['disconnected'] += previous[0] and not connected
            summary[serial]['reconnected'] += not previous[0] and connected
            if valid and not previous[1] and serial in ever_tracked:
                summary[serial]['reacquired'] += 1
                reacquisitions[serial].append(row['capture_time_ns'])
        if valid:
            ever_tracked.add(serial)
        states[serial] = connected, valid
    return {'counts': {key: dict(value) for key, value in summary.items()},
            'reacquisition_capture_times_ns': dict(reacquisitions)}


def compare(replay, reference, warmup_s=10, lag_s=0, bounds=None, events=None):
    if bounds:
        start, end = bounds
    else:
        start = min(samples[0][0] for samples in replay.values() if samples)
        end = max(samples[-1][0] for samples in replay.values() if samples)
    begin = start + round(warmup_s*1e9)
    split = begin + (end-begin)//3
    pairs, coverage = [], {}
    for serial in sorted(set(replay) | set(reference)):
        samples = replay.get(serial, [])
        refs = reference.get(serial, [])
        times = [row[0] for row in refs]
        for t, pose in samples:
            if t < begin:
                continue
            ref = interpolate(refs, times, t+round(lag_s*1e9))
            if ref:
                pairs.append((serial, pose, ref, t))
        # Coverage denominator is valid reference time, not emitted replay poses.
        libtimes = [row[0] for row in samples]
        good = total = 0
        previous = 0
        for t, pose in refs:
            t -= round(lag_s*1e9)
            if pose is None or t < begin or t > end or t-previous < 20_000_000:
                continue
            previous = t
            total += 1
            k = bisect.bisect_left(libtimes, t)
            nearest = min((abs(libtimes[j]-t) for j in (k-1, k) if 0 <= j < len(libtimes)), default=1e20)
            good += nearest < 20_000_000
        coverage[serial] = {'matched_reference_bins': good, 'valid_reference_bins': total,
                            'fraction': good/total if total else None,
                            'first_replay_pose_after_start_s': (samples[0][0]-start)/1e9 if samples else None,
                            'first_valid_reference_after_start_s':
                                next(((t-start)/1e9 for t, p in refs if t >= start and p is not None), None)}
    training = [row for row in pairs if row[3] < split]
    transform = fit(training)
    per_object = {}
    for serial in coverage:
        rows = [row for row in pairs if row[0] == serial and row[3] >= split]
        positions, angles = [], []
        errors = []
        for _, pose, ref, t in rows:
            predicted = compose(transform, pose)
            positions.append(distance(predicted[0], ref[0]))
            angles.append(math.degrees(angle(predicted[1], ref[1])))
            errors.append((t, positions[-1], angles[-1]))
        per_object[serial] = {'position_difference_m': distribution(positions),
                              'orientation_difference_deg': distribution(angles), 'coverage': coverage[serial],
                              'association': ('no_replay_poses' if not replay.get(serial) else
                                              'no_reference_poses' if not reference.get(serial) else 'matched')}
        if events:
            recovery = []
            for event_ns in events['reacquisition_capture_times_ns'].get(serial, []):
                if not split <= event_ns <= end:
                    continue
                streak, previous, recovered = [], None, None
                for t, pos, ang in errors:
                    if t < event_ns:
                        continue
                    if t-event_ns > 5e9:
                        break
                    if pos <= .05 and ang <= 5:
                        if previous is None or t-previous > 50_000_000:
                            streak = []
                        streak.append(t)
                        if len(streak) == 3:
                            recovered = (streak[0]-event_ns)/1e9
                            break
                        previous = t
                    else:
                        streak, previous = [], None
                recovery.append({'reference_event_monotonic_ns': event_ns, 'delay_s': recovered})
            per_object[serial]['recovery_after_reference_reacquisition'] = {
                'criterion': '3 samples within 50mm and 5deg; gaps <=50ms; search <=5s; diagnostic, not equivalence',
                'events': recovery}
    return {'reference_is_ground_truth': False, 'fitted_time_lag_s': lag_s,
            'time_lag_was_fitted': False, 'warmup_s': warmup_s, 'alignment_train_end_monotonic_ns': split,
            'alignment_train_samples': len(training), 'alignment_method': 'common SE3; positions + 0.25m orientation axes',
            'world_from_survive': {'position_m': transform[0], 'orientation_wxyz': transform[1]},
            'evaluation_samples': sum(row[3] >= split for row in pairs), 'objects': per_object,
            'reference_state_events': events}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=pathlib.Path)
    parser.add_argument('recording', type=pathlib.Path)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--warmup', type=float, default=10)
    parser.add_argument('--lag', type=float, default=0, help='Explicit reference-minus-replay lag, seconds; not fitted')
    parser.add_argument('--reference-time', choices=['pose_time_estimate_ns', 'capture_time_ns'], default='pose_time_estimate_ns')
    args = parser.parse_args()
    manifest = json.loads((args.session/'manifest.json').read_text())
    if manifest.get('status') != 'complete' or manifest.get('errors'):
        parser.error('A finalized session without recording errors is required')
    clock = ClockMap(list(jsonl(args.session/'clock_sync.jsonl')))
    replay, timing = load_replay(args.recording, clock)
    reference, rejected = load_reference(args.session, args.reference_time)
    result = compare(replay, reference, args.warmup, args.lag,
                     bounds=(timing['capture_start_monotonic_ns'], timing['capture_end_monotonic_ns']),
                     events=load_reference_events(args.session))
    result.update(session=args.session.name, reference_time=args.reference_time, timing=timing,
                  reference_rejected=rejected, recording_loss=manifest.get('loss'),
                  timing_caveat='Sensor ticks mapped to local USB arrival times; sensor/driver latency and prediction are unknown')
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(result['objects'], indent=2))


if __name__ == '__main__':
    main()
