"""Compare native IMU residuals, body velocity and stationary-update pose steps.

Requires NumPy; --plot also requires Matplotlib. This measures internal
consistency, not ground-truth tracking error. It never modifies a capture.
"""

import argparse
import collections
import gzip
import hashlib
import json
import sys
from pathlib import Path

import numpy as np


def conjugate(q):
    return q * np.array([1, -1, -1, -1])


def rotate(q, v):
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    return v + 2 * np.cross(q[..., 1:], np.cross(q[..., 1:], v) + q[..., :1] * v)


def rotation_vector(q):
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    q = q * np.where(q[..., :1] < 0, -1, 1)
    n = np.linalg.norm(q[..., 1:], axis=-1, keepdims=True)
    return q[..., 1:] * np.divide(
        2 * np.arctan2(n, q[..., :1]), n, out=np.full_like(n, 2), where=n > 1e-12
    )


def relative_rotation(a, b):
    a = conjugate(a)
    return np.concatenate([
        a[..., :1] * b[..., :1] - np.sum(a[..., 1:] * b[..., 1:], axis=-1, keepdims=True),
        a[..., :1] * b[..., 1:] + b[..., :1] * a[..., 1:] + np.cross(a[..., 1:], b[..., 1:]),
    ], axis=-1)


def rms(v):
    return float(np.sqrt(np.mean(np.sum(v * v, axis=-1))))


def cosine(a, b):
    n = np.linalg.norm(a) * np.linalg.norm(b)
    return float(np.dot(a, b) / n) if n > 1e-20 else None


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read_capture(session, serial, allow_truncated):
    arrays = collections.defaultdict(list)
    options, configs, last_state, pending_imu = {}, {}, {}, {}
    device = None
    anchors, events = [], []
    counts = collections.Counter()
    eof_error, incomplete, line_count, last_receive = None, 0, 0, None
    native = session / 'native.rec.gz'
    input_sha = sha256(native)
    # INPUT_TIME follows the matching calibrated I record. Join the full tick
    # here, rather than using a global low-32-bit dictionary across clock wraps.
    try:
        with gzip.open(native, 'rt') as stream:
            for line_count, line in enumerate(stream, 1):
                if not line.endswith('\n'):
                    incomplete += 1
                    continue
                f = line.split()
                if len(f) < 3 or '\0' in line:
                    raise ValueError(f'invalid native row {line_count}')
                t, d, kind = float(f[0]), f[1], f[2]
                if not np.isfinite(t):
                    raise ValueError(f'nonfinite timestamp at row {line_count}')
                last_receive = t
                counts['OPTION' if d == 'OPTION' else kind] += 1
                if d == 'OPTION':
                    options[kind] = ' '.join(f[4:])
                    continue
                if kind == 'CONFIG':
                    configs[d] = json.loads(line.split(' CONFIG ', 1)[1])
                    if configs[d].get('device_serial_number') == serial:
                        if device is not None and device != d:
                            raise ValueError('serial appears with multiple native names')
                        device = d
                if kind == 'INPUT_TIME' and len(anchors) < 32:
                    anchors.append(int(f[4]) * 1000 - t * 1e9)
                if kind == 'LH_POSE' or d == 'INFO':
                    events.append(line.strip())
                if d != device:
                    continue
                if kind == 'POSE_STATE':
                    if len(f) != 15:
                        raise ValueError('unexpected POSE_STATE width')
                    arrays['state'].append([t, *map(float, f[3:15])])
                    last_state[d] = (t, int(f[3]))
                elif kind in ('POSE_BIAS', 'POSE_BIAS_VAR'):
                    if len(f) != (18 if kind == 'POSE_BIAS' else 10):
                        raise ValueError('unexpected bias row width')
                    arrays['bias' if kind == 'POSE_BIAS' else 'var'].append([t, *map(float, f[3:])])
                elif kind == 'VELOCITY':
                    if len(f) != 9 or d not in last_state or t < last_state[d][0]:
                        raise ValueError('unexpected velocity row/order')
                    arrays['vel'].append([t, last_state[d][1], *map(float, f[3:9])])
                elif kind == 'I':
                    if d in pending_imu or len(f) != 15:
                        raise ValueError('unexpected calibrated IMU row/order')
                    pending_imu[d] = [t, int(f[4]), *map(float, f[5:11])]
                elif kind == 'INPUT_TIME':
                    imu = pending_imu.pop(d, None)
                    tick = int(f[3])
                    if imu is None or imu[1] != (tick & 0xffffffff) or t < imu[0]:
                        raise ValueError('IMU/INPUT_TIME tick or order mismatch')
                    arrays['imu'].append([imu[0], tick, *imu[2:]])
                elif kind == 'B':
                    if len(f) != 8:
                        raise ValueError('unexpected calibrated light row width')
                    # Gen2 B: channel sensor tick plane angle. The plane is
                    # neither a lighthouse index nor a channel identifier.
                    arrays['light'].append([t, int(f[3]), int(f[4])])
    except EOFError as error:
        if not allow_truncated:
            raise
        eof_error = str(error)
    if incomplete and not allow_truncated:
        raise ValueError('incomplete final native row')
    if device is None or not anchors:
        raise ValueError('serial or INPUT_TIME anchors missing')
    if sha256(native) != input_sha:
        raise ValueError('source changed while reading')
    epoch = float(np.median(anchors))
    markers = []
    if (session / 'scene-markers.jsonl').exists():
        with gzip.open(session / 'clock_sync.jsonl.gz', 'rt') as stream:
            clock = json.loads(next(stream))
        for line in (session / 'scene-markers.jsonl').read_text().splitlines():
            m = json.loads(line)
            markers.append(dict(label=m['label'], native_receive_s=(
                m['capture_time_ns'] + clock['offset_realtime_minus_monotonic_ns'] - epoch
            ) / 1e9))
    meta = dict(
        source=str(session.resolve()), serial=serial, native_name=device,
        model=configs[device].get('model_number'), native_sha256=input_sha,
        complete_gzip=eof_error is None, gzip_eof_error=eof_error,
        incomplete_final_rows_excluded=incomplete, pending_imu_excluded=len(pending_imu),
        complete_native_rows=line_count - incomplete, last_receive_s=last_receive,
        record_counts=dict(counts), options=options, markers=markers, scene_and_info_records=events,
        clock_mapping=dict(native_epoch_ns=epoch, anchor_range_us=(max(anchors) - min(anchors)) / 1000),
        companion_sha256={n: sha256(session / n) for n in [
            'calibration.initial.json', 'scene-markers.jsonl', 'clock_sync.jsonl.gz',
        ] if (session / n).exists()},
    )
    return {k: np.asarray(v) for k, v in arrays.items()}, meta


def join_rows(arrays):
    for name in ('state', 'bias', 'var', 'imu', 'vel'):
        a = arrays[name]
        if not np.all(np.isfinite(a)) or not np.all(np.diff(a[:, 1]) > 0):
            raise ValueError(f'{name}: nonfinite data or nonincreasing sensor ticks')
    state = arrays['state']
    original_count = len(state)
    keep = np.ones(len(state), dtype=bool)
    for name in ('bias', 'var', 'imu', 'vel'):
        keep &= np.isin(state[:, 1], arrays[name][:, 1])
    state = state[keep]
    if not len(state):
        raise ValueError('no full-tick-matched rows')
    joined = {'state': state}
    for name in ('bias', 'var', 'imu', 'vel'):
        a = arrays[name]
        joined[name] = a[np.searchsorted(a[:, 1], state[:, 1])]
    q, bias, imu = state[:, 9:13], joined['bias'], joined['imu']
    if np.any(np.linalg.norm(q, axis=1) < 1e-12) or np.any(bias[:, 9:12] <= 0):
        raise ValueError('invalid quaternion or accelerometer scale')
    if np.max(np.abs(bias[:, 12:16] - [1, 0, 0, 0])) > 1e-10:
        raise ValueError('this body/gyro comparison requires identity IMUCorrection')
    joined['residual'] = (imu[:, 2:5] - bias[:, 6:9]) / bias[:, 9:12] - rotate(conjugate(q), [0, 0, 1])
    joined['gyro'] = imu[:, 5:8] - bias[:, 3:6]
    joined['body_velocity'] = rotate(conjugate(q), joined['vel'][:, 2:5])
    joined['light'] = arrays.get('light', np.empty((0, 3)))
    # Use sensor time for differentiation; receive time only selects labelled windows.
    dt = np.diff(state[:, 1]) / state[:-1, 2]
    if np.any(state[:, 2] != state[0, 2]) or np.any(dt <= 0):
        raise ValueError('changing timebase or invalid tick delta')
    normalized_q = q / np.linalg.norm(q, axis=1, keepdims=True)
    dot = np.sum(normalized_q[:-1] * normalized_q[1:], axis=1, keepdims=True)
    midpoint_q = normalized_q[:-1] + normalized_q[1:] * np.where(dot < 0, -1, 1)
    vout = rotate(conjugate(midpoint_q), np.diff(state[:, 6:9], axis=0) / dt[:, None])
    vmodel = rotate(conjugate(midpoint_q), (joined['vel'][:-1, 2:5] + joined['vel'][1:, 2:5]) / 2)
    joined.update(dt=dt, vout=vout, vmodel=vmodel, feedback=vout - vmodel)
    return joined, dict(native_state_rows=original_count, matched_rows=len(state),
                        unmatched_rows_excluded=int(sum(~keep)), quaternion_sign_changes=int(sum(dot[:, 0] < 0)))


def summarize(a, lo, hi):
    s, bias, r, dt = a['state'], a['bias'], a['residual'], a['dt']
    t, q = s[:, 0], s[:, 9:13]
    k = (t >= lo) & (t <= hi)
    pairs = (t[:-1] >= lo) & (t[1:] <= hi)
    if sum(k) < 3 or not np.any(pairs):
        raise ValueError('window has insufficient matched rows')
    if t[0] > lo or t[-1] < hi or np.max(dt[pairs]) > .02 or np.max(np.diff(t)[pairs]) > .02:
        raise ValueError('window not covered or sensor/receive gap exceeds 20 ms')
    mean = lambda v: np.mean(v[k], axis=0)
    pairmean = lambda v: np.average(v[pairs], weights=dt[pairs], axis=0)
    residual, feedback = mean(r), pairmean(a['feedback'])
    active = s[:, 4] > .1 * s[:, 2]
    onset = np.flatnonzero(active[1:] & ~active[:-1]) + 1
    onset = onset[(t[onset - 1] >= lo) & (t[onset] <= hi)]
    step = rotate(conjugate(q[:-1]), np.diff(s[:, 6:9], axis=0)) * 1000
    step_norm = np.linalg.norm(step, axis=1)
    events = []
    for i in onset:
        before = a['body_velocity'][i - 1] * 1000
        events.append(dict(
            receive_s=float(t[i]), previous_receive_s=float(t[i - 1]),
            sensor_tick=int(s[i, 1]), previous_sensor_tick=int(s[i - 1, 1]),
            sensor_dt_ms=float(dt[i - 1] * 1000), step_body_mm=step[i - 1].tolist(),
            step_mm=float(step_norm[i - 1]), velocity_before_body_mm_s=before.tolist(),
            velocity_after_body_mm_s=(a['body_velocity'][i] * 1000).tolist(),
            residual_before_body_mg=(r[i - 1] * 1000).tolist(),
            step_against_velocity_cosine=cosine(step[i - 1], -before),
            step_against_residual_cosine=cosine(step[i - 1], -r[i - 1]),
            gyro_norm_rad_s=float(np.linalg.norm(a['gyro'][i])),
            orientation_step_deg=float(np.linalg.norm(rotation_vector(relative_rotation(q[i - 1], q[i]))) * 180 / np.pi),
        ))
    onset_mask = np.zeros(len(dt), dtype=bool)
    onset_mask[onset - 1] = True
    qrate = rotation_vector(relative_rotation(q[:-1], q[1:])) / dt[:, None]
    gyro_pair = (a['gyro'][:-1] + a['gyro'][1:]) / 2
    light = a['light'][(a['light'][:, 0] >= lo) & (a['light'][:, 0] <= hi)]
    optical = dict(records=len(light), by_channel={
        str(int(channel)): int(sum(light[:, 1] == channel)) for channel in np.unique(light[:, 1])
    }, max_receive_gap_ms=float(max(np.diff(light[:, 0])) * 1000) if len(light) > 1 else None)
    return dict(
        window_receive_s=[lo, hi], samples=int(sum(k)), pairs=int(sum(pairs)),
        first_sensor_tick=int(s[k][0, 1]), last_sensor_tick=int(s[k][-1, 1]),
        first_q_wxyz=q[k][0].tolist(), last_q_wxyz=q[k][-1].tolist(),
        max_sensor_gap_ms=float(max(dt[pairs]) * 1000),
        max_receive_gap_ms=float(max(np.diff(t)[pairs]) * 1000),
        stationary_fraction=float(np.mean(active[k])),
        optical_inputs=optical,
        position_radial_rms_mm=rms(s[k, 6:9] - mean(s[:, 6:9])) * 1000,
        residual_acc_body_mean_mg=(residual * 1000).tolist(),
        residual_acc_body_std_mg=(np.std(r[k], axis=0) * 1000).tolist(),
        factory_acc_norm_mean_g=float(np.mean(np.linalg.norm(a['imu'][k, 2:5], axis=1))),
        factory_gyro_rms_rad_s=rms(a['imu'][k, 5:8]),
        corrected_gyro_rms_rad_s=rms(a['gyro'][k]),
        factory_gyro_mean_rad_s=mean(a['imu'][:, 5:8]).tolist(),
        corrected_gyro_mean_rad_s=mean(a['gyro']).tolist(),
        pose_angular_rate_rms_rad_s=rms(qrate[pairs]),
        pose_gyro_disagreement_rms_rad_s=rms(qrate[pairs] - gyro_pair[pairs]),
        model_velocity_body_mean_mm_s=(pairmean(a['vmodel']) * 1000).tolist(),
        output_velocity_body_mean_mm_s=(pairmean(a['vout']) * 1000).tolist(),
        feedback_velocity_body_mean_mm_s=(feedback * 1000).tolist(),
        residual_against_feedback_cosine=cosine(residual, -feedback),
        acc_bias_first_mg=(bias[k][0, 6:9] * 1000).tolist(),
        acc_bias_last_mg=(bias[k][-1, 6:9] * 1000).tolist(),
        acc_scale_first=bias[k][0, 9:12].tolist(), acc_scale_last=bias[k][-1, 9:12].tolist(),
        acc_bias_sigma_last_mg=(np.sqrt(a['var'][k][-1, 5:8]) * 1000).tolist(),
        equivalent_constant_bias_at_estimated_attitude_mg=(np.mean((bias[:, 6:9] + r * bias[:, 9:12])[k], axis=0) * 1000).tolist(),
        onsets=events, onset_count=len(events),
        onset_step_mm_median=float(np.median(step_norm[onset - 1])) if len(onset) else None,
        onset_step_mm_max=float(max(step_norm[onset - 1])) if len(onset) else None,
        other_step_mm_median=float(np.median(step_norm[pairs & ~onset_mask])) if np.any(pairs & ~onset_mask) else None,
        onset_step_energy_fraction=float(sum(step_norm[onset - 1] ** 2) / sum(step_norm[pairs] ** 2)),
    )


def plot(a, lo, hi, path, serial):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    s, t = a['state'], a['state'][:, 0]
    k = (t >= lo) & (t <= hi)
    pos = rotate(conjugate(s[k][0, 9:13]), s[k, 6:9] - s[k][0, 6:9]) * 1000
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True, layout='constrained')
    colors = ['#c83049', '#00798c', '#a58200']
    for ax, values, label in [
        (axes[0], pos, 'Position, start IMU axes\n(mm)'),
        (axes[1], a['residual'][k] * 1000, 'Acc minus gravity, IMU axes\n(mg)'),
        (axes[2], a['body_velocity'][k] * 1000, 'Model velocity, IMU axes\n(mm/s)'),
    ]:
        for j in range(3):
            ax.plot(t[k], values[:, j], color=colors[j], lw=.8, label='XYZ'[j])
        ax.set_ylabel(label)
        ax.legend(loc='upper right', ncols=3)
    midpoint_t = (t[:-1] + t[1:]) / 2
    bins, values = [], []
    for start in np.arange(lo, hi, .1):
        take = (midpoint_t >= start) & (midpoint_t < start + .1) & (a['dt'] <= .02)
        if np.any(take):
            bins.append(start + .05)
            values.append(np.average(a['feedback'][take], weights=a['dt'][take], axis=0) * 1000)
    for j in range(3):
        axes[3].plot(bins, np.asarray(values)[:, j], color=colors[j])
    axes[3].set_ylabel('Output minus model velocity\n0.1 s average (mm/s)')
    axes[4].plot(t[k], s[k, 4] / s[k, 2], color='black', lw=.8, label='stationary duration')
    axes[4].axhline(.1, color='green', ls='--', label='stationary-update threshold')
    axes[4].set_ylabel('Stationary duration\n(s)')
    axes[4].legend(loc='upper right')
    active = s[:, 4] > .1 * s[:, 2]
    onsets = np.flatnonzero(active[1:] & ~active[:-1]) + 1
    for ax in axes:
        ax.grid(alpha=.2)
        for i in onsets[(t[onsets] >= lo) & (t[onsets] <= hi)]:
            ax.axvline(t[i], color='gray', ls=':', lw=.6, alpha=.5)
    axes[0].set_xlim(lo, hi)
    axes[-1].set_xlabel('Native receive time (s); derivatives use sensor ticks')
    fig.suptitle(f'{serial}: residual acceleration and repeated returns\n'
                 'Physical motion contributes; output-minus-model velocity is not an isolated optical correction.')
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--window', action='append', required=True, help='label:start:end in native receive seconds')
    parser.add_argument('--allow-truncated-prefix', action='store_true', help='explicitly accept gzip EOFError; retain the error in output')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', type=Path)
    parser.add_argument('--plot-window', help='start:end in native receive seconds')
    args = parser.parse_args()
    arrays, metadata = read_capture(args.session, args.serial, args.allow_truncated_prefix)
    joined, coverage = join_rows(arrays)
    results = {}
    for spec in args.window:
        label, lo, hi = spec.split(':')
        if label in results or not float(lo) < float(hi):
            parser.error('windows must have unique labels and start < end')
        results[label] = summarize(joined, float(lo), float(hi))
    report = dict(input=metadata, joins=coverage, windows=results,
                  analysis=dict(tool_sha256=sha256(Path(__file__)), python=sys.version, numpy=np.__version__), method=dict(
        acceleration='(I.acc - POSE_BIAS.AccBias) / AccScale - R(POSE_STATE.q)^-1 [0,0,1]',
        derivative='adjacent full-precision raw IMU poses / positive full sensor tick delta',
        feedback='body-frame pose derivative minus midpoint model VELOCITY; includes all measurement updates and output timing effects',
        means='pair velocities are sensor-dt weighted; IMU residual means use matched output samples',
        onset='recorded stationary duration crosses from <=100 ms to >100 ms',
        limitations=[
            'No independent physical ground truth; actual hand acceleration contributes.',
            'Residual acceleration also contains attitude/gravity, scale and optical-model errors.',
            'Output-minus-model velocity is not an isolated optical innovation or proof of its cause.',
            'Stationary-onset association is observational, not a replay that isolates the update.',
            'Equivalent constant bias assumes recorded attitude and zero physical acceleration; not a calibration to apply.',
            'Native rows used by this analysis are checked; this is not a full native grammar validator.',
            'Prefix observations cannot establish final queue loss or full gzip CRC validity.',
        ],
    ))
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    if args.plot:
        if not args.plot_window:
            parser.error('--plot requires --plot-window')
        lo, hi = map(float, args.plot_window.split(':'))
        summarize(joined, lo, hi)
        plot(joined, lo, hi, args.plot, args.serial)


if __name__ == '__main__':
    main()
