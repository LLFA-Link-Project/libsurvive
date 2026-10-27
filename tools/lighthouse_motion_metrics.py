#!/usr/bin/env python3
"""Measure pose, derivative-state and prediction variation on native sensor clocks.

Requires POSE_STATE and POSE_MOTION from record-tracking-state. Variation includes
real motion. The centered trend subtraction is analysis only, never a pose filter.
"""
import argparse
import collections
import gzip
import json
from pathlib import Path

import numpy as np
from lighthouse_body_drift import relative_rotation, rotation_vector


def rms(v):
    return float(np.sqrt(np.mean(np.sum(v * v, axis=-1))))


def highpass(v, samples=101):
    if len(v) < samples:
        raise ValueError('Window shorter than centered trend support')
    trend = np.stack([np.convolve(v[:, i], np.ones(samples) / samples, 'valid')
                      for i in range(v.shape[1])], axis=-1)
    half = samples // 2
    return v[half:-half] - trend


def vector_stats(v):
    n = np.linalg.norm(v, axis=-1)
    return dict(rms=rms(v), p95=float(np.percentile(n, 95)), maximum=float(np.max(n)))


def frequency_band_rms(t, values, bands=((.5, 2), (2, 5), (5, 15), (15, 50))):
    """Hann-weighted vector RMS by frequency; real motion can occupy any band.

    Remove a fitted straight line before the FFT. Normalize by window energy,
    so summing power over all bins gives the window-weighted time-domain power.
    Bands include their lower bound and exclude their upper bound, except Nyquist.
    This is a diagnostic on a uniform grid, never an output smoothing filter.
    """
    t, values = np.asarray(t), np.asarray(values)
    if len(t) < 4 or values.ndim != 2 or len(values) != len(t):
        raise ValueError('Frequency analysis needs matching time and vector samples')
    dt = np.diff(t)
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(values)) or dt[0] <= 0 or \
            not np.allclose(dt, dt[0], rtol=1e-7, atol=1e-10):
        raise ValueError('Frequency analysis requires finite uniformly spaced samples')
    centered = t - np.mean(t)
    trend = np.mean(values, axis=0) + centered[:, None] * \
        np.sum(centered[:, None] * values, axis=0) / np.sum(centered * centered)
    window = np.hanning(len(t))
    spectrum = np.fft.rfft((values - trend) * window[:, None], axis=0)
    power = np.sum(abs(spectrum) ** 2, axis=1) / (len(t) * np.sum(window * window))
    power[1:] *= 2
    if len(t) % 2 == 0:
        power[-1] *= .5
    frequencies = np.fft.rfftfreq(len(t), dt[0])
    nyquist = .5 / dt[0]
    result = {}
    for lo, hi in bands:
        if not 0 <= lo < hi <= nyquist + 1e-7:
            raise ValueError('Frequency band exceeds the sampled frequency range')
        selected = (frequencies >= lo) & ((frequencies < hi) |
            (np.isclose(frequencies, nyquist) & np.isclose(hi, nyquist)))
        result[f'{lo:g}-{hi:g}_hz'] = float(np.sqrt(np.sum(power[selected])))
    return result


def predict(p, q, v, w, a, horizon):
    """Predict an IMU-origin pose using world-frame derivative states."""
    predicted = p + horizon * v + .5 * horizon * horizon * a
    half_angle = np.linalg.norm(w, axis=1, keepdims=True) * horizon / 2
    dq = np.concatenate([np.cos(half_angle), w * horizon / 2 * np.sinc(half_angle / np.pi)], axis=1)
    predicted_q = np.concatenate([
        dq[:, :1] * q[:, :1] - np.sum(dq[:, 1:] * q[:, 1:], axis=1, keepdims=True),
        dq[:, :1] * q[:, 1:] + q[:, :1] * dq[:, 1:] + np.cross(dq[:, 1:], q[:, 1:])], axis=1)
    return predicted, predicted_q


def window_metrics(t, p, q, v, w, a):
    """All inputs use the same uniform 200 Hz sensor-time grid and world frame."""
    angles = rotation_vector(relative_rotation(q[len(q) // 2], q))
    result = dict(samples=len(t), position_hp_mm=vector_stats(highpass(p) * 1000),
        orientation_hp_deg=vector_stats(highpass(angles) * 180 / np.pi),
        velocity_hp_mm_s=vector_stats(highpass(v) * 1000),
        angular_velocity_hp_deg_s=vector_stats(highpass(w) * 180 / np.pi),
        acceleration_hp_m_s2=vector_stats(highpass(a)),
        acceleration_m_s2=vector_stats(a),
        angular_acceleration_deg_s2=vector_stats(np.diff(w, axis=0) / np.diff(t)[:, None] * 180 / np.pi),
        jerk_m_s3=vector_stats(np.diff(a, axis=0) / np.diff(t)[:, None]),
        frequency_bands=dict(method='Linear detrend, Hann-window energy normalization; '
            'vector RMS on the uniform sensor-time grid; variation includes real motion',
            position_mm=frequency_band_rms(t, p * 1000),
            orientation_deg=frequency_band_rms(t, angles * 180 / np.pi),
            velocity_mm_s=frequency_band_rms(t, v * 1000),
            angular_velocity_deg_s=frequency_band_rms(t, w * 180 / np.pi),
            acceleration_m_s2=frequency_band_rms(t, a)),
        predictions={})
    for horizon in (0, .01, .02, .03):
        predicted, predicted_q = predict(p, q, v, w, a, horizon)
        predicted_angles = rotation_vector(relative_rotation(predicted_q[len(q) // 2], predicted_q))
        result['predictions'][str(horizon)] = dict(position_hp_mm=vector_stats(highpass(predicted) * 1000),
            orientation_hp_deg=vector_stats(highpass(predicted_angles) * 180 / np.pi),
            frequency_bands=dict(position_mm=frequency_band_rms(t, predicted * 1000),
                orientation_deg=frequency_band_rms(t, predicted_angles * 180 / np.pi)))
    return result


def read(path):
    states, motions, serials = collections.defaultdict(dict), collections.defaultdict(dict), {}
    with gzip.open(path, 'rt') as stream:
        for line in stream:
            if '\0' in line or not line.endswith('\n'):
                raise ValueError('Damaged recording row')
            f = line.split(maxsplit=3)
            if len(f) != 4:
                continue
            receive, obj, kind, value = f
            if kind == 'CONFIG':
                serial = json.loads(value).get('device_serial_number', obj)
                if obj in serials and serial != serials[obj]:
                    raise ValueError('Receiver identity changed; split recording before evaluation')
                serials[obj] = serial
            elif kind == 'POSE_STATE':
                v = value.split()
                if len(v) != 12 or int(v[1]) <= 0:
                    raise ValueError('Malformed POSE_STATE or invalid sensor frequency')
                states[obj][int(v[0])] = (float(receive), int(v[1]), *map(float, v[5:12]))
            elif kind == 'POSE_MOTION':
                v = value.split()
                if len(v) != 11 or int(v[1]) <= 0:
                    raise ValueError('Malformed POSE_MOTION or invalid sensor frequency')
                motions[obj][int(v[0])] = (int(v[1]), *map(float, v[2:]))
    return states, motions, serials


def evaluate(path, windows):
    states, motions, serials = read(path)
    report = dict(path=str(path), method='200Hz sensor-time interpolation, centered 101-sample trend; '
        'variation includes real motion; prediction uses p+v*h+a*h*h/2; no reference accuracy or latency claim', objects={})
    for obj, by_tick in states.items():
        serial = serials.get(obj, obj)
        output = report['objects'][serial] = {}
        for name, (lo, hi) in windows.items():
            selected = [(tick, row, motions[obj].get(tick)) for tick, row in sorted(by_tick.items()) if lo <= row[0] <= hi]
            try:
                if len(selected) < 104 or any(row[2] is None for row in selected):
                    raise ValueError('Insufficient paired POSE_MOTION/POSE_STATE samples')
                if selected[0][1][0] - lo > .02 or hi - selected[-1][1][0] > .02:
                    raise ValueError('Output does not cover the declared receive-time window')
                hz = selected[0][1][1]
                if any(row[1][1] != hz or row[2][0] != hz for row in selected):
                    raise ValueError('Sensor frequency changes within window')
                # Subtract integer ticks before converting to double.
                times = np.array([(row[0] - selected[0][0]) / hz for row in selected])
                if np.any(np.diff(times) <= 0) or np.any(np.diff(times) > .02):
                    raise ValueError('Sensor gap exceeds 20ms or time does not increase')
                values = np.array([(*row[1][2:], *row[2][1:]) for row in selected])
                if not np.all(np.isfinite(values)):
                    raise ValueError('Nonfinite motion state')
                q = values[:, 3:7].copy()
                norms = np.linalg.norm(q, axis=1, keepdims=True)
                if np.any(abs(norms - 1) > .01):
                    raise ValueError('Nonunit orientation')
                q /= norms
                q[1:] *= np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1, 1))[:, None]
                values[:, 3:7] = q
                grid = np.arange(0, times[-1], .005)
                samples = np.stack([np.interp(grid, times, values[:, i]) for i in range(values.shape[1])], axis=-1)
                samples[:, 3:7] /= np.linalg.norm(samples[:, 3:7], axis=1, keepdims=True)
                output[name] = window_metrics(grid, samples[:, :3], samples[:, 3:7], samples[:, 7:10],
                                               samples[:, 10:13], samples[:, 13:16])
                output[name].update(receive_bounds=[selected[0][1][0], selected[-1][1][0]], sensor_hz=hz)
            except ValueError as error:
                output[name] = dict(unavailable=str(error))
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording', type=Path)
    p.add_argument('--window', action='append', required=True, help='label:start:end receive seconds')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    windows = {}
    for value in args.window:
        name, lo, hi = value.split(':')
        windows[name] = float(lo), float(hi)
    args.output.write_text(json.dumps(evaluate(args.recording, windows), indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
