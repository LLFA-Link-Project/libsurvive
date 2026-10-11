#!/usr/bin/env python3
"""Summarize raw or replayed recordings without printing embedded device JSON."""

import argparse
import collections
import gzip
import hashlib
import json
import math
import statistics


def summarize(path, stationary_start):
    counts = collections.Counter()
    angles = collections.defaultdict(list)
    poses = []
    velocities = []
    light_times = []
    angle_hash = hashlib.sha256()
    pose_hash = hashlib.sha256()
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 3 or fields[1] != "T20":
                continue
            time, kind = float(fields[0]), fields[2]
            counts[kind] += 1
            if kind == "C":
                light_times.append(time)
            elif kind == "A":
                angle_hash.update((" ".join(fields[3:]) + "\n").encode())
                key = f"{fields[8]}.{'y' if int(fields[4]) & 1 else 'x'}"
                angles[key].append((time, int(fields[3]), float(fields[7])))
            elif kind == "POSE":
                pose_hash.update((" ".join(fields[3:]) + "\n").encode())
                poses.append((time, list(map(float, fields[3:10]))))
            elif kind == "VELOCITY":
                velocities.append((time, list(map(float, fields[3:9]))))
    result = {
        "file": str(path),
        "counts": dict(counts),
        "angle_sha256": angle_hash.hexdigest(),
        "pose_sha256": pose_hash.hexdigest(),
        "angles": {},
    }
    if light_times:
        result["raw_light_time_range_s"] = [light_times[0], light_times[-1]]
    if poses:
        result["pose_time_range_s"] = [poses[0][0], poses[-1][0]]
    def velocity_summary(samples):
        return {
            "count": len(samples),
            "linear_speed_m_s_max": max(math.sqrt(sum(x * x for x in v[:3])) for v in samples),
            "angular_speed_rad_s_max": max(math.sqrt(sum(x * x for x in v[3:])) for v in samples),
        }
    if velocities:
        result["velocity"] = velocity_summary([v for _, v in velocities])
    for key in ("0.x", "0.y", "1.x", "1.y"):
        values = angles[key]
        entry = {
            "count": len(values),
            "sensor_count": len({sensor for _, sensor, _ in values}),
        }
        if stationary_start is not None:
            per_sensor = collections.defaultdict(list)
            for time, sensor, angle in values:
                if time >= stationary_start:
                    per_sensor[sensor].append(angle)
            deviations = [
                math.degrees(statistics.stdev(samples))
                for samples in per_sensor.values() if len(samples) >= 50
            ]
            if deviations:
                entry["stationary_sensor_std_deg_median"] = statistics.median(deviations)
                entry["stationary_sensor_std_deg_max"] = max(deviations)
        result["angles"][key] = entry
    if stationary_start is not None:
        selected_velocities = [v for time, v in velocities if time >= stationary_start]
        if selected_velocities:
            result["stationary_velocity"] = velocity_summary(selected_velocities)
        selected = [pose for time, pose in poses if time >= stationary_start]
        result["stationary_start_s"] = stationary_start
        if selected:
            q0 = selected[0][3:]
            q0_norm = math.sqrt(sum(x * x for x in q0))
            rotations = []
            for pose in selected:
                q = pose[3:]
                norm = q0_norm * math.sqrt(sum(x * x for x in q))
                if norm > 0:
                    dot = min(1, abs(sum(x * y for x, y in zip(q0, q))) / norm)
                    rotations.append(math.degrees(2 * math.acos(dot)))
            result["stationary_pose"] = {
                "count": len(selected),
                "xyz_peak_to_peak_m": [
                    max(pose[i] for pose in selected) - min(pose[i] for pose in selected)
                    for i in range(3)
                ],
                "rotation_from_first_deg_max": max(rotations) if rotations else None,
            }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--stationary-start", type=float, help="Known static interval start, in recording seconds")
    args = parser.parse_args()
    print(json.dumps([summarize(path, args.stationary_start) for path in args.recordings], indent=2))
