"""Audit completed native replays and callback timing without claiming ground truth.

A run directory must contain run.json (command/returncode) and record.rec.gz.
Incomplete/failed runs are excluded. Original capture directories are read only.
"""
import argparse
import collections
import gzip
import hashlib
import json
import math
import pathlib


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def native(path):
    counts = collections.Counter()
    invalid = collections.Counter()
    maximum = collections.defaultdict(float)
    previous = {}
    jumps = collections.Counter()
    pairs = collections.Counter()
    maxstep = collections.defaultdict(float)
    first, last = {}, {}
    digest = hashlib.sha256()
    logs = []
    nul_bytes = 0
    malformed_lines = 0
    malformed_examples = []
    with gzip.open(path, "rt") as stream:
        for line_number, line in enumerate(stream, 1):
            if "\0" in line:
                nul_bytes += line.count("\0")
                malformed_lines += 1
                if len(malformed_examples) < 8:
                    malformed_examples.append(dict(line=line_number, text_repr=repr(line[:450])))
                # The record boundary/object cannot be trusted at this point.
                previous.clear()
                continue
            fields = line.split()
            if len(fields) < 4:
                continue
            if fields[2] == "LOG":
                logs.append(line.strip())
            if fields[2] != "POSE":
                continue
            digest.update(line.encode())
            device = fields[1]
            values = list(map(float, fields[3:10]))
            timestamp = float(fields[0])
            counts[device] += 1
            if (len(values) != 7 or not all(map(math.isfinite, values))
                    or not any(values[3:])):
                invalid[device] += 1
                previous.pop(device, None)
                continue
            maximum[device] = max(maximum[device], math.sqrt(sum(x * x for x in values[:3])))
            first.setdefault(device, timestamp)
            last[device] = timestamp
            if device in previous:
                dt = timestamp - previous[device][0]
                if 0 < dt <= .02:
                    distance = math.dist(values[:3], previous[device][1])
                    pairs[device] += 1
                    jumps[device] += distance > .1
                    maxstep[device] = max(maxstep[device], distance)
            previous[device] = (timestamp, values[:3])
    return dict(
        count=dict(counts), invalid=dict(invalid), first_s=first, last_s=last,
        max_distance_from_origin_m=dict(maximum),
        adjacent_pairs_dt_positive_at_most_20ms=dict(pairs),
        jumps_over_100mm=dict(jumps), maximum_step_m=dict(maxstep),
        pose_stream_sha256=digest.hexdigest(),
        nul_byte_count=nul_bytes, malformed_line_count=malformed_lines,
        malformed_examples=malformed_examples,
        gss_quality=[line for line in logs if "GSS_QUALITY " in line or "GSS_LIGHTHOUSE_GRAVITY " in line],
        gss_success=[line for line in logs if "MPFIT success" in line and " iters, up err " in line],
        gss_failures=sum("MPFIT status failure" in line for line in logs),
        gss_gravity_rejections=sum("opposite hemisphere" in line for line in logs),
        tracking_resets=sum("reseting calibration" in line for line in logs),
    )


def callback(path):
    groups = {}
    with gzip.open(path, "rt") as stream:
        for line in stream:
            record = json.loads(line)
            if record["stage"] != "backend_received":
                continue
            key = record["serial"] + (" valid" if record["pose_is_valid"] else " invalid")
            stats = groups.setdefault(key, dict(
                count=0, min_callback_minus_pose_s=float("inf"),
                max_callback_minus_pose_s=-float("inf"), gap_over_one_second=0,
            ))
            delta = (record["capture_time_ns"] - record["pose_time_ns"]) / 1e9
            stats["count"] += 1
            stats["min_callback_minus_pose_s"] = min(stats["min_callback_minus_pose_s"], delta)
            stats["max_callback_minus_pose_s"] = max(stats["max_callback_minus_pose_s"], delta)
            stats["gap_over_one_second"] += abs(delta) > 1
    return groups


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_root", type=pathlib.Path)
    parser.add_argument("--source", action="append", type=pathlib.Path, required=True)
    parser.add_argument("--baseline-build-proof", type=pathlib.Path)
    parser.add_argument("--prior-corpus-gravity-audit", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    root = args.run_root.resolve()
    results = {}
    for path in sorted(root.glob("*/record.rec.gz")):
        metadata = path.parent / "run.json"
        if not metadata.exists():
            continue
        run = json.loads(metadata.read_text())
        if run.get("returncode") != 0:
            continue
        results[path.parent.name] = dict(run=run, native=native(path))
    sources = {}
    for directory in args.source:
        directory = directory.resolve()
        sources[directory.name] = dict(
            native=native(directory / "native.rec.gz"),
            callback_gaps=callback(directory / "poses.jsonl.gz"),
            sha256={p.name: sha256_file(p) for p in directory.iterdir() if p.is_file()},
        )
    builds = {p.parent.name: json.loads(p.read_text()) for p in root.glob("*/build-proof.json")}
    if args.baseline_build_proof:
        builds["baseline"] = json.loads(args.baseline_build_proof.read_text())
    output = dict(
        schema_version=1,
        method="Complete native recordings, no ground truth. NUL-bearing lines are reported and excluded, breaking adjacency; this is not a complete format validator. Jump pairs use positive native receive-time gaps <=20 ms and steps >100 mm; nonfinite/zero-quaternion poses break adjacency. Finite nonzero quaternions do not certify tracking quality. Distances from origin depend on the world frame. Callback gaps are split by the logger's pose_is_valid flag, which does not certify calibration accuracy.",
        sources=sources, runs=results, builds=builds,
    )
    if args.prior_corpus_gravity_audit:
        output["prior_corpus_gravity_audit"] = json.loads(args.prior_corpus_gravity_audit.read_text())
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(f"Audited {len(results)} completed replays and {len(sources)} captures: {args.output}")


if __name__ == "__main__":
    main()
