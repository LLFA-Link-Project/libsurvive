#!/usr/bin/env python3
"""Repeat native Lighthouse captures and report replay/pose stability (stdlib only).

Use --compare for reference disagreement and coverage. See docs/lighthouse-replay.md.
"""
import argparse
import collections
import gzip
import hashlib
import json
import math
import pathlib
import subprocess
import tempfile
import time


def summarize(recording):
    objects = collections.defaultdict(lambda: {
        "poses": 0, "nonfinite_poses": 0, "max_quaternion_norm_error": 0,
        "first_receive_time_s": None, "last_receive_time_s": None,
        "max_receive_gap_s": 0, "receive_gaps_over_100ms": 0,
    })
    digest = hashlib.sha256()
    with gzip.open(recording, "rt") as f:
        for line in f:
            fields = line.split(maxsplit=3)
            if len(fields) < 4:
                continue
            if fields[2] == "CONFIG":
                config = json.loads(fields[3])
                objects[fields[1]]["tracked_object_serial"] = config.get("device_serial_number")
            elif fields[2] == "POSE":
                pose = [float(value) for value in fields[3].split()]
                if len(pose) != 7:
                    raise ValueError("Expected seven pose components")
                obj = objects[fields[1]]
                obj["poses"] += 1
                t = float(fields[0])
                digest.update(line.strip().encode())
                if not all(math.isfinite(value) for value in pose):
                    obj["nonfinite_poses"] += 1
                else:
                    error = abs(math.sqrt(sum(value * value for value in pose[3:])) - 1)
                    obj["max_quaternion_norm_error"] = max(obj["max_quaternion_norm_error"], error)
                if obj["first_receive_time_s"] is None:
                    obj["first_receive_time_s"] = t
                previous = obj["last_receive_time_s"]
                if previous is not None:
                    gap = t - previous
                    obj["max_receive_gap_s"] = max(obj["max_receive_gap_s"], gap)
                    obj["receive_gaps_over_100ms"] += gap > .1
                obj["last_receive_time_s"] = t
    return {"objects": dict(objects), "pose_stream_sha256": digest.hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=pathlib.Path)
    parser.add_argument("sessions", nargs="+", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True,
                        help="New directory for results, logs and pose recordings")
    parser.add_argument("--seconds", type=float, default=0, help="Replay prefix; zero means entire capture")
    parser.add_argument("--timeout", type=float, default=300, help="Wall time limit per run")
    parser.add_argument("--repeat", type=int, default=2)
    parser.add_argument("--compare", action="store_true", help="Compare against timed Valve reference estimates")
    parser.add_argument("--warmup", type=float, default=10, help="Comparison warmup in seconds from first input")
    args = parser.parse_args()
    if args.repeat < 1 or args.seconds < 0 or args.timeout <= 0:
        parser.error("repeat and timeout must be positive; seconds must be nonnegative")
    binary = args.binary.resolve(strict=True)
    sessions = set()
    for path in args.sessions:
        if (path / "manifest.json").is_file():
            sessions.add(path.resolve())
        else:
            sessions.update(p.parent.resolve() for p in path.glob("*/manifest.json"))
    if not sessions:
        parser.error("No session manifests found")
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"metric_basis": "libsurvive replay receive clock; not sensor measurement time",
              "accuracy_against_reference": ("held-out disagreement with reference estimates; not ground truth"
                                             if args.compare else "not computed; use --compare"),
              "sessions": []}
    failed = False
    for session in sorted(sessions):
        manifest = json.loads((session / "manifest.json").read_text())
        entry = {"session": session.name, "manifest_status": manifest.get("status"),
                 "recording_loss": manifest.get("loss"), "runs": []}
        report["sessions"].append(entry)
        if manifest.get("status") != "complete" or manifest.get("errors"):
            entry["skipped"] = "Session is incomplete or reports recording errors"
            continue
        capture = session / "usb.pcap.gz"
        if not capture.is_file() or not pathlib.Path(str(capture) + ".usbdevs").is_file():
            entry["skipped"] = "Replay capture or device listing is missing"
            continue
        if args.compare:
            import lighthouse_compare as comparison
            clock = comparison.ClockMap(list(comparison.jsonl(session / "clock_sync.jsonl")))
            reference, rejected = comparison.load_reference(session, 'pose_time_estimate_ns')
            events = comparison.load_reference_events(session)
        for repetition in range(args.repeat):
            stem = args.output / (session.name + "-" + str(repetition))
            record = pathlib.Path(str(stem) + ".rec.gz")
            # A fresh config per run prevents a previous solve from seeding the next run.
            with tempfile.TemporaryDirectory(prefix="survive-replay-") as temp:
                command = [str(binary), "--usbmon-playback", str(capture), "--playback-factor", "0",
                           "--configfile", str(pathlib.Path(temp) / "config.json"),
                           "--no-threaded-posers", "--no-gss-threaded", "--record", str(record),
                           "--no-record-rawlight", "--no-record-imu", "--no-record-angle"]
                if args.seconds > 0:
                    command += ["--run-time", str(args.seconds)]
                if args.compare:
                    command += ["--record-pose-timing"]
                start = time.monotonic()
                result = {"command": command}
                with pathlib.Path(str(stem) + ".log").open("wb") as log:
                    try:
                        proc = subprocess.run(command, stdout=log, stderr=log, timeout=args.timeout)
                        result["returncode"] = proc.returncode
                    except subprocess.TimeoutExpired:
                        result["timeout"] = True
                result["wall_time_s"] = time.monotonic() - start
            if result.get("returncode") == 0 and record.is_file():
                try:
                    result.update(summarize(record))
                    if args.compare:
                        replay, timing = comparison.load_replay(record, clock)
                        result['reference_comparison'] = comparison.compare(replay, reference, args.warmup,
                            bounds=(timing['capture_start_monotonic_ns'], timing['capture_end_monotonic_ns']), events=events)
                        result['reference_comparison'].update(timing=timing, reference_rejected=rejected,
                            timing_caveat='Input arrival mapping; sensor/driver latency and prediction remain unknown')
                except (ValueError, EOFError, OSError) as error:
                    result["error"] = str(error)
            result["ok"] = (result.get("returncode") == 0 and "error" not in result
                            and any(obj["poses"] for obj in result.get("objects", {}).values())
                            and not any(obj["nonfinite_poses"] for obj in result.get("objects", {}).values()))
            failed |= not result["ok"]
            entry["runs"].append(result)
            print(session.name, repetition, "ok" if result["ok"] else "FAILED", flush=True)
        entry["identical_pose_streams"] = (all(run["ok"] for run in entry["runs"])
                                            and len({run["pose_stream_sha256"] for run in entry["runs"]}) == 1)
    (args.output / "results.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
