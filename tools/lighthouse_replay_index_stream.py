"""Bounded-memory replay indexing with exact compressed external sorting.

Keep every input anchor and timed pose until sorting by full sensor tick and
original record order. Only the existing50Hz mapped replay table goes into
SQLite. This avoids storing/indexing all high-rate raw rows in SQLite and
simultaneously creating its large sort files. Original recordings are read-only.
"""
import collections
import contextlib
import gzip
import heapq
import json
import math
from pathlib import Path
import statistics
import struct
import tempfile

import lighthouse_compare as comparison


INPUT = struct.Struct('<qqq')  # tick, original order, receive ns
POSE = struct.Struct('<qqq7d')  # tick, original order, Hz, normalized pose


class SortedRuns:
    def __init__(self, folder, chunk_rows=65536):
        if chunk_rows < 1:
            raise ValueError('Positive sort chunk size required')
        self.folder, self.chunk_rows = Path(folder), chunk_rows
        self.pending = collections.defaultdict(list)
        self.runs = collections.defaultdict(list)
        self.rows, self.sequence = 0, 0

    def add(self, obj, kind, row):
        self.pending[(obj, kind)].append(row)
        self.rows += 1
        if self.rows >= self.chunk_rows:
            self.flush()

    def flush(self):
        for key, rows in self.pending.items():
            if not rows:
                continue
            rows.sort(key=lambda row: row[:2])
            path = self.folder / f'run-{self.sequence}.bin.gz'
            self.sequence += 1
            layout = INPUT if key[1] == 'inputs' else POSE
            with gzip.open(path, 'wb', compresslevel=6) as stream:
                for row in rows:
                    stream.write(layout.pack(*row))
            self.runs[key].append(path)
        self.pending.clear()
        self.rows = 0

    @staticmethod
    def read(path, layout):
        with gzip.open(path, 'rb') as stream:
            while True:
                payload = stream.read(layout.size)
                if not payload:
                    return
                if len(payload) != layout.size:
                    raise ValueError('Truncated derived sort run')
                yield layout.unpack(payload)

    def merged(self, obj, kind):
        self.flush()
        layout = INPUT if kind == 'inputs' else POSE
        paths = list(self.runs.get((obj, kind), ()))
        # Bound the number of simultaneously open gzip streams. Intermediate
        # files are disposable products in this function's temporary directory.
        while len(paths) > 32:
            next_paths = []
            for start in range(0, len(paths), 32):
                group = paths[start:start + 32]
                path = self.folder / f'merge-{self.sequence}.bin.gz'
                self.sequence += 1
                with gzip.open(path, 'wb', compresslevel=6) as stream:
                    rows = heapq.merge(*(self.read(p, layout) for p in group), key=lambda row: row[:2])
                    for row in rows:
                        stream.write(layout.pack(*row))
                for original in group:
                    original.unlink()
                next_paths.append(path)
            paths = next_paths
        try:
            yield from heapq.merge(*(self.read(p, layout) for p in paths), key=lambda row: row[:2])
        finally:
            for path in paths:
                path.unlink()
            self.runs.pop((obj, kind), None)


def replay_index(db, path, clock, sample_hz=50, allow_unbracketed_span=False, *,
                 chunk_rows=65536, scratch_parent=None):
    identities, pending, segments, serials = {}, {}, {}, {}
    counts = collections.Counter()
    span = [None, None]
    mapped_span = [None, None]
    objects = set()
    if sample_hz <= 0:
        raise ValueError('Positive replay sampling rate required')
    with tempfile.TemporaryDirectory(prefix='lh-sorted-replay-', dir=scratch_parent) as folder:
        runs = SortedRuns(folder, chunk_rows)
        with gzip.open(path, 'rt') as stream:
            for order, line in enumerate(stream):
                row = line.split(maxsplit=3)
                if len(row) != 4:
                    continue
                _, receiver, op, value = row
                obj = receiver + '#' + str(segments.get(receiver, 0))
                if op == 'CONFIG':
                    serial = json.loads(value).get('device_serial_number')
                    if receiver in serials and serials[receiver] != serial:
                        segments[receiver] = segments.get(receiver, 0) + 1
                        obj = receiver + '#' + str(segments[receiver])
                        counts['receiver_identity_changes'] += 1
                    identities[obj] = serial
                    serials[receiver] = serial
                elif op == 'INPUT_TIME':
                    tick, receive = map(int, value.split())
                    if tick <= 0:
                        continue
                    receive *= 1000
                    runs.add(obj, 'inputs', (tick, order, receive))
                    span[0] = receive if span[0] is None else min(span[0], receive)
                    span[1] = receive if span[1] is None else max(span[1], receive)
                    if allow_unbracketed_span and clock.convert(receive) is not None:
                        mapped_span[0] = receive if mapped_span[0] is None else min(mapped_span[0], receive)
                        mapped_span[1] = receive if mapped_span[1] is None else max(mapped_span[1], receive)
                elif op == 'POSE':
                    values = tuple(map(float, value.split()))
                    if len(values) != 7 or not all(math.isfinite(v) for v in values):
                        raise ValueError('Nonfinite or malformed replay pose')
                    pending[obj] = (*values[:3], *comparison.unit(values[3:]))
                elif op == 'POSE_TIMING':
                    tick, hz = map(int, value.split()[:2])
                    if hz <= 0 or obj not in pending:
                        raise ValueError('Unpaired or malformed pose timing')
                    runs.add(obj, 'poses', (tick, order, hz, *pending.pop(obj)))
                    objects.add(obj)
        runs.flush()
        if span[0] is None and not objects:
            counts['no_replay_input_or_poses'] = 1
            db.execute('CREATE INDEX replay_times ON replay(serial,t,id)')
            db.commit()
            return dict(counts)
        for i, name in enumerate(('capture_start_monotonic_ns', 'capture_end_monotonic_ns')):
            converted = clock.convert(span[i]) if span[i] is not None else None
            if converted is None and allow_unbracketed_span:
                converted = clock.convert(mapped_span[i]) if mapped_span[i] is not None else None
                counts['unbracketed_input_span_endpoints'] += 1
            if converted is None:
                raise ValueError('Input span is not bracketed by clock samples')
            counts[name] = converted[0]
        for obj in sorted(objects):
            serial = identities.get(obj)
            if not serial:
                raise ValueError('Missing tracked-object identity')
            inputs = runs.merged(obj, 'inputs')
            before = collections.deque(maxlen=8)
            after = collections.deque()

            def replenish():
                while len(after) < 8:
                    item = next(inputs, None)
                    if item is None:
                        break
                    after.append((item[0], item[2]))

            replenish()
            previous, batch = None, []
            with contextlib.closing(inputs), contextlib.closing(runs.merged(obj, 'poses')) as poses:
                for tick, _, hz, *pose in poses:
                    while after and after[0][0] < tick:
                        before.append(after.popleft())
                        replenish()
                    anchors = [(tc, ns) for tc, ns in (*before, *after) if abs(tc - tick) / hz < .05]
                    if not anchors:
                        counts['unbracketed_sensor_time'] += 1
                        continue
                    estimates = [ns + round((tick - tc) * 1e9 / hz) for tc, ns in anchors]
                    spread = max(estimates) - min(estimates)
                    if spread > 20_000_000:
                        counts['ambiguous_sensor_clock'] += 1
                        continue
                    converted = clock.convert(int(statistics.median(estimates)))
                    if converted is None:
                        counts['unmapped_clock'] += 1
                        continue
                    ns, error = converted
                    if previous is not None and ns - previous < 1e9 / sample_hz:
                        continue
                    previous = ns
                    batch.append((serial, ns, *pose))
                    counts['max_clock_mapping_estimate_ns'] = max(counts['max_clock_mapping_estimate_ns'], error)
                    counts['max_local_input_arrival_spread_ns'] = max(counts['max_local_input_arrival_spread_ns'], spread)
                    if len(batch) >= 4000:
                        db.executemany('INSERT INTO replay(serial,t,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?)', batch)
                        batch.clear()
            db.executemany('INSERT INTO replay(serial,t,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?)', batch)
        db.execute('CREATE INDEX replay_times ON replay(serial,t,id)')
        db.commit()
    return dict(counts)
