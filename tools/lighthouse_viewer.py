#!/usr/bin/env python3
"""Read-only, LAN-capable Lighthouse recording viewer (Python standard library).

Originals are never modified. Each selected stream is indexed once into a
disposable SQLite cache; browsers request bounded windows at original cadence.
"""

import argparse
import concurrent.futures
import gzip
import hashlib
import http.server
import json
import math
import mimetypes
import os
from pathlib import Path
import signal
import sqlite3
import statistics
import tempfile
import threading
import urllib.parse
import zlib

REPO = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).with_name('lighthouse-viewer')
VERSION = 3
MAX_WINDOW = 60
COLUMNS = ['t', 'tick', 'hz', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz',
           'stationary_s', 'vx', 'vy', 'vz', 'bias_gx', 'bias_gy', 'bias_gz',
           'gyro_x', 'gyro_y', 'gyro_z', 'valid']
SQL_COLUMNS = ','.join(COLUMNS)


class IndexCancelled(Exception):
    pass


def load_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default


def open_text(path):
    return gzip.open(path, 'rt', encoding='utf-8') if path.suffix == '.gz' else path.open(encoding='utf-8')


def stream_file(folder, name):
    for suffix in ('.gz', ''):
        p = folder / (name + suffix)
        if p.is_file():
            return p
    return None


def sha256(path, cancel=None):
    digest = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            if cancel is not None and cancel.is_set():
                raise IndexCancelled()
            digest.update(chunk)
    return digest.hexdigest()


def read_lines(path, health, cancel=None):
    """Keep complete readable lines, explicitly report trailer/CRC/row damage."""
    try:
        with open_text(path) as f:
            for line in f:
                if cancel is not None and cancel.is_set():
                    raise IndexCancelled()
                health['lines'] += 1
                if not line.endswith('\n') or '\0' in line:
                    health['damaged_rows'] += 1
                    continue
                yield line
        health['stream_integrity'] = 'complete'
    except (EOFError, OSError, zlib.error, UnicodeError) as error:
        health['stream_integrity'] = 'readable-prefix'
        health['stream_error'] = str(error)


def unit(q):
    n = math.sqrt(sum(x * x for x in q))
    if n < 1e-12 or not math.isfinite(n):
        raise ValueError('invalid quaternion')
    return [x / n for x in q]


def numbers(fields, count):
    values = list(map(float, fields))
    if len(values) != count or not all(map(math.isfinite, values)):
        raise ValueError('invalid numeric sample')
    return values


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def rotate(q, p):
    return multiply(multiply(q, [0, *p]), [q[0], -q[1], -q[2], -q[3]])[1:]


def compose(a, b):
    p, q = a
    r, s = b
    return ([x+y for x, y in zip(p, rotate(q, r))], multiply(q, s))


def xyzw(q):
    return unit([q[3], *q[:3]])


class Writer:
    def __init__(self, db):
        self.db, self.tracks, self.batch = db, {}, []

    def add(self, identity, description, row):
        key = hashlib.sha256(identity.encode()).hexdigest()[:20]
        if len(row) != len(COLUMNS):
            raise ValueError('unexpected sample schema')
        if not all(v is None or isinstance(v, (float, int)) and math.isfinite(v) for v in row):
            raise ValueError('nonfinite sample')
        info = self.tracks.setdefault(key, dict(id=key, **description, count=0,
                                              start=row[0], end=row[0], invalid=0))
        info['count'] += 1
        info['start'] = min(info['start'], row[0])
        info['end'] = max(info['end'], row[0])
        info['invalid'] += not row[-1]
        self.batch.append((key, *row))
        if len(self.batch) >= 4000:
            self.flush()

    def flush(self):
        self.db.executemany(f'INSERT INTO samples VALUES({",".join("?" for _ in range(len(COLUMNS)+1))})', self.batch)
        self.batch.clear()


def native_index(source, writer, health, cancel=None):
    identities, pending, configs, options, epochs = {}, {}, {}, {}, []
    bases = {}

    def flush(device):
        item = pending.pop(device, None)
        if item:
            row, state, identity = item
            serial, segment = identity
            mode = 'POSE_STATE' if state else 'POSE'
            writer.add(f'{device}:{serial}:{segment}:{mode}', dict(
                serial=serial, device=device, stage=mode,
                frame='libsurvive world Z-up', reference='raw IMU before floor' if state else 'reported pose (floor / profile may apply)',
                clock='native receive seconds; sensor tick/Hz retained',
                label=f'{serial} · {device} · {mode}', role=configs.get(device, {}).get('device_class', 'unknown')), row)

    for line in read_lines(source['path'], health, cancel):
        fields = line.split(maxsplit=3)
        if len(fields) != 4:
            continue
        ts, device, kind, value = fields
        if kind not in ('CONFIG', 'POSE', 'POSE_STATE', 'POSE_TIMING', 'POSE_BIAS',
                        'VELOCITY', 'I', 'INPUT_TIME', 'LH_POSE') and device != 'OPTION':
            continue
        try:
            t = float(ts)
            if not math.isfinite(t):
                raise ValueError('invalid receive time')
            f = value.split()
            if device == 'OPTION':
                options[kind] = value.split(maxsplit=1)[-1].strip()
            elif kind == 'CONFIG':
                c = json.loads(value)
                serial = c.get('device_serial_number') or device
                old = identities.get(device)
                if old is None or old[0] != serial:
                    flush(device)
                    identities[device] = (serial, 0 if old is None else old[1]+1)
                configs[device] = c
            elif kind == 'INPUT_TIME' and len(epochs) < 32:
                epochs.append(int(f[1])*1000-round(t*1e9))
            elif kind == 'LH_POSE':
                p = list(map(float, f[:7]))
                if len(p) == 7 and all(map(math.isfinite, p)):
                    bases[device] = dict(t=t, position=p[:3], quaternion=unit(p[3:]))
            elif kind == 'POSE':
                flush(device)
                p = numbers(f, 7)
                q = unit(p[3:])
                row = [t, None, None, *p[:3], *q, *([None]*10), 1]
                pending[device] = [row, False, identities.get(device, (device, 0))]
            elif device in pending:
                row = pending[device][0]
                if kind == 'POSE_STATE':
                    tick, hz = int(f[0]), int(f[1])
                    if hz <= 0:
                        raise ValueError('invalid sensor frequency')
                    p = numbers(f[5:12], 7)
                    row[:11] = [t, tick, hz, *p[:3], *unit(p[3:]), int(f[3])/hz]
                    pending[device][1] = True
                elif kind == 'POSE_TIMING' and row[1] is None:
                    row[1:3] = [int(f[0]), int(f[1])]
                    if row[2] <= 0:
                        raise ValueError('invalid sensor frequency')
                elif kind == 'POSE_BIAS' and row[1] == int(f[0]):
                    row[14:17] = numbers(f[2:5], 3)
                elif kind == 'VELOCITY':
                    row[11:14] = numbers(f[:3], 3)
                elif kind == 'I' and row[1] is not None and row[1] & 0xffffffff == int(f[1]):
                    row[17:20] = numbers(f[5:8], 3)
        except (ValueError, TypeError, IndexError, OverflowError):
            health['damaged_rows'] += 1
            # Retain an explicit invalid sample when its callback is known, so
            # short invalid intervals cannot disappear behind interpolation.
            item = pending.get(device)
            if item:
                item[0][3:10] = [0, 0, 0, 1, 0, 0, 0]
                item[0][10:20] = [None]*10
                item[0][-1] = 0
                if kind == 'POSE_STATE':
                    item[1] = True
    for device in list(pending):
        try:
            flush(device)
        except ValueError:
            health['damaged_rows'] += 1
    return dict(options=options, bases=bases,
                native_epoch_realtime_ns=round(statistics.median(epochs)) if epochs else None,
                time_basis='native recording receive seconds (not sensor measurement time)')


def jsonl_index(source, writer, health, cancel=None):
    folder = source['path'].parent
    clock = stream_file(folder, 'clock_sync.jsonl')
    origin = None
    offset = None
    origin_basis = 'first decoded callback (clock_sync unavailable)'
    if clock:
        try:
            with open_text(clock) as f:
                c = json.loads(f.readline())
                origin = (c['monotonic_before_ns']+c['monotonic_after_ns'])//2
                offset = c.get('offset_realtime_minus_monotonic_ns')
                origin_basis = 'clock_sync start'
        except (OSError, EOFError, ValueError, KeyError, TypeError) as error:
            health.setdefault('metadata_warnings', []).append(f'clock_sync: {error}')
    for line in read_lines(source['path'], health, cancel):
        try:
            r = json.loads(line)
            capture = r['capture_time_ns']
            if not isinstance(capture, int):
                raise ValueError('invalid callback clock')
            if origin is None:
                origin = capture
            t = (capture-origin)/1e9
            serial = r.get('serial') or r['device_id']
            stage = r.get('stage', 'valve_driver_pose')
            reference = r.get('reference_point', 'driver tracking reference')
            frame = r.get('coordinate_frame', 'unknown')
            input_name = r.get('input_name') or ''
            valid = bool(r.get('pose_is_valid'))
            validity = r.get('validity') or {}
            valid &= validity.get('position') is not False and validity.get('orientation') is not False
            p = r['position_m']
            try:
                q = xyzw(r['orientation_xyzw'])
            except (ValueError, TypeError):
                q = [1, 0, 0, 0]
                valid = False
                health['damaged_rows'] += 1
            v = r.get('linear_velocity_m_s') or [None]*3
            if r.get('source') == 'steamvr_lh':
                # Match the documented world-from-driver / driver-from-head
                # composition. Never treat SteamVR poses as ground truth.
                valid &= bool(r.get('device_is_connected')) and r.get('tracking_result') == 200
                try:
                    p, q = compose((r['world_from_driver_translation_m'], xyzw(r['world_from_driver_orientation_xyzw'])),
                                   compose((p, q), (r['driver_from_head_translation_m'], xyzw(r['driver_from_head_orientation_xyzw']))))
                except (ValueError, TypeError):
                    p, q, valid = [0, 0, 0], [1, 0, 0, 0], False
                    health['damaged_rows'] += 1
                frame = 'Valve composed world Y-up (before chaperone / stabilization)'
                reference = 'driver-from-head composed reference'
                v = [None]*3  # Point offset changes velocity; not recomputed here.
            # Keep native axes; the client uses metadata only to orient its camera.
            values = [*p, *q]
            if len(values) != 7 or not all(isinstance(x, (float, int)) and math.isfinite(x) for x in values):
                values, valid = [0, 0, 0, 1, 0, 0, 0], False
                health['damaged_rows'] += 1
            if len(v) != 3 or not all(x is None or isinstance(x, (float, int)) and math.isfinite(x) for x in v):
                v = [None]*3
            row = [t, None, None, *values, None, *v, *([None]*6), int(valid)]
            description = dict(serial=serial, stage=stage, frame=frame, reference=reference,
                               clock=f'CLOCK_MONOTONIC callback relative to {origin_basis}',
                               role=r.get('role', str(r.get('device_class', 'unknown'))),
                               label=f'{serial} · {stage} {input_name} · {reference}')
            writer.add(f'{serial}:{stage}:{input_name}:{frame}:{reference}', description, row)
        except (KeyError, ValueError, TypeError, IndexError, OverflowError):
            health['damaged_rows'] += 1
    return dict(time_basis=f'callback CLOCK_MONOTONIC seconds since {origin_basis} (not native receive seconds)',
                callback_origin_ns=origin, clock_offset_realtime_minus_monotonic_ns=offset)


def index_source(source, output, cancel=None):
    before = source['path'].stat()
    health = dict(lines=0, damaged_rows=0, stream_integrity='unknown')
    db = sqlite3.connect(output)
    try:
        if cancel is not None:
            db.set_progress_handler(lambda: int(cancel.is_set()), 10000)
        db.execute('PRAGMA journal_mode=OFF')
        db.execute('PRAGMA synchronous=OFF')
        db.execute('PRAGMA cache_size=-16384')
        db.execute('CREATE TABLE samples(track TEXT,' + ','.join(
            f'{c} {"INTEGER" if c in ("tick", "hz", "valid") else "REAL"}' for c in COLUMNS) + ')')
        db.execute('CREATE TABLE metadata(value TEXT)')
        writer = Writer(db)
        details = (native_index if source['kind'] == 'native' else jsonl_index)(source, writer, health, cancel)
        writer.flush()
        db.execute('CREATE INDEX sample_times ON samples(track,t)')
        digest = sha256(source['path'], cancel)
        after = source['path'].stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError('Source changed during indexing; retry after recording stops')
        info = dict(version=VERSION, source_sha256=digest, source_bytes=before.st_size,
                    source_mtime_ns=before.st_mtime_ns, health=health,
                    tracks=list(writer.tracks.values()), **details)
        if source.get('session_path'):
            markers = source['session_path'] / 'scene-markers.jsonl'
            info['markers_raw'] = []
            if markers.exists():
                for line in markers.read_text().splitlines():
                    if line.strip():
                        try:
                            info['markers_raw'].append(json.loads(line))
                        except ValueError:
                            health.setdefault('metadata_warnings', []).append('Unreadable scene marker row')
            clock = stream_file(source['session_path'], 'clock_sync.jsonl')
            if clock:
                try:
                    with open_text(clock) as f:
                        c = json.loads(f.readline())
                    offset = c.get('offset_realtime_minus_monotonic_ns')
                except (OSError, EOFError, ValueError, TypeError) as error:
                    offset = None
                    health.setdefault('metadata_warnings', []).append(f'marker clock_sync: {error}')
                epoch = info.get('native_epoch_realtime_ns')
                for marker in info['markers_raw']:
                    ns = marker.get('capture_time_ns')
                    if ns is not None:
                        if source['kind'] == 'native' and epoch is not None and offset is not None:
                            marker['viewer_time_s'] = (ns+offset-epoch)/1e9
                        elif source['kind'] == 'jsonl' and info.get('callback_origin_ns') is not None:
                            marker['viewer_time_s'] = (ns-info['callback_origin_ns'])/1e9
        db.execute('INSERT INTO metadata VALUES(?)', (json.dumps(info, allow_nan=False),))
        db.commit()
        return info
    finally:
        db.close()


def catalogue(dump_root, repo):
    sources = {}

    def add(identifier, path, label, session, **extra):
        if not path or not path.is_file():
            return
        sources[identifier] = dict(id=identifier, path=path.resolve(), label=label,
                                   session=session, bytes=path.stat().st_size, **extra)

    for folder in sorted(dump_root.iterdir(), reverse=True):
        if not folder.is_dir() or not (folder/'manifest.json').is_file():
            continue
        manifest = load_json(folder/'manifest.json', {})
        status = manifest.get('status', 'unknown')
        native = stream_file(folder, 'native.rec')
        common = dict(session_path=folder, manifest_status=status,
                      revision=manifest.get('libsurvive_revision'), group='recordings')
        add(f'dump:{folder.name}:native', native, f'{folder.name} / native', folder.name,
            kind='native', input_group=folder.name, **common)
        add(f'dump:{folder.name}:poses', stream_file(folder, 'poses.jsonl'),
            f'{folder.name} / {"Monado stages" if native else "SteamVR reference"}', folder.name,
            kind='jsonl', input_group=None, **common)
    hand = repo/'.lh-evaluation/handheld-fix-2026-10-25'
    a = '2026-10-25_211424_libsurvive_158643023_1366788'
    r4 = '2026-10-24_203705_libsurvive_907114938_814822'
    for run, session, label in [('baseline', a, 'A / 旧設定 replay'),
                                ('tracker-final', a, 'A / tracker constraints replay'),
                                ('r4-before', r4, 'R4 / 旧設定 replay'),
                                ('r4-tracker-final', r4, 'R4 / tracker constraints replay')]:
        add(f'replay:handheld:{run}', hand/run/'record.rec.gz', label, session,
            kind='native', group='handheld replays', input_group=session,
            manifest_status='derived replay; see run.json', session_path=dump_root/session)
    full = repo/'.lh-evaluation/full-2026-10-24'
    for run in sorted(full.glob('replay-*')):
        if not run.is_dir():
            continue
        for path in sorted(run.glob('*-0.rec.gz')):
            session = path.name.removesuffix('-0.rec.gz')
            add(f'replay:corpus:{session}', path, f'{session} / full-corpus historical replay', session,
                kind='native', group='historical corpus replays', input_group=None,
                manifest_status='historical derived replay; not current core', session_path=dump_root/session)
    return sources


class Store:
    def __init__(self, dump_root, cache, repo=REPO):
        self.dump_root, self.repo = dump_root, repo
        self.sources = catalogue(dump_root, repo)
        self.cache = cache
        cache.mkdir(parents=True, exist_ok=True)
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix='pose-index')
        self.lock, self.jobs = threading.Lock(), {}
        self.cancel = threading.Event()

    def refresh(self):
        self.sources = catalogue(self.dump_root, self.repo)

    def public(self, source):
        return {k: v for k, v in source.items() if k not in ('path', 'session_path')}

    def db_path(self, source):
        s = source['path'].stat()
        key = f'{VERSION}:{source["path"]}:{s.st_size}:{s.st_mtime_ns}'
        return self.cache/(hashlib.sha256(key.encode()).hexdigest()+'.sqlite')

    def ensure(self, identifier):
        source = self.sources[identifier]
        path = self.db_path(source)
        with self.lock:
            if path.exists():
                with sqlite3.connect(f'file:{path}?mode=ro', uri=True) as db:
                    info = json.loads(db.execute('SELECT value FROM metadata').fetchone()[0])
                return dict(state='ready', source=self.public(source), **info)
            job = self.jobs.get(path)
            if job is None:
                def build():
                    fd, name = tempfile.mkstemp(prefix=path.stem+'-', suffix='.building', dir=self.cache)
                    os.close(fd)
                    temp = Path(name)
                    try:
                        index_source(source, temp, self.cancel)
                        os.replace(temp, path)
                    finally:
                        if temp.exists():
                            temp.unlink()
                job = self.pool.submit(build)
                self.jobs[path] = job
            if job.done():
                error = job.exception()
                if error:
                    return dict(state='error', error=str(error), source=self.public(source))
            return dict(state='indexing', queued=not job.running(), source=self.public(source))

    def window(self, identifier, track, lo, hi):
        if not all(map(math.isfinite, (lo, hi))) or hi <= lo or hi-lo > MAX_WINDOW:
            raise ValueError(f'Window must be > 0 and <= {MAX_WINDOW} seconds')
        info = self.ensure(identifier)
        if info['state'] != 'ready':
            return info
        if track not in {x['id'] for x in info['tracks']}:
            raise ValueError('Unknown track')
        path = self.db_path(self.sources[identifier])
        with sqlite3.connect(f'file:{path}?mode=ro', uri=True) as db:
            rows = db.execute(f'SELECT {SQL_COLUMNS} FROM samples WHERE track=? AND t>=? AND t<=? ORDER BY t,rowid LIMIT 100001',
                              (track, lo, hi)).fetchall()
        if len(rows) > 100000:
            raise ValueError('Too many samples; choose a shorter window')
        return dict(state='ready', columns=COLUMNS, rows=rows, source_sha256=info['source_sha256'],
                    track=next(x for x in info['tracks'] if x['id'] == track),
                    lo=lo, hi=hi, decimation=False)


class Handler(http.server.BaseHTTPRequestHandler):
    server_version = 'LighthouseViewer/1'

    def do_GET(self):
        parsed = urllib.parse.urlsplit(self.path)
        query = urllib.parse.parse_qs(parsed.query)
        try:
            if parsed.path == '/api/sources':
                self.server.store.refresh()
                self.json(dict(sources=[self.server.store.public(s) for s in self.server.store.sources.values()],
                               max_window_s=MAX_WINDOW))
            elif parsed.path == '/api/source':
                self.json(self.server.store.ensure(query['id'][0]))
            elif parsed.path == '/api/window':
                self.json(self.server.store.window(query['id'][0], query['track'][0], float(query['lo'][0]), float(query['hi'][0])))
            elif parsed.path in ('/', '/index.html', '/viewer.js', '/viewer.css'):
                name = 'index.html' if parsed.path == '/' else parsed.path[1:]
                self.send_data((ASSETS/name).read_bytes(), mimetypes.guess_type(name)[0] or 'application/octet-stream')
            else:
                self.send_error(404)
        except (KeyError, ValueError, IndexError) as error:
            self.json(dict(error=str(error)), status=400)
        except (OSError, sqlite3.Error) as error:
            self.json(dict(error=str(error)), status=500)

    def json(self, value, status=200):
        data = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()
        compress = len(data) > 2048 and 'gzip' in self.headers.get('Accept-Encoding', '')
        self.send_data(gzip.compress(data, compresslevel=1) if compress else data,
                       'application/json; charset=utf-8', status, compress)

    def send_data(self, data, mime, status=200, compressed=False):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
        if compressed:
            self.send_header('Content-Encoding', 'gzip')
            self.send_header('Vary', 'Accept-Encoding')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dumps', type=Path, default=Path.home()/'LH-Dump')
    p.add_argument('--cache', type=Path, default=REPO/'.lh-evaluation/viewer-cache')
    p.add_argument('--host', default='127.0.0.1', help='Use 0.0.0.0 for LAN access')
    p.add_argument('--port', type=int, default=8765)
    p.add_argument('--index', action='append', default=[], help='Pre-index a source ID; repeatable')
    args = p.parse_args()
    store = Store(args.dumps.expanduser().resolve(), args.cache.expanduser().resolve())
    for identifier in args.index:
        store.ensure(identifier)
    server = http.server.ThreadingHTTPServer((args.host, args.port), Handler)
    server.store = store
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    print(f'Lighthouse viewer: http://{args.host}:{server.server_port} ({len(store.sources)} streams)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        store.cancel.set()
        store.pool.shutdown(wait=True, cancel_futures=True)


if __name__ == '__main__':
    main()
