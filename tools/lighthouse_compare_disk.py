#!/usr/bin/env python3
"""Disk-backed native capture comparison; original files remain read-only.

Retains all reference validity transitions and all input clock anchors in SQLite.
Error evaluation defaults to 5 Hz; coverage uses the 50 Hz mapped replay stream.
"""
import argparse
import collections
import gzip
import json
import math
import pathlib
import sqlite3
import statistics
import time

import lighthouse_compare as comparison


try:
    import orjson as fast_json
    JSON_LOADS = fast_json.loads
    JSON_PARSER = 'orjson '+fast_json.__version__
except ImportError:
    JSON_LOADS = json.loads
    JSON_PARSER = 'Python standard json'


class Samples:
    def __init__(self, db, table, serial):
        self.db, self.table, self.serial = db, table, serial

    @staticmethod
    def unpack(row):
        return row[0], None if row[1] is None else (tuple(row[1:4]), tuple(row[4:8]))

    def __iter__(self):
        for row in self.db.execute(f'SELECT t,x,y,z,w,qx,qy,qz FROM {self.table} WHERE serial=? ORDER BY t,id', (self.serial,)):
            yield self.unpack(row)

    def __len__(self):
        return self.db.execute(f'SELECT COUNT(*) FROM {self.table} WHERE serial=?', (self.serial,)).fetchone()[0]

    def __bool__(self):
        return self.db.execute(f'SELECT 1 FROM {self.table} WHERE serial=? LIMIT 1', (self.serial,)).fetchone() is not None

    def __getitem__(self, index):
        if index not in (0, -1):
            raise IndexError('Disk samples support first/last indexing only')
        order = 'ASC' if index == 0 else 'DESC'
        row = self.db.execute(f'SELECT t,x,y,z,w,qx,qy,qz FROM {self.table} WHERE serial=? ORDER BY t {order},id {order} LIMIT 1', (self.serial,)).fetchone()
        if row is None:
            raise IndexError(index)
        return self.unpack(row)

    def interpolate(self, t, max_gap_ns):
        # Indexed neighbors retain invalid samples, so interpolation cannot bridge loss.
        a = self.db.execute(f'SELECT t,x,y,z,w,qx,qy,qz FROM {self.table} WHERE serial=? AND t<=? ORDER BY t DESC,id DESC LIMIT 1', (self.serial,t)).fetchone()
        if a is not None and a[0] == t:
            return self.unpack(a)[1]
        b = self.db.execute(f'SELECT t,x,y,z,w,qx,qy,qz FROM {self.table} WHERE serial=? AND t>? ORDER BY t,id LIMIT 1', (self.serial,t)).fetchone()
        if a is None or b is None:
            return None
        rows = [self.unpack(a), self.unpack(b)]
        return comparison.interpolate(rows, [a[0],b[0]], t, max_gap_ns)


def create_db(path):
    if path.exists():
        raise ValueError('Database path must be new')
    db = sqlite3.connect(path, uri=True)
    db.execute('PRAGMA journal_mode=OFF')  # Disposable derived index; originals are never written.
    db.execute('PRAGMA synchronous=OFF')
    db.execute('PRAGMA cache_size=-32768')
    db.execute('PRAGMA temp_store=FILE')
    db.execute('CREATE TABLE inputs(obj TEXT,tick INTEGER,receive INTEGER)')
    db.execute('CREATE TABLE poses(id INTEGER PRIMARY KEY,obj TEXT,tick INTEGER,hz INTEGER,x REAL,y REAL,z REAL,w REAL,qx REAL,qy REAL,qz REAL)')
    for table in ('replay','reference'):
        db.execute(f'CREATE TABLE {table}(id INTEGER PRIMARY KEY,serial TEXT,t INTEGER,x REAL,y REAL,z REAL,w REAL,qx REAL,qy REAL,qz REAL)')
    return db


def replay_index(db, path, clock, sample_hz=50, allow_unbracketed_span=False):
    identities, pending, segments, serials = {}, {}, {}, {}
    counts = collections.Counter()
    batch_inputs, batch_poses = [], []
    def flush():
        db.executemany('INSERT INTO inputs VALUES(?,?,?)', batch_inputs)
        db.executemany('INSERT INTO poses(obj,tick,hz,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?,?)', batch_poses)
        batch_inputs.clear(); batch_poses.clear()
    with gzip.open(path,'rt') as f:
        for line in f:
            row=line.split(maxsplit=3)
            if len(row)!=4:
                continue
            _,receiver,op,value=row
            obj=receiver+"#"+str(segments.get(receiver,0))
            if op=='CONFIG':
                serial=json.loads(value).get('device_serial_number')
                if receiver in serials and serials[receiver]!=serial:
                    segments[receiver]=segments.get(receiver,0)+1
                    obj=receiver+'#'+str(segments[receiver])
                    counts['receiver_identity_changes']+=1
                identities[obj]=serial
                serials[receiver]=serial
            elif op=='INPUT_TIME':
                tick,receive=map(int,value.split())
                if tick>0:
                    batch_inputs.append((obj,tick,receive*1000))
            elif op=='POSE':
                values=tuple(map(float,value.split()))
                if len(values)!=7 or not all(math.isfinite(v) for v in values):
                    raise ValueError('Nonfinite or malformed replay pose')
                pending[obj]=(*values[:3],*comparison.unit(values[3:]))
            elif op=='POSE_TIMING':
                tick,hz=map(int,value.split()[:2])
                if hz<=0 or obj not in pending:
                    raise ValueError('Unpaired or malformed pose timing')
                batch_poses.append((obj,tick,hz,*pending.pop(obj)))
            if len(batch_inputs)+len(batch_poses)>=4000:
                flush()
    flush();db.commit()
    db.execute('CREATE INDEX input_ticks ON inputs(obj,tick)')
    db.execute('CREATE INDEX pose_ticks ON poses(obj,tick,id)')
    span=db.execute('SELECT MIN(receive),MAX(receive) FROM inputs').fetchone()
    for name,value in zip(('capture_start_monotonic_ns','capture_end_monotonic_ns'),span):
        converted=clock.convert(value) if value is not None else None
        if converted is None and allow_unbracketed_span:
            order='ASC' if name.startswith('capture_start') else 'DESC'
            for candidate, in db.execute(f'SELECT DISTINCT receive FROM inputs ORDER BY receive {order}'):
                converted=clock.convert(candidate)
                if converted is not None:break
            counts['unbracketed_input_span_endpoints']+=1
        if converted is None:
            raise ValueError('Input span is not bracketed by clock samples')
        counts[name]=converted[0]
    for obj, in db.execute('SELECT DISTINCT obj FROM poses').fetchall():
        serial=identities.get(obj)
        if not serial:
            raise ValueError('Missing tracked-object identity')
        inputs=iter(db.execute('SELECT tick,receive FROM inputs WHERE obj=? ORDER BY tick,rowid',(obj,)))
        before=collections.deque(maxlen=8)
        after=collections.deque()
        def replenish():
            while len(after)<8:
                item=next(inputs,None)
                if item is None: break
                after.append(item)
        replenish()
        previous=None;batch=[]
        for tick,hz,*pose in db.execute('SELECT tick,hz,x,y,z,w,qx,qy,qz FROM poses WHERE obj=? ORDER BY tick,id',(obj,)):
            while after and after[0][0]<tick:
                before.append(after.popleft());replenish()
            anchors=[(tc,ns) for tc,ns in (*before,*after) if abs(tc-tick)/hz<.05]
            if not anchors:
                counts['unbracketed_sensor_time']+=1;continue
            estimates=[ns+round((tick-tc)*1e9/hz) for tc,ns in anchors]
            spread=max(estimates)-min(estimates)
            if spread>20_000_000:
                counts['ambiguous_sensor_clock']+=1;continue
            converted=clock.convert(int(statistics.median(estimates)))
            if converted is None:
                counts['unmapped_clock']+=1;continue
            ns,error=converted
            if previous is not None and ns-previous<1e9/sample_hz:continue
            previous=ns
            batch.append((serial,ns,*pose))
            counts['max_clock_mapping_estimate_ns']=max(counts['max_clock_mapping_estimate_ns'],error)
            counts['max_local_input_arrival_spread_ns']=max(counts['max_local_input_arrival_spread_ns'],spread)
            if len(batch)>=4000:
                db.executemany('INSERT INTO replay(serial,t,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?)',batch);batch=[]
        db.executemany('INSERT INTO replay(serial,t,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?)',batch)
    db.execute('CREATE INDEX replay_times ON replay(serial,t,id)');db.commit()
    return dict(counts)


def reference_index(db, session, time_field='pose_time_estimate_ns', allow_truncated=False):
    rejected=collections.Counter();batch=[];rows=0;integrity='complete'
    try:
        for row in comparison.jsonl(session/'poses.jsonl', loads=JSON_LOADS):
            rows+=1
            if row.get('device_class')==4:continue
            t=row.get(time_field)
            if t is None:
                rejected['missing_time']+=1;continue
            pose=None
            if row['pose_is_valid'] and row['device_is_connected'] and row['tracking_result']==200:
                try:
                    p=row['position_m']
                    if len(p)!=3 or not all(v is not None and math.isfinite(v) for v in p):raise ValueError('Invalid position')
                    pose=comparison.compose((row['world_from_driver_translation_m'],comparison.xyzw(row['world_from_driver_orientation_xyzw'])),
                        comparison.compose((p,comparison.xyzw(row['orientation_xyzw'])),
                        (row['driver_from_head_translation_m'],comparison.xyzw(row['driver_from_head_orientation_xyzw']))))
                    if not all(math.isfinite(v) for v in pose[0]):raise ValueError('Invalid offset')
                except (ValueError,TypeError):rejected['invalid_components']+=1
            else:rejected['invalid_or_not_running_ok']+=1
            values=(*pose[0],*pose[1]) if pose else (None,)*7
            batch.append((row['device_id'],t,*values))
            if len(batch)>=4000:
                db.executemany('INSERT INTO reference(serial,t,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?)',batch);batch=[]
    except (EOFError,OSError,json.JSONDecodeError) as error:
        if not allow_truncated:raise
        integrity='partial: '+str(error)
    db.executemany('INSERT INTO reference(serial,t,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?)',batch)
    db.execute('CREATE INDEX reference_times ON reference(serial,t,id)');db.commit()
    return {'rows_read':rows,'integrity':integrity,'rejected':dict(rejected),'json_parser':JSON_PARSER}


def run(session, recording, database, warmup=10, evaluation_hz=5, allow_truncated=False,
        reference_database=None, reference_info=None):
    db=create_db(database)
    started=time.monotonic()
    try:
        clock_rows=[];clock_integrity='complete'
        try:
            for row in comparison.jsonl(session/'clock_sync.jsonl'):clock_rows.append(row)
        except (EOFError,OSError,json.JSONDecodeError) as error:
            if not allow_truncated:raise
            clock_integrity='partial: '+str(error)
        clock=comparison.ClockMap(clock_rows)
        timing=replay_index(db,recording,clock,allow_unbracketed_span=allow_truncated)
        reference_table='reference'
        if reference_database is None:
            reference=reference_index(db,session,allow_truncated=allow_truncated)
        else:
            if reference_info is None or reference_info.get('session')!=session.name:
                raise ValueError('Matching cached reference metadata required')
            db.execute('ATTACH DATABASE ? AS native',('file:'+str(reference_database.resolve())+'?mode=ro',))
            reference_table='native.reference'
            reference=reference_info
        print(session.name,'indexed',reference['rows_read'],'reference callbacks',flush=True)
        replay={s:Samples(db,'replay',s) for s, in db.execute('SELECT DISTINCT serial FROM replay')}
        refs={s:Samples(db,reference_table,s) for s, in db.execute(f'SELECT DISTINCT serial FROM {reference_table}')}
        events_integrity='complete'
        try:events=comparison.load_reference_events(session)
        except (EOFError,OSError,json.JSONDecodeError):
            if not allow_truncated:raise
            events=None;events_integrity='partial; event summary unavailable'
        result=comparison.compare(replay,refs,warmup_s=warmup,evaluation_hz=evaluation_hz,
            bounds=(timing['capture_start_monotonic_ns'],timing['capture_end_monotonic_ns']),events=events,diagnostic_bin_s=60)
        result.update(timing=timing,reference_index=reference,clock_integrity=clock_integrity,events_integrity=events_integrity,wall_time_s=time.monotonic()-started,
            sampling='Replay sorted by full device ticks; 50Hz mapping/coverage, sampled error evaluation',
            timing_caveat='Input arrival mapping; device/USB/driver latency and prediction remain unknown')
        return result
    finally:db.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session',type=pathlib.Path);p.add_argument('recording',type=pathlib.Path)
    p.add_argument('--database',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True)
    p.add_argument('--evaluation-hz',type=float,default=5);p.add_argument('--warmup',type=float,default=10)
    p.add_argument('--allow-truncated',action='store_true')
    a=p.parse_args()
    if a.output.exists() or not 0<a.evaluation_hz<=50 or a.warmup<0:p.error('New output and valid sample rate/warmup required')
    result=run(a.session,a.recording,a.database,a.warmup,a.evaluation_hz,a.allow_truncated)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
