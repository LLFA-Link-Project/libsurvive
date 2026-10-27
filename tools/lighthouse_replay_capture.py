"""Capture an analysis recording without storing unused callback streams.

The original executable still writes its complete gzip stream. A FIFO consumer
checks that stream's CRC and hashes every byte before retaining selected rows.
No callback, estimator state or timestamp is changed. The retained file is an
analysis artifact; reproduce the full recording from the recorded command.
"""
import collections
import gzip
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time


DEFAULT_DROP_KINDS = frozenset({b'EXTERNAL_POSE', b'OPTICAL_TIMING', b'VELOCITY'})


def capture(command, destination, log, *, environment=None, timeout=7200,
            drop_kinds=DEFAULT_DROP_KINDS):
    """Run a --record command, preserving all rows except named analysis exclusions."""
    command = list(map(str, command))
    try:
        record_index = command.index('--record') + 1
        command[record_index]
    except (ValueError, IndexError):
        raise ValueError('Capture requires an explicit --record filename') from None
    destination = Path(destination).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(destination)
    drop_kinds = frozenset(drop_kinds)
    started = threading.Event()
    failures, evidence = [], {}

    with tempfile.TemporaryDirectory(prefix='capture-', dir=destination.parent) as scratch:
        fifo = Path(scratch)/'full-recording.rec.gz'
        os.mkfifo(fifo)
        # Keep a writer endpoint open until the real producer has written its
        # first bytes, or exited. This also releases the reader on startup errors.
        keeper = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)

        def consume():
            compressed, full, retained = (hashlib.sha256() for _ in range(3))
            compressed_size = 0
            counts, sizes = collections.Counter(), collections.Counter()
            kept_counts = collections.Counter()

            class DigestReader:
                def __init__(self, source):
                    self.source = source

                def read(self, size=-1):
                    nonlocal compressed_size
                    block = self.source.read(size)
                    if block:
                        compressed_size += len(block)
                        compressed.update(block)
                        started.set()
                    return block

            try:
                with fifo.open('rb', buffering=0) as source, \
                        gzip.GzipFile(fileobj=DigestReader(source)) as decoded, \
                        gzip.GzipFile(filename=str(destination), mode='wb',
                                      compresslevel=9, mtime=0) as output:
                    for line in decoded:
                        full.update(line)
                        if b'\0' in line or not line.endswith(b'\n'):
                            raise ValueError('Damaged native recording row')
                        fields = line.split(maxsplit=3)
                        kind = fields[2] if len(fields) >= 3 else b'<short>'
                        counts[kind] += 1
                        sizes[kind] += len(line)
                        if kind not in drop_kinds:
                            output.write(line)
                            retained.update(line)
                            kept_counts[kind] += 1
                evidence.update(
                    input_gzip_crc_checked=compressed_size > 0,
                    full_compressed_bytes=compressed_size,
                    full_compressed_sha256=compressed.hexdigest(),
                    full_uncompressed_sha256=full.hexdigest(),
                    retained_uncompressed_sha256=retained.hexdigest(),
                    input_rows=sum(counts.values()), retained_rows=sum(kept_counts.values()),
                    dropped_kinds=sorted(x.decode('ascii') for x in drop_kinds),
                    input_counts={k.decode('ascii'): v for k, v in sorted(counts.items())},
                    input_bytes_by_kind={k.decode('ascii'): v for k, v in sorted(sizes.items())},
                    retained_counts={k.decode('ascii'): v for k, v in sorted(kept_counts.items())})
            except Exception as error:
                failures.append(error)

        reader = threading.Thread(target=consume, daemon=True)
        reader.start()
        command[record_index] = str(fifo)
        process = None
        try:
            with Path(log).open('w') as stream:
                process = subprocess.Popen(command, env=environment, stdout=stream, stderr=stream)
                startup_deadline = time.monotonic() + min(30, timeout)
                while not started.wait(.01) and process.poll() is None:
                    if time.monotonic() > startup_deadline:
                        raise TimeoutError('Producer did not open its recording FIFO')
                os.close(keeper)
                keeper = None
                returncode = process.wait(timeout=timeout)
        finally:
            if keeper is not None:
                os.close(keeper)
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
            reader.join(timeout=30)
        if reader.is_alive():
            raise TimeoutError('Recording consumer did not finish')
        if failures:
            raise ValueError(f'Recording capture failed (producer exit {process.returncode}): '
                             f'{failures[0]}') from failures[0]
        if returncode == 0 and not evidence['input_gzip_crc_checked']:
            raise ValueError('Successful producer did not write a gzip stream')
        evidence.update(command=command, returncode=returncode,
                        retained_file_bytes=destination.stat().st_size)
        return evidence
