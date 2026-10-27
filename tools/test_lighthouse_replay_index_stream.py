from functools import partial
import gzip
from pathlib import Path
import random
import tempfile
import unittest

import lighthouse_compare_disk as disk
import lighthouse_replay_index_stream as streaming


class Clock:
    def convert(self, value):
        if 900_000_000 <= value <= 2_100_000_000:
            return value + 123456, 17
        return None


class StreamIndexTests(unittest.TestCase):
    def compare(self, lines, allow_unbracketed_span=False):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            recording = root/'record.rec.gz'
            with gzip.open(recording, 'wt') as f:
                f.writelines(lines)
            results = []
            for index, function in enumerate((disk.replay_index, partial(streaming.replay_index,
                                                                        chunk_rows=7, scratch_parent=root))):
                db = disk.create_db(root/f'{index}.sqlite')
                try:
                    counts = function(db, recording, Clock(), allow_unbracketed_span=allow_unbracketed_span)
                    rows = list(db.execute('SELECT serial,t,x,y,z,w,qx,qy,qz FROM replay ORDER BY serial,t,id'))
                    results.append((counts, rows))
                finally:
                    db.close()
            self.assertEqual(results[0], results[1])
            self.assertFalse(list(root.glob('lh-sorted-replay-*')))
            return results[1]

    def test_unordered_full_ticks_ties_identity_changes_and_many_merge_runs(self):
        rng = random.Random(45821)
        lines = []
        for segment, serial in enumerate(('old-device', 'new-device')):
            lines.append(f'0 WM0 CONFIG {{"device_serial_number":"{serial}"}}\n')
            lines.append(f'0 WM1 CONFIG {{"device_serial_number":"{serial}"}}\n')
            messages = []
            for obj in ('WM0', 'WM1'):
                for i in range(270):
                    tick = 5_000_000_000 + segment * 480_000_000 + i * 96_000
                    receive_us = 1_000_000 + i * 2000 + (i % 3)
                    if i == 94:
                        receive_us += 50_000  # Preserve the ambiguous-anchor rejection.
                    messages.append([f'0 {obj} INPUT_TIME {tick} {receive_us}\n'])
                    if i % 2 == 0:
                        pose = f'{i*.0001} {segment*.1} {obj == "WM0":d} 0.8 0.2 -0.4 0.4'
                        message = [f'0 {obj} POSE {pose}\n',
                                   f'0 {obj} POSE_TIMING {tick+480} 48000000 0 0\n']
                        messages.append(message)
                        if i % 10 == 0:
                            # Identical ticks with different payloads require the original order.
                            messages.append([f'0 {obj} POSE 0 0 0 1 0 0 0\n',
                                             f'0 {obj} POSE_TIMING {tick+480} 48000000 0 0\n'])
            rng.shuffle(messages)
            lines.extend(line for message in messages for line in message)
        counts, rows = self.compare(lines)
        self.assertEqual(counts['receiver_identity_changes'], 2)
        self.assertGreater(counts['ambiguous_sensor_clock'], 0)
        self.assertGreater(len(rows), 20)

    def test_unbracketed_endpoints_use_same_convertible_input_span(self):
        lines = ['0 WM0 CONFIG {"device_serial_number":"A"}\n']
        for i, receive in enumerate((800_000, 1_000_000, 1_010_000, 1_020_000, 2_200_000)):
            tick = 48_000_000 + i * 48_000
            lines.extend([f'0 WM0 INPUT_TIME {tick} {receive}\n',
                          '0 WM0 POSE 0 0 0 1 0 0 0\n',
                          f'0 WM0 POSE_TIMING {tick} 48000000 0 0\n'])
        counts, _ = self.compare(lines, allow_unbracketed_span=True)
        self.assertEqual(counts['unbracketed_input_span_endpoints'], 2)

    def test_no_output_retains_empty_capture_case(self):
        self.assertEqual(self.compare(['0 WM0 CONFIG {"device_serial_number":"A"}\n'])[0],
                         {'no_replay_input_or_poses': 1})

    def test_malformed_pose_rejected(self):
        for function in (disk.replay_index, streaming.replay_index):
            with self.subTest(function=function.__module__), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                recording = root/'record.rec.gz'
                with gzip.open(recording, 'wt') as f:
                    f.write('0 WM0 POSE 0 0 nan 1 0 0 0\n')
                db = disk.create_db(root/'test.sqlite')
                try:
                    with self.assertRaisesRegex(ValueError, 'Nonfinite or malformed'):
                        function(db, recording, Clock())
                finally:
                    db.close()


if __name__ == '__main__':
    unittest.main()
