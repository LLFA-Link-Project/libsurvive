import gzip
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

from lighthouse_replay_capture import capture


@unittest.skipUnless(sys.platform.startswith('linux'), 'FIFO capture requires Linux')
class ReplayCaptureTests(unittest.TestCase):
    def run_capture(self, root, compressed, exit_code=0, fail_before_open=False):
        source = root/'source.gz'
        source.write_bytes(compressed)
        writer = root/'writer.py'
        writer.write_text('''import pathlib,sys
if sys.argv[1] == 'fail': sys.exit(2)
record = sys.argv[sys.argv.index('--record')+1]
source = sys.argv[sys.argv.index('--input')+1]
with open(record,'wb') as output: output.write(pathlib.Path(source).read_bytes())
sys.exit(int(sys.argv[1]))
''')
        command = [sys.executable, str(writer), 'fail' if fail_before_open else str(exit_code),
                   '--record', str(root/'original.gz'), '--input', str(source)]
        return capture(command, root/'retained.gz', root/'run.log', timeout=5)

    def test_crc_hashes_and_exact_retained_timestamps_and_unknown_rows(self):
        rows = [b'0.0 OPTION setting 1\n', b'1.001 WM0 POSE 1 2 3 1 0 0 0\n',
                b'1.001 WM0 POSE_TIMING 192000 48000000\n',
                b'1.002 WM0 EXTERNAL_POSE raw diagnostics\n',
                b'1.003 WM0 OPTICAL_TIMING original tick\n',
                b'1.004 WM0 VELOCITY 1 2 3\n', b'1.005 WM0 FUTURE_TAG keep me\n']
        expected = b''.join(rows[:3] + rows[6:])
        compressed = gzip.compress(b''.join(rows), mtime=0)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            result = self.run_capture(root, compressed)
            self.assertEqual(result['returncode'], 0)
            self.assertTrue(result['input_gzip_crc_checked'])
            self.assertEqual(gzip.decompress((root/'retained.gz').read_bytes()), expected)
            self.assertEqual(result['full_compressed_sha256'], hashlib.sha256(compressed).hexdigest())
            self.assertEqual(result['full_uncompressed_sha256'], hashlib.sha256(b''.join(rows)).hexdigest())
            self.assertEqual(result['retained_uncompressed_sha256'], hashlib.sha256(expected).hexdigest())
            self.assertEqual(result['input_rows'], 7)
            self.assertEqual(result['retained_rows'], 4)

    def test_writer_failure_before_open_releases_consumer(self):
        with tempfile.TemporaryDirectory() as folder:
            result = self.run_capture(Path(folder), b'', fail_before_open=True)
            self.assertEqual(result['returncode'], 2)
            self.assertFalse(result['input_gzip_crc_checked'])
            self.assertEqual(result['input_rows'], 0)

    def test_successful_writer_without_a_gzip_stream_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, 'did not write a gzip stream'):
                self.run_capture(Path(folder), b'')

    def test_truncated_gzip_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, 'capture failed'):
                self.run_capture(Path(folder), gzip.compress(b'0 WM0 POSE content\n')[:-5])

    def test_damaged_native_row_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, 'Damaged native recording row'):
                self.run_capture(Path(folder), gzip.compress(b'0 WM0 POSE no-newline'))

    def test_requires_record_argument(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                capture([sys.executable, '-V'], Path(folder)/'out.gz', Path(folder)/'log')


if __name__ == '__main__':
    unittest.main()
