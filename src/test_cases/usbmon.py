"""Exercise usbmon using synthetic captures, without USB hardware or private data."""
import gzip
import json
import pathlib
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import zlib

BINARY = sys.argv.pop(1)


def header(urb_id, event=b"C", endpoint=0x81, length=0, setup=None, seconds=1700000000):
    return struct.pack("<QccBBHccqiiII8siiii", urb_id, event, b"\x02" if setup else b"\x01",
                       endpoint, 1, 1, b"\0" if setup and event == b"S" else b"-", b"\0", seconds, 0, 0,
                       length, length, setup or bytes(8), 0, 0, 0, 0)


def config_packets():
    config = json.dumps({"device_class": "controller", "device_serial_number": "TEST-REPLAY",
                         "device_vid": 10462, "device_pid": 8192,
                         "lighthouse_config": {"channelMap": [0, 1, 2, 3],
                                               "modelPoints": [[0, 0, 0]] * 4,
                                               "modelNormals": [[0, 0, 1]] * 4}}).encode()
    data = zlib.compress(config)
    chunks = [data[i:i + 62] for i in range(0, len(data), 62)] + [b""]
    packets = []
    for urb_id, chunk in enumerate(chunks, 1):
        setup = struct.pack("<BBHHH", 0xa1, 1, 0x311, 0, 64)
        packets.append(header(urb_id, event=b"S", endpoint=0x80, setup=setup))
        payload = bytes([0x11, len(chunk)]) + chunk
        packets.append(header(urb_id, endpoint=0x80, setup=bytes(8), length=len(payload)) + payload)
    return packets


class USBMon(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def capture(self, packets):
        path = self.root / "usb.pcap.gz"
        with gzip.open(path, "wb") as f:
            f.write(struct.pack("<IHHIIII", 0xa1b2c3d4, 2, 4, 0, 0, 65535, 220))
            for packet in packets:
                f.write(struct.pack("<IIII", 1700000000, 0, len(packet), len(packet)))
                f.write(packet)
        pathlib.Path(str(path) + ".usbdevs").write_text("10462 8449 1 1 WM\n")
        return path

    def command(self, path, factor="0"):
        return [BINARY, "--usbmon-playback", str(path), "--playback-factor", factor,
                "--configfile", str(self.root / "config.json"), "--no-threaded-posers",
                "--no-gss-threaded", "--record", str(self.root / "out.rec.gz")]

    def test_malformed_packets(self):
        setup = struct.pack("<BBHHH", 0xa1, 1, 0x311, 0, 64)
        packets = [header(100, event=b"S", endpoint=0x80, setup=setup),
                   header(100, endpoint=0x80, length=2) + b"\x11\xff"]
        packets += config_packets()
        packets += [b"short", header(90, length=100), header(91, length=65) + bytes(65),
                    header(92, endpoint=0x80, setup=bytes(8), length=1) + b"\x05"]
        path = self.capture(packets)
        result = subprocess.run(self.command(path), capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))
        output = (result.stdout + result.stderr).decode(errors="replace")
        self.assertIn("Short USB header skipped", output)
        self.assertIn("Truncated USB payload skipped", output)
        self.assertIn("Oversized USB report skipped", output)
        self.assertIn("usbmon loaded", output)
        self.assertIn("Invalid config chunk", output)

    def test_interrupt_capture_gap(self):
        path = self.capture([header(1), header(2, seconds=1700003600)])
        with subprocess.Popen(self.command(path, "1"), stdout=subprocess.PIPE, stderr=subprocess.PIPE) as proc:
            try:
                time.sleep(0.2)
                self.assertIsNone(proc.poll())
                proc.send_signal(signal.SIGTERM)
                out, err = proc.communicate(timeout=3)
                self.assertEqual(proc.returncode, 0, (out + err).decode(errors="replace"))
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.communicate()

    def test_only_short_headers(self):
        path = self.capture([b"short"] * 10)
        result = subprocess.run(self.command(path), capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))


if __name__ == "__main__":
    unittest.main()
