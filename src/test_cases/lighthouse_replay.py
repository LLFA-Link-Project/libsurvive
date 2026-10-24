import gzip
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

RUNNER = pathlib.Path(__file__).resolve().parents[2]/'tools/lighthouse_replay.py'


class ReplayRunner(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)
        self.binary = self.root/'fake-cli'
        self.binary.write_text('''#!/usr/bin/env python3
import gzip,pathlib,sys
root=pathlib.Path(__file__).parent
counter=root/'calls'
counter.write_text(str(int(counter.read_text())+1 if counter.exists() else 1))
record=sys.argv[sys.argv.index('--record')+1]
with gzip.open(record,'wt') as f:
 f.write('0 a CONFIG {"device_serial_number":"object"}\\n')
 f.write('1 a POSE 1 2 3 1 0 0 0\\n')
''')
        self.binary.chmod(0o755)
        self.session = self.root/'capture'
        self.session.mkdir()
        (self.session/'usb.pcap.gz').write_bytes(b'fixture')
        (self.session/'usb.pcap.gz.usbdevs').write_text('fixture')
        self.manifest = self.session/'manifest.json'
        self.manifest.write_text(json.dumps(dict(status='complete', errors=[])))

    def tearDown(self):
        self.temp.cleanup()

    def run_replay(self, output, *flags):
        return subprocess.run([sys.executable,str(RUNNER),str(self.binary),str(self.session),
                               '--repeat','1','--output',str(output),*flags],capture_output=True,text=True)

    def test_resume_does_not_rerun_and_rejects_changed_build(self):
        output=self.root/'results'
        first=self.run_replay(output,'--record-timing')
        self.assertEqual(first.returncode,0,first.stderr)
        report=json.loads((output/'results.json').read_text())
        self.assertTrue(report['sessions'][0]['qualified_capture'])
        self.assertFalse(report['sessions'][0]['reproducibility_checked'])
        self.assertIsNone(report['sessions'][0]['identical_pose_streams'])
        self.assertEqual(self.run_replay(output,'--record-timing','--resume').returncode,0)
        self.assertEqual((self.root/'calls').read_text(),'1')
        before=(output/'results.json').read_bytes()
        self.binary.write_text(self.binary.read_text()+'\n')
        changed=self.run_replay(output,'--record-timing','--resume')
        self.assertNotEqual(changed.returncode,0)
        self.assertIn('build differ',changed.stderr)
        self.assertEqual((output/'results.json').read_bytes(),before)

    def test_incomplete_is_explicit_and_opt_in(self):
        self.manifest.write_text(json.dumps(dict(status='recording',errors=[])))
        output=self.root/'default'
        self.assertEqual(self.run_replay(output).returncode,0)
        self.assertIn('skipped',json.loads((output/'results.json').read_text())['sessions'][0])
        self.assertFalse((self.root/'calls').exists())
        output=self.root/'partial'
        self.assertEqual(self.run_replay(output,'--include-incomplete').returncode,0)
        result=json.loads((output/'results.json').read_text())['sessions'][0]
        self.assertFalse(result['qualified_capture'])
        self.assertEqual(result['manifest_status'],'recording')
        self.assertTrue(result['runs'][0]['ok'])


if __name__=='__main__':unittest.main()
