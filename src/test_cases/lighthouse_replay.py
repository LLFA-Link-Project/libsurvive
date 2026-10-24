import gzip
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

TOOLS = pathlib.Path(__file__).resolve().parents[2]/'tools'
sys.path.insert(0,str(TOOLS))
import lighthouse_corpus_report as corpus
RUNNER = TOOLS/'lighthouse_replay.py'


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

    def test_resume_rejects_changed_capture(self):
        output=self.root/'results'
        self.assertEqual(self.run_replay(output).returncode,0)
        before=(output/'results.json').read_bytes()
        (self.session/'usb.pcap.gz').write_bytes(b'changed input')
        result=self.run_replay(output,'--resume')
        self.assertNotEqual(result.returncode,0)
        self.assertEqual((output/'results.json').read_bytes(),before)
        self.assertEqual((self.root/'calls').read_text(),'1')

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


class CorpusLedger(unittest.TestCase):
    def test_missing_work_and_unqualified_results_remain_visible(self):
        with tempfile.TemporaryDirectory() as temp:
            root=pathlib.Path(temp)
            rows=[dict(session='good',status='complete',bytes=10,errors=[]),
                  dict(session='partial',status='recording',bytes=20,errors=[])]
            def write(path,value):
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text(json.dumps(value))
            write(root/'inventory.json',rows)
            write(root/'source-integrity.json',dict(bytes_read=30,sessions=[dict(session=r['session'],files=[],missing_manifest_files=[]) for r in rows]))
            write(root/'gzip-integrity.json',dict(sessions=[dict(session=r['session'],files=[]) for r in rows]))
            entries=[]
            for row in rows:
                name=row['session']
                write(root/'reference-index'/(name+'.json'),dict(rows_read=10,index_state='complete'))
                entries.append(dict(session=name,identical_pose_streams=True,runs=[dict(returncode=0,ok=True,objects={'a':dict(poses=10)})]))
            write(root/'replay-0/results.json',dict(settings={},sessions=entries))
            for name,error in [('good',.1),('partial',10)]:
                if name=='partial':
                    self.assertFalse(corpus.assemble(root)['complete'])
                metric=dict(n=3,rmse=error)
                comparison=dict(evaluation_samples=3,evaluation_sample_hz=5,
                                objects={'a':dict(position_difference_m=metric,orientation_difference_deg=metric,coverage={'fraction':1})})
                write(root/'comparison'/(name+'.json'),dict(comparison=comparison))
            result=corpus.assemble(root)
            self.assertTrue(result['complete'])
            self.assertEqual(result['summary']['sessions'],2)
            pooled=result['summary']['pooled_heldout_disagreement_qualified_only']['position_difference_m']
            self.assertAlmostEqual(pooled['rmse'],.1)
            self.assertEqual(pooled['n'],3)
            self.assertFalse(result['sessions'][1]['eligible_for_summary'])
            self.assertIsNone(result['sessions'][0]['replay']['identical_pose_streams'])
            corpus.write_report(result,root/'report')
            self.assertEqual(len((root/'report/objects.csv').read_text().splitlines()),3)
            write(root/'reference-index/partial.json',dict(index_state='building'))
            self.assertFalse(corpus.assemble(root)['complete'])


if __name__=='__main__':unittest.main()
