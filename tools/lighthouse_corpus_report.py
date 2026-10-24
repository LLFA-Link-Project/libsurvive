#!/usr/bin/env python3
"""Assemble an auditable corpus ledger from inventory, replay and comparison checkpoints."""
import argparse
import collections
import csv
import datetime
import json
import math
import pathlib
import re


def read_json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def assemble(root):
    inventory=read_json(root/'inventory.json',[])
    source=read_json(root/'source-integrity.json',{'sessions':[]})
    gzip_checks=read_json(root/'gzip-integrity.json',{'sessions':[]})
    source_by_name={s['session']:s for s in source['sessions']}
    gzip_by_name={s['session']:s for s in gzip_checks['sessions']}
    reference_cache={p.stem:read_json(p) for p in (root/'reference-index').glob('*.json')}
    replay={};builds={}
    for path in sorted(root.glob('replay-*/results.json')):
        report=read_json(path)
        builds[path.parent.name]=report.get('settings',{})
        for entry in report['sessions']:replay[entry['session']]=(path.parent,entry)
    sessions=[];pose_total=0;reference_callbacks=0
    sums={'position_difference_m':[0.,0], 'orientation_difference_deg':[0.,0]}
    for row in inventory:
        name=row['session'];item=dict(row)
        item['qualified_capture']=row.get('status')=='complete' and not row.get('errors')
        item['replay_state']='pending';item['comparison_state']='pending'
        cache=reference_cache.get(name,{})
        item['reference_index_state']=cache.get('index_state', 'complete' if 'rows_read' in cache else ('error' if 'error' in cache else 'pending'))
        item['reference_rows_read']=cache.get('rows_read',0)
        if 'error' in cache:item['reference_index_error']=cache['error']
        if 'integrity' in cache:item['reference_integrity']=cache['integrity']
        audit=source_by_name.get(name)
        if audit:
            item['source_hash_check']={'declared_verified':sum(f['matches_manifest'] is True for f in audit['files']),
                'mismatches':[f['name'] for f in audit['files'] if f['matches_manifest'] is False],
                'changed_files':[f['name'] for f in audit['files'] if f['changed_while_reading']],
                'missing_files':audit['missing_manifest_files']}
        gz=gzip_by_name.get(name)
        if gz:
            item['gzip_errors']={f['name']:f['error'] for f in gz['files'] if f['returncode']}
        item['eligible_for_summary']=(item['qualified_capture'] and audit is not None and gz is not None
            and not item['gzip_errors'] and not any(item['source_hash_check'][k] for k in ('mismatches','changed_files','missing_files')))
        run=None
        if name in replay:
            directory,entry=replay[name]
            if entry.get('skipped'):
                item['replay_state']='unavailable';item['replay_reason']=entry['skipped']
            elif entry.get('runs'):
                run=entry['runs'][0]
                if run.get('timeout'):item['replay_state']='timeout'
                elif run.get('returncode')!=0:item['replay_state']='process_failed'
                elif run.get('error'):item['replay_state']='output_error'
                elif run.get('ok'):item['replay_state']='finite_output'
                else:item['replay_state']='no_finite_poses'
                item['replay']={k:v for k,v in run.items() if k!='command'}
                item['replay']['reproducibility_checked']=len(entry['runs'])>=2
                item['replay']['identical_pose_streams']=entry.get('identical_pose_streams') if len(entry['runs'])>=2 else None
                pose_total+=sum(o['poses'] for o in run.get('objects',{}).values())
        # A file being written is progress evidence, not a completed replay result.
        for directory in root.glob('replay-*'):
            log=directory/(name+'-0.log')
            if item['replay_state']=='pending' and log.exists():
                item['replay_state']='running'
                with log.open('rb') as f:
                    f.seek(max(0,log.stat().st_size-4096));tail=f.read().decode(errors='replace')
                matches=re.findall(r'^([0-9]+\.[0-9]+) ',tail,re.MULTILINE)
                if matches:item['latest_logged_receive_s']=float(matches[-1])
        comp=read_json(root/'comparison'/(name+'.json'))
        if comp:
            if 'comparison' in comp:
                item['comparison']=comp['comparison']
                # Sparse error evaluation cannot diagnose the 50ms recovery criterion.
                if item['comparison'].get('evaluation_sample_hz',50)<20:
                    for obj in item['comparison']['objects'].values():
                        obj['recovery_after_reference_reacquisition']={'unavailable':'Error sampling below 20Hz cannot resolve the <=50ms recovery criterion'}
                item['comparison_state']='available' if comp['comparison']['evaluation_samples'] else 'no_evaluation_samples'
                item['reference_rows_read']=max(item['reference_rows_read'],comp['comparison'].get('reference_index',{}).get('rows_read',0))
                if item['eligible_for_summary']:
                    for obj in comp['comparison']['objects'].values():
                        for metric,(sum_sq,n) in list(sums.items()):
                            value=obj.get(metric)
                            if value:
                                sums[metric]=[sum_sq+value['rmse']**2*value['n'],n+value['n']]
            else:
                item['comparison_state']='unavailable'
                item['comparison_reason']=comp.get('error',comp.get('unavailable'))
        reference_callbacks+=item['reference_rows_read']
        sessions.append(item)
    counts=collections.Counter(s['replay_state'] for s in sessions)
    comp_counts=collections.Counter(s['comparison_state'] for s in sessions)
    index_counts=collections.Counter(s['reference_index_state'] for s in sessions)
    complete=(not any(index_counts[k] for k in ('pending','queued','building')) and bool(inventory) and counts['pending']==0 and counts['running']==0 and comp_counts['pending']==0
              and len(source_by_name)==len(inventory) and len(gzip_by_name)==len(inventory))
    return {'generated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'complete':complete,'reference_is_ground_truth':False,
        'protocol':'Full USB replay once per capture; all references indexed when comparable; 5Hz held-out error and 50Hz output coverage; no fitted time lag',
        'summary':{'sessions':len(sessions),'source_bytes':sum(s['bytes'] for s in sessions),
            'manifest_states':dict(collections.Counter(s.get('status') for s in sessions)),
            'replay_states':dict(counts),'comparison_states':dict(comp_counts),'emitted_poses':pose_total,
            'reference_callbacks_indexed':reference_callbacks,'reference_index_states':dict(index_counts),'source_bytes_hashed':source.get('bytes_read',0),
            'gzip_sessions_checked':len(gzip_by_name),
            'pooled_heldout_disagreement_qualified_only':{metric:{'n':n,'rmse':math.sqrt(ss/n) if n else None} for metric,(ss,n) in sums.items()},
            'pooling_note':'Weighted by matched error samples; each recording fits its own training transform; long/static recordings can dominate'},
        'builds':builds,'sessions':sessions}


def write_report(report, output):
    output.mkdir(parents=True,exist_ok=True)
    temp=output/'results.json.tmp';temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');temp.replace(output/'results.json')
    columns=['session','manifest_status','eligible_for_summary','source_bytes','duration_s','replay_state','comparison_state','object','position_rmse_mm','orientation_rmse_deg','coverage','evaluation_samples','reason']
    with (output/'objects.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(columns)
        for s in report['sessions']:
            objects=s.get('comparison',{}).get('objects',{}) or {'':{}}
            for serial,obj in objects.items():
                pos=obj.get('position_difference_m');rot=obj.get('orientation_difference_deg')
                writer.writerow([s['session'],s.get('status'),s['eligible_for_summary'],s['bytes'],s.get('duration_s'),s['replay_state'],s['comparison_state'],serial,pos['rmse']*1000 if pos else '',rot['rmse'] if rot else '',obj.get('coverage',{}).get('fraction',''),pos['n'] if pos else '',s.get('comparison_reason',s.get('replay_reason',''))])
    summary=report['summary'];state='COMPLETE' if report['complete'] else 'IN PROGRESS'
    lines=[f'# Lighthouse entire-corpus evaluation — {state}','',
        f"{summary['sessions']} sessions; {summary['source_bytes']:,} source bytes. Manifest states: {summary['manifest_states']}.",'',
        f"Replay outcomes: {summary['replay_states']}. Reference comparisons: {summary['comparison_states']}.",'',
        'Valve poses are reference estimates, not ground truth. This is one full replay per session; it does not establish repeatability. Error is sampled at 5 Hz on held-out data; coverage uses mapped 50 Hz output. Partial/error-marked captures remain unqualified. Unavailable comparisons are listed, not omitted.','',
        '| Session | Manifest | Replay | Comparison | Pose differences: position mm / angle degrees |','| --- | --- | --- | --- | --- |']
    for s in report['sessions']:
        metrics=[]
        for serial,obj in s.get('comparison',{}).get('objects',{}).items():
            pos=obj.get('position_difference_m');rot=obj.get('orientation_difference_deg')
            metrics.append(f"{serial}: {pos['rmse']*1000:.1f} / {rot['rmse']:.2f}" if pos and rot else f'{serial}: no held-out matches')
        reason=s.get('comparison_reason',s.get('replay_reason','')) or ''
        values='; '.join(metrics) or reason.replace('|','/') or 'pending'
        lines.append(f"| `{s['session']}` | {s.get('status')} | {s['replay_state']} | {s['comparison_state']} | {values} |")
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    template=pathlib.Path(__file__).with_name('lighthouse_report.html').read_text()
    # JSON is data inside a script element; escape '<' to prevent an embedded end tag.
    data=json.dumps(report,allow_nan=False).replace('<','\\u003c')
    (output/'report.html').write_text(template.replace('__LIGHTHOUSE_REPORT_DATA__',data))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=pathlib.Path);p.add_argument('--output',type=pathlib.Path,required=True)
    a=p.parse_args();r=assemble(a.root);write_report(r,a.output);print(json.dumps({'complete':r['complete'],**r['summary']},indent=2))

if __name__=='__main__':main()
