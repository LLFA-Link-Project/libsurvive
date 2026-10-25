"""Analytical checks, independent of the recorded tracking output."""
import gzip,json,tempfile
from pathlib import Path
import numpy as np
import lighthouse_body_drift as m
checks={}
# Nontrivial rotation: +90deg around Y maps body X to world -Z.
q=np.array([2**-.5,0,2**-.5,0]);R=np.array([[0,0,1],[0,1,0],[-1,0,0]])
v=np.array([.12,-.07,.035]);np.testing.assert_allclose(m.rotate(q,v),R@v,atol=1e-14)
np.testing.assert_allclose(m.rotate(m.conjugate(q),R@v),v,atol=1e-14)
checks['rotation_against_explicit_matrix']='pass'
# Constant physical world velocity, known additive body residual and nonunit scales.
# Cross low-32-bit rollover; full-tick association must preserve it.
hz=48_000_000;n=301;tick0=2**32-10*192000;full=tick0+np.arange(n)*192000;t=np.arange(n)*.004
world_v=np.array([.01,-.02,.03]);body_v=R.T@world_v
acc_bias=np.array([.023,-.017,.041]);scale=np.array([.98,1.02,.97]);res=np.array([.004,-.006,.008]);gyro_bias=np.array([.002,.003,-.004]);raw=scale*(R.T@np.array([0,0,1])+res)+acc_bias
with tempfile.TemporaryDirectory(prefix='lh-body-drift-check-') as td:
 p=Path(td);lines=['0 WM9 CONFIG '+json.dumps(dict(device_serial_number='synthetic'))+'\n']
 for i in range(n):
  pp=world_v*t[i];active=.05 if i<150 else .12
  # Known 2 mm output-only world X correction at the stationary onset.
  if i>=150:pp=pp+np.array([.002,0,0])
  def row(tag,values):lines.append(f'{t[i]:.9f} WM9 {tag} '+' '.join(format(float(x),'.17g') for x in values)+'\n')
  row('POSE_STATE',[full[i],hz,full[i]-int(active*hz),int(active*hz),0,*pp,*q])
  row('POSE_BIAS',[full[i],12,*gyro_bias,*acc_bias,*scale,1,0,0,0])
  row('POSE_BIAS_VAR',[full[i],*([1e-6]*6)])
  row('VELOCITY',[*world_v,0,0,0])
  row('I',[3,int(full[i])&0xffffffff,*raw,*gyro_bias,0,0,0,0])
  row('INPUT_TIME',[full[i],1_700_000_000_000_000+int(t[i]*1e6)])
 with gzip.open(p/'native.rec.gz','wt') as f:f.writelines(lines)
 arrays,meta=m.read_capture(p,'synthetic',False);a,j=m.join_rows(arrays);r=m.summarize(a,.1,1.1)
 np.testing.assert_allclose(a['residual'],np.broadcast_to(res,(n,3)),atol=1e-13)
 np.testing.assert_allclose(a['body_velocity'],np.broadcast_to(body_v,(n,3)),atol=1e-13)
 assert r['onset_count']==1
 # Body-frame location of the injected world-X step is [0,0,2] mm;
 # constant physical displacement remains explicitly present in raw step.
 np.testing.assert_allclose(r['onsets'][0]['step_body_mm'],R.T@(world_v*.004+np.array([.002,0,0]))*1000,atol=1e-10)
 kk=np.ones(n-1,bool);kk[149]=False
 np.testing.assert_allclose(a['feedback'][kk],0,atol=1e-12)
 assert np.array_equal(a['state'][:,1],full)
 checks['nonunit_scale_bias_gravity_inverse']='pass'
 checks['full_tick_join_across_uint32_rollover']='pass'
 checks['constant_velocity_zero_feedback_except_known_step']='pass'
 checks['known_stationary_onset_and_body_step']='pass'
 # Quaternion signs encode the same physical rotation; alternating signs must
 # not change midpoint velocity, residuals or event statistics.
 flipped={k:v.copy() for k,v in arrays.items()}
 flipped['state'][::2,9:13]*=-1
 f,j=m.join_rows(flipped)
 np.testing.assert_allclose(f['feedback'],a['feedback'],atol=1e-12)
 np.testing.assert_allclose(f['residual'],a['residual'],atol=1e-12)
 checks['quaternion_sign_invariance']='pass'
 # A missing calibrated-IMU tick must not be substituted with a nearby sample.
 short=dict(arrays);short['imu']=short['imu'][np.arange(n)!=10]
 b,j=m.join_rows(short);assert j['unmatched_rows_excluded']==1 and full[10] not in b['state'][:,1]
 checks['unmatched_tick_excluded']='pass'
 # Explicit prefix handling, without inventing a CRC/EOF trailer.
 native=p/'native.rec.gz';native.write_bytes(native.read_bytes()[:-8])
 try:m.read_capture(p,'synthetic',False)
 except EOFError:checks['truncated_input_rejected_by_default']='pass'
 else:raise AssertionError('truncated gzip accepted without flag')
 _,meta=m.read_capture(p,'synthetic',True);assert not meta['complete_gzip'] and meta['gzip_eof_error']
 checks['explicit_prefix_preserves_eof_failure']='pass'
 # Different pose/IMU gap must reject the affected analysis window.
 missing=np.arange(n)[(np.arange(n)<100)|(np.arange(n)>108)]
 gaps={k:v[missing] for k,v in arrays.items()};b,_=m.join_rows(gaps)
 try:m.summarize(b,.1,1.1)
 except ValueError:checks['sensor_gap_over20ms_rejected']='pass'
 else:raise AssertionError('interpolated over a missing input interval')
print(json.dumps(checks,indent=2))
