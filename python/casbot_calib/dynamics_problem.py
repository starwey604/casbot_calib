"""Qualified output-torque to existing Pinocchio rigid-regression boundary."""
import json
from pathlib import Path
import numpy as np
from scipy.linalg import lstsq
from .dynamics_qualification import qualified_observations
from .vendor_identity import sha256


def fourier_projection(time_ns,values,origin_ns,period_s,harmonics,phase_points,derivatives=False,*,weights=None,phase_offset_s=0.,diagnostics=False):
    """Least squares on supplied event times, then analytic derivatives.

    Integer origins are subtracted before float conversion. Caller supplies one
    contiguous segment; this helper never concatenates across a missing interval.
    S3 callers supply qualified acquisition times; the H4 adapter explicitly uses
    unqualified callback availability. This numerical helper grants no timing qualification.
    """
    t=(np.asarray(time_ns,dtype=np.int64)-int(origin_ns))*1e-9
    if len(t)<4*(2*harmonics+1) or np.ptp(t)<1.8*period_s or np.any(np.diff(t)<=0):
        raise ValueError('At least two contiguous steady periods with distinct times required')
    if period_s<=0 or harmonics<1 or phase_points<=2*harmonics+1:raise ValueError('Invalid periodic fit settings')
    def design(times,order=0):
        columns=[np.ones(len(times)) if order==0 else np.zeros(len(times))]
        for k in range(1,harmonics+1):
            w=2*np.pi*k/period_s
            columns.extend([w**order*np.sin(w*times+order*np.pi/2),w**order*np.cos(w*times+order*np.pi/2)])
        return np.column_stack(columns)
    basis=design(t);target=np.asarray(values)
    if weights is not None:
        weights=np.asarray(weights,dtype=float)
        if weights.shape!=t.shape or not np.isfinite(weights).all() or np.any(weights<=0):raise ValueError('Positive Fourier row weights required')
        factor=np.sqrt(weights);basis=basis*factor[:,None];target=target*factor[:,None] if target.ndim==2 else target*factor
    if not np.isfinite(phase_offset_s):raise ValueError('Finite phase evaluation offset required')
    coef,_,rank,singular=lstsq(basis,target,cond=1e-10,lapack_driver='gelsd')
    if rank!=basis.shape[1]:raise ValueError('Rank deficient Fourier observation fit')
    phase=np.arange(phase_points)*period_s/phase_points
    projected=[design(phase+phase_offset_s,k)@coef for k in range(3 if derivatives else 1)]
    metadata=dict(rank=int(rank),singular_values=singular.tolist(),phase_s=phase)
    if diagnostics:
        residual=design(t)@coef-np.asarray(values)
        metadata['unmodelled_event_signal_rmse']=np.sqrt(np.average(residual**2,axis=0,weights=weights)).tolist()
    return projected,metadata


def _project(values,harmonics):
    spectrum=np.fft.rfft(values,axis=0);spectrum[harmonics+1:]=0
    return np.fft.irfft(spectrum,n=len(values),axis=0)


def periodic_observation_problem(meta,data,report):
    from .dual_model import nominal_dual_arm
    contract=meta['contract']
    experiment=contract.get('experiment',{})
    if experiment.get('kind')!='periodic':raise ValueError('A declared periodic experiment window is required')
    period=float(experiment['period_s']);harmonics=int(experiment['harmonics']);points=int(experiment.get('phase_points',1024))
    if harmonics/period>report['requested_band_hz']+1e-9:raise ValueError('Fit band exceeds the qualified frequency range')
    begin=int(experiment['begin_ns']);end=int(experiment['end_ns']);discard=int(experiment.get('discard_periods',2))
    if period<=0 or discard<0 or begin>=end:raise ValueError('Invalid periodic acquisition window')
    steady=begin+round(discard*period*1e9);time=data['aligned_position_time_ns'];signal=data['signal_time_ns']
    active=np.flatnonzero((time>=steady)&(time<end)&(signal>=steady)&(signal<end))
    if len(active)<2:raise ValueError('No complete steady measurement window')
    first,last=int(active[0]),int(active[-1])+1
    candidates=report['stage_data_eligibility']['inertia']['candidate_segments']
    if not any(row['begin']<=first and row['end']>=last for row in candidates):
        raise ValueError('Periodic window crosses an invalid/missing acquisition segment')
    if len(active)!=last-first:raise ValueError('Noncontiguous periodic row selection')
    engine,source=nominal_dual_arm()
    if contract['identity']['model_sha256']!=source['sha256'] or meta['joint_names']!=[j.name for j in engine.model.joints]:
        raise ValueError('The rigid regressor bridge requires the same declared nominal geometry and joint order')
    v=data['dq'][active];direction=np.sign(v);transition=np.exp(-(v/.002)**2)*direction
    raw_features=np.column_stack([v,direction-transition,transition])
    position,temporal=fourier_projection(time[active],np.column_stack([data['q'][active],raw_features]),begin,period,harmonics,points,True)
    torque,_=fourier_projection(signal[active],data['torque_nm'][active],begin,period,harmonics,points)
    q=position[0][:,:14];dq=position[1][:,:14];ddq=position[2][:,:14];features=position[0][:,14:]
    native=engine.regressor_batch(q,dq,ddq);regression=_project(native['regressor'],harmonics);rotor=_project(native['rotor_torque'],harmonics)
    auxiliary=np.zeros((points,14,42))
    for j in range(14):
        for group in range(3):auxiliary[:,j,group*14+j]=features[:,group*14+j]
    columns=regression.reshape(-1,140);norm=np.linalg.norm(columns,axis=0);safe=np.maximum(norm,1e-12)
    singular=np.linalg.svd(columns/safe,compute_uv=False);rank=int(np.count_nonzero(singular>singular[0]*1e-8))
    result=dict(schema_version='casbot_qualified_rigid_problem_v1',passed=True,kind='periodic_inertia_observations',
                model_source=source,
                samples=len(active),phase_points=points,period_s=period,harmonics=harmonics,fit_band_hz=harmonics/period,
                raw_position_time_known=True,raw_torque_time_known=True,torque_source=report['signal_origin'],
                mechanical_parameters_fitted=False,hardware_executable=False,regressor_backend='Pinocchio',
                rank=rank,columns=140,unique_body_parameters=rank==140,singular_values=singular.tolist(),
                target='output torque Nm minus separately accounted rotor torque; no current-shaped alias',
                friction_feature_order='measured raw velocity -> nonlinear features -> same declared Fourier band',
                solver_boundary='regression and target feed existing rigid_identification.fit_rigid; staged priors/validation remain S4 gates',
                temporal_fit_rank=temporal['rank'],limitations=['Only an observation equation, not an accepted calibration or motion permission',
                    'Static-friction feature is not a unique zero-speed torque law; S4 must use frozen operational bounds'])
    return dict(regression=regression,friction_basis=auxiliary,rotor=rotor,
                target=torque[0]-rotor,torque_nm=torque[0],q=q,dq=dq,ddq=ddq,phase_s=temporal['phase_s']),result


def build_periodic_problem(qualification,output):
    meta,data,report=qualified_observations(qualification,'inertia')
    arrays,result=periodic_observation_problem(meta,data,report)
    result['source_qualification_sha256']=sha256(Path(qualification)/'manifest.json')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'problem.npz',**arrays)
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'manifest.json').write_text(json.dumps({name:sha256(output/name) for name in ('problem.npz','report.json')},indent=2)+'\n')
    return result
