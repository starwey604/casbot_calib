"""Direction/software mapping and frozen driver fits for staged recipes."""
import json
from pathlib import Path
import numpy as np
from .data import write_json
from .dual_preflight import mock_encoder_channels,_single_axis
from .identification import identify_current_response,current_response_residual,_current_response_problem
from .staged_collection import collect_trial,load_trial
from .staged_identification import recordings


def timing_transform(nominal,home):
    baseline=nominal.gravity(np.asarray(home));amplitude=np.tile([.15,.15,.1,.1,.03,.02,.015],2)
    def transform(t,commands,meta):
        probe=t<.3;validation=meta['conditions']['validation']
        pattern=np.array([1.,-1.,1.,1.,-1.,-1.] if validation else [1.,-1.,-1.,1.])
        width=.0173 if validation else .01;signal=pattern[(np.floor(t/width).astype(int))%len(pattern)]
        commands[probe,:,2]=baseline+signal[probe,None]*amplitude*(.8 if validation else 1.)
        commands[probe,:,3:]=0.
        return commands
    return transform


def check_mapping(root,config,output=None):
    root=Path(root);destination=Path(output) if output is not None else root;destination.mkdir(parents=True,exist_ok=True);rows=[]
    for name,meta,obs in recordings(root,'direction'):
        j=meta['conditions']['joint'];raw=mock_encoder_channels(obs);_,curve=load_trial(root,name)
        ticks=raw['raw_position'][:,j];direction=1 if j%2==0 else -1;zero=65536+512*j;unit=2*np.pi/131072
        decoded=direction*(ticks-zero)*unit;reference=curve(obs['sample_ns']*1e-9)[:,j]
        covariance=float((ticks-ticks.mean())@(reference-reference.mean()));inferred=1 if covariance>0 else -1
        amplitude=config['pose_amplitude_rad'][j];span=float(np.ptp(obs['q'][:,j]));error=float(max(abs(decoded-obs['q'][:,j])))
        end=obs['sample_ns']>obs['sample_ns'][-1]-200000000;return_error=float(abs(decoded[end].mean()-config['home_rad'][j]))
        path=root/'runs'/name/'raw_telemetry.npz'
        if path.exists():
            with np.load(path,allow_pickle=False) as old:
                if set(old.files)!=set(raw) or any(not np.array_equal(old[k],raw[k]) for k in raw):raise ValueError('Independent encoder raw recording changed')
        elif output is not None:raise ValueError('Independent encoder raw data missing during reanalysis')
        else:np.savez_compressed(path,**raw)
        rows.append(dict(joint=j,direction=direction,inferred_direction=inferred,software_zero_ticks=zero,unit_rad_per_tick=unit,
            decoded_error_rad=error,observed_span_rad=span,return_error_rad=return_error,
            passed=bool(error<1e-10 and inferred==direction and span>=1.5*amplitude and return_error<.05)))
    report=dict(passed=len(rows)==14 and all(r['passed'] for r in rows),axes=rows,synthetic_only=True,
        physical_zero='not independently observed; no geometric zero calibration',physical_direction='requires independent real physical observation',
        producer='independent mock tick direction/zero conventions; not manufacturer encoder specifications')
    write_json(destination/'direction_check.json',report);return report


def check_driver(root,config,output=None):
    root=Path(root);destination=Path(output) if output is not None else root;destination.mkdir(parents=True,exist_ok=True);cases={};gain=config['drive']['gain_nm_a'];fits=[]
    for name,meta,obs in recordings(root,'timing'):
        keep=obs['sample_ns']<300000000;selected={key:value[keep] for key,value in obs.items()}
        with np.load(root/'runs'/name/'commands.npz',allow_pickle=False) as data:
            active=data['send_ns']<300000000;times=data['send_ns'][active];commands=data['commands'][active]
        cases[name]=(selected,times,commands)
    train=cases['timing_train'];predictions={key:np.zeros_like(o['current']) for key,(o,_,_) in cases.items()}
    for j in range(14):
        single,values=_single_axis(train[0],train[2],j)
        fit=identify_current_response(single,train[1],values,gain,(0.,.005),(.0002,.006));validation={}
        for name,(obs,times,commands) in cases.items():
            channel,values=_single_axis(obs,commands,j);_,evaluate=_current_response_problem(channel,times,values,gain)
            predictions[name][:,j]=evaluate(fit['current_tau_s'],fit['command_delay_s'])
            if name!='timing_train':validation[name]=current_response_residual(channel,times,values,gain,fit['current_tau_s'],fit['command_delay_s'])
        fits.append(dict(joint=j,fit=fit,validation=validation,passed=bool(fit['success'] and fit['rmse_a']<=.0015 and all(r['rmse_a']<=.0015 for r in validation.values()))))
    for name,(obs,_,_) in cases.items():
        path=(destination/'predictions'/name) if output is not None else root/'runs'/name
        path.mkdir(parents=True,exist_ok=True);np.savez_compressed(path/'current_prediction.npz',sample_ns=obs['sample_ns'],current=predictions[name])
    report=dict(passed=len(cases)==3 and all(r['passed'] for r in fits),axes=fits,parameters_frozen_for_validation=True,
        training_hz=1e9/config['control_period_ns'],validation_hz=[1e9/config['control_period_ns'],250.],
        method='torque-only windows / known acquisition clock / exact ZOH plus first-order current response',
        frequency_scope='deterministic C++ virtual-time drive test; real ROS wall-clock rates measured separately',
        gain_status=config['drive']['gain_status'],mechanical_parameters_fitted=False)
    write_json(destination/'driver_timing.json',report);return report


def collect_preflight(root,plant,nominal,config,profile,*,resume=False):
    root=Path(root);plan=json.loads((root/'plan_report.json').read_text())
    for stage in ('direction','timing'):
        for name in plan['selected_trials']:
            metadata,_=load_trial(root,name)
            if metadata['stage']!=stage:continue
            transform=timing_transform(nominal,config['home_rad']) if stage=='timing' else None
            if not (resume and (root/'runs'/name/'metadata.json').exists()):collect_trial(root,name,plant,nominal,config,profile,command_transform=transform)
        report=check_mapping(root,config) if stage=='direction' else check_driver(root,config)
        if not report['passed']:raise RuntimeError(stage+' preflight failed; mechanical collection blocked')
    write_json(root/'preflight.json',dict(passed=True,checks=['software_mapping','torque_only_driver_timing'],
        hardware_executable=False,ros_frequency_qualification='separate ROS benchmark; no real network qualification inferred'))
