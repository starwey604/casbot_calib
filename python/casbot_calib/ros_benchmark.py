"""ROS stream experiment orchestration and analysis. Python never sends per-cycle commands."""
from collections import Counter
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import numpy as np
from .data import write_json
from .dual_model import nominal_dual_arm
from .identification import identify_current_response, current_response_residual


def table(path):
    rows=np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding='utf-8',ndmin=1)
    if not len(rows):raise ValueError(f'Empty stream log: {path}')
    return rows


def vectors(rows,prefix):return np.column_stack([rows[f'{prefix}{j}'] for j in range(14)])


def distribution(values):
    values=np.asarray(values,dtype=float)
    if not len(values):return None
    return dict(count=len(values),minimum=float(values.min()),p50=float(np.quantile(values,.5)),
        p95=float(np.quantile(values,.95)),p99=float(np.quantile(values,.99)),maximum=float(values.max()),std=float(values.std()))


def rate(times):
    times=np.asarray(times,dtype=np.int64)
    if len(times)<2:return dict(hz=None,interval_ms=None)
    delta=np.diff(times)
    return dict(hz=float((len(times)-1)*1e9/(times[-1]-times[0])) if times[-1]>times[0] else None,interval_ms=distribution(delta*1e-6),nonpositive_intervals=int(np.count_nonzero(delta<=0)))


def latency(later,earlier,shared_clock):
    # No cross-clock arithmetic, including when the numeric values look plausible.
    if not shared_clock:return dict(observable=False,reason='clock_domains_not_aligned',milliseconds=None)
    return dict(observable=True,milliseconds=distribution((np.asarray(later,dtype=np.int64)-np.asarray(earlier,dtype=np.int64))*1e-6))


def fingerprint(folder):
    names=['case.json','clocks.json','plan.csv','client_sent.csv','client_observations.csv','client_events.csv',
           'host_events.csv','host_writes.csv','host_samples.csv']
    return {name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in names}


def case_metrics(folder):
    folder=Path(folder);config=json.loads((folder/'case.json').read_text());clocks=json.loads((folder/'clocks.json').read_text())
    tx=table(folder/'client_sent.csv');states=table(folder/'client_observations.csv');events=table(folder/'client_events.csv')
    host=table(folder/'host_events.csv');writes=table(folder/'host_writes.csv');samples=table(folder/'host_samples.csv')
    sent=tx[tx['publish_ns']>0];primary=tx[~np.isin(tx['injection'],['duplicate','old','omitted'])]
    received=host[host['kind']==0];accepted=host[host['kind']==4];adopted=host[host['kind']==1]
    rejected=host[host['kind']==2];watchdog=host[host['kind']==3]
    shared=clocks['client']==clocks['host'] and bool(np.all(states['clock_match']))
    begin=int(primary['send_ns'].min());end=int(primary['send_ns'].max())
    active=(states['receive_ns']>=begin)&(states['receive_ns']<=end)
    callback_events=events[(events['kind']==0)&(events['source_send_ns']>=begin)&(events['source_clock_match']==1)]
    missing=sorted(set(map(int,sent['sequence']))-set(map(int,received['command_sequence'])))
    published=states['publish_sequence'].astype(np.int64)
    result=dict(name=config['name'],scope=config['scope'],shared_clock=shared,
        source_age_expiry_supported=shared,
        command_publish=rate(primary['send_ns']),command_receive=rate(received['receive_ns']),command_adoption=rate(adopted['decision_ns']),
        host_control=rate(writes['write_ns']),device_sampling=rate(samples['sample_ns']),feedback_publish=rate(states['publish_ns'][active]),feedback_receive=rate(states['receive_ns'][active]),
        command_transport=latency(received['receive_ns'],received['source_send_ns'],shared),
        command_receive_to_decision=latency(accepted['decision_ns'],accepted['receive_ns'],True),
        command_age_at_adoption=latency(adopted['decision_ns'],adopted['source_send_ns'],shared),
        feedback_transport=latency(states['receive_ns'],states['publish_ns'],shared),
        acquisition_to_driver_read=latency(states['driver_receive_ns'],states['sample_ns'],bool(np.all(states['sample_time_known']) and np.all(states['sample_clock_match']))),
        callback_round_trip=latency(callback_events['client_receive_ns'],callback_events['source_send_ns'],True),
        callback_round_trip_definition='client send -> host callback event -> client receive; never divided by two',
        counts=dict(planned=len(table(folder/'plan.csv')),primary_published=len(primary),intentional_omissions=int(np.count_nonzero(tx['injection']=='omitted')),
            publication_copies=len(sent),received=len(received),accepted=len(accepted),adopted=len(adopted),rejected=len(rejected),
            accepted_superseded=len(accepted)-len(adopted),transport_missing_sequences=missing,
            client_schedule_skips=max(0,len(table(folder/'plan.csv'))-len(primary)-int(np.count_nonzero(tx['injection']=='omitted'))),
            received_duplicate_or_reordered=int(np.count_nonzero(np.diff(received['command_sequence'].astype(np.int64))<=0)),
            feedback_publish_gaps=int(np.maximum(np.diff(published)-1,0).sum()),
            repeated_acquisition_frames=int(np.count_nonzero(np.diff(states['sequence'])==0)),
            diagnostic_event_gaps=int(np.maximum(np.diff(events['event_sequence'].astype(np.int64))-1,0).sum())),
        rejection_reasons=dict(Counter(map(str,rejected['reason']))),watchdog_after_stream=bool(np.any(watchdog['decision_ns']>end)),
        maximum_command_age_ms=float((adopted['decision_ns']-adopted['source_send_ns']).max()*1e-6) if shared and len(adopted) else None,
        acquisition_gaps_note='Acquisition sequence skips at a lower publication rate are not classified as DDS packet loss',
        delivery_gaps_note='Unobserved command sequences include DDS history eviction/executor load/transport effects; they are not a packet-loss measurement',
        input_sha256=fingerprint(folder))
    delivered=len(set(map(int,received['command_sequence'])))
    delivery_ratio=delivered/max(1,len(set(map(int,sent['sequence']))))
    result['frequency_qualification']=dict(qualified=bool(not config.get('faults') and delivery_ratio>=.99 and
        result['command_adoption']['hz'] is not None and result['command_adoption']['hz']>=config['command_hz']*.9 and
        result['feedback_receive']['hz'] is not None and result['feedback_receive']['hz']>=config['feedback_hz']*.9),
        delivery_ratio=delivery_ratio,minimum_delivery_ratio=.99,minimum_rate_ratio=.9,
        scope='This local instrumented profile only; synthetic screening, not a hardware or hard-real-time guarantee')
    raw=dict(sequence=states['sequence'].astype(np.int64),sample_ns=states['sample_ns'].astype(np.int64),
        receive_ns=states['receive_ns'].astype(np.int64),publish_ns=states['publish_ns'].astype(np.int64),
        sample_time_known=states['sample_time_known'].astype(bool),sample_clock_matches_host=states['sample_clock_match'].astype(bool),
        publish_sequence=states['publish_sequence'].astype(np.int64),q=vectors(states,'q'),dq=vectors(states,'dq'),current=vectors(states,'current'),status=vectors(states,'status').astype(np.uint32))
    return result,raw


def driver_data(folder):
    """Inputs and samples use the host clock, even if client/host clocks differ.

    Actual host write history is a required capability. No evaluation parameters.
    Integer origin is removed before conversion to seconds by the existing solver.
    """
    folder=Path(folder);clocks=json.loads((folder/'clocks.json').read_text())
    if not clocks['sample_time_known'] or clocks.get('sample')!=clocks['host']:raise ValueError('Driver identification needs acquisition timestamps aligned to the host write clock')
    states=table(folder/'client_observations.csv');writes=table(folder/'host_writes.csv');samples=table(folder/'host_samples.csv')
    states=states[(states['sample_time_known']==1)&(states['sample_clock_match']==1)]
    _,indices=np.unique(states['sample_ns'],return_index=True);states=states[np.sort(indices)]
    order=np.argsort(states['sample_ns']);states=states[order]
    origin=min(int(samples['sample_ns'].min()),int(writes['write_ns'].min()))
    return dict(sample_ns=states['sample_ns'].astype(np.int64)-origin,current=vectors(states,'current'),status=vectors(states,'status').astype(int)),writes['write_ns'].astype(np.int64)-origin,vectors(writes,'effort')


def analyse_suite(folder,output):
    folder=Path(folder);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    session=json.loads((folder/'session.json').read_text());names=session['joint_names'];results=[];snapshots={}
    write_json(output/'observation_schema.json',dict(schema_version='ros_stream_v1',joint_names=names,
        units=dict(q='rad',dq='rad/s',current='A',time='int64 monotonic ns'),
        clock_policy='Each case retains separate client/host clock IDs; raw times are not rewritten',
        estimated_effort_source='current times synthetic declared output gain; not an independent sensor'))
    for config in session['cases']:
        path=folder/config['name'];metrics,raw=case_metrics(path);results.append(metrics);snapshots[config['name']]=metrics['input_sha256']
        np.savez_compressed(output/(config['name']+'_observations.npz'),**raw)
    training=[c for c in session['cases'] if c.get('train')]
    if len(training)!=1:raise ValueError('Exactly one independent driver training run required')
    train,send,torque=driver_data(folder/training[0]['name']);fits=[]
    for j,name in enumerate(names):
        observed=dict(current_sample_ns=train['sample_ns'],current=train['current'][:,j],status=train['status'][:,j])
        commands=np.zeros((len(send),6));commands[:,3]=torque[:,j]
        fit=identify_current_response(observed,send,commands,session['gain_nm_a'],(0.,.005),(.0002,.006))
        fit['gain_status']=session['gain_status'];fits.append(dict(joint=name,**fit))
    for config,result in zip(session['cases'],results,strict=True):
        observed,send,torque=driver_data(folder/config['name']);errors=[]
        for j,fit in enumerate(fits):
            commands=np.zeros((len(send),6));commands[:,3]=torque[:,j]
            channel=dict(current_sample_ns=observed['sample_ns'],current=observed['current'][:,j],status=observed['status'][:,j])
            error=current_response_residual(channel,send,commands,session['gain_nm_a'],fit['current_tau_s'],fit['command_delay_s'])
            errors.append(error['rmse_a'])
        result['frozen_driver_current_rmse_a']=errors;result['maximum_current_rmse_a']=max(errors)
        result['driver_passed']=max(errors)<=session['maximum_current_rmse_a']
        result['parameters_refitted']=False
        result['checks']=dict(frozen_driver=result['driver_passed'],watchdog=result['watchdog_after_stream'],
            fault_rejections=(not config.get('faults') or all(result['rejection_reasons'].get(reason,0)>0 for reason in ['expired','duplicate_or_old','nonfinite_or_synthetic_limit','joint_order','mode','experiment_or_model'])))
        result['passed']=all(result['checks'].values())
        result['passed_scope']='Protocol/watchdog and frozen current prediction; frequency qualification is reported separately'
        write_json(output/(config['name']+'.json'),result)
        if result['input_sha256']!=fingerprint(folder/config['name']):raise ValueError('Raw data changed during analysis')
    report=dict(passed=all(r['passed'] for r in results) and all(f['success'] for f in fits),schema_version=1,
        scope=session['scope'],hardware_tested=False,physical_network_tested=False,
        passed_scope='Implementation/identification checks, not qualification of every requested frequency',
        driver_training=training[0]['name'],driver_parameters=fits,cases=results,
        clock_policy='Cross-host one-way latency requires matching declared clock domains; RTT is not halved',
        qualified_local_profiles=[r['name'] for r in results if r['frequency_qualification']['qualified']],
        operating_frequency_recommendation=None,qualification_note='Single captured batch per profile; repeats and randomized profile order are required before selecting an operating frequency',
        limitations=['C++ wall-timer endpoint, not a ros2_control hardware/controller plugin',
          'Zero-gravity nominal 14-axis synthetic plant; no mechanical parameter calibration in this benchmark',
          'Inline CSV and ROS diagnostics are part of the measured load; no hard-real-time claim',
          'No collision certification; external real-robot stop policy and interfaces remain unconfirmed'],
        source_sha256=snapshots)
    write_json(output/'report.json',report)
    with (output/'summary.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['case','command_actual_hz','feedback_actual_hz','host_control_hz','adoption_age_p99_ms','maximum_current_rmse_a','passed'])
        for r in results:
            age=r['command_age_at_adoption']['milliseconds'];writer.writerow([r['name'],r['command_publish']['hz'],r['feedback_receive']['hz'],r['host_control']['hz'],age['p99'] if age else '',r['maximum_current_rmse_a'],r['passed']])
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig=Figure(figsize=(12,7));FigureCanvasAgg(fig);axes=fig.subplots(2,1)
    x=np.arange(len(results));axes[0].plot(x,[r['command_publish']['hz'] for r in results],'o-',label='command publish')
    axes[0].plot(x,[r['feedback_receive']['hz'] for r in results],'o-',label='feedback receive');axes[0].set_ylabel('Measured Hz');axes[0].legend()
    axes[1].plot(x,[1000*r['maximum_current_rmse_a'] for r in results],'o-',label='frozen driver current prediction')
    axes[1].axhline(session['maximum_current_rmse_a']*1000,color='r',linestyle='--',label='synthetic acceptance');axes[1].set_ylabel('Maximum RMSE / mA');axes[1].legend()
    for axis in axes:axis.set_xticks(x,[r['name'] for r in results],rotation=25,ha='right');axis.grid(alpha=.2)
    fig.tight_layout();fig.savefig(output/'frequency_and_prediction.png',dpi=140)
    (output/'README.md').write_text('# M2：ROS通信与频率验证\n\n'+f"结果：{'通过' if report['passed'] else '失败'}；范围：{session['scope']}。\n\n"+
        '[完整报告](report.json) · [频率/电流预测图](frequency_and_prediction.png) · [指标CSV](summary.csv)\n\n'+
        'C++发端通过ROS持续发送，C++主机端独立控制循环推进统一Pinocchio/Boost Mock。'
        '电流模型仅在训练批次拟合，其他命令/反馈频率、QoS及负载批次冻结预测。'
        '单向延迟要求已声明同一时钟；RTT不除以二。当前是本机回环，不代表物理以太网或实机性能。'
        '本端点尚非甲方ros2_control适配器，不含新机械参数辨识或碰撞门禁。\n',encoding='utf-8')
    return report


def stop(process):
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:process.kill();process.wait()


def run_suite(folder,config):
    from ament_index_python.packages import get_package_prefix
    from rclpy.utilities import get_rmw_implementation_identifier
    if config.get('schema_version')!=1 or config.get('scope')!='localhost_ros_mock':raise ValueError('Only an explicit local ROS Mock benchmark may be automatically launched')
    if config['gain_nm_a']!=2.0 or config['gain_status']!='synthetic_declared':raise ValueError('The M2 synthetic host declares 2 Nm/A; change the host capability profile before changing its scale')
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    model,source=nominal_dual_arm();names=[j.name for j in model.model.joints]
    from importlib.resources import files
    model_path=folder/'nominal.urdf';model_path.write_bytes(files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf').read_bytes())
    session={**config,'joint_names':names,'model_id':'casbot_20251015_nominal:'+source['sha256'][:12],
             'rmw':get_rmw_implementation_identifier(),
             'instrumentation':'inline C++ CSV; ROS event diagnostics', 'zero_gravity':True}
    write_json(folder/'session.json',session)
    from casbot_calib_interfaces.msg import JointCommand
    from rclpy.serialization import serialize_message
    probe=JointCommand();probe.joint_names=names;probe.model_id=session['model_id'];probe.experiment_id='calib_'+'0'*16
    probe.clock_id='linux_monotonic:'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()+':'+os.readlink('/proc/self/ns/time')
    session['command_cdr_payload_bytes']=len(serialize_message(probe));write_json(folder/'session.json',session)
    prefix=Path(get_package_prefix('casbot_calib_transport'))/'lib/casbot_calib_transport'
    environment=dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(10+uuid.uuid4().int%80))
    for case in config['cases']:
        name=case['name'];path=folder/name;path.mkdir();identifier='calib_'+uuid.uuid4().hex[:16];namespace='/casbot_calib_bench/'+identifier
        settings={**config,**case,'namespace':namespace,'experiment_id':identifier,'model_id':session['model_id'],
                  'rmw':session['rmw'],'ros_domain_id':environment['ROS_DOMAIN_ID'],'zero_gravity':True}
        settings.pop('cases');write_json(path/'case.json',settings)
        period=1_000_000_000//case['command_hz'];times=np.arange(0,round(config['duration_s']*1e9),period,dtype=np.int64)
        pattern=np.array([1.,-1.,-1.,1.]) if case.get('train') else np.array([1.,-1.,1.,1.,-1.,-1.])
        width=10_000_000 if case.get('train') else 17_300_000
        amplitude=np.tile([.03,.03,.02,.02,.01,.01,.01],2)
        torque=pattern[(times//width)%len(pattern),None]*amplitude
        q=np.zeros_like(torque);dq=q.copy();kp=q.copy();kd=q.copy()
        if case.get('mode',0)==1:
            q[:]=.02*np.sin(2*np.pi*times[:,None]*1e-9);dq[:]=.04*np.pi*np.cos(2*np.pi*times[:,None]*1e-9);kp[:]=.005;kd[:]=.001
        plan=np.column_stack([times,np.full(len(times),case.get('mode',0)),q,dq,torque,kp,kd])
        np.savetxt(path/'plan.csv',plan,delimiter=',',header='relative_ns,mode'+''.join(','+prefix+str(j) for prefix in ['q','v','tau','kp','kd'] for j in range(14)),comments='',fmt=['%d','%d']+['%.17g']*70)
        evaluation=path/'evaluation';evaluation.mkdir();write_json(evaluation/'plant.json',dict(current_tau_s=.001,drive_delay_ns=1_350_000,feedback_delay_ns=700_000,model='nominal zero gravity'))
        common=['--ros-args','-r','__ns:='+namespace,'-p','experiment_id:='+identifier,'-p','model_id:='+session['model_id'],'-p','model_path:='+str(model_path),'-p','output_dir:='+str(path),'-p','reliable:='+str(case.get('reliable',True)).lower()]
        host_args=[str(prefix/'mock_host'),*common,'-p',f"control_hz:={case.get('control_hz',config['control_hz'])}",'-p',f"feedback_hz:={case['feedback_hz']}",'-p',f"inject_every:={case.get('inject_every',0)}",'-p',f"inject_delay_ns:={case.get('inject_delay_ns',0)}"]
        client_args=[str(prefix/'stream_client'),*common,'-p','plan_file:='+str(path/'plan.csv'),'-p',f"command_hz:={case['command_hz']}",'-p',f"tail_s:={config['tail_s']}",'-p',f"valid_for_ns:={case.get('valid_for_ns',100_000_000)}",'-p','faults:='+str(case.get('faults',False)).lower()]
        processes=[];logs=[]
        try:
            for number in range(case.get('load_workers',0)):
                processes.append(subprocess.Popen([sys.executable,'-c','while True: sum(i*i for i in range(10000))'],env=environment,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
            host_log=(path/'host.log').open('w');logs.append(host_log);host=subprocess.Popen(host_args,env=environment,stdout=host_log,stderr=subprocess.STDOUT);processes.append(host)
            if case.get('record'):
                log=(path/'rosbag.log').open('w');logs.append(log);processes.append(subprocess.Popen(['ros2','bag','record','-o',str(path/'bag'),namespace+'/commands',namespace+'/observations',namespace+'/events'],env=environment,stdout=log,stderr=subprocess.STDOUT))
            client_log=(path/'client.log').open('w');logs.append(client_log)
            print('ROS stream: '+name,flush=True)
            client=subprocess.Popen(client_args,env=environment,stdout=client_log,stderr=subprocess.STDOUT);processes.append(client)
            code=client.wait(timeout=config['duration_s']+18)
            if code or host.poll() is not None:raise RuntimeError(f'{name}: ROS endpoint failed; inspect host.log/client.log')
            time.sleep(.1)
        finally:
            for process in reversed(processes):stop(process)
            for log in logs:log.close()
        write_json(path/'capture.json',dict(host_returncode=host.returncode,client_returncode=code,host_command=host_args,client_command=client_args,
            load_workers=case.get('load_workers',0),record_requested=case.get('record',False),bag_created=(path/'bag/metadata.yaml').exists()))
        if case.get('record') and not (path/'bag/metadata.yaml').exists():raise RuntimeError('Requested rosbag recording did not complete')
    return analyse_suite(folder,folder/'analysis')
