"""Synthetic-only K2 scoring. Never imported by the measurement controller."""
import csv
import json
from pathlib import Path
import numpy as np
from .kinematics import nominal_kinematics,ball_world
from .data import write_json


def evaluate(folder, output):
    folder=Path(folder);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    config=json.loads((folder/'profile.json').read_text());engine,_=nominal_kinematics(config)
    measurements=json.loads((folder/'measurements.json').read_text());events=json.loads((folder/'events.json').read_text())
    with np.load(folder/'evaluation/plant_truth.npz',allow_pickle=False) as data:truth={key:data[key] for key in data.files}
    initial={};current=None
    for event in events:
        if event['state']=='preflight':current=event['visit']
        if event['state']=='read_offset' and current not in initial:initial[current]=max(abs(np.array(event['offset_sensor_m'])))
    rows=[]
    for item in measurements:
        window=item['window'];selected=(truth['sample_ns']>=window['start_ns'])&(truth['sample_ns']<=window['end_ns'])
        positions=engine.frame_batch(truth['q'][selected],item['frame'])['poses'][:,:3,3]
        error=np.linalg.norm(positions-ball_world(config,item['ball_id']),axis=1)
        rows.append(dict(visit=item['visit'],initial_indicator_max_um=float(initial[item['visit']]*1e6),
            final_indicator_max_um=float(max(abs(np.array(item['offset_sensor_m'])))*1e6),
            evaluated_position_mean_um=float(np.mean(error)*1e6),evaluated_position_max_um=float(error.max()*1e6)))
    passed=bool(rows) and max(r['evaluated_position_max_um'] for r in rows)<=config['workflow']['centering_tolerance_m']*1e6+5
    result=dict(passed=passed,synthetic_truth_only=True,calibration_performed=False,measurements=rows,
        note='True plant TCP within accepted static window; not real-device accuracy or geometry identification')
    write_json(output/'report.json',result)
    with (output/'measurements.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]) if rows else ['visit']);writer.writeheader();writer.writerows(rows)
    return result


def plot_observations(folder, output):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt,font_manager
    from .presentation import _font
    from .trical_data import load_observations
    font=font_manager.FontProperties(fname=_font(12).path)
    folder=Path(folder);output=Path(output);arrays,_=load_observations(folder/'observations')
    with np.load(folder/'raw/row_decision_times.npz',allow_pickle=False) as data:times=data['time_ns']*1e-9
    measurements=json.loads((folder/'measurements.json').read_text());fig,axes=plt.subplots(len(measurements),1,figsize=(10,2.6*len(measurements)),squeeze=False)
    for axis,item in zip(axes[:,0],measurements,strict=True):
        selected=(arrays['tool_frame']==item['frame'])&(arrays['ball_id']==item['ball_id'])
        time=times[selected];values=arrays['indicator_readings_m'][selected].copy()*1e6
        values[arrays['indicator_status'][selected]!=0]=np.nan
        for j,name in enumerate(['X表','Y表','Z表']):axis.plot(time,values[:,j],label=name,lw=1)
        axis.axhline(30,color='gray',ls='--',lw=.8);axis.axhline(-30,color='gray',ls='--',lw=.8)
        axis.axvline(item['accepted_ns']*1e-9,color='green',ls=':',lw=1)
        axis.set_yscale('symlog',linthresh=30);axis.set_title(('左臂' if item['frame'].startswith('left') else '右臂')+' / '+item['ball_id']+'：读表、闭环修正、稳定采样',fontproperties=font)
        axis.set_ylabel('偏差（µm，对称对数）',fontproperties=font);axis.legend(prop=font,ncol=3);axis.grid(alpha=.2)
    axes[-1,0].set_xlabel('虚拟时间（秒）；无效接触区段断开显示',fontproperties=font)
    fig.suptitle('K2 合成测量闭环；不是几何参数标定或实机精度验收',fontproperties=font)
    fig.tight_layout();fig.savefig(output,dpi=140);plt.close(fig)
