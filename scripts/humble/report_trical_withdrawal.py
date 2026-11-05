#!/usr/bin/env python3
"""Plot the exported 500 Hz local withdrawal and its orientation, without a plant."""
import argparse,csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from casbot_calib.dvt2_bundle import read,load_bundle
from casbot_calib.presentation import _font
from casbot_calib.data import write_json


def report(plan,bundle,output):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    plan=Path(plan);out=Path(output);out.mkdir(parents=True,exist_ok=False);engine,meta=load_bundle(bundle);protocol=read(plan/'protocol.json')
    if protocol['bundle_sha256']!=meta['bundle_sha256']:raise ValueError('Report/model mismatch')
    font=FontProperties(fname=_font(18).path);fig=plt.figure(figsize=(15,9),layout='constrained');rows=[]
    for i,(side,bid) in enumerate([('left','D'),('right','C'),('right','A')]):
        case=plan/(side+'_'+bid);m=read(case/'motion.json');labels=m['labels'];knots=np.array(m['knots_s']);states=np.array(m['states']);h=labels.index('contact_hold');spec=m['conditions']['withdrawal']
        indices=[k for k,label in enumerate(labels) if label.startswith('withdraw_')];start=knots[indices[0]];end=knots[indices[-1]+1]
        turn_index=next(k for k in indices if labels[k]=='withdraw_released');turn=knots[turn_index]
        with np.load(case/'reference.npz',allow_pickle=False) as data:
            keep=np.flatnonzero((data['time_ns']/1e9>=start-1e-10)&(data['time_ns']/1e9<=end+1e-10));keep=keep[::max(1,len(keep)//400)]
            q=data['q'][keep];t=data['time_ns'][keep]/1e9
        frame=side+'_trical_tcp';poses=engine.frame_batch(q,frame)['poses'];T0=engine.frame_pose(states[h,0],frame)
        xyz=(poses[:,:3,3]-T0[:3,3])*1000;angle=np.rad2deg(Rotation.from_matrix(np.einsum('ij,njk->nik',T0[:3,:3].T,poses[:,:3,:3])).magnitude());before=t<=turn+1e-10
        ax=fig.add_subplot(2,3,i+1,projection='3d')
        ax.plot(*xyz[before].T,color='#d78927',lw=3,label='接触区／朝向受限');ax.plot(*xyz[~before].T,color='#078b8e',lw=3,label='正间隙区／允许转向')
        release=engine.frame_pose(states[turn_index,0],frame)[:3,3];release_xyz=(release-T0[:3,3])*1000
        ax.scatter([0],[0],[0],color='#ba3636',s=45);ax.scatter(*release_xyz,color='#078b8e',s=40)
        ax.set_title(('左臂 ' if side=='left' else '右臂 ')+bid+' 球：实际参考TCP路径',fontproperties=font)
        for function,label in [(ax.set_xlabel,'X / mm'),(ax.set_ylabel,'Y / mm'),(ax.set_zlabel,'Z / mm')]:function(label)
        ax.view_init(elev=25,azim=-55);ax.legend(prop=font,loc='upper left',fontsize=9)
        ax=fig.add_subplot(2,3,i+4)
        ax.axvspan(0,turn-start,color='#f6e9d5');ax.plot(t-start,angle,color='#24649b',lw=2)
        ax.axvline(turn-start,color='#078b8e',ls='--',label='切换：三测头正间隙已证明')
        ax.plot([0,turn-start],np.rad2deg([spec['contact_orientation_tolerance_rad']]*2),color='#b78c45',ls=':',label='接触区1°包络上限')
        ax.set_xlabel('退出时间 / s',fontproperties=font);ax.set_ylabel('相对测量姿态转角 / °',fontproperties=font);ax.grid(alpha=.2);ax.legend(prop=FontProperties(fname=_font(14).path),loc='upper left')
        rows.append(dict(case=side+'_'+bid,withdrawal_duration_s=float(end-start),release_after_s=float(turn-start),release_TCP_distance_mm=float(np.linalg.norm(release_xyz)),
            maximum_sampled_orientation_change_deg=float(angle.max()),minimum_model_released_tip_clearance_mm=spec['minimum_released_tip_clearance_m']*1000))
    fig.suptitle('三表工具：先退出接触区，再允许转向\n当前工作状态CAD模型；三测头脱离后保持≥0.5 mm，实机自由测杆包络尚待量测',fontproperties=font,fontsize=19)
    fig.savefig(out/'release_then_turn_zh.png',dpi=135,bbox_inches='tight',pad_inches=.15);plt.close(fig)
    with (out/'withdrawal.csv').open('w',encoding='utf-8-sig',newline='') as f:w=csv.DictWriter(f,list(rows[0]));w.writeheader();w.writerows(rows)
    write_json(out/'summary.json',dict(cases=rows,source=str(plan),hardware_executable=False));return rows


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['plan','bundle','output']:p.add_argument('--'+name,required=True)
    a=p.parse_args();print(report(a.plan,a.bundle,a.output))
