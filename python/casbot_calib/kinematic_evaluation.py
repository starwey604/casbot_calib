"""Independent synthetic geometry scoring, deliberately outside the estimator."""
import json
from pathlib import Path
import numpy as np
from .kinematics import nominal_kinematics
from .kinematic_parameters import KinematicParameters
from .kinematic_identification import load_batch,residual_metrics
from .data import write_json


def evaluate(folder,output):
    root=Path(folder);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    public=json.loads((root/'train/profile.json').read_text());hidden=json.loads((root/'train/evaluation/mock.json').read_text())
    nominal,_=nominal_kinematics(public);parameters=KinematicParameters(nominal)
    geometry=np.array(hidden['geometry_scaled']);truth=geometry.copy();offset=np.array(hidden['encoder_offset_rad'])
    for i,definition in enumerate(parameters.definitions):
        if definition['zero_stage']:
            index=parameters.chart['sides'][definition['side']]['indices'][definition['row']]
            truth[i]+=offset[index]/definition['scale']
    physical=parameters.build(geometry);equivalent=parameters.build(truth)
    rng=np.random.default_rng(20261031);joint_names=[j.name for j in nominal.model.joints]
    with np.load(root/'identification/solution.npz',allow_pickle=False) as raw:estimated=raw['scaled_parameters'];jac=raw['training_jacobian_scaled']
    fitted=parameters.build(estimated);q=np.array(public['workflow']['home_rad'])+rng.uniform(-.35,.35,(64,14))
    for index,joint in enumerate(nominal.model.joints):q[:,index]=np.clip(q[:,index],joint.lower+.03,joint.upper-.03)
    original=[];corrected=[];equivalence=0.
    for frame in ('left_trical_tcp','right_trical_tcp'):
        actual=physical.frame_batch(q+offset,frame)['poses']
        alternate=equivalent.frame_batch(q,frame)['poses'];equivalence=max(equivalence,float(np.max(abs(actual-alternate))))
        original.append(nominal.frame_batch(q,frame)['poses'][:,:3,3]-actual[:,:3,3])
        corrected.append(fitted.frame_batch(q,frame)['poses'][:,:3,3]-actual[:,:3,3])
    if equivalence>1e-11:raise ValueError('Independent physical/encoder model is inconsistent with the calibration chart')
    _,s,vh=np.linalg.svd(jac,full_matrices=False);active=s>s[0]/public['k3']['maximum_condition']
    observable_error=float(np.linalg.norm(vh[active]@(estimated-truth)))
    result=dict(synthetic_truth_only=True,truth_used_for_fitting=False,physical_encoder_chart_equivalence_error=equivalence,
        off_design_fk_diagnostic=dict(nominal=residual_metrics(np.concatenate(original)),calibrated=residual_metrics(np.concatenate(corrected)),
            scope='Offline static FK queries only; no collision-certified motion, no metrology acceptance'),
        observable_scaled_parameter_error_norm=observable_error,
        individual_parameters_unique=False,joint_names=joint_names,
        note='Full raw parameter errors are gauge-dependent; independent artifact validation is reported by the estimator')
    write_json(output/'report.json',result);np.savez_compressed(output/'parameters_truth.npz',truth_scaled=truth,estimated_scaled=estimated)
    return result


def plot_result(folder,path):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt,font_manager
    from .presentation import _font
    font=font_manager.FontProperties(fname=_font(12).path);root=Path(folder)
    config=json.loads((root/'train/profile.json').read_text());nominal,_=nominal_kinematics(config);parameters=KinematicParameters(nominal)
    with np.load(root/'identification/solution.npz',allow_pickle=False) as raw:values=[np.zeros(parameters.size),raw['zero_stage_scaled_parameters'],raw['scaled_parameters']]
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for axis,split in zip(axes,('train','validation'),strict=True):
        data=load_batch(root/split,split)
        for x,label in zip(values,('名义模型','零位层','几何层'),strict=True):
            p,_=parameters.evaluate(x,data['q'],data['frames'],data['offsets']);axis.plot(np.arange(len(p))+1,np.linalg.norm(p-data['targets'],axis=1)*1e6,'o-',ms=3,label=label)
        axis.set_yscale('log');axis.set_title('训练批次' if split=='train' else '独立验证批次',fontproperties=font)
        axis.set_xlabel('完整静态测点编号',fontproperties=font);axis.set_ylabel('基准球位置约束残差（µm）',fontproperties=font);axis.grid(alpha=.2);axis.legend(prop=font)
    fig.suptitle('K3 合成几何辨识；位置约束验证不代表实机绝对或姿态精度',fontproperties=font)
    fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)
