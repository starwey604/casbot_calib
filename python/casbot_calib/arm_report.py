"""Reproducible R1 model checks; no simulated acquisition or calibration claims."""
from importlib.resources import files
import csv
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from . import _core
from .arm_model import load_arm_model,default_arm_config,planar_two_link_config
from .arm_reference import planar_terms
from .data import write_json


def _plot_probe(folder,probe):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(6,5));FigureCanvasAgg(figure);ax=figure.subplots()
    im=ax.imshow(probe.mass_matrix,cmap="coolwarm")
    ax.set(xlabel="Joint index",ylabel="Joint index",title="Pinocchio mass matrix / kg m²")
    figure.colorbar(im,ax=ax);figure.tight_layout();figure.savefig(folder/"mass_matrix.png",dpi=150)
    figure=Figure(figsize=(9,4));FigureCanvasAgg(figure);ax=figure.subplots()
    x=np.arange(7)
    for offset,name in enumerate(("inertia","velocity","gravity","friction")):
        ax.bar(x+(offset-1.5)*.18,getattr(probe,name),width=.18,label=name)
    ax.plot(x,probe.total,"ko",label="total")
    ax.set(xlabel="Joint index",ylabel="Torque / Nm",title="Declared model probe (not acquired telemetry)")
    ax.legend();ax.grid(axis="y",alpha=.2);figure.tight_layout();figure.savefig(folder/"torque_components.png",dpi=150)


def _pinocchio_probes(engine,q,dq,ddq):
    """Model-only checks, not parameter estimation or a claim of observability."""
    regression=engine.regressor_batch(q,dq,ddq)
    predicted=regression['regressor']@engine.rigid_parameters()+regression['rotor_torque']+regression['friction']
    inverse=engine.evaluate_batch(q,dq,ddq)['total']
    regression_error=float(np.max(np.abs(predicted-inverse)))
    design=regression['regressor'].reshape(-1,10*q.shape[1])
    scaling=np.linalg.norm(design,axis=0)
    singular=np.linalg.svd(design/np.where(scaling>0.,scaling,1.),compute_uv=False)
    rank=int(np.count_nonzero(singular>singular[0]*1e-8))
    derivative_absolute=0.;derivative_relative=0.;jacobian_error=0.;workspace_error=0.
    runtime=engine.workspace();h=1e-6
    for position,velocity,acceleration in zip(q[:8],dq[:8],ddq[:8],strict=True):
        torque=engine.inverse(position,velocity,acceleration)
        di=runtime.inverse_derivatives(position,velocity,acceleration)
        df=runtime.forward_derivatives(position,velocity,torque)
        def derivative(function,x):
            return np.column_stack([(function(x+h*axis)-function(x-h*axis))/(2*h) for axis in np.eye(len(x))])
        pairs=[(di.dtau_dq,derivative(lambda x:engine.inverse(x,velocity,acceleration),position)),
            (di.dtau_dv,derivative(lambda x:engine.inverse(position,x,acceleration),velocity)),
            (di.dtau_da,engine.mass_matrix(position)),
            (df.ddq_dq,derivative(lambda x:engine.forward(x,velocity,torque),position)),
            (df.ddq_dv,derivative(lambda x:engine.forward(position,x,torque),velocity)),
            (df.ddq_dtau,derivative(lambda x:engine.forward(position,velocity,x),torque))]
        for actual,expected in pairs:
            error=float(np.max(np.abs(actual-expected)))
            derivative_absolute=max(derivative_absolute,error)
            derivative_relative=max(derivative_relative,error/max(1.,float(np.max(np.abs(expected)))))
        jacobian=runtime.tool_jacobian(position);pose=engine.fk(position)[-1]
        translation=derivative(lambda x:engine.fk(x)[-1][:3,3],position)
        angular=[]
        for axis in np.eye(len(position)):
            dr=(engine.fk(position+h*axis)[-1][:3,:3]-engine.fk(position-h*axis)[-1][:3,:3])/(2*h)
            skew=dr@pose[:3,:3].T
            angular.append([skew[2,1],skew[0,2],skew[1,0]])
        jacobian_error=max(jacobian_error,float(np.max(np.abs(jacobian-np.vstack([translation,np.array(angular).T])))))
        # Alternating algorithms must not reuse stale Data fields.
        workspace_error=max(workspace_error,float(np.max(np.abs(runtime.components(position,velocity,acceleration).total-torque))))
    metrics=dict(maximum_regressor_torque_error_nm=regression_error,
        maximum_derivative_absolute_error=derivative_absolute,maximum_derivative_normalized_error=derivative_relative,
        maximum_tcp_jacobian_error=jacobian_error,maximum_workspace_component_error_nm=workspace_error,
        rigid_parameter_count=len(scaling),probe_regressor_rank=rank,probe_regressor_column_scales=scaling.tolist(),
        probe_regressor_singular_values=singular.tolist(),probe_regressor_rank_relative_threshold=1e-8,
        derivative_normalization='max absolute block error / max(1, max absolute finite-difference block); mixed derivative units, not a physical accuracy metric')
    checks=dict(pinocchio_regressor_identity=regression_error<1e-10,
        analytic_dynamics_derivatives=derivative_relative<1e-6,
        tcp_jacobian_world_convention=jacobian_error<1e-8,reusable_workspace_consistency=workspace_error<1e-10)
    contract=dict(schema_version=1,backend=engine.backend_version(),model_version=engine.model.model_version,
        joint_names=[j.name for j in engine.model.joints],parameter_names=regression['parameter_names'],
        units_per_joint=['kg']+['kg m']*3+['kg m^2']*6,
        equation='tau = Y @ pi + rotor_inertia * ddq + friction(dq)',
        parameter_convention='mass, first moments hx/hy/hz, then Ixx/Ixy/Iyy/Ixz/Iyz/Izz about joint origin; not COM inertia',
        payload='merged once into last moving body in joint coordinates',
        exclusions=['armature is separate from pi','friction is separate from pi','drive current gain is separate'],
        rank_status='probe matrix only; not a calibration or full individual-link identifiability claim',
        tcp_jacobian='LOCAL_WORLD_ALIGNED at TCP origin; rows world linear then world angular',
        derivatives='state/input derivatives of full inverse/forward including explicit smooth friction; not derivatives with respect to inertial parameters')
    return metrics,checks,regression,contract


def run_arm_checks(folder,*,render=False):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    config=default_arm_config();engine=_core.ArmDynamics(load_arm_model(config))
    planar=_core.ArmDynamics(load_arm_model(planar_two_link_config()))
    rng=np.random.default_rng(20261020);errors={name:0. for name in ('mass_matrix','gravity','velocity','inverse','forward')}
    for _ in range(64):
        q,v,a=rng.uniform(-1.,1.,size=(3,2));expected=planar_terms(q,v,a)
        actual=dict(mass_matrix=planar.mass_matrix(q),gravity=planar.gravity(q),velocity=planar.velocity_bias(q,v),
                    inverse=planar.inverse(q,v,a),forward=planar.forward(q,v,expected['total']))
        for name,value in actual.items():
            target=expected['total'] if name=='inverse' else a if name=='forward' else expected[name]
            errors[name]=max(errors[name],float(np.max(np.abs(value-target))))
    reference=json.loads(files('casbot_calib').joinpath('assets/casbot_right_arm_fk_reference.json').read_text(encoding='utf-8'))
    source_matches=reference['source_sha256']==config['provenance']['source_sha256'] and reference['joint_names']==config['joint_names']
    fk_error=max(float(np.max(np.abs(np.array(engine.fk(np.array(x['q'])))-np.array(x['world_poses'])))) for x in reference['samples'])
    limits=np.array([[j['limits']['lower'],j['limits']['upper']] for j in config['joints']])
    q=rng.uniform(limits[:,0]*.7,limits[:,1]*.7,size=(96,7))
    dq=rng.uniform(-.8,.8,size=(96,7));ddq=rng.uniform(-1.,1.,size=(96,7))
    start=perf_counter();terms=engine.evaluate_batch(q,dq,ddq);elapsed=perf_counter()-start
    eigen=np.linalg.eigvalsh(terms['mass_matrix'])
    symmetry=float(np.max(np.abs(terms['mass_matrix']-terms['mass_matrix'].transpose(0,2,1))))
    sum_error=float(np.max(np.abs(terms['total']-sum(terms[k] for k in ('inertia','velocity','gravity','friction')))))
    inverse_error=max(float(np.max(np.abs(engine.inverse(qq,vv,aa)-total))) for qq,vv,aa,total in zip(q,dq,ddq,terms['total'],strict=True))
    zero=np.zeros(7)
    zero_velocity=max(float(np.max(np.abs(engine.velocity_bias(qq,zero)))) for qq in q)
    static_error=max(float(np.max(np.abs(engine.inverse(qq,zero,zero)-engine.gravity(qq)))) for qq in q)
    off_diagonal=terms['mass_matrix'].copy();off_diagonal[:,np.arange(7),np.arange(7)]=0.
    coupling=float(np.max(np.abs(off_diagonal)))
    velocity_coupling=float(np.max(np.abs(terms['velocity'])))
    # Differential work-energy identity checked against a direction in state space.
    work_error=0.
    for qq,vv,aa in zip(q[:12],dq[:12],ddq[:12],strict=True):
        h=1e-6;e1=engine.energy(qq+h*vv,vv+h*aa);e0=engine.energy(qq-h*vv,vv-h*aa)
        de=(e1.kinetic+e1.potential-e0.kinetic-e0.potential)/(2*h)
        power=float((engine.inverse(qq,vv,aa)-engine.friction(vv))@vv)
        work_error=max(work_error,abs(de-power))
    checks=dict(analytic_two_link=max(errors.values())<1e-8,reference_source_matches=source_matches,
        source_urdf_fk=fk_error<1e-12,mass_positive_definite=bool(eigen.min()>0.),mass_symmetric=symmetry<1e-12,
        static_gravity=static_error<1e-12,zero_speed_bias=zero_velocity<1e-12,
        components_equal_rnea=inverse_error<1e-10,component_sum=sum_error<1e-12,
        nonzero_mass_coupling=coupling>1e-3,nonzero_velocity_coupling=velocity_coupling>1e-3,
        differential_work_energy=work_error<1e-7)
    metrics=dict(planar_max_absolute_errors=errors,fk_max_absolute_error=fk_error,
        minimum_mass_eigenvalue=float(eigen.min()),maximum_mass_condition=float(np.max(eigen[:,-1]/eigen[:,0])),
        maximum_mass_symmetry_error=symmetry,maximum_inverse_component_error=inverse_error,
        maximum_off_diagonal_mass=coupling,maximum_velocity_bias_nm=velocity_coupling,
        maximum_work_energy_error_w=work_error,
        batch_samples=len(q),batch_wall_s=elapsed,batch_us_per_probe=elapsed/len(q)*1e6)
    extra_metrics,extra_checks,regression,contract=_pinocchio_probes(engine,q,dq,ddq)
    metrics.update(extra_metrics);checks.update(extra_checks)
    np.savez_compressed(folder/'model_probes.npz',q=q,dq=dq,ddq=ddq,**terms,
        torque_regressor=regression['regressor'],rotor_torque=regression['rotor_torque'])
    write_json(folder/'regressor_contract.json',contract)
    write_json(folder/'arm_model.json',config)
    write_json(folder/'fk_reference.json',reference)
    with (folder/'reference_pose_components.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(['joint','q_rad','inertia_Nm','velocity_Nm','gravity_Nm','friction_Nm','total_Nm'])
        probe_q=np.array(config['reference_pose']);probe_v=np.linspace(-.5,.5,7);probe_a=np.linspace(.6,-.6,7)
        probe=engine.components(probe_q,probe_v,probe_a)
        for j,name in enumerate(config['joint_names']):
            writer.writerow([name,probe_q[j],*(getattr(probe,key)[j] for key in ('inertia','velocity','gravity','friction','total'))])
    _plot_probe(folder,probe)
    geometry=None
    if render:
        from .arm_viewer import render_arm_snapshot
        try:
            geometry=render_arm_snapshot(config,engine,folder)
        except Exception as exc:
            geometry=dict(passed=False,error=f'{type(exc).__name__}: {exc}')
        checks['geometry_only_viewer_fk']=geometry['passed']
    report=dict(schema_version=1,stage='R1',passed=all(checks.values()),checks=checks,metrics=metrics,
        backend=engine.backend_version(),robot_model_id=config['robot_model_id'],model_version=config['model_version'],
        synthetic_only=True,scope='model evaluation and independent correctness probes; no collected telemetry, no calibration',
        provenance=config['provenance'],geometry=geometry,
        runtime_policy='immutable model; one reusable Pinocchio Data per batch/workspace; stateless scalar convenience calls use temporary workspace; Python workspace retains GIL; no hard-real-time claim',
        limitations=['R2 acquisition not implemented','R3 calibration not implemented','hardware inertias/friction not verified'])
    write_json(folder/'report.json',report)
    content='# R1：统一整臂模型检查\n\n'
    content+=f"后端：{report['backend']}；七轴固定基座右臂；全部数值为合成模型探针，不是实机标定结果。\n\n"
    content+='| 检查 | 结果 |\n| --- | --- |\n'+''.join(f'| {name} | {"通过" if passed else "失败"} |\n' for name,passed in checks.items())
    content+='\n- [数值报告](report.json)\n- [模型与来源](arm_model.json)\n- [分项力矩](reference_pose_components.csv)\n- [回归矩阵与参数契约](regressor_contract.json)\n'
    content+=f"\n回归探针矩阵秩：{metrics['probe_regressor_rank']} / {metrics['rigid_parameter_count']}。这不是逐连杆参数已可辨识或标定完成的结论。\n"
    if render:content+='- [整臂几何快照](arm_geometry.png)\n- [几何一致性](geometry_check.json)\n'
    (folder/'report.md').write_text(content,encoding='utf-8')
    evidence={str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
    write_json(folder/'evidence_sha256.json',evidence)
    return report
