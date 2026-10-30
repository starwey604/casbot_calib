"""Versioned, collision-gated experiment recipes, without a device connection.

Septic Hermite segments are represented in SciPy's Bernstein basis. Derivative
coefficient convex hulls give conservative continuous-time velocity bounds;
sampled maxima are never used as interval certificates.
"""
from dataclasses import dataclass, field
import numpy as np
from scipy.interpolate import BPoly,PPoly
from math import factorial
from .collision import certify_reference


@dataclass
class Motion:
    name: str
    stage: str
    home: np.ndarray
    conditions: dict = field(default_factory=dict)
    knots: list = field(default_factory=lambda: [0.])
    states: list = field(default_factory=list)
    labels: list = field(default_factory=list)

    def __post_init__(self):
        self.home=np.asarray(self.home,dtype=float)
        self.states=[np.stack([self.home,*[np.zeros_like(self.home) for _ in range(3)]])]

    def add(self,duration,q,velocity=None,label='transition'):
        if duration<=0:raise ValueError('Segment duration must be positive')
        q=np.asarray(q,dtype=float);v=np.zeros_like(q) if velocity is None else np.asarray(velocity,dtype=float)
        if q.shape!=self.home.shape or v.shape!=q.shape or not np.isfinite(q).all() or not np.isfinite(v).all():raise ValueError('Invalid motion state')
        self.knots.append(self.knots[-1]+float(duration));self.states.append(np.stack([q,v,np.zeros_like(q),np.zeros_like(q)]));self.labels.append(label)
        return self

    @property
    def duration(self):return self.knots[-1]

    def polynomial(self):return BPoly.from_derivatives(self.knots,self.states)

    def at(self,t):
        curve=self.polynomial();t=np.asarray(t,dtype=float)
        if np.any(t<0) or np.any(t>self.duration+1e-8):raise ValueError('Outside trajectory')
        return tuple(curve(np.clip(t,0,self.duration),nu=d) for d in range(4))

    def windows(self,label):return [(self.knots[i],self.knots[i+1]) for i,x in enumerate(self.labels) if x==label]

    def metadata(self):
        return dict(name=self.name,stage=self.stage,duration_s=self.duration,conditions=self.conditions,
            knots_s=self.knots,states=np.asarray(self.states).tolist(),labels=self.labels,
            initial_final_home=bool(np.allclose(self.states[-1][0],self.home) and np.allclose(self.states[-1][1:],0)),
            interpolation='septic Hermite / Bernstein; q,dq,ddq,jerk continuous at segment boundaries')


def restricted_coefficients(curve,order=0,subdivisions=8):
    """Exact Bernstein coefficients on a fixed subdivision, without sampling."""
    power=PPoly.from_bernstein_basis(curve.derivative(order));degree=len(power.c)-1
    knots=np.r_[(power.x[:-1,None]+np.diff(power.x)[:,None]*np.arange(subdivisions)/subdivisions).ravel(),power.x[-1]]
    coefficients=np.stack([power.derivative(k)(knots[:-1])/factorial(k) for k in range(degree,-1,-1)])
    restricted=BPoly.from_power_basis(PPoly(coefficients,knots))
    return restricted.c.reshape(degree+1,len(power.x)-1,subdivisions,-1)


def derivative_bounds(curve,order=1,subdivisions=8):
    """Bernstein convex bounds after exact polynomial restriction to subcells.

    SciPy's power/Bernstein conversions and Taylor derivatives represent the
    same polynomial on smaller intervals. This tightens the often very loose
    original control-point bound without substituting a sampled maximum.
    """
    bound=np.max(abs(restricted_coefficients(curve,order,subdivisions)),axis=(0,2))
    return bound*(1+1e-10)+1e-12


def gate_motion(profile,motion,clearance_m=.01):
    curve=motion.polynomial();bounds=derivative_bounds(curve)
    allowance_labels=motion.conditions.get('allowance_labels',('static_dwell','torque_ramp'))
    lower=np.min(curve.c,axis=(0,1));upper=np.max(curve.c,axis=(0,1))
    if motion.conditions.get('refine_position_bounds',False):
        coefficients=restricted_coefficients(curve)
        padding=1e-10*(1+np.max(abs(coefficients),axis=(0,1,2)))
        lower=coefficients.min(axis=(0,1,2))-padding;upper=coefficients.max(axis=(0,1,2))+padding
    if 'position_allowance_rad' in motion.conditions:
        allowance=np.asarray(motion.conditions['position_allowance_rad'])
        for i,label in enumerate(motion.labels):
            if label in allowance_labels:
                lower=np.minimum(lower,curve.c[:,i].min(axis=0)-allowance);upper=np.maximum(upper,curve.c[:,i].max(axis=0)+allowance)
    joints=profile.engine.model.joints;limits=np.array([[j.lower,j.upper] for j in joints])
    limits_passed=bool(np.all(lower>=limits[:,0]) and np.all(upper<=limits[:,1]))
    limit_report=dict(passed=limits_passed,lower_rad=lower.tolist(),upper_rad=upper.tolist(),
        source='nominal URDF travel limits only; manufacturer speed/effort capabilities unconfirmed',method='Bernstein coefficient convex hull')
    if not limits_passed:
        distance=float(profile.check([motion.home])['distances'].min())
        return dict(passed=False,reason='position_envelope_outside_urdf_travel',joint_limits=limit_report,
            segments=[],minimum_distance_m=distance,queries=1,wall_s=0.,motion=motion.name)
    if motion.stage=='inertia_probe':
        peaks=[float(np.max(derivative_bounds(curve,k))) for k in (1,2,3)]
        measure=[i for i,label in enumerate(motion.labels) if label=='probe_measure'];j=motion.conditions['joint'];sign=motion.conditions['direction']
        minimum_velocity=float(np.min(sign*curve.derivative().c[:,measure,j]))
        if any(a>b for a,b in zip(peaks,[1.,8.,200.])) or minimum_velocity<=.02:
            return dict(passed=False,reason='dedicated_probe_envelope',joint_limits=limit_report,segments=[],
                minimum_distance_m=float(profile.check([motion.home])['distances'].min()),queries=1,wall_s=0.,motion=motion.name)
    rows=[]
    for i,(a,b) in enumerate(zip(motion.knots[:-1],motion.knots[1:])):
        allowance=motion.conditions.get('position_allowance_rad') if motion.labels[i] in allowance_labels else None
        if allowance is not None and motion.conditions.get('joint_box_refinement',False):
            from .collision import certify_reference_box
            report=certify_reference_box(profile,lambda t,a=a:curve(t+a),b-a,bounds[i],allowance,clearance_m)
        else:report=certify_reference(profile,lambda t,a=a:curve(t+a),b-a,bounds[i],clearance_m=clearance_m,initial_step_s=b-a,position_allowance=allowance)
        report.update(segment=i,label=motion.labels[i],start_s=a,end_s=b)
        rows.append(report)
        if not report['passed']:break
    return dict(passed=all(r['passed'] for r in rows) and len(rows)==len(motion.labels),segments=rows,
        minimum_distance_m=min(r['minimum_sampled_distance_m'] for r in rows),queries=sum(r['queries'] for r in rows),
        wall_s=sum(r['wall_s'] for r in rows),motion=motion.name,
        joint_limits=limit_report,
        includes=['initial hold','approach','active motion','braking','return home','final hold'])


def _trial(name,stage,home,pose,config,conditions=None):
    motion=Motion(name,stage,home,conditions or {})
    motion.add(config['settle_s'],home,label='initial_hold').add(config['transition_s'],pose,label='approach')
    return motion


def _finish(motion,config):
    return motion.add(config['transition_s'],motion.home,label='return_home').add(config['settle_s'],motion.home,label='final_hold')


def add_friction_sweep(motion,joint,extent,speed,ramp=.25,reverse_hold=.15):
    """Shared C3 speed ramps, exact uniform plateaus and braking in both directions."""
    for sign in (1,-1):
        v=np.zeros_like(motion.home);v[joint]=sign*speed
        q=motion.states[-1][0].copy();q[joint]+=sign*speed*ramp/2
        motion.add(ramp,q,v,label='speed_ramp')
        q=q.copy();distance=2*extent-speed*ramp;q[joint]+=sign*distance
        if distance<=0:raise ValueError('Friction scan too short for configured speed')
        motion.add(distance/speed,q,v,label='constant_velocity')
        q=q.copy();q[joint]+=sign*speed*ramp/2
        motion.add(ramp,q,label='brake').add(reverse_hold,q,label='reverse_hold')
    return motion


def build_recipes(config):
    """Every independent trial begins/ends at a common declared home, at rest.

    A real adapter must observe and verify home before a trial. The Mock uses the
    same home initial condition; there is no hidden teleport inside a trial.
    """
    home=np.asarray(config['home_rad'],dtype=float);amplitude=np.asarray(config['pose_amplitude_rad'],dtype=float)
    if home.shape!=(14,) or amplitude.shape!=(14,) or np.any(amplitude<=0):raise ValueError('14 positive pose amplitudes required')
    result=[]
    for j in range(14):
        m=Motion(f'direction_{j:02}','direction',home,dict(joint=j))
        m.add(config['settle_s'],home,label='initial_hold')
        for factor in (1,-1,0):
            q=home.copy();q[j]+=factor*amplitude[j]
            m.add(config['transition_s'],q,label='jog').add(.2,q,label='jog_hold')
        m.add(config['settle_s'],home,label='final_hold');result.append(m)
    for name,period in [('timing_train',config['control_period_ns']),('timing_validation',config['control_period_ns']),('timing_frequency_250',4000000)]:
        m=Motion(name,'timing',home,dict(command_period_ns=period,validation=name!='timing_train',
            integration_ns=config.get('timing_integration_ns',25000)))
        m.add(.05,home,label='initial_hold').add(.25,home,label='torque_probe')
        result.append(m.add(config['transition_s'],home,label='recovery').add(config['settle_s'],home,label='final_hold'))
    # Train and validation trajectories have disjoint deterministic configurations.
    for batch,count,seed in [('train',config['gravity_train_poses'],config.get('gravity_train_seed',71)),('validation',config['gravity_validation_poses'],config.get('gravity_validation_seed',171))]:
        rng=np.random.default_rng(seed)
        for p in range(count*config.get('gravity_candidate_multiplier',2)):
            pose=home+rng.uniform(-1,1,14)*amplitude
            for approach in (-1,1):
                near=pose.copy();near+=approach*amplitude*.08
                m=_trial(f'gravity_{batch}_{p:02}_{approach:+}','gravity',home,near,config,dict(batch=batch,pose=p,approach=approach))
                m.add(.8,pose,label='paired_approach').add(config['gravity_hold_s'],pose,label='static_measure')
                result.append(_finish(m,config))
    for batch,seed in [('train',config.get('friction_train_seed',92)),('validation',config.get('friction_validation_seed',192)),('sampling',config.get('friction_validation_seed',192))]:
        rng=np.random.default_rng(seed)
        for p in range(1 if batch=='sampling' else config['friction_poses']):
            pose=home+rng.uniform(-.35,.35,14)*amplitude
            for speed in config['friction_speeds_rad_s']:
                for j in range(14):
                    extent=min(amplitude[j]*.65,config.get('friction_stroke_half_rad',.08));start=pose.copy();start[j]-=extent
                    m=_trial(f'friction_{batch}_{p:02}_{j:02}_{speed:g}','friction',home,start,config,dict(batch=batch,pose=p,joint=j,speed=speed,
                        sample_period_ns=config.get('friction_convergence_sample_period_ns',125000) if batch=='sampling' else config.get('friction_sample_period_ns',config['sample_period_ns'])))
                    # Ramp to speed on each side; exact constant velocity central
                    # segment uses matching Hermite derivatives at both ends.
                    add_friction_sweep(m,j,extent,speed)
                    result.append(_finish(m,config))
    breakaway_poses=[home+offset*amplitude for offset in (-0.25,0.25)]
    if 'breakaway_pose_seed' in config:
        rng=np.random.default_rng(config['breakaway_pose_seed'])
        breakaway_poses=[home+offset*amplitude for offset in rng.uniform(-.25,.25,(2,14))]
    for p,pose in enumerate(breakaway_poses):
        for j in range(14):
            for sign in (-1,1):
                for dwell in config['breakaway_dwell_s']:
                    for slope in config.get('breakaway_slopes_nm_s',[.12,.24]):
                        for repeat in range(config.get('breakaway_repeats',2)):
                            allowance=config.get('breakaway_tracking_allowance_nm',.08)/np.asarray(config['drive']['kp'],dtype=float)
                            allowance[j]+=config.get('breakaway_motion_allowance_rad',.003)
                            conditions=dict(pose=p,joint=j,direction=sign,dwell_s=dwell,slope_nm_s=slope,repeat=repeat,
                                acquisition_seed=config.get('acquisition_seed',27)+repeat*101,position_allowance_rad=allowance.tolist())
                            m=_trial(f'breakaway_{p}_{j:02}_{sign:+}_{dwell:g}_{slope:g}_{repeat}','breakaway',home,pose,config,conditions)
                            m.add(dwell,pose,label='static_dwell').add(config['breakaway_ramp_s'],pose,label='torque_ramp')
                            result.append(_finish(m,config))
    # Dedicated sustained-motion probes keep friction away from its set-valued
    # zero-speed region while increasing the wrist inertia signal. Geometry and
    # speed remain independently gated; the jerk envelope is explicitly separate
    # from the slow full-arm multisine, not a manufacturer capability claim.
    probe=config.get('wrist_probe',dict(frequency_hz=5.,amplitude_rad=.004,bias_speed_rad_s=.18,duration_s=2.))
    for p,offset in enumerate((-.35,0.,.35)):
        pose=home.copy();pose[[5,6,12,13]]=offset
        for joint in (4,11):
            for sign in (-1,1):
                result.append(wrist_probe_motion(f'wrist_probe_{p}_{joint:02}_{sign:+}',home,pose,config,joint,sign,
                    dict(pose=p,batch='validation' if p==1 else 'train')))
    dynamic=[('train',seed,'both') for seed in config.get('dynamic_candidate_seeds',[27])]
    dynamic += [('validation',127,'both'),('left_validation',128,'left'),('right_validation',129,'right')]
    dynamic += [(f'tuning_{k}',227,'both') for k in range(5)]
    dynamic += [(f'control_{model}_{gain}',327,'both') for model in ('nominal','identified') for gain in ('original','tuned')]
    for batch,seed,active in dynamic:
        period=config['dynamic_period_s'];periods=config['dynamic_periods']
        stage='inertia';name='inertia_'+batch;conditions=dict(batch=batch,period_s=period,periods=periods,seed=seed,active=active,
            sample_period_ns=config.get('inertia_sample_period_ns',250000))
        if batch=='train':name=f'inertia_train_candidate_{seed}'
        elif batch.startswith('tuning_'):
            stage='control_tuning';name='control_'+batch;k=int(batch.rsplit('_',1)[1]);conditions.update(kp_factor=[.5,.75,1.,1.25,1.5][k],feedforward='nominal')
        elif batch.startswith('control_'):
            stage='control_validation';name=batch;_,ff,gain=batch.split('_');conditions.update(feedforward=ff,gain_strategy=gain)
        result.append(periodic_motion(name,stage,home,home,config,seed,batch!='train',active,conditions))
    return result


def wrist_probe_motion(name,home,pose,config,joint,sign,conditions=None):
    """Shared non-reversing acceleration probe; native mechanics remain unchanged."""
    home=np.asarray(home,dtype=float);pose=np.asarray(pose,dtype=float)
    probe=config.get('wrist_probe',dict(frequency_hz=5.,amplitude_rad=.004,bias_speed_rad_s=.18,duration_s=2.))
    frequency=probe['frequency_hz'];omega=2*np.pi*frequency;probe_amplitude=probe['amplitude_rad'];bias=probe['bias_speed_rad_s'];duration=probe['duration_s'];ramp=1.
    start=pose.copy();start[joint]-=sign*bias*duration/2
    entry=start.copy();entry[joint]-=sign*bias*ramp/2
    conditions={**(conditions or {}), 'joint':joint, 'direction':sign, 'frequency_hz':frequency,
        'sample_period_ns':config.get('inertia_sample_period_ns',250000),
        'protocol':'sustained signed speed plus small sinusoid; no zero crossing in identification window'}
    m=_trial(name,'inertia_probe',home,entry,config,conditions)
    def smooth(u):
        return (35*u**4-84*u**5+70*u**6-20*u**7,140*u**3-420*u**4+420*u**5-140*u**6,
            420*u**2-1680*u**3+2100*u**4-840*u**5,840*u-5040*u**2+8400*u**3-4200*u**4,
            7*u**5-14*u**6+10*u**7-2.5*u**8)
    def add_state(qj,vj,aj,jj,span,label):
        q=pose.copy();v=np.zeros(14);a=np.zeros(14);jerk=np.zeros(14)
        q[joint]=qj;v[joint]=vj;a[joint]=aj;jerk[joint]=jj
        m.add(span,q,v,label=label);m.states[-1]=np.stack([q,v,a,jerk])
    step=1/128
    for mode,length in [('entry',ramp),('probe',duration),('exit',ramp)]:
        count=round(length/step);span=length/count
        for index in range(1,count+1):
            t=index*span
            if mode=='probe':
                angle=omega*t;qj=start[joint]+sign*(bias*t+probe_amplitude*np.sin(angle));vj=sign*(bias+probe_amplitude*omega*np.cos(angle));aj=-sign*probe_amplitude*omega**2*np.sin(angle);jj=-sign*probe_amplitude*omega**3*np.cos(angle)
            else:
                u=t/ramp;s,d,dd,ddd,integral=smooth(u)
                if mode=='entry':
                    phase=t-ramp;base=entry[joint]+sign*bias*ramp*integral;vb=sign*bias*s;ab=sign*bias*d/ramp;jb=sign*bias*dd/ramp**2
                else:
                    phase=duration+t;base=start[joint]+sign*bias*duration+sign*bias*ramp*(u-integral);vb=sign*bias*(1-s);ab=-sign*bias*d/ramp;jb=-sign*bias*dd/ramp**2
                    s,d,dd,ddd=1-s,-d,-dd,-ddd
                f=probe_amplitude*np.sin(omega*phase);v=probe_amplitude*omega*np.cos(omega*phase);a=-omega**2*f;j=-omega**2*v
                qj=base+sign*s*f;vj=vb+sign*(s*v+d/ramp*f);aj=ab+sign*(s*a+2*d/ramp*v+dd/ramp**2*f)
                jj=jb+sign*(s*j+3*d/ramp*a+3*dd/ramp**2*v+ddd/ramp**3*f)
            add_state(qj,vj,aj,jj,span,'probe_measure' if mode=='probe' else 'probe_'+mode)
    return _finish(m,config)


def periodic_motion(name,stage,home,center,config,seed,validation=False,active='both',conditions=None):
    """Shared analytic multisine and complete C3 entry/braking/return path."""
    if config.get('smooth_periodic_envelope',False):
        return enveloped_periodic_motion(name,stage,home,center,config,seed,validation,active,conditions)
    from .dual_excitation import periodic_reference
    home=np.asarray(home,dtype=float);center=np.asarray(center,dtype=float)
    period=config['dynamic_period_s'];periods=config['dynamic_periods'];step=period/64
    t=np.arange(periods*64+1)*step
    q,v,a,jrk=periodic_reference(t,center,period,np.asarray(config['pose_amplitude_rad']),seed,validation,active)
    conditions={**(conditions or {}), 'period_s':period,'periods':periods,'seed':seed,'active':active,
        'sample_period_ns':config.get('inertia_sample_period_ns',250000)}
    m=Motion(name,stage,home,conditions)
    m.add(config['settle_s'],home,label='initial_hold')
    transition=config.get('dynamic_transition_s',2.)
    m.add(transition,q[0],v[0],label='enter_periodic')
    m.states[-1]=np.stack([q[0],v[0],a[0],jrk[0]])
    for k in range(1,len(t)):
        m.add(step,q[k],v[k],label='periodic');m.states[-1]=np.stack([q[k],v[k],a[k],jrk[k]])
    return _finish(m,{**config,'transition_s':transition})


def enveloped_periodic_motion(name,stage,home,center,config,seed,validation=False,active='both',conditions=None):
    """C3 amplitude envelope; entry/braking cannot amplify the multisine span.

    Unlike matching nonzero high derivatives across a long rest-to-motion
    segment, multiplying by a bounded envelope preserves the reference range.
    SciPy supplies the envelope polynomial and its analytic derivatives.
    """
    from .dual_excitation import periodic_reference
    home=np.asarray(home,dtype=float);center=np.asarray(center,dtype=float)
    period=config['dynamic_period_s'];periods=config['dynamic_periods'];ramp=config['dynamic_ramp_s'];transition=config['dynamic_transition_s']
    conditions={**(conditions or {}),'period_s':period,'periods':periods,'seed':seed,'active':active,
        'entry_braking':'C3 bounded amplitude envelope; static center transitions',
        'sample_period_ns':config.get('inertia_sample_period_ns',250000)}
    m=Motion(name,stage,home,conditions);m.add(config['settle_s'],home,label='initial_hold').add(transition,center,label='to_excitation_center')
    envelope=BPoly.from_derivatives([0.,ramp],[[0.,0.,0.,0.],[1.,0.,0.,0.]])
    for mode,duration in [('entry',ramp),('periodic',periods*period),('exit',ramp)]:
        count=int(np.ceil(duration/(period/64)));span=duration/count;t=np.arange(1,count+1)*span
        phase=t-ramp if mode=='entry' else t if mode=='periodic' else periods*period+t
        q,v,a,j=periodic_reference(phase,center,period,np.asarray(config['pose_amplitude_rad']),seed,validation,active)
        delta=q-center
        if mode=='periodic':weights=[np.ones((count,1)),*[np.zeros((count,1)) for _ in range(3)]]
        else:
            weights=[np.asarray(envelope(t,nu=d))[:,None] for d in range(4)]
            if mode=='exit':weights=[1-weights[0],*[-w for w in weights[1:]]]
        w,d,dd,ddd=weights
        states=np.stack([center+w*delta,w*v+d*delta,w*a+2*d*v+dd*delta,w*j+3*d*a+3*dd*v+ddd*delta],axis=1)
        for state in states:
            m.add(span,state[0],state[1],label='periodic' if mode=='periodic' else 'excitation_'+mode);m.states[-1]=state
    return m.add(transition,home,label='return_home').add(config['settle_s'],home,label='final_hold')
