"""Continuous references are sampled at each control period, with no rescaling."""
from __future__ import annotations
import numpy as np

def multisine(t, frequencies=(.5, 1.5, 2.5), amplitudes=(.3, .2, .14), phases=(.2, 1.1, -.6)):
    t = np.asarray(t)
    q, v, a = (np.zeros_like(t, dtype=float) for _ in range(3))
    for f, amp, phase in zip(frequencies, amplitudes, phases, strict=True):
        w = 2*np.pi*f
        q += amp*np.sin(w*t+phase)
        v += amp*w*np.cos(w*t+phase)
        a -= amp*w*w*np.sin(w*t+phase)
    return q, v, a

def torque_commands(duration_s=24., period_ns=1_000_000, frequencies=(.5, 1.5, 2.5),
                    amplitudes=(.3, .2, .14), phases=(.2, 1.1, -.6)):
    ts = np.arange(0, round(duration_s*1e9), period_ns, dtype=np.int64)
    values = np.zeros((len(ts), 6))
    values[:, 3] = multisine(ts*1e-9, frequencies, amplitudes, phases)[0]
    return ts, values

def feedforward(q, dq, ddq, model, spring_zero=0., friction_velocity=.03):
    return (model.get("inertia", 0)*ddq + model.get("damping", 0)*dq
            + model.get("spring", 0)*(q-spring_zero)
            + model.get("coulomb", 0)*np.tanh(dq/friction_velocity)
            + model.get("gravity_sin", 0)*np.sin(q)+model.get("gravity_cos", 0)*np.cos(q))

def tracking_commands(model, kp, kd, duration_s=6., period_ns=1_000_000, validation=False,
                      spring_zero=0., friction_velocity=.03):
    ts = np.arange(0, round(duration_s*1e9), period_ns, dtype=np.int64)
    args = dict(frequencies=(.5, 1., 2.), amplitudes=(.08, .045, .022), phases=(0., 0., 0.))
    if validation:
        args = dict(frequencies=(.75, 1.25, 2.75), amplitudes=(.07, .04, .018), phases=(0., 0., 0.))
    q, v, a = multisine(ts*1e-9, **args)
    values = np.column_stack((np.ones(len(ts)), q, v, feedforward(q, v, a, model,spring_zero,friction_velocity),
                              np.full(len(ts), kp), np.full(len(ts), kd)))
    return ts, values, args

def guitar_reference(t, lever=.2, speed=1., stroke=.1, reversal=.05, direction=1.):
    """100 ms straight segments + 50 ms constant-acceleration turns.

    x=lever*q, a one-joint equivalent mapping, not seven-axis kinematics.
    """
    t = np.asarray(t)
    straight = stroke/speed
    phase = t % (2*(straight+reversal))
    x = np.zeros_like(t); v = np.zeros_like(t); a = np.zeros_like(t)
    m = phase < straight
    x[m] = -stroke/2+speed*phase[m]; v[m] = speed
    m = (phase >= straight) & (phase < straight+reversal)
    u = phase[m]-straight
    x[m] = stroke/2+speed*u-speed/reversal*u*u
    v[m] = speed-2*speed/reversal*u; a[m] = -2*speed/reversal
    m = (phase >= straight+reversal) & (phase < 2*straight+reversal)
    u = phase[m]-straight-reversal
    x[m] = stroke/2-speed*u; v[m] = -speed
    m = phase >= 2*straight+reversal
    u = phase[m]-2*straight-reversal
    x[m] = -stroke/2-speed*u+speed/reversal*u*u
    v[m] = -speed+2*speed/reversal*u; a[m] = 2*speed/reversal
    return direction*x/lever, direction*v/lever, direction*a/lever
