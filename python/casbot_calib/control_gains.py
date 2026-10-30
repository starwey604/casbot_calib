"""Declared nominal PD starting gains; no identification or plant-truth access."""
import numpy as np


def nominal_pd_gains(engine,pose,frequency,damping,minimum_kp,minimum_kd):
    diagonal=np.diag(engine.mass_matrix(np.asarray(pose)))
    kp=np.maximum(diagonal*frequency**2,minimum_kp)
    kd=np.maximum(2*damping*frequency*diagonal,minimum_kd)
    if np.max(kp)>1000 or np.max(kd)>100:raise ValueError('Nominal starting gains exceed declared range')
    return kp,kd
