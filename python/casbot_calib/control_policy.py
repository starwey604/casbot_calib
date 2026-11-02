"""Shared experiment policy; timing, observation qualification and acceptance stay with callers."""
import numpy as np

PD_FACTORS=(.5,.75,1.,1.25,1.5)
CONTROL_GROUPS=tuple((model,gains) for model in ('prior','identified') for gains in ('original','tuned'))


def pd_multipliers(factor):
    if not np.isfinite(factor) or factor<=0:raise ValueError('Positive finite PD factor required')
    return float(factor),float(np.sqrt(factor))


def scale_pd(kp,kd,factor):
    p,d=pd_multipliers(factor)
    return np.asarray(kp)*p,np.asarray(kd)*d


def select_pd_factor(scores):
    """Choose from (factor, admissible, RMSE), preserving grid order for ties."""
    if len(scores)!=len(PD_FACTORS) or {row[0] for row in scores}!=set(PD_FACTORS):
        raise ValueError('Exactly the five predeclared PD factors required')
    allowed=[row for row in scores if row[1]]
    if not allowed:raise ValueError('No PD candidate satisfies the declared constraints')
    if any(not np.isfinite(row[2]) or row[2]<0 for row in allowed):raise ValueError('Finite nonnegative tracking RMSE required')
    return min(allowed,key=lambda row:row[2])[0]


def comparison_reductions(prior_original,identified_original,prior_tuned,identified_tuned,*,denominator_floor=0.):
    denominator=lambda value:max(value,denominator_floor)
    return (1-identified_original/denominator(prior_original),1-prior_tuned/denominator(prior_original),
            1-identified_tuned/denominator(identified_original))
