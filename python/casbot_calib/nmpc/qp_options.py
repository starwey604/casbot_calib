"""Winora's fast HPIPM profile for the local SQP-RTI solver."""

PROFILES = {
    'fast': {
        'tol_stat': 1e-4,
        'tol_eq': 1e-7,
        'tol_ineq': 1e-7,
        'tol_comp': 1e-7,
        'warm_start': 0,
    },
}


def profile_options(name):
    if name not in PROFILES:
        raise ValueError(f'Unknown QP profile: {name}')
    return PROFILES[name].copy()
