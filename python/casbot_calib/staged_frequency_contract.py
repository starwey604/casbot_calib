"""Independent synthetic noise and analytic torque qualification for P0 R0.

No moving-mask concatenation, plant state, fitted model or test observations.
Thresholds qualify torque bins; measured velocity/acceleration qualification and
model attribution are R3 work. This is not a hardware bandwidth measurement.
"""
from pathlib import Path
import numpy as np
from .data import write_json


def harmonic_rms(values, harmonics):
    values = np.asarray(values, dtype=float)
    if values.ndim < 2 or not np.isfinite(values).all() or values.shape[-2] <= 2 * harmonics:
        raise ValueError('Complete uniformly sampled period required')
    coefficients = np.fft.rfft(values, axis=-2) / values.shape[-2]
    # Parseval: interior one-sided bins have energy 2*abs(c)**2.
    return np.sqrt(2.) * abs(coefficients[..., 1:harmonics + 1, :])


def qualify_frequency(output, config):
    settings = config['p0']['frequency_qualification']
    count = round(config['dynamic_period_s'] * 1e9 / config['inertia_sample_period_ns'])
    if not np.isclose(count * config['inertia_sample_period_ns'] * 1e-9, config['dynamic_period_s'], atol=1e-12, rtol=0):
        raise ValueError('Whole raw sampling periods required')
    harmonics = config['p0']['preprocessing']['harmonics']
    rng = np.random.default_rng(settings['noise_seed'])
    sigma = config['drive']['current_noise_a'] * config['drive']['gain_nm_a']
    noise = rng.normal(0., sigma, (settings['noise_periods'], count, 14))
    noise_bins = harmonic_rms(noise, harmonics)
    threshold = np.maximum(settings['minimum_bin_rms_nm'],
                           settings['noise_multiplier'] * np.max(noise_bins, axis=0))
    # Independent scalar torque law tau=J*qdd, not a round trip through Mock or
    # Pinocchio. Exact full-period Fourier amplitudes fix normalization/units.
    time = np.arange(count) * config['inertia_sample_period_ns'] * 1e-9
    h = 3; inertia = .02; amplitude = .03
    omega = 2 * np.pi * h / config['dynamic_period_s']
    acceleration = -amplitude * omega**2 * np.sin(omega * time)
    torque = inertia * acceleration[:, None]
    measured = float(harmonic_rms(torque, harmonics)[h - 1, 0])
    expected = inertia * amplitude * omega**2 / np.sqrt(2)
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / 'noise.npz', torque_nm=noise,
                        sample_period_ns=np.int64(config['inertia_sample_period_ns']))
    report = dict(schema_version=1, passed=bool(abs(measured - expected) < 1e-12),
        source='independent configured white current-sensor noise; no plant/test observations',
        noise_seed=settings['noise_seed'], independent_noise_periods=settings['noise_periods'],
        raw_sample_hz=1e9 / config['inertia_sample_period_ns'], period_s=config['dynamic_period_s'],
        harmonics=list(range(1, harmonics + 1)), frequency_hz=(np.arange(1, harmonics + 1) / config['dynamic_period_s']).tolist(),
        threshold_bin_rms_nm=threshold.tolist(),
        rule='max(minimum_bin_rms_nm, noise_multiplier * maximum independent noise-period bin RMS)',
        settings=settings, analytic_oracle=dict(inertia_kg_m2=inertia, amplitude_rad=amplitude,
            harmonic=h, expected_rms_nm=expected, measured_rms_nm=measured),
        hardware_bandwidth_qualified=False,
        limitations=['Synthetic white sensor noise only; not drive ripple, clock error or derivative noise',
                    'R3 must additionally qualify measured speed/acceleration and report insufficient evidence',
                    'Full-period same-band residual attribution and raw out-of-band diagnostics pending R3'])
    write_json(output / 'report.json', report)
    return report
