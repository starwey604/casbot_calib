"""H3 Inertia stage stub (periodic Fourier excitation + 140-param PSD fit, P6)."""
from ...pipeline.stage import Stage
from ...shm_data import ROOT


class H3InertiaStage(Stage):
    name = 'h3'
    title = 'H3 惯量标定'
    default_config = ROOT / 'configs/dynamics/h3.yaml'
    implemented = False
