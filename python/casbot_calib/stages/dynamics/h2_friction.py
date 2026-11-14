"""H2 Friction stage stub (constant-velocity + breakaway static friction, P6)."""
from ...pipeline.stage import Stage
from ...shm_data import ROOT


class H2FrictionStage(Stage):
    name = 'h2'
    title = 'H2 摩擦标定'
    default_config = ROOT / 'configs/dynamics/h2.yaml'
    implemented = False
