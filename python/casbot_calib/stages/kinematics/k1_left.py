from .common import AcquisitionStage


class K1LeftStage(AcquisitionStage):
    name = 'k1'
    title = '左臂采集'
    side = 'left'
    stop_after = True
