"""CAD interchange retains metric vertices and source face counts."""
import numpy as np
import pytest
from casbot_calib.mesh_io import read_stl,write_stl


def test_binary_metric_roundtrip_and_reject_corrupt_payload(tmp_path):
    path=tmp_path/'part.stl';v=np.array([[0,0,0],[.01,0,0],[0,.02,0],[0,0,.03]])
    f=np.array([[0,1,2],[0,3,1],[0,2,3],[1,3,2]])
    write_stl(path,v,f);points,faces=read_stl(path)
    np.testing.assert_allclose(points[faces],v[f],atol=2e-9,rtol=0)
    path.write_bytes(path.read_bytes()[:-1])
    with pytest.raises(ValueError,match='Invalid binary STL'):read_stl(path)
