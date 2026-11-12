"""Damage follows changed triangles and texture pixels, not just vertex positions."""
import numpy as np
from casbot_calib.fieldwork_damage import Damage


def triangle(points):
    data=np.zeros((3,20),dtype=np.uint8)
    data[:,:8]=np.asarray(points,dtype=np.float32).view(np.uint8).reshape(3,8)
    return data


def test_static_ui_has_no_damage_and_image_upload_has_its_own_region():
    damage=Damage();a=triangle([[10,10],[30,10],[10,30]])
    key=(1,0,0,1000,1000)
    initial=damage.regions([(key,a)],(1000,1000))
    assert sum((r[2]-r[0])*(r[3]-r[1]) for r in initial)==1000000
    assert damage.regions([(key,a.copy())],(1000,1000))==[]
    changed=damage.regions([(key,a.copy())],(1000,1000),texture=(2,1,(500,100,880,385)))
    assert changed and all(r[0]>=480 and r[2]<=912 for r in changed)


def test_glyph_uv_change_and_motion_cover_original_and_new_geometry():
    damage=Damage();key=(1,0,0,1000,1000)
    first=triangle([[10,10],[30,10],[10,30]])
    damage.regions([(key,first)],(1000,1000))
    glyph=first.copy();glyph[0,8]=1
    assert damage.regions([(key,glyph)],(1000,1000))==[(0,0,48,48)]
    moved=triangle([[210,10],[230,10],[210,30]])
    regions=damage.regions([(key,moved)],(1000,1000))
    assert (0,0,48,48) in regions and (192,0,240,48) in regions


def test_layout_and_resize_damage_are_complete():
    damage=Damage();key=(1,0,0,100,100);a=triangle([[1,1],[2,1],[1,2]])
    damage.regions([(key,a)],(100,100))
    regions=damage.regions([((1,0,0,120,100),a)],(120,100))
    assert sum((r[2]-r[0])*(r[3]-r[1]) for r in regions)==12000
