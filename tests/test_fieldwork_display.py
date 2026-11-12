"""Display optimizations keep feedback fidelity and the live stop channel separate."""
import json
import os
from types import SimpleNamespace

import pytest

from casbot_calib import fieldwork_display as display


def test_graphics_thread_default_respects_explicit_override(monkeypatch):
    monkeypatch.delenv('LP_NUM_THREADS',raising=False)
    monkeypatch.setenv('OMP_NUM_THREADS','3')
    display.configure_graphics_environment()
    assert os.environ['LP_NUM_THREADS']=='0' and os.environ['OMP_NUM_THREADS']=='3'
    monkeypatch.setenv('LP_NUM_THREADS','4');display.configure_graphics_environment()
    assert os.environ['LP_NUM_THREADS']=='4'


def test_display_mailbox_atomic_without_evidence_fsync(tmp_path,monkeypatch):
    monkeypatch.setattr(os,'fsync',lambda *_:pytest.fail('disposable view is not evidence'))
    path=tmp_path/'pose.json'
    display.latest_json(path,dict(q=[.1,.2],selected=1))
    display.latest_json(path,dict(q=[.3,.4],selected=0))
    assert json.loads(path.read_text())==dict(q=[.3,.4],selected=0)
    assert not path.with_name('pose.json.pending').exists()
    # Serialization failure leaves the previously published frame complete.
    with pytest.raises(ValueError):display.latest_json(path,dict(q=[float('nan')]))
    assert json.loads(path.read_text())['q']==[.3,.4]


def test_geometry_ignores_timestamp_but_not_position_or_selection():
    pose=dict(q=[0.,.1],selected=0,receive_ns=1,source='SDK_feedback')
    assert not display.geometry_changed(dict(pose,receive_ns=999),pose)
    assert display.geometry_changed(dict(pose,q=[0.,.11]),pose)
    assert display.geometry_changed(dict(pose,selected=1),pose)


def test_pipe_pulses_every_frame_and_cancel_precedes_archival_io(tmp_path,monkeypatch):
    from casbot_calib import fieldwork_gui as gui
    desk=gui.Desk.__new__(gui.Desk)
    desk.local_clock='unit_clock';desk.owner_id='unit_owner';desk.closing=False
    desk.heartbeat=tmp_path/'heartbeat.json';desk.heartbeat_archived_at=None
    desk.control_cancel=None;desk.observer_cancel=None
    rows=[];writes=[];now=[0.]
    desk.task=SimpleNamespace(pulse=lambda row:rows.append(('control',row)))
    desk.observer=SimpleNamespace(pulse=lambda row:rows.append(('observer',row)))
    desk.renderer=SimpleNamespace(pulse=lambda row:rows.append(('renderer',row)))
    monkeypatch.setattr(gui.time,'monotonic',lambda:now[0])
    def archive(path,row):
        assert len(rows)%3==0 and len(rows)>len(writes)*3
        writes.append(row)
    monkeypatch.setattr(gui,'latest_json',archive)
    desk.pulse();now[0]=.02;desk.pulse()
    assert len(rows)==6 and len(writes)==1
    now[0]=1.;desk.pulse();assert len(writes)==2
    desk.pulse('operator_cancel')
    assert rows[-3][1]['cancel']=='operator_cancel'
    assert rows[-2][1]['cancel'] is None and rows[-1][1]['cancel'] is None
    desk.closing=True;desk.pulse()
    assert all(row['cancel']=='frontend_closed' for _,row in rows[-3:])


def test_disk_failure_does_not_hide_cancel(tmp_path,monkeypatch):
    from casbot_calib import fieldwork_gui as gui
    desk=gui.Desk.__new__(gui.Desk)
    desk.local_clock='unit_clock';desk.owner_id='unit_owner';desk.closing=False
    desk.heartbeat=tmp_path/'heartbeat.json';desk.heartbeat_archived_at=None
    desk.control_cancel=None;desk.observer_cancel=None;desk.observer=desk.renderer=None
    rows=[];desk.task=SimpleNamespace(pulse=rows.append)
    def full(*_):raise OSError('full disk')
    monkeypatch.setattr(gui,'latest_json',full)
    desk.pulse('operator_cancel')
    assert rows[-1]['cancel']=='operator_cancel' and 'full disk' in desk.recording_error


def test_display_assets_keep_source_coordinate_extrema_and_face_inventory():
    from pathlib import Path
    import struct
    import numpy as np
    root=Path(display.__file__).parent/'assets/fieldwork_display/dvt2_v1'
    manifest=json.loads((root/'manifest.json').read_text())
    source=Path(display.__file__).parent/'assets/models/casbot_hl12p1_dvt2/source/assets/visual/CASBOTHL12'
    for row in manifest['meshes']:
        vertices=np.asarray([[float(x) for x in line.split()[1:4]]
            for line in (source/row['source']).read_text().splitlines() if line.startswith('v ')])
        # Independently confirm Blender kept original OBJ axes, units and origin.
        original=np.array([vertices.min(axis=0),vertices.max(axis=0)])
        assert np.max(np.abs(original-row['bounds_source']))<1e-6
        assert np.max(np.abs(original-row['bounds_display']))<=.000501
        path=root/row['file']
        with path.open('rb') as stream:
            stream.seek(80);count=struct.unpack('<I',stream.read(4))[0]
        assert count==row['triangles'] and path.stat().st_size==84+50*count
        assert row['max_sampled_deviation_m']<=.002
    assert sum(r['triangles'] for r in manifest['meshes'])<.2*sum(r['source_triangles'] for r in manifest['meshes'])


def test_lightweight_scene_changes_only_visual_resource_paths(tmp_path):
    import copy
    from pathlib import Path
    import xml.etree.ElementTree as ET
    from casbot_calib.model_context import resolve_context
    import yaml
    config=yaml.safe_load(Path('configs/fieldwork_mock.yaml').read_text())
    context=resolve_context(config['model_context']);canonical=tmp_path/'scene.xml'
    context.write_view_scene(canonical)
    before=ET.parse(canonical);render=display.lightweight_scene(context,canonical);after=ET.parse(render)
    assert render!=canonical
    for node in after.findall('./asset/mesh'):
        assert (render.parent/node.get('file')).is_file()
    # All hierarchy, axes, placements, scales, materials and geometry are identical.
    for tree in (before,after):
        for node in tree.findall('./asset/mesh'):node.attrib.pop('file')
    assert ET.tostring(before.getroot())==ET.tostring(after.getroot())
    unsupported=SimpleNamespace(identity=copy.deepcopy(context.identity))
    unsupported.identity['model_id']='another_model'
    assert display.lightweight_scene(unsupported,canonical)==canonical


def test_visual_deadband_keeps_cumulative_motion_and_does_not_quantize_feedback():
    previous=dict(q=[0.,.1],selected=0,receive_ns=1)
    noisy=dict(q=[1e-6,.100001],selected=0,receive_ns=2)
    assert not display.geometry_changed(noisy,previous)
    assert display.geometry_changed(dict(noisy,selected=1),previous)
    # Each increment may be below epsilon; compare to last actual rendered pose.
    assert display.geometry_changed(dict(noisy,q=[0.,.10011]),previous)
    assert noisy['q']==[1e-6,.100001] # Never round the delivered/recorded positions.
    assert display.geometry_changed(noisy,None)
