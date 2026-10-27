import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from casbot_calib.demo import run_demo,score_control


def test_workflow_collects_without_mujoco_or_truth_access(tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,"mujoco",None)
    original=np.load
    def load(path,*args,**kwargs):
        assert Path(path).name!="evaluation_truth.npz"
        return original(path,*args,**kwargs)
    monkeypatch.setattr(np,"load",load)
    folder=tmp_path/"workflow"
    result=run_demo(folder,progress=lambda *_:None)
    assert result["passed"] and result["heldout"]["torque_rmse"]<.01
    assert result["direction"]["physical_direction"].startswith("unconfirmed")
    assert len(result["controls"])==4 and len(result["pd_candidates"])==50
    assert result["fixed_pd_improvement"]>=.2
    assert result["repeatability"]["total"]==10 and result["repeatability"]["passed_count"]>=9
    assert len(result["frequency_validation"])==3
    assert result["driver_heldout"]["parameters_frozen"]
    assert result["driver_response"]["gain_status"]=="fixed_independently_anchored_gain"
    assert result["gravity"]["identified"]["parameters"]["gravity_cos"]<0
    session=result["session"]
    assert all(row["drive_sha256"]==session["drive_sha256"] for row in session["records"])
    assert len(session["records"])==len(list((folder/"records").glob("*/session.json")))
    clips=json.loads((folder/"storyboard.json").read_text(encoding="utf-8"))["clips"]
    assert len(clips)==8
    for path,digest in json.loads((folder/"evidence_sha256.json").read_text(encoding="utf-8")).items():
        assert hashlib.sha256((folder/path).read_bytes()).hexdigest()==digest
    events=[json.loads(line) for line in (folder/"workflow_events.jsonl").read_text(encoding="utf-8").splitlines()]
    assert events[-1]["stage"]=="complete" and events[-1]["state"]=="passed"
