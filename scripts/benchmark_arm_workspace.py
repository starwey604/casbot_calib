"""Repeatable desktop batch timing; optional frozen before-change numerical comparison."""
import argparse
import hashlib
import platform
from pathlib import Path
from time import perf_counter
import numpy as np
from casbot_calib import _core
from casbot_calib.arm_model import load_arm_model
from casbot_calib.data import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--baseline',type=Path,help='Directory with benchmark.json and frozen probes.npz')
    parser.add_argument('--samples',type=int,default=4096)
    parser.add_argument('--repeat',type=int,default=15)
    args=parser.parse_args()
    if args.samples<1 or args.repeat<3:parser.error('Need positive samples and at least three repeats')
    engine=_core.ArmDynamics(load_arm_model());rng=np.random.default_rng(20261027)
    reference=None
    if args.baseline:
        with np.load(args.baseline/'probes.npz',allow_pickle=False) as data:
            q,v,a=(data[key].copy() for key in ('q','dq','ddq'))
            reference={key:data[key].copy() for key in ('mass_matrix','inertia','velocity','gravity','friction','total')}
    else:q,v,a=rng.uniform(-.5,.5,(3,args.samples,7))
    args.output.mkdir(parents=True,exist_ok=False)
    output=engine.evaluate_batch(q,v,a)  # warm-up and numerical comparison
    errors={} if reference is None else {key:float(np.max(np.abs(output[key]-reference[key]))) for key in reference}
    wall=[]
    for _ in range(args.repeat):
        start=perf_counter();engine.evaluate_batch(q,v,a);wall.append(perf_counter()-start)
    report=dict(backend=engine.backend_version(),platform=platform.platform(),python=platform.python_version(),
        batch_size=len(q),repeat=args.repeat,wall_s=wall,median_us_per_sample=float(np.median(wall)/len(q)*1e6),
        p95_batch_s=float(np.percentile(wall,95)),maximum_frozen_output_errors=errors,
        passed=all(error<1e-12 for error in errors.values()),
        scope='Desktop end-to-end Python batch API timing including output allocation; no hard-real-time guarantee; timings are not acceptance gates')
    if args.baseline:
        import json
        prior=json.loads((args.baseline/'benchmark.json').read_text(encoding='utf-8'))
        report['baseline_median_us_per_sample']=prior['median_us_per_sample']
        report['baseline_over_current_median_ratio']=prior['median_us_per_sample']/report['median_us_per_sample']
        report['baseline_probes_sha256']=hashlib.sha256((args.baseline/'probes.npz').read_bytes()).hexdigest()
    write_json(args.output/'benchmark.json',report)
    np.savez_compressed(args.output/'probes.npz',q=q,dq=v,ddq=a,**output)
    print(f"passed={report['passed']}; median={report['median_us_per_sample']:.3f} us/sample; {args.output}")
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
