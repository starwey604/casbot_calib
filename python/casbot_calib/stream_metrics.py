"""Clock-aware statistics for recorded event streams; no device access."""
import numpy as np


def distribution(values):
    values=np.asarray(values,dtype=float)
    if not len(values):return None
    return dict(count=len(values),minimum=float(values.min()),p50=float(np.quantile(values,.5)),
        p95=float(np.quantile(values,.95)),p99=float(np.quantile(values,.99)),maximum=float(values.max()),std=float(values.std()))


def rate(times):
    times=np.asarray(times,dtype=np.int64)
    if len(times)<2:return dict(hz=None,interval_ms=None)
    delta=np.diff(times)
    return dict(hz=float((len(times)-1)*1e9/(times[-1]-times[0])) if times[-1]>times[0] else None,interval_ms=distribution(delta*1e-6),nonpositive_intervals=int(np.count_nonzero(delta<=0)))


def latency(later,earlier,shared_clock):
    # No cross-clock arithmetic, including when the numeric values look plausible.
    if not shared_clock:return dict(observable=False,reason='clock_domains_not_aligned',milliseconds=None)
    return dict(observable=True,milliseconds=distribution((np.asarray(later,dtype=np.int64)-np.asarray(earlier,dtype=np.int64))*1e-6))


def position_rates(times):
    times=np.asarray(times,dtype=np.int64)
    if len(times)<2:return dict(hz=None,interval_ms=None)
    dt=np.diff(times)*1e-6
    return dict(hz=float((len(times)-1)*1e9/(int(times[-1])-int(times[0]))),
                interval_ms={key:float(value) for key,value in zip(('p50','p95','p99','maximum'),[*np.quantile(dt,[.5,.95,.99]),max(dt)])})


