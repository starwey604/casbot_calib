"""GUI-owned localhost Mock. Parent EOF closes only the instance we started."""
import argparse
from pathlib import Path
import time

from .fieldwork_ipc import PipeHeartbeat,emit


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,required=True);p.add_argument('--owner-id',required=True)
    p.add_argument('--profile',type=Path,required=True);p.add_argument('--pause-file',type=Path)
    args=p.parse_args();check=PipeHeartbeat(0,args.owner_id,timeout_s=None)
    from .sdk_mock import SDKMockSession
    from .fieldwork_storage import CacheMonitor
    monitor=CacheMonitor(args.output.parent) if (args.output.parent/'cache.json').exists() else None
    try:
        with SDKMockSession('configs/fieldwork_mock.yaml',args.output,args.domain_id,profile=args.profile,
                recording_pause_file=args.pause_file,cancel_check=check) as mock:
            emit('mock_ready',profile=str(mock.client_profile),endpoint=mock.endpoint)
            while not check():
                if any(process.poll() is not None for process in mock.processes):raise RuntimeError('本地Mock进程提前退出')
                time.sleep(.05)
        return 0
    except Exception as exc:emit('mock_error',message=str(exc));return 1
    finally:
        if monitor:monitor.close()


if __name__=='__main__':raise SystemExit(main())
