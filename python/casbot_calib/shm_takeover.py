"""Bounded, explicit Jetson handoff; no SHM access or automatic recovery."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

SERVICES=('hl_orin_app.service','zero_calibration.service','arm_debug_tool.service')


class HandoffError(RuntimeError):
    pass


class Host:
    def __init__(self,vendor=Path('/workspace/embed_fusion/bin'),proc=Path('/proc')):
        self.vendor,self.proc=Path(vendor),Path(proc)

    def capture(self,command,timeout=3):
        try:
            result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout)
            return result.stdout.strip()
        except (OSError,subprocess.TimeoutExpired) as error:return 'unavailable: '+str(error)

    def services(self):
        return {unit:self.capture(['systemctl','is-active',unit]) for unit in SERVICES}

    def processes(self):
        found=[]
        for folder in self.proc.glob('[0-9]*'):
            try:
                comm=(folder/'comm').read_text().strip()
                if comm not in ('ethercat_statio','ethercat_station','hlorin'):continue
                # Capture every thread, not only a possibly-zombie group leader.
                threads=[]
                for task in (folder/'task').iterdir():
                    try:
                        status=(task/'status').read_text().splitlines()
                        state=next(line.split()[1] for line in status if line.startswith('State:'))
                        threads.append(dict(tid=int(task.name),state=state))
                    except FileNotFoundError:continue
                found.append(dict(pid=int(folder.name),name=comm,threads=threads,
                                  start_ticks=(folder/'stat').read_text().rsplit(')',1)[1].split()[19]))
            except FileNotFoundError:continue
        return found

    def terminate_master(self,process):
        # Recheck the same PID/start time immediately before this targeted signal.
        current={p['pid']:p for p in self.processes()}.get(process['pid'])
        if current is None:return
        if current['name']=='hlorin' or current['start_ticks']!=process['start_ticks']:
            raise HandoffError('主站PID身份已变化，未发送信号')
        subprocess.run(['sudo','-n','kill','-TERM',str(process['pid'])],check=True,timeout=3)

    def diagnostics(self,root):
        data=dict(services=self.services(),processes=self.processes())
        (root/'diagnosis.json').write_text(json.dumps(data,indent=2)+'\n')
        text=[]
        for unit in SERVICES:
            text.append(self.capture(['systemctl','show',unit,'-p','ActiveState','-p','SubState','-p','ControlPID','-p','MainPID']))
        for process in data['processes']:
            for thread in process['threads']:
                path=self.proc/str(process['pid'])/'task'/str(thread['tid'])/'stack'
                text.append(str(path)+'\n'+self.capture(['sudo','-n','cat',str(path)]))
        (root/'diagnosis.txt').write_text('\n\n'.join(text)+'\n')

    def start(self,root):
        with (root/'station.log').open('wb') as log:
            child=subprocess.Popen(['sudo','-n','bash',str(self.vendor/'run_ethercat_station.bash')],
                                   stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (root/'station_launcher.pid').write_text(str(child.pid)+'\n')


def alive(process):
    return any(t['state'] not in ('Z','X') for t in process['threads'])


def run_logged(command,path,*,timeout=120):
    """Inherit interactive stdin, but never pipe stdout held by vendor descendants."""
    with path.open('wb') as writer,path.open('rb') as reader:
        child=subprocess.Popen(command,stdout=writer,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+timeout;notice=time.monotonic()+5
        def output():
            data=reader.read()
            if data:sys.stdout.buffer.write(data);sys.stdout.buffer.flush()
        try:
            while child.poll() is None:
                output()
                if time.monotonic()>=deadline:raise HandoffError('停机命令超时；未启动新主站')
                if time.monotonic()>=notice:
                    print('仍在等待停机步骤；不会把厂家成功提示当作退出证明。',flush=True);notice=time.monotonic()+5
                time.sleep(.05)
            output();return child.returncode
        except BaseException:
            if child.poll() is None:
                # Only the Popen helper owned here, never the master or unrelated PIDs.
                child.terminate()
                try:child.wait(timeout=2)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
            output();raise


def graceful_master(host,root,timeout=15):
    masters=[p for p in host.processes() if p['name']!='hlorin' and alive(p)]
    for process in masters:
        print(f'请求旧主站PID {process["pid"]} 正常退出（SIGTERM），不使用SIGKILL。',flush=True)
        host.terminate_master(process)
    deadline=time.monotonic()+timeout
    while any(p['name']!='hlorin' and alive(p) for p in host.processes()):
        if time.monotonic()>=deadline:raise HandoffError('旧主站15秒内未退出；不硬杀、不启动新主站。请查看线程栈，安排现场恢复')
        time.sleep(.1)
    (root/'master_exit_confirmed.txt').write_text('No live master threads; before vendor launcher stop.\n')


def wait_stopped(host,root,timeout=120):
    deadline=time.monotonic()+timeout;notice=time.monotonic()+5
    while True:
        states=host.services();running=[p for p in host.processes() if alive(p)]
        (root/'stopped_services.txt').write_text(''.join(f'{unit} {state}\n' for unit,state in states.items()))
        if all(s in ('inactive','failed') for s in states.values()) and not running:return
        if time.monotonic()>=deadline:raise HandoffError('服务/旧主站尚未退出；未启动新主站。查看diagnosis.json/txt')
        if time.monotonic()>=notice:
            print(f'等待实际退出：{states}；存活进程{[p["pid"] for p in running]}',flush=True);notice=time.monotonic()+5
        time.sleep(.2)


def prepare(root,*,start_only=False,diagnose=False,host=None,runner=run_logged):
    root=Path(root).resolve();host=Host() if host is None else host
    if start_only:
        if not (root/'stop.log').is_file():raise ValueError('需要本次已有停业务日志目录')
    else:root.mkdir(parents=True,exist_ok=False)
    result=dict(schema='casbot_shm_takeover_v2',completed=False,station_start_requested=False)
    try:
        if diagnose:
            host.diagnostics(root);result['reason']='diagnosis_only_no_control';return 0
        if not all((host.vendor/name).is_file() for name in ('launcher.bash','run_ethercat_station.bash')):
            raise HandoffError('厂家入口不存在，未停止业务')
        if any(s in ('deactivating','activating') for s in host.services().values()):
            raise HandoffError('已有服务启停未完成，不重复停机、不启动主站；先处理旧会话')
        if not start_only:
            if runner(['sudo','-n','true'],root/'sudo.log',timeout=3):raise HandoffError('sudo -n不可用，未停止业务')
            print('停止独立服务；现场支撑继续承担失能后的载荷。',flush=True)
            if runner(['sudo','-n','systemctl','stop',*SERVICES[1:]],root/'independent_stop.log',timeout=100):
                raise HandoffError('独立服务停止失败，未继续接管')
            graceful_master(host,root)
            code=runner(['sudo','-n','bash',str(host.vendor/'launcher.bash'),'stop'],root/'stop.log',timeout=120)
            (root/'stop.returncode').write_text(str(code)+'\n')
            print(f'厂家stop返回{code}；现在核对实际服务和线程退出。',flush=True)
        wait_stopped(host,root)
        host.start(root)
        result.update(completed=True,station_start_requested=True,reason='station_start_requested_not_ready_proof')
        print(f'已发起新主站启动：{root}/station.log；确认日志后再单独activate。',flush=True)
        return 0
    except KeyboardInterrupt:
        result['reason']='operator_interrupt';print('已中断准备流程；未自动恢复或继续启动。',file=sys.stderr);return 130
    except (HandoffError,OSError,subprocess.SubprocessError) as error:
        result['reason']=str(error);print('准备失败：'+str(error),file=sys.stderr);return 2
    finally:
        if not result['completed'] and not diagnose:
            try:host.diagnostics(root)
            except (OSError,subprocess.SubprocessError) as error:result['diagnostics_error']=str(error)
        (root/'prepare_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group();group.add_argument('--start-only',action='store_true');group.add_argument('--diagnose',action='store_true')
    parser.add_argument('logs',type=Path);args=parser.parse_args(argv)
    interrupted=False
    def interrupt(signum,frame):
        nonlocal interrupted
        if not interrupted:
            interrupted=True
            raise KeyboardInterrupt
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
    try:return prepare(args.logs,start_only=args.start_only,diagnose=args.diagnose)
    except (ValueError,OSError) as error:print(str(error),file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
