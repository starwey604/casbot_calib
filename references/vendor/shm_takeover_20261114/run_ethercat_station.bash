#!/bin/bash

current_dir=$(cd $(dirname $0); pwd)
root_dir=$(cd $current_dir/; pwd)

source $current_dir/embed_fusion_source.env
source $current_dir/common_function.bash

#状态文件路径
state_file="/tmp/ethercat/prepare_env_done.flag"

# 日志目录路径
log_dir="/var/log/robot/current/"

function ensure_ethercat_master_ready()
{
    # 期望的主站数量（可通过配置文件或自动检测，此处默认检测所有存在的）
    local expected_masters=${1:-0}   # 如果传入参数则使用，否则自动检测

    # 如果未指定数量，尝试从 /dev/EtherCAT* 或配置文件获取
    if [ $expected_masters -eq 0 ]; then
        # 方法1：通过 ethercat master 命令获取
        if command -v ethercat &>/dev/null; then
            expected_masters=$(ethercat master 2>/dev/null | grep -c "^Master")
        fi
        # 若获取失败，默认为1
        [ $expected_masters -eq 0 ] && expected_masters=1
    fi

    echo "# ensure_ethercat_master_ready(): expecting $expected_masters master(s)"

    # 如果没有任何 /dev/EtherCAT* 设备，先启动主站
    if [ ! -e /dev/EtherCAT0 ] && [ ! -e /dev/EtherCAT1 ]; then
        echo "# No EtherCAT device found, starting ethercatctl"
        $sudocmd ethercatctl start
        sleep 6
    fi

    local timeout=15   # 总超时秒数
    local count=0
    local ready=0
    echo "expected_masters=$expected_masters"
    while [ $count -lt $timeout ]; do
        # 检查当前已有的设备数量
        local dev_count=$(ls /dev/EtherCAT* 2>/dev/null | wc -l)
        if [ $dev_count -ge $expected_masters ]; then
            # 所有主站设备已存在
            ready=1
            break
        fi
        echo -ne "# Waiting for /dev/EtherCAT*: ${count}s / ${timeout}s, found $dev_count of $expected_masters \r"
        sleep 1
        count=$((count + 1))
    done
    echo ""

    if [ $ready -eq 1 ]; then
        # 可选：进一步验证主站是否已经扫描到从站
        # 尝试执行 ethercat slaves，看是否能列出从站（可能需要额外等待几秒）
        local slaves_ok=0
        local retry=3
        while [ $retry -gt 0 ]; do
            if ethercat slaves 2>/dev/null | grep -q "[0-9]"; then
                slaves_ok=1
                break
            fi
            echo "# Waiting for slaves to be scanned..."
            sleep 2
            retry=$((retry - 1))
        done

        if [ $slaves_ok -eq 1 ]; then
            echo "# All masters ready and slaves detected."
            # 修改权限（根据需要）
            for dev in /dev/EtherCAT*; do
                $sudocmd chmod 777 "$dev" 2>/dev/null
            done
            return 0
        else
            echo "# ERROR: Masters created but no slaves found after scanning."
            return 1
        fi
    else
        echo "# ERROR: Timeout waiting for /dev/EtherCAT* devices (expected $expected_masters)."
        return 1
    fi
}
function kill_pid_list()
{
    pid_list="$1"
    for pid in $pid_list; do
        if [ "$pid"x == ""x -o "$pid"x == "1"x -o "$pid"x == "$$"x -o "$pid"x == "$PPID"x ]; then
            continue
        fi
        $sudocmd kill -TERM "$pid" 1>/dev/null 2>&1
    done
    sleep 1
    for pid in $pid_list; do
        if [ "$pid"x == ""x -o "$pid"x == "1"x -o "$pid"x == "$$"x -o "$pid"x == "$PPID"x ]; then
            continue
        fi
        $sudocmd kill -KILL "$pid" 1>/dev/null 2>&1
    done
}

function kill_ethercat_device_users()
{
    if [ ! -e /dev/EtherCAT0 ]; then
        return
    fi

    pids=""
    if command -v fuser 1>/dev/null 2>&1; then
        pids=$($sudocmd fuser /dev/EtherCAT0 2>/dev/null)
    elif command -v lsof 1>/dev/null 2>&1; then
        pids=$($sudocmd lsof -t /dev/EtherCAT0 2>/dev/null | sort -u)
    fi

    if [ "$pids"x != ""x ]; then
        echo "# kill_ethercat_device_users(): $pids"
        kill_pid_list "$pids"
    fi
}

function stop_ethercat_stack()
{
    echo "# stop_ethercat_stack() start"

    # 先停业务进程
    kill_process "ethercat_station"
    kill_process "$EMBED_FUSION_BIN_PATH/ethercat_station"

    # 再停所有可能占用主站设备的查询命令
    kill_process "watch -n 1 ethercat slaves"
    kill_process "ethercat slaves"
    kill_process "ethercat pdos"
    kill_process "ethercat xml"

    # 兜底：直接按命令行匹配清理
    $sudocmd pkill -f "watch -n 1 ethercat slaves" 1>/dev/null 2>&1
    $sudocmd pkill -f "$EMBED_FUSION_BIN_PATH/ethercat_station" 1>/dev/null 2>&1

    # 停主站服务
    # $sudocmd ethercatctl stop 1>/dev/null 2>&1
    # sleep 1

    # 若仍有设备占用，再清理一次
    kill_ethercat_device_users
    # $sudocmd ethercatctl stop 1>/dev/null 2>&1

    # 等待内核模块引用计数清零（避免 restart 时 rmmod ec_generic in use）
    # wait_sec=8
    # while [ $wait_sec -gt 0 ]; do
    #     ec_generic_use=$(lsmod | awk '/^ec_generic /{print $3}')
    #     if [ "$ec_generic_use"x == ""x -o "$ec_generic_use"x == "0"x ]; then
    #         break
    #     fi
    #     echo -ne "# waiting ec_generic release: use_count=$ec_generic_use, remain ${wait_sec}s \r"
    #     sleep 1
    #     wait_sec=$((wait_sec - 1))
    # done
    # echo ""
    echo "# stop_ethercat_stack() done"
}

function prepare_ethercat_orin()
{
    #方法1
    # if [ ! -f $state_file ]; then
    #     $sudocmd $EMBED_FUSION_BIN_PATH/ethercat_fusion_config.sh
    #     touch $state_file
    #     sleep 8
    # fi
    #方法2
    if [ ! -d /mnt/1 ]; then
        echo "prepare_env() 1" ; date
        $sudocmd mount -o bind /proc /mnt
        $sudocmd $EMBED_FUSION_BIN_PATH/ethercat_fusion_config.sh
        sleep 8
        echo "prepare_env() 2" ; date
        # 等待ethercat从站启动
        wait_ethercat_seconds=12
        while [ $wait_ethercat_seconds -gt 0 ]; do
            if [ -e /dev/EtherCAT0 ]; then
                break
            fi
            echo -ne "#       ethercat 主站 等待时间：$wait_ethercat_seconds 秒 \r"
            sleep 1
            wait_ethercat_seconds=$((wait_ethercat_seconds - 1))
        done
        echo "prepare_env() 3" ; date
        sleep 1
        ethercat slaves  # 查看状态
    fi

    # 无论是否执行过首次配置，都要确保主站设备存在
    ensure_ethercat_master_ready
}

function prepare_ethercat_with_port()
{
    port=$1
    ifconfig $port 1>/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "# 网卡 $port 配置 start"
        ensure_ethercat_master_ready
    else
        echo "# 网卡 $port 已配置"
    fi
}

function prepare_ethercat_x86()
{
    # 第一版本工控机
    # prepare_ethercat_with_port eno1
    # 第二版本工控机
    # prepare_ethercat_with_port enp8s0    

    # 工控机网口不确定，这里不做限制
    ensure_ethercat_master_ready
}

function prepare_env()
{
    prepare_ros_env

    # 检查并创建日志目录
    if [ ! -d "$log_dir" ]; then
        echo "创建日志目录: $log_dir"
        $sudocmd mkdir -p "$log_dir"
        $sudocmd chmod 755 "$log_dir"  # 设置适当权限
    else
        echo "日志目录已存在: $log_dir"
    fi

    $sudocmd rm -f /dev/shm/sem.shm_mc* /dev/shm/shm_mc

    if [ $IS_ORIN == true ]; then
        prepare_ethercat_orin
    elif [ $IS_X86 == true ]; then
        prepare_ethercat_x86
    fi
}

function print_slaves()
{
    while true; do
        echo "## ethercat slaves info ##"
        ethercat slaves
        sleep 1
    done
}

function monitor_windows()
{
    if [ "$background"x == "1"x ]; then
        return
    fi

    ps -o cmd= -p $PPID | grep run_ethercat_station.bash 1>/dev/null  # 是通过一键启动脚本启动的
    is_launcher=$?
    
    ps -ef | grep "watch -n 1 ethercat slaves" | grep -v grep 1>/dev/null
    monitor_is_running=$?
    if [ $monitor_is_running -eq 0 ]; then
        return 0
    fi

    if [ $is_launcher -eq 0 ]; then
        # print_slaves &
        xterm -T "# 1. Ethercat 基站 - 电驱状态" -e "watch -n 1 ethercat slaves" &  # 子窗口显示
    else
        xterm -T "# 1. Ethercat 基站 - 电驱状态" -e "watch -n 1 ethercat slaves" &  # 子窗口显示
    fi
}

function main()
{
    # 杀掉旧的运行脚本实例（保留当前实例）
    kill_process "run_ethercat_station.bash"

    # 完整停止主站与占用者，避免 ethercatctl restart 报 ec_generic in use
    stop_ethercat_stack

    if [ "$argv_1"x == "kill"x -o "$argv_1"x == "stop"x ]; then
        echo "# kill $argv_0"
        exit 0
    fi

    prepare_env

    # monitor_windows

    # ethercat_station 需要检查op等状态
    $sudocmd $sudoenv  $EMBED_FUSION_BIN_PATH/ethercat_station

    sleep 5  # 退出等待看错误信息
}

argv_0=$0
argv_1=$1
main
