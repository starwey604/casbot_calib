#!/bin/bash

current_dir=$(cd $(dirname $0); pwd)
root_dir=$(cd $current_dir/; pwd)


export is_ubuntu_20_04=$(grep -q 'DISTRIB_RELEASE=20.04' /etc/lsb-release && echo "true" || echo "false")
export is_ubuntu_22_04=$(grep -q 'DISTRIB_RELEASE=22.04' /etc/lsb-release && echo "true" || echo "false")
# echo "# is_ubuntu_20_04: $is_ubuntu_20_04"

if [ $(uname -m) == "x86_64" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    export sudocmd="sudo "
else
    if [ "$is_ubuntu_22_04" == "true" ]; then
        export sudocmd="sudo "
        source /workspace/embed_fusion/setup.bash
        source /opt/ros/humble/setup.bash
    fi
fi

export sudoenv="-E LD_LIBRARY_PATH=$LD_LIBRARY_PATH -E PATH=$PATH "


function kill_process()
{
    process_name=$1
    ignore_field=$2
    care_field_a=$3
    care_field_b=$4
    if [[ "$process_name" =~ ".bash" ]]; then
        # 是要杀掉脚本文件, 需要排除掉当前脚本启动的 sudo xxx 进程
        exclude_pppid=$(ps -o ppid= -p $PPID)
        exclude_str="nnnnnnnn_ignore_nnnnnnnnnnnnnn"
    else
        exclude_pppid="nnnnnnnn_ignore_nnnnnnnnnnnnnn"
        exclude_str=".bash"
    fi

    if [ "$process_name"x == "monitor_process.bash"x ]; then
        exclude_sec_str="nnnnnnnn_ignore_nnnnnnnnnnnnnn"
    else
        exclude_sec_str="monitor_process.bash"
    fi

    if [[ "$ignore_field"x == ""x ]]; then
        ignore_field="nnnnnnnn_ignore_nnnnnnnnnnnnnn"
    fi
    
    # process_desc=$(ps -ef | grep "$1" | grep -v grep)
    # echo -e "process_desc($process_name):<\n$process_desc \n>"
    # echo "now: $$  $PPID exclude_pppid:$exclude_pppid exclude_str:$exclude_str"
    
    # 测试打印
    # ps -ef | grep "$1" | grep -v grep | grep -v "$exclude_str" | awk '{print $2}' | grep -v $$ | grep -v $PPID | grep -v $exclude_pppid
    
    current_pid=$$
    current_ppid=$PPID
    # 过滤当前进程ID、父进程ID
    # process_ids=
    process_ids=$(ps -ef | grep "$process_name" \
                         | grep "$care_field_a" | grep "$care_field_b" \
                         | grep -v grep | grep -v "$exclude_str" | grep -v "$exclude_sec_str" | grep -v "$ignore_field" \
                         | awk '{print $2}' | grep -v $current_pid | grep -v $current_ppid | grep -v $exclude_pppid )
    # echo -e "  @@@ process_ids: <$process_ids > \n"
    if [ "$process_ids"x == ""x ]; then
        return 0
    fi

    while IFS= read -r pid; do
        # 对每个 PID 执行 kill 命令
        if [ "$pid"x == ""x ]; then
            continue
        elif [ "$pid"x == "$current_pid"x ] || [ "$pid"x == "$current_ppid"x ] || [ "$pid"x == "$exclude_pppid"x ] ; then
            continue
        elif [ "$pid"x == "1"x ]; then
            continue
        fi
        # echo "# kill_process() kill -9 $pid for process_name:$process_name"
        $sudocmd kill -9 "$pid"  1>/dev/null 2>&1
    done <<< "$process_ids"
}

function print_and_exit_if_ret()
{
    ret=$1
    msg=$2
    desc=${@:3}
    if [ $ret -ne 0 ]; then
        echo "########################################### "
        echo "# $msg() ${desc} fail !!!!!! "
        echo "########################################### "
        sleep 5 ; exit 1   # 直接退出
    fi
}

function mkdir_directory()
{
    dir=$1
    $sudocmd mkdir -p $dir
    $sudocmd chmod a+w $dir
    $sudocmd chmod a+r $dir
}

function remove_large_log()
{
    log_file=( "/var/log/auth.log" "/var/log/syslog" "/var/log/kern.log" "/var/log/dpkg.log" )
    max_size=4096000
    for file in ${log_file[@]}; do
        if [ -f $file ]; then
            log_size=$(du -sb $file | awk '{print $1}')
            if [ $log_size -gt $max_size ]; then
                echo "# remove_large_log() rm $file (size:$log_size)"
                echo '# delete' | $sudocmd tee $file > /dev/null
            fi
        fi
    done
}

function copy_if_changed()
{
    local src="$1"
    local dst="$2"

    if [ ! -f "$dst" ] || ! cmp -s "$src" "$dst"; then
        cp "$src" "$dst"
    fi
}

function prepare_udev_env()
{
    qingsheng_src_rules="/workspace/embed_fusion/config/udev/99-qingsheng-ttyacm.rules"
    qingsheng_dst_rules="/etc/udev/rules.d/99-qingsheng-ttyacm.rules"
    copy_if_changed $qingsheng_src_rules $qingsheng_dst_rules

    hand_src_rules="/workspace/embed_fusion/config/udev/99-hand.rules"
    hand_dst_rules="/etc/udev/rules.d/99-hand.rules"
    copy_if_changed $hand_src_rules $hand_dst_rules

    obsensor_src_rules="/workspace/embed_fusion/share/orbbec_camera/udev/99-obsensor-libusb.rules"
    obsensor_dst_rules="/etc/udev/rules.d/99-obsensor-libusb.rules"
    copy_if_changed $obsensor_src_rules $obsensor_dst_rules
}

function prepare_etc_hosts_env()
{
    host_name=$(hostname)
    check_str=$(cat /etc/hosts | grep "^127.0.1.1" | grep ${host_name})
    if [ "$check_str"x == ""x ]; then
        echo "# prepare_etc_hosts_env() add "127.0.1.1 $host_name" to /etc/hosts"
        echo "127.0.1.1 $host_name" | tee -a /etc/hosts > /dev/null
    fi
}

function prepare_isolation_aarch64_cpu_core_env()
{
    if [ $IS_ORIN == true ]; then
        # 检查并添加三个参数
        config="/boot/extlinux/extlinux.conf"
        params=("isolcpus" "irqaffinity" "rcu_nocbs")
        values=("4,5" "0,1,2,3,6,7,8,9,10,11" "4,5")
        modified=0

        for i in "${!params[@]}"; do
            if [[ $(grep -c "APPEND.*\<${params[i]}=" "$config") -eq 0 ]]; then
                sed -i "s|\(^[[:space:]]*APPEND.*\)|\1 ${params[i]}=${values[i]}|" "$config"
                echo "prepare_isolation_aarch64_cpu_core_env() add ${params[i]}=${values[i]} to $config"
                modified=1
            fi
        done
        if [[ $modified -eq 1 ]]; then
            sync
            echo "prepare_isolation_aarch64_cpu_core_env() 配置已修改，执行 sync"
        fi

        # 设置中断运行核心
        irq=`cat /proc/interrupts | grep eqos_0.vm0 | awk '{print $1}' | tr -d ':'`
        echo "prepare_isolation_aarch64_cpu_core_env() 设置中断运行核心"
        echo 4,5 | tee /proc/irq/${irq}/smp_affinity_list
    fi
}

function prepare_isolation_x86_cpu_core_env()
{
    if [ $IS_X86 == true ]; then
        # 检查并添加 x86 grub 参数
        config="/etc/default/grub"
        params=("isolcpus" "nohz_full" "rcu_nocbs" "irqaffinity" "intel_idle.max_cstate" "processor.max_cstate" "transparent_hugepage" "skew_tick")
        values=("4,5" "4,5" "4,5" "0-3,6-17" "1" "1" "never" "1")
        modified=0

        if [[ $(grep -c "^GRUB_CMDLINE_LINUX_DEFAULT=" "$config") -eq 0 ]]; then
            echo 'GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"' >> "$config"
            modified=1
        fi

        for i in "${!params[@]}"; do
            if [[ $(grep -c "^GRUB_CMDLINE_LINUX_DEFAULT=.*\<${params[i]}=" "$config") -eq 0 ]]; then
                sed -i "/^GRUB_CMDLINE_LINUX_DEFAULT=/ s|\"$| ${params[i]}=${values[i]}\"|" "$config"
                echo "prepare_isolation_x86_cpu_core_env() add ${params[i]}=${values[i]} to $config"
                modified=1
            fi
        done
        if [[ $modified -eq 1 ]]; then
            $sudocmd update-grub
            sync
            echo "prepare_isolation_x86_cpu_core_env() 配置已修改，已执行 update-grub 和 sync"
        fi

        # 设置中断运行核心
        irq=`cat /proc/interrupts | grep eqos_0.vm0 | awk '{print $1}' | tr -d ':'`
        echo "prepare_isolation_x86_cpu_core_env() 设置中断运行核心"
        echo 4,5 | tee /proc/irq/${irq}/smp_affinity_list
    fi
}
function print_etc_hosts_env()
{
    host_name=$(hostname)
    check_str=$(cat /etc/hosts | grep "^127.0.1.1" | grep ${host_name})
    if [ "$check_str"x == ""x ]; then
        echo "# print_etc_hosts_env() error!! please add "127.0.1.1 $host_name" to /etc/hosts"
    fi
}

function prepare_first_reboot_env()
{
    # 开机只执行一次
    first_reboot_flag="/tmp/launcher/launcher_first_reboot.flag"
    if [ -f $first_reboot_flag ]; then
        return
    fi
    touch $first_reboot_flag

    # 删除ros日志
    rm -rf /root/.ros/log/*
    rm -rf /home/casbot/.ros/log/*
    rm -rf /home/nvidia/.ros/log/*
    # 关闭 gdm3 服务，CPU占用约35%
    $sudocmd systemctl stop gdm3
}

function prepare_disk_env()
{
    # 判断空间占用
    root_use_percent=$(df -h / |  awk 'NR==2 {print $5}' | sed 's/%//')
    if [ $root_use_percent -gt 95 ]; then
        echo "# "
        echo "# root_use_percent(使用率):$root_use_percent"
        echo "# / 磁盘空间快满了，请先清理磁盘空间"
        echo "# "
        exit 1
    fi
}

function prepare_target_board_env()
{
    export IS_TRUE=true
    export IS_FALS=false
    export IS_ORIN=false
    export IS_3588=false
    export IS_X86=false
    export IS_W1=false
    export IS_RCU0=false
    export IS_N305=false
    export IS_ULTRA5=false
    lspci  | grep Rockchip | grep 3588 1>/dev/null
    if [ $? -eq 0 ]; then
        export IS_3588=true
    fi
    # 新版本系统没有lspci的标志，但是有/rockchip_test
    ls /  | grep -i rockchip 1>/dev/null 2>&1
    if [ $? -eq 0 ]; then
        export IS_3588=true
    fi

    lspci  | grep 'PCI bridge: NVIDIA Corporation Device' 1>/dev/null
    if [ $? -eq 0 ]; then
        export IS_ORIN=true
    fi

    cat /proc/cpuinfo | grep -i 'model name' | grep -i 'i3-N305' > /dev/null
    if [ $? -eq 0 ]; then
        export IS_N305=true
    fi
    cat /proc/cpuinfo | grep -i 'model name' | grep -i 'Ultra 5' > /dev/null
    if [ $? -eq 0 ]; then
        export IS_N305=true
        export IS_ULTRA5=true
    fi

    if [ $(uname -m) == "x86_64" ]; then
        export IS_X86=true
    fi

    if [ "$ROBOT_ID"x == ""x ]; then
        robotid_env="/workspace/embed_fusion/bin/factory_tool/robot_id/robotid_source.env"
        if [ -f $robotid_env ]; then
            source $robotid_env
        fi
    fi

    # W1/W2
    model_line=$(awk '/model_info:/{f=1;next}/^[^[:space:]]/{f=0} f&&/model: "/' /workspace/global_robot.yaml)   # 第一步 awk 获取 model_info 下完整model行
    robot_model=$(echo "$model_line" | grep -o '"[^"]*"' | tr -d '"')   # 第二步 grep 提取双引号中间内容
    model_low=${robot_model,,}  # 转小写统一匹配
    # echo "prepare_target_board_env() robot_model:$robot_model model_low:$model_low"
    if [[ $model_low == "w1" ]]; then
        export IS_W1=true
    elif [[ -f /workspace/robot_w1.mark ]]; then
        export IS_W1=true
    elif [[ $model_low == "w2" ]]; then
        export IS_W1=true
    else
        export IS_W1=false
    fi

    # 组合，ORIN_HM 人形 ， ORIN_W1 轮式
    export ORIN_HM=false
    export ORIN_W1=false
    export RCU0_W1=false
    if [ $IS_ORIN == true -a $IS_W1 == true ]; then
        export ORIN_W1=true
        export RCU0_W1=true
    elif [ $IS_X86 == true -a $IS_W1 == true ]; then
        export RCU0_W1=true
    elif [ $IS_ORIN == true -a $IS_W1 == false ]; then
        export ORIN_HM=true
    fi

    # 组合，RCU板，包含 orin x86
    if [ $IS_ORIN == true -o $IS_X86 == true ]; then
        export IS_RCU0=true
    fi

    # 组合，orin hru
    export ORIN_HR=false
    if [ $IS_X86 == false ]; then
        export ORIN_HR=true
    fi

    # OEM 标记
    export IS_OEM_CCTV=false
    if [ -f /workspace/robot_oem_cctv.mark ]; then
        export IS_OEM_CCTV=true
    fi
}

function prepare_base_env()
{
    prepare_target_board_env
    print_etc_hosts_env
    
    opt_dir="/opt/humble/bin/"
    file_bash="change_thread_proirity.bash"
    if [ ! -f $opt_dir/$file_bash ]; then
        mkdir_directory $opt_dir
        ln -sf $current_dir/$file_bash  $opt_dir/$file_bash
        chmod 777 $opt_dir/$file_bash
    fi

    # 开机自启动，这个环境变量没有
    if [ "$HOME"x == ""x ]; then
        export HOME=/root
    fi

    mkdir_directory "/tmp/launcher"
    mkdir_directory "/tmp/launcher/tmp"
    mkdir_directory "/var/log/robot/current"
    remove_large_log
    prepare_udev_env
    prepare_first_reboot_env
    prepare_disk_env
}

function prepare_mcu_env()
{
    prepare_base_env

    mkdir_directory "/tmp/embed_fusion"
    mkdir_directory "/tmp/embed_fusion/control"
    mkdir_directory "/tmp/embed_fusion/control/mcu"
    mkdir_directory "/tmp/embed_fusion/control/mcu/old"
    mkdir_directory "/tmp/embed_fusion/log"
    mkdir_directory "/tmp/embed_fusion/log/mcu"

    $sudocmd chmod a+w "/workspace/embed_fusion/firware/ota_files/"

    $sudocmd chmod 777 /dev/spidev0.0 2>/dev/null
}

function prepare_ros_env()
{
    prepare_base_env
    
    orin_hru_username=nvidia
    if id casbot &> /dev/null; then
    	orin_hru_username=casbot
    fi

    mkdir_directory "/tmp/.ros"
    mkdir_directory "/tmp/.ros/log"
    mkdir_directory "/home/${orin_hru_username}/.ros"
    mkdir_directory "/home/${orin_hru_username}/.ros/log"

    # 承接 root 用户的 domainid
    whoami=$(whoami)
    if [ $whoami == "root" ]; then
        # set_domainid_cmd=$(cat /home/${orin_hru_username}/.bashrc | grep export | grep ROS_DOMAIN_ID)
        set_domainid_cmd=$(cat /root/.bashrc | grep export | grep ROS_DOMAIN_ID)
        $set_domainid_cmd
        echo "prepare_ros_env() root $set_domainid_cmd"
    fi

    # 开机自启动的情况下，HOME 环境变量为空，则设置为 /root
    if [ "$HOME"x == ""x ]; then
        export HOME=/root
    fi
}

function prepare_imu_env()
{
    $sudocmd chmod 777 /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>/dev/null
}

function prepare_conda_env()
{
    env_name=$1
    export PATH="/opt/miniconda3/bin:$PATH"
    source /opt/miniconda3/etc/profile.d/conda.sh
    conda init
    if [ "$env_name"x != ""x ]; then
        conda activate $env_name
        if [ $? -eq 0 ]; then
            export PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH"
            echo "# prepare_conda_env() PYTHONPATH: $PYTHONPATH"
        fi
    fi
    echo "# prepare_conda_env() conda env list:$(conda env list)"
    echo "# prepare_conda_env() python:$(which python)"
}

function check_need_ssh_x()
{
    if [ $(uname -m) == "x86_64" ]; then
        return 0
    fi

    if [ "$DISPLAY"x == ""x ]; then
        echo ""
        echo "DISPLAY=    # is not set"
        echo "# Please ssh nvidia@***** -> ssh -X nvidia@***** , add -X "
        echo "# 请 ssh nvidia@***** 替换成 ssh -X nvidia@***** , 加上 -X "
        echo ""
        echo "# 退出 in check_need_ssh_x()"
        echo ""
        sleep 5 ; exit 1
    fi
}

function check_need_pip_package()
{
    package=$1
    version=$2
    if [ "$version"x == ""x ]; then
        pip3 list | grep $package 1>/dev/null
    else
        pip3 list | grep $package | grep "$version"  1>/dev/null
    fi
    ret=$?

    if [ $ret -ne 0 ]; then
        echo ""
        echo "# pip3 package:$package is not installed !!!!!!"
        echo "# sudo pip3 install $package -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com "
        echo "# 请按照以上命令安装"
        echo ""
        echo "# 退出 in check_need_pip_package()"
        echo ""
        sleep 5 ; exit 1
    fi
}

# 判断进程是否运行的函数，使用ps命令
function is_process_running()
{
    local process_name="$1"
    local process_count=$(ps -ef | grep -v grep | grep "$process_name" | wc -l)
    if [ $process_count -gt 0 ]; then
        if [ "$PROCESS_SUMMARY_RUNNING"x != ""x ]; then
            PROCESS_SUMMARY_RUNNING="${PROCESS_SUMMARY_RUNNING} , "
        fi
        PROCESS_SUMMARY_RUNNING="${PROCESS_SUMMARY_RUNNING}${process_name}"
        return 0
    else
        if [ "$PROCESS_SUMMARY_STOPPED"x != ""x ]; then
            PROCESS_SUMMARY_STOPPED="${PROCESS_SUMMARY_STOPPED} , "
        fi
        PROCESS_SUMMARY_STOPPED="${PROCESS_SUMMARY_STOPPED}${process_name}"
        return 1
    fi
}

function unset_process_summary()
{
    PROCESS_SUMMARY_RUNNING=""
    PROCESS_SUMMARY_STOPPED=""
}

function print_process_summary()
{
    local green_color='\033[32m'
    local red_color='\033[31m'
    local reset_color='\033[0m'
    echo -e "${green_color}正在运行:${PROCESS_SUMMARY_RUNNING} ${reset_color} \r\n"
    echo -e "${red_color}未在运行:${PROCESS_SUMMARY_STOPPED} ${reset_color} \r\n"
}

function publish_launcher_status()
{
    msg=$1
    if [ "$msg"x == ""x ]; then
        echo "publish_launcher_status() msg is null"
        return 1
    fi

    if [ "$IS_ORIN"x == "true"x ]; then
        timeout 2 ros2 topic pub /hru/embed_fusion/rcu_launcher_status std_msgs/msg/String "{data: '$msg'}" --once
    fi
}

function sleep_func()
{
    sleep_seconds=$1
    if [ "$sleep_seconds"x == ""x ]; then
        sleep_seconds=1
    fi
    total_seconds=$sleep_seconds

    while [ $sleep_seconds -gt 0 ]; do
        echo "## wait for $sleep_seconds (total $total_seconds) seconds ... "
        sleep 1
        let sleep_seconds=sleep_seconds-1
    done
}

function wait_and_check_network_after_embed_station()
{
    return 0   # 不用这里的判断了
    
    if [ $IS_3588 == true ]; then
        echo "# wait_and_check_network_after_embed_station"
        date ; cat /proc/uptime
        myppid=$PPID
        if [ $myppid -eq 1 ]; then

            # 测试
            # 计算掩码：CPU0-3 -> 二进制 1111 -> 十六进制 f
            # sh -c "echo f > /sys/class/net/eth0/queues/rx-0/rps_cpus"
            # 增大网络设备积压队列（默认通常为1000）
            sysctl -w net.core.netdev_max_backlog=8192
            # 关闭网卡的GRO（巨型帧接收，千兆网卡GRO会导致FIFO溢出，RK板卡禁用必好）
            sudo ethtool -K eth0 gro off
            # 增大socket读取缓冲区
            # sysctl -w net.core.rmem_default=262144
            # sysctl -w net.core.rmem_max=16777216
            # 增大网络设备接收缓存
            ethtool -G eth0 rx 1024
            # 启用RX方向流控（如果支持）
            sudo ethtool -A eth0 rx on
            echo "wait_and_check_network_after_embed_station sysctl ethtool set done"


            # HRU 开机启动会概率性出现eth0端口异常的问题
            echo "# wait_and_check_network_after_embed_station waiting..."
            sleep_func 70
            date ; cat /proc/uptime
            echo "# wait_and_check_network_after_embed_station restart network eth0 ..."
            # 最近的设备会出现有线网卡overflow，重启网卡才能正常工作
            bash -c "ethtool -S eth0 | grep -E 'rx_pkt_n|rx_missed|overflow|error'"   # 打印
            ifconfig eth0 down
            sleep 1
            ifconfig eth0 up

            return 0   # 直接重启网卡后返回

            TARGET_IP="172.16.0.10"
            NETWORK_INTERFACE="eth0"
            ping -c 1 -W 1 $TARGET_IP > /dev/null 2>&1  # -W 2：超时2秒，避免等待过久
            date ; cat /proc/uptime
            if [ $? -ne 0 ]; then
                echo "# launcher.bash before_main ping $TARGET_IP faild, reset network $NETWORK_INTERFACE"
                # 重启网卡
                ethtool -r $NETWORK_INTERFACE
            else
                echo "# launcher.bash before_main ping $TARGET_IP success"
            fi
        fi
    fi
}

