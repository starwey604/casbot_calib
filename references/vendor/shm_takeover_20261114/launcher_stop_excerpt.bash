# Reference excerpts only; not an executable launcher.

function process_in_list()
{
    processes[0]="占位-用于从1开始"
    processes[1]="ethercat_station run_ethercat_station.bash"
    processes[2]="embed_station run_embed_station.bash"
    processes[3]="remote_cmd_bridge_node remote_cmd_bridge_scale_node task_cmd_bridge_node run_agv_ws_bridge.bash"
    processes[4]="arm_hand_control_node run_w1_control_realman.bash"
    processes[5]="hlorin run_hlmotion.bash"
    processes[6]="run_camera_rgbd_station.bash run_camera_orbbec_station.bash orbbec_camera_node"
    processes[7]="png_streamer_msg_server stereo_camera_node run_camera_stereo_rgb_station.bash"
    processes[8]="robot_interface_node voice_interface_node media_node run_prod_hru.bash"
    processes[9]="bt_executor fsm_executor casbot02_basic.launch.py run_prod_casbot02_basic_btexecutor.bash"
    processes[10]="hand_gesture_server run_prod_casbot02_basic_rockscissors.bash"
    processes[11]="shake_hand_node run_prod_casbot02_basic_shakehand.bash"
    processes[12]="casbot_human_perception_node run_prod_casbot02_basic_facetrack.bash"
    processes[13]="fisheyes_camera_node run_camera_fisheyes_station.bash"
    processes[14]="joystick_node run_joystick_station.bash"
    processes[15]="joystick_cmd_sub run_prod_casbot02_basic_joystick.bash"
    processes[16]="diag_station run_diagnostic.bash"
    processes[17]="crb_collector run_prod_casbot02_basic_collector.bash"
    processes[18]="crb_collector run_w1_crb_collector.bash"
    processes[19]="crb_motion_node run_w1_crb_motion.bash"
    processes[20]="crb_controller run_w1_crb_controller.bash"
    processes[21]="humanoid_nav_bridge_node run_prod_casbot02_basic_navbridge.bash"
    processes[22]="run_main_ctrl.bash"
    processes[23]="run_casbot_rslidar_station.bash"
    processes[24]="control_mux_node run_control_mux_station.bash"



    tmp_module_index=$1
    module_index=$((tmp_module_index+1))

    unkonw_process="未知进程名"
    if [ $module_index -ge 0 ] && [ $module_index -lt ${#processes[@]} ]; then
        echo ${processes[$module_index]}
    else
        return $unkonw_process
    fi
}


function stop_apps()
{
    input_array=()
    index_write_array=0
    # 初始化数组
    for ((i = 1; i <= module_num; i++)); do
        input_array+=("n")
    done

    if [ -f ${manual_run_flag_filename} ]; then
        echo "# 手动启动的程序，可选择停止"
        ask_one "$IS_RCU0" "stop" "run_ethercat_station.bash"            "$(module_name 1)"
        ask_one "$ORIN_HR" "stop" "run_embed_station.bash"               "$(module_name 2)"
        ask_one "$RCU0_W1" "stop" "run_agv_ws_bridge.bash"               "$(module_name 3)"
        ask_one "$RCU0_W1" "stop" "run_w1_control_realman.bash"          "$(module_name 4)"
        ask_one "$IS_RCU0" "stop" "run_hlmotion.bash"                    "$(module_name 5)"
        ask_one "$IS_RCU0" "stop" "run_camera_orbbec_station.bash"         "$(module_name 6)"
        ask_one "$IS_ORIN" "stop" "run_camera_stereo_rgb_station.bash"   "$(module_name 7)"
        ask_one "$IS_3588" "stop" "run_prod_hru.bash"                    "$(module_name 8)"
        ask_one "$ORIN_HM" "stop" "run_prod_casbot02_basic_btexecutor.bash"    "$(module_name 9)"
        ask_one "$ORIN_HM" "stop" "run_prod_casbot02_basic_rockscissors.bash"  "$(module_name 10)"
        ask_one "$ORIN_HM" "stop" "run_prod_casbot02_basic_shakehand.bash"     "$(module_name 11)"
        ask_one "$ORIN_HM" "stop" "run_prod_casbot02_basic_facetrack.bash"     "$(module_name 12)"
        ask_one "$IS_RCU0" "stop" "run_camera_fisheyes_station.bash"     "$(module_name 13)"
        ask_one "$IS_RCU0" "stop" "run_joystick_station.bash"            "$(module_name 14)"
        ask_one "$ORIN_HM" "stop" "run_prod_casbot02_basic_joystick.bash"      "$(module_name 15)"
        ask_one "$IS_RCU0" "stop" "run_diagnostic.bash"                  "$(module_name 16)"
        ask_one "$ORIN_HM" "stop" "run_prod_casbot02_basic_collector.bash"     "$(module_name 17)"
        ask_one "$RCU0_W1" "stop" "run_w1_crb_collector.bash"            "$(module_name 18)"
        ask_one "$RCU0_W1" "stop" "run_w1_crb_motion.bash"               "$(module_name 19)"
        ask_one "$RCU0_W1" "stop" "run_w1_crb_controller.bash"           "$(module_name 20)"
        ask_one "$ORIN_HM" "stop" "run_prod_casbot02_basic_navbridge.bash"     "$(module_name 21)"
        ask_one "$IS_RCU0" "stop" "run_main_ctrl.bash"                   "$(module_name 22)"
        ask_one "$IS_RCU0" "stop" "run_casbot_rslidar_station.bash"      "$(module_name 23)"
        ask_one "$IS_RCU0" "stop" "run_control_mux_station.bash"         "$(module_name 24)"


    else
        echo "# kill all 停止全部应用程序"   # 直接填充默认 y
        for ((i=0; i<$module_num; i++)); do
            input_array[$i]="y"
        done
    fi

    # 先杀掉监控进程
    kill_process "monitor_process.bash"

    # EtherCAT 基站必须走专用停止流程，确保 /dev/EtherCAT0 占用被释放
    need_stop_ethercat=${input_array[0]}
    if [ "$need_stop_ethercat" == "Y" ] || [ "$need_stop_ethercat" == "y" ]; then
        echo "# 1) 停止 EtherCAT：调用 run_ethercat_station.bash stop"
        if [ -f "$current_dir/run_ethercat_station.bash" ]; then
            bash "$current_dir/run_ethercat_station.bash" stop
        else
            echo "# WARN: $current_dir/run_ethercat_station.bash 不存在，回退到 kill_process"
            kill_process "ethercat_station"
            kill_process "run_ethercat_station.bash"
        fi
    fi
    
    # 遍历 input_array 和 processes 数组
    for i in "${!input_array[@]}"; do
        start_module=${input_array[$i]}
        if [ "$start_module" == "Y" ] || [ "$start_module" == "y" ]; then
            # EtherCAT 已执行专用停止逻辑，这里跳过重复处理
            if [ $i -eq 0 ]; then
                continue
            fi
            processes=$(process_in_list $i)
            echo "# $((i+1)) 停止进程 < ${processes} >"
            for process in ${processes}; do
                # 调用脚本 stop 子程序
                if [[ "$process" == *.bash ]] && [ -f "$current_dir/${process}" ]; then
                    if [ "${process}" == "run_ethercat_station.bash" ]; then
                        true    # 上面已经处理了，跳过
                    else
                        echo "# $((i+1)) 停止脚本 < ${process} >"
                        bash "$current_dir/${process}" stop
                    fi
                fi
                # echo "  # kill ${process}"
                kill_process "$process"
            done
        fi
    done
}


function auto_stop()
{
    # 杀掉应用程序
    stop_apps

    # 停止服务
    systemctl status hl_orin_app.service | grep 'active (running)' 1>/dev/null
    if [ $? -eq 0 ]; then
        echo "Service is running, stop..."
        systemctl stop hl_orin_app.service &
        sleep 1
    fi

    echo "Service stopped successfully."
    kill_process "launcher.bash" "set_cmd"
}
