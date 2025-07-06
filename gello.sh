#!/bin/bash

# ================== 配置区域 ==================
# 全局变量
ROOT_DIR="$HOME/data_save"
# 计算 ROOT_DIR 下的文件夹数量
if [ -d "$ROOT_DIR" ]; then
    TRAJ_NUMBER=$(find "$ROOT_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l)
else
    echo -e "${RED}错误：目录不存在 $ROOT_DIR${NC}"
    return 1
fi

PROJ_MAIN_DIR="$HOME/Prometheus"
GELLO_DIR="$HOME/Prometheus/gello_teleoperation"
ROBOTIQ_DIR="$HOME/Prometheus/Robotiq-Gripper"
RECORD_DIR="$HOME/Prometheus/xArm-Python-SDK/control"
CAMERA_DIR="$HOME/Prometheus/data_collection"
TACTILE_DIR="$HOME/Prometheus/data_collection"
TACTILE_CORE="$HOME/Prometheus/Tac3D/Tac3D-SDK-v3.3.0/Tac3D-Core/linux-x86_64/Tac3D"
TACTILE_CONFIG="$HOME/Prometheus/Tac3D/Tac3D-SDK-v3.3.0/Tac3D-Core/linux-x86_64/config"
TAC_SN1="DL1-GWM0001"
TAC_SN2="DL1-GWM0002"

# 命令配置
LAUNCH_CMD="python experiments/launch_nodes.py --robot xarm"
GELLO_OFFSET_CMD="python scripts/gello_get_offset.py"
GELLO_MAIN_CMD="python experiments/run_env.py --agent=gello"
ROBOTIQ_INIT_CMD="python gripper_init.py"
ROBOTIQ_CMD="python gripper_force_control_node.py"
TACTILE_PUB_CMD="python ./Robotiq-Gripper/tactile_pub.py"
RECORD_CMD="python ./gello_record_trajectory.py --root_dir=${ROOT_DIR} --traj_number=${TRAJ_NUMBER}"
CAMERA_CMD="python ./collect_rgbd.py --root_dir=${ROOT_DIR} --traj_number=${TRAJ_NUMBER}"
GRIPPRER_INFO_CMD="python ./save_gripper_info.py --root_dir=${ROOT_DIR} --traj_number=${TRAJ_NUMBER}"
TAC_SENSOR1_START="${TACTILE_CORE} -c ${TACTILE_CONFIG}/${TAC_SN1} -i 127.0.0.1 -p 9988"
TAC_SENSOR2_START="${TACTILE_CORE} -c ${TACTILE_CONFIG}/${TAC_SN2} -i 127.0.0.1 -p 9988"
ROS_CMD="roscore"

# Gello设备配置
GELLO_DEVICE_PATTERN="usb-FTDI_USB__-__Serial_Converter"
DEFAULT_GELLO_PORT="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAA08CH-if00-port0"

# 终端窗口管理
MAIN_TERMINAL_TITLE="Gello-xArm控制台"
DATA_RECORD_TITLE="数据录制"
TACTILE_TERMINAL_TITLE="触觉传感器控制台"
ROBOTIQ_TERMINAL_TITLE="Robotiq夹爪控制台"

# ================== 颜色定义 ==================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ================== 功能函数 ==================

# 查找Gello设备端口
find_gello_port() {
    local ports=($(ls /dev/serial/by-id/$GELLO_DEVICE_PATTERN* 2>/dev/null))
    
    if [ ${#ports[@]} -eq 0 ]; then
        echo -e "${YELLOW}警告：未检测到Gello设备，尝试使用默认端口${NC}"
        echo "$DEFAULT_GELLO_PORT"
    else
        echo "${ports[0]}"
    fi
}

# 初始化Gello设备
init_gello_device() {
    local port=$1
    
    echo -e "${BLUE}[初始化] 设置Gello设备权限...${NC}"
    if ! sudo chmod 777 "$port"; then
        echo -e "${RED}错误：无法设置设备权限${NC}"
        return 1
    fi
    
    echo -e "${BLUE}[初始化] 获取Gello偏移量...${NC}"
    if ! (cd "$GELLO_DIR" && $GELLO_OFFSET_CMD); then
        echo -e "${RED}错误：偏移量获取失败${NC}"
        return 1
    fi
    
    sleep 1
    return 0
}

# 关闭所有相关终端
close_all_terminals() {
    echo -e "${YELLOW}正在关闭所有终端窗口...${NC}"
    pkill -f "gnome-terminal.*$MAIN_TERMINAL_TITLE" 2>/dev/null
    pkill -f "gnome-terminal.*$DATA_RECORD_TITLE" 2>/dev/null
    pkill -f "gnome-terminal.*$TACTILE_TERMINAL_TITLE" 2>/dev/null
    pkill -f "gnome-terminal.*$ROBOTIQ_TERMINAL_TITLE" 2>/dev/null
    pkill -f "$LAUNCH_CMD" 2>/dev/null
    pkill -f "$GELLO_MAIN_CMD" 2>/dev/null
    pkill -f "$ROBOTIQ_CMD" 2>/dev/null
    pkill -f "$ROBOTIQ_INIT_CMD" 2>/dev/null
    pkill -f "$TACTILE_PUB_CMD" 2>/dev/null
    pkill -f "$CAMERA_CMD" 2>/dev/null
    pkill -f "$GRIPPRER_INFO_CMD" 2>/dev/null
    pkill -f "$TAC_SENSOR1_START" 2>/dev/null
    pkill -f "$TAC_SENSOR2_START" 2>/dev/null
    pkill -f "$RECORD_CMD" 2>/dev/null
    pkill -f "$ROS_CMD" 2>/dev/null
    sleep 1
}

# 启动触觉传感器终端（所有触觉程序在一个窗口的不同标签页）
start_tactile_terminal() {
    echo -e "${YELLOW}[步骤] 启动触觉传感器控制台...${NC}"
    
    # 关闭已存在的触觉终端
    pkill -f "gnome-terminal.*$TACTILE_TERMINAL_TITLE" 2>/dev/null
    sleep 0.5
    
    # 创建新终端窗口（第一个标签页 - 触觉发布程序）
    gnome-terminal --title="$TACTILE_TERMINAL_TITLE" --tab -- bash -c \
        "$ROS_CMD; exec bash"

    # 添加第二个标签页（触觉传感器1）
    gnome-terminal -- bash -c \
        "cd '$PROJ_MAIN_DIR' && echo -e '${YELLOW}启动触觉传感器1...${NC}' && $TAC_SENSOR1_START; exec bash"
    
    # 添加第三个标签页（触觉传感器2）
    gnome-terminal -- bash -c \
        "cd '$PROJ_MAIN_DIR' && echo -e '${YELLOW}启动触觉传感器2...${NC}' && $TAC_SENSOR2_START; exec bash"
        
    gnome-terminal --tab -- bash -c \
        "cd '$PROJ_MAIN_DIR' && echo -e '${YELLOW}启动触觉发布程序...${NC}' && $TACTILE_PUB_CMD; exec bash"
    
    sleep 0.5
    return 0
}

# 启动Robotiq控制终端（独立窗口）
start_robotiq_terminal() {
    echo -e "${YELLOW}[步骤] 启动Robotiq夹爪控制台...${NC}"
    
    # 关闭已存在的Robotiq终端
    pkill -f "gnome-terminal.*$ROBOTIQ_TERMINAL_TITLE" 2>/dev/null
    sleep 0.2

    gnome-terminal --title="$ROBOTIQ_TERMINAL_TITLE" --tab -- bash -c \
        "cd '$ROBOTIQ_DIR' && echo -e '${YELLOW}启动Robotiq控制...${NC}' && $ROBOTIQ_CMD; exec bash" &
    
    sleep 0.5
    return 0
}

# 在主终端中运行命令（作为标签页）
run_in_main_terminal() {
    local tab_name=$1
    local cmd=$2
    local dir=$3
    
    echo -e "${YELLOW}[步骤] ${BLUE}${tab_name}...${NC}"
    echo -e "执行命令: ${GREEN}${cmd}${NC}"
    echo -e "工作目录: ${dir}"
    
    if [ ! -d "$dir" ]; then
        echo -e "${RED}错误：目录不存在 $dir${NC}"
        return 1
    fi
    
    # 检查主终端是否已存在
    existing_pid=$(ps aux | grep "gnome-terminal.*$MAIN_TERMINAL_TITLE" | grep -v grep | awk '{print $2}')

    if [ -z "$existing_pid" ]; then
        # 创建新主终端（第一个标签页）
        gnome-terminal --title="$MAIN_TERMINAL_TITLE" --tab -- bash -c \
            "cd '$dir' && echo -e '${YELLOW}启动: ${tab_name}${NC}' && $cmd; exec bash"
    else
        # 向现有主终端添加新标签页
        gnome-terminal --tab -- bash -c \
            "cd '$dir' && echo -e '${YELLOW}启动: ${tab_name}${NC}' && $cmd; exec bash"
    fi
    sleep 0.5
    return 0
}

# 在独立终端中运行数据录制命令
run_data_record() {
    echo -e "${YELLOW}[步骤] 数据录制...${NC}"

    TRAJ_NUMBER=$(find "$ROOT_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l)
    RECORD_CMD="python ./gello_record_trajectory.py --root_dir=${ROOT_DIR} --traj_number=${TRAJ_NUMBER}"
    CAMERA_CMD="python ./collect_rgbd.py --root_dir=${ROOT_DIR} --traj_number=${TRAJ_NUMBER}"
    GRIPPRER_INFO_CMD="python ./save_gripper_info.py --root_dir=${ROOT_DIR} --traj_number=${TRAJ_NUMBER}"
    
    # 关闭已存在的数据录制终端
    pkill -f "gnome-terminal.*$DATA_RECORD_TITLE" 2>/dev/null
    sleep 0.2

    
    # 启动相机采集（同一终端的新标签页）
    if [ ! -d "$CAMERA_DIR" ]; then
        echo -e "${RED}错误：目录不存在 $CAMERA_DIR${NC}"
        return 1
    fi
    gnome-terminal --tab -- bash -c \
        "cd '$CAMERA_DIR' && echo -e '${YELLOW}执行相机采集...${NC}' && $CAMERA_CMD; exec bash" &
    
    # 启动触觉采集（同一终端的新标签页）
    if [ ! -d "$TACTILE_DIR" ]; then
        echo -e "${RED}错误：目录不存在 $TACTILE_DIR${NC}"
        return 1
    fi
    gnome-terminal --title="$DATA_RECORD_TITLE"  -- bash -c \
        "cd '$TACTILE_DIR' && echo -e '${YELLOW}执行触觉采集...${NC}' && setsid $GRIPPRER_INFO_CMD & wait \$!; exec bash" &

    # 启动轨迹录制（新终端窗口）
    if [ ! -d "$RECORD_DIR" ]; then
        echo -e "${RED}错误：目录不存在 $RECORD_DIR${NC}"
        return 1
    fi
    gnome-terminal --title="$DATA_RECORD_TITLE" -- bash -c \
        "cd '$RECORD_DIR' && echo -e '${YELLOW}执行轨迹录制...${NC}' && setsid $RECORD_CMD & wait \$!; exec bash" &
    
    sleep 0.5
    return 0
}

stop_data_record(){
    echo "Stopping all data collection processes..."
    pkill -f "$RECORD_CMD"
    sleep 3
    pkill -f "$GRIPPRER_INFO_CMD"
    sleep 3
    pkill -f "$CAMERA_CMD"

    echo "Data collection stopped."
}

# ================== 主菜单 ==================
show_menu() {
    clear
    echo -e "${GREEN}=== Gello-xArm-Dexhand 分步控制系统 ===${NC}"
    echo -e "1. 启动 xArm"
    echo -e "2. 启动 Gello 遥操作"
    echo -e "3. 启动触觉传感器控制台"
    echo -e "4. 启动 Robotiq 控制台"
    echo -e "5. 顺序执行所有步骤"
    echo -e "6. 开始录制数据"
    echo -e "7. 停止录制数据"
    echo -e "0. 退出并关闭所有终端"
    echo -n "请选择: "
}

# ================== 步骤执行 ==================
execute_step() {
    case $1 in
        1) run_in_main_terminal "xArm仿真" "$LAUNCH_CMD" "$GELLO_DIR" ;;
        2)
            port=$(find_gello_port)
            echo -e "${BLUE}检测到Gello端口: ${GREEN}$port${NC}"
            
            if ! init_gello_device "$port"; then
                echo -e "${RED}Gello初始化失败，无法继续${NC}"
                return 1
            fi
            
            run_in_main_terminal "Gello遥操作" "$GELLO_MAIN_CMD --gello_port=$port" "$GELLO_DIR"
            ;;
        3) start_tactile_terminal ;;
        4) start_robotiq_terminal ;;
        5)
            run_in_main_terminal "xArm仿真" "$LAUNCH_CMD" "$GELLO_DIR"
            sleep 2
            
            port=$(find_gello_port)
            echo -e "${BLUE}检测到Gello端口: ${GREEN}$port${NC}"
            
            init_gello_device "$port"
            sleep 1
            
            run_in_main_terminal "Gello遥操作" "$GELLO_MAIN_CMD --gello_port=$port" "$GELLO_DIR"
            run_in_main_terminal "夹爪激活" "$ROBOTIQ_INIT_CMD" "$ROBOTIQ_DIR"
            sleep 5
            start_tactile_terminal
            start_robotiq_terminal
            ;;
        6) run_data_record ;;
        7) stop_data_record ;;
        0) 
            close_all_terminals
            echo -e "${GREEN}系统已安全退出${NC}"
            exit 0 
            ;;
        *) echo -e "${RED}无效输入，请重试${NC}"; sleep 1 ;;
    esac
}

# ================== 主循环 ==================
while true; do
    show_menu
    read -r choice
    case $choice in
        [1-7]) execute_step "$choice" ;;
        0) execute_step 0 ;;
        *) echo -e "${RED}无效输入，请重试${NC}"; sleep 1 ;;
    esac
    echo -e "${YELLOW}按回车继续...${NC}"
    read -r
done