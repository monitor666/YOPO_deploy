#!/bin/bash
# ============================================================================
# YOPO 传感器快速检查脚本
# ============================================================================
#
# 使用方法:
#   ./quick_check.sh           # 检查所有传感器
#   ./quick_check.sh camera    # 只检查相机
#   ./quick_check.sh imu       # 只检查 IMU
#
# ============================================================================

GREEN='\033[92m'
RED='\033[91m'
YELLOW='\033[93m'
RESET='\033[0m'
BOLD='\033[1m'

echo ""
echo "=============================================="
echo "       YOPO 传感器快速检查"
echo "=============================================="

# 检查 ROS Master
echo ""
echo -n "🔍 检查 ROS Master... "
if rostopic list &> /dev/null; then
    echo -e "${GREEN}✅ 正常${RESET}"
else
    echo -e "${RED}❌ 未运行${RESET}"
    echo "   请先运行: roscore"
    exit 1
fi

# 检查函数
check_topic() {
    local topic=$1
    local name=$2
    local timeout=${3:-2}
    
    echo -n "  检查 $name... "
    
    # 检查话题是否存在
    if ! rostopic info "$topic" &> /dev/null; then
        echo -e "${RED}❌ 话题不存在${RESET}"
        return 1
    fi
    
    # 检查是否有数据
    local result
    result=$(timeout "$timeout" rostopic hz "$topic" 2>&1 | head -3)
    
    if echo "$result" | grep -q "average rate"; then
        local freq
        freq=$(echo "$result" | grep "average rate" | awk '{print $3}')
        echo -e "${GREEN}✅ ${freq} Hz${RESET}"
        return 0
    else
        echo -e "${YELLOW}⚠️  无数据${RESET}"
        return 1
    fi
}

# 相机检查
check_camera() {
    echo ""
    echo -e "${BOLD}📷 RealSense 相机${RESET}"
    echo "----------------------------------------------"
    
    local camera_ok=0
    
    check_topic "/camera/depth/image_rect_raw" "深度图像" 3 && ((camera_ok++))
    check_topic "/camera/infra1/image_rect_raw" "红外图像 1" 3 && ((camera_ok++))
    check_topic "/camera/infra2/image_rect_raw" "红外图像 2" 3 && ((camera_ok++))
    
    if [ $camera_ok -eq 3 ]; then
        echo -e "  ${GREEN}相机状态: 全部正常 ✅${RESET}"
        return 0
    elif [ $camera_ok -gt 0 ]; then
        echo -e "  ${YELLOW}相机状态: 部分正常 ⚠️${RESET}"
        return 1
    else
        echo -e "  ${RED}相机状态: 异常 ❌${RESET}"
        echo "  提示: roslaunch realsense2_camera yopo_d455f_camera.launch"
        return 1
    fi
}

# IMU 检查
check_imu() {
    echo ""
    echo -e "${BOLD}🎯 MAVROS IMU${RESET}"
    echo "----------------------------------------------"
    
    local imu_ok=0
    
    check_topic "/mavros/imu/data_raw" "IMU 原始数据" 3 && ((imu_ok++))
    check_topic "/mavros/imu/data" "IMU 滤波数据" 3 && ((imu_ok++))
    
    if [ $imu_ok -ge 1 ]; then
        echo -e "  ${GREEN}IMU 状态: 正常 ✅${RESET}"
        return 0
    else
        echo -e "  ${RED}IMU 状态: 异常 ❌${RESET}"
        echo "  提示: roslaunch mavros px4_yopo.launch"
        return 1
    fi
}

# 列出所有相关话题
list_topics() {
    echo ""
    echo -e "${BOLD}📋 当前活跃的传感器话题${RESET}"
    echo "----------------------------------------------"
    
    echo "相机话题:"
    rostopic list 2>/dev/null | grep -E "^/camera" | head -20 | sed 's/^/  /'
    
    echo ""
    echo "MAVROS 话题:"
    rostopic list 2>/dev/null | grep -E "^/mavros/imu" | sed 's/^/  /'
}

# 主逻辑
case "${1:-all}" in
    camera)
        check_camera
        ;;
    imu)
        check_imu
        ;;
    list)
        list_topics
        ;;
    all|*)
        check_camera
        camera_result=$?
        
        check_imu
        imu_result=$?
        
        echo ""
        echo "=============================================="
        echo "📋 总结"
        echo "----------------------------------------------"
        
        if [ $camera_result -eq 0 ] && [ $imu_result -eq 0 ]; then
            echo -e "${GREEN}✅ 所有传感器正常！${RESET}"
            exit 0
        else
            echo -e "${YELLOW}⚠️  部分传感器异常，请检查上方详情${RESET}"
            exit 1
        fi
        ;;
esac

echo ""
