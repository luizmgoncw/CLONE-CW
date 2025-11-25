#!/bin/bash

CLONE_DIR="/home/luizmarques/Documents/CLONE"
cd "$CLONE_DIR" || exit 1

# 1. Source ROS2 Humble
echo "📦 Loading ROS2 Humble..."
source /opt/ros/humble/setup.bash

# 2. Source Unitree ROS2 workspace
echo "📦 Loading Unitree ROS2..."
source "$CLONE_DIR/unitree_ros2/install/setup.bash"

echo "=== ENVIRONMENT DIAGNOSTICS ==="
echo ""

echo "1. Python Path:"
which python
python --version
echo ""

echo "2. ROS2 Environment:"
printenv | grep ROS
echo ""

echo "3. CycloneDDS Config:"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI set: $([ -n "$CYCLONEDDS_URI" ] && echo 'YES' || echo 'NO')"
echo ""

echo "4. Python Packages (key ones):"
python -c "import torch; print(f'PyTorch: {torch.__version__}')"
python -c "import numpy; print(f'NumPy: {numpy.__version__}')"
python -c "import rclpy; print('rclpy: OK')"
echo ""

echo "5. PYTHONPATH:"
echo "$PYTHONPATH"
echo ""

echo "6. Virtual Environment:"
echo "VIRTUAL_ENV: $VIRTUAL_ENV"
echo ""

echo "7. Network Interface:"
ip addr show enx00e6021980f3 2>/dev/null || echo "Interface enx00e6021980f3 NOT FOUND!"
echo ""

echo "8. ROS2 Nodes Running:"
ros2 node list 2>/dev/null || echo "Cannot list ROS2 nodes"
echo ""

echo "9. ROS2 Topics:"
ros2 topic list 2>/dev/null || echo "Cannot list ROS2 topics"
echo ""

echo "=== ROBOT COMMUNICATION DIAGNOSTICS ==="
echo ""

echo "10. Checking lowcmd Topics:"
ros2 topic list 2>/dev/null | grep -E "(lowcmd|lowstate)" || echo "No low* topics found"
echo ""

echo "11. Topic Publishing Rates:"
echo "Checking /lowcmd_buffer (expect ~50Hz):"
timeout 2 ros2 topic hz /lowcmd_buffer 2>/dev/null || echo "/lowcmd_buffer not publishing"
echo ""
echo "Checking /lowcmd (expect ~1000Hz):"
timeout 2 ros2 topic hz /lowcmd 2>/dev/null || echo "/lowcmd not publishing"
echo ""

echo "12. Sample Motor Command from /lowcmd:"
ros2 topic echo /lowcmd --once 2>/dev/null | head -20 || echo "Cannot capture /lowcmd"
echo ""

echo "13. Robot State from /lowstate:"
ros2 topic echo /lowstate --once 2>/dev/null | head -20 || echo "Cannot capture /lowstate"
echo ""
