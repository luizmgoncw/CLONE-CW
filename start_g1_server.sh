#!/bin/bash

echo "🚀 Starting CLONE G1 Server"
echo "===================================="

# Diretório base
CLONE_DIR="/home/luizmarques/Documents/CLONE"
cd "$CLONE_DIR" || exit 1

# 1. Source ROS2 Humble
echo "📦 Loading ROS2 Humble..."
source /opt/ros/humble/setup.bash

# 2. Source Unitree ROS2 workspace
echo "📦 Loading Unitree ROS2..."
source "$CLONE_DIR/unitree_ros2/install/setup.bash"

# 3. Configurar CycloneDDS
echo "🔧 Configuring CycloneDDS..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enx00e6021980f3" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

# 4. Ativar Python venv
echo "🐍 Activating Python venv..."
source "$CLONE_DIR/unitree_ros2/venv/bin/activate"

# 5. Ir para deploy
cd "$CLONE_DIR/CLONE/deploy" || exit 1

# 6. Executar g1_server
echo ""
echo "✅ Environment loaded successfully!"
echo "===================================="
echo "🤖 Starting g1_server.py..."
echo ""

python g1_server.py
