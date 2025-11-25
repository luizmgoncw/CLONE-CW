#!/bin/bash

# Script para visualizar localização no RViz2
# Inicia o bridge ZeroMQ→ROS2 e o RViz2

set -e

CLONE_DIR="/home/luizmarques/Documents/CLONE"

echo "🚀 Starting Localization Visualization"

# Carregar ambiente ROS2
source /opt/ros/humble/setup.bash
source "$CLONE_DIR/unitree_ros2/install/setup.bash"

# Ativar venv
source "$CLONE_DIR/unitree_ros2/venv/bin/activate"

echo ""
echo "📡 Starting Localization Bridge (ZeroMQ → ROS2)..."
echo "   Connecting to G1 PC2 at 192.168.123.164:6006"
echo ""

# Iniciar bridge em background
cd "$CLONE_DIR/CLONE/deploy"
python3 localization_bridge.py &
BRIDGE_PID=$!

# Aguardar bridge inicializar
sleep 2

echo ""
echo "🎨 Starting RViz2..."
echo "   Config: $CLONE_DIR/localization_visualization.rviz2"
echo ""

# Iniciar RViz2
rviz2 -d "$CLONE_DIR/localization_visualization.rviz2"

# Quando RViz fechar, matar o bridge
echo ""
echo "🛑 Stopping Localization Bridge..."
kill $BRIDGE_PID 2>/dev/null || true

echo "✅ Localization Visualization stopped"
