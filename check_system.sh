#!/bin/bash

echo "🔍 CLONE System Check"
echo "===================================="
echo ""

CLONE_DIR="/home/luizmarques/Documents/CLONE"
ERRORS=0

# 1. Verificar ROS2
echo -n "📦 ROS2 Humble: "
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✅ OK"
else
    echo "❌ NOT FOUND"
    ERRORS=$((ERRORS+1))
fi

# 2. Verificar Unitree ROS2 compilado
echo -n "📦 Unitree ROS2: "
if [ -f "$CLONE_DIR/unitree_ros2/install/setup.bash" ]; then
    echo "✅ OK"
else
    echo "❌ NOT COMPILED"
    ERRORS=$((ERRORS+1))
fi

# 3. Verificar Python venv
echo -n "🐍 Python venv: "
if [ -f "$CLONE_DIR/unitree_ros2/venv/bin/activate" ]; then
    echo "✅ OK"
else
    echo "❌ NOT FOUND"
    ERRORS=$((ERRORS+1))
fi

# 4. Verificar ngrok
echo -n "📡 Ngrok: "
if command -v ngrok &> /dev/null; then
    echo "✅ OK ($(ngrok version))"
else
    echo "❌ NOT INSTALLED"
    ERRORS=$((ERRORS+1))
fi

# 5. Verificar ngrok auth
echo -n "🔑 Ngrok auth: "
if ngrok config check &> /dev/null; then
    echo "✅ AUTHENTICATED"
else
    echo "⚠️  NOT AUTHENTICATED"
    echo "   Run: ngrok config add-authtoken YOUR_TOKEN"
fi

# 6. Verificar interface de rede
echo -n "🌐 Network interface (enx00e6021980f3): "
if ip link show enx00e6021980f3 &> /dev/null; then
    echo "✅ OK"
else
    echo "⚠️  NOT FOUND (check interface name)"
fi

# 7. Verificar conectividade G1
echo -n "🤖 G1 PC2 (192.168.123.164): "
if ping -c 1 -W 1 192.168.123.164 &> /dev/null; then
    echo "✅ REACHABLE"
else
    echo "❌ NOT REACHABLE"
    ERRORS=$((ERRORS+1))
fi

# 8. Verificar scripts
echo -n "📜 Deployment scripts: "
if [ -f "$CLONE_DIR/start_lowcmd.sh" ] && [ -f "$CLONE_DIR/start_g1_server.sh" ]; then
    echo "✅ OK"
else
    echo "❌ MISSING"
    ERRORS=$((ERRORS+1))
fi

# 9. Verificar config.py
echo -n "⚙️  Config backend: "
if [ -f "$CLONE_DIR/CLONE/deploy/config.py" ]; then
    BACKEND=$(grep "VISION_WRAPPER_BACKEND" "$CLONE_DIR/CLONE/deploy/config.py" | cut -d"'" -f2)
    echo "✅ $BACKEND"
else
    echo "❌ config.py NOT FOUND"
    ERRORS=$((ERRORS+1))
fi

# 10. Verificar dependências Python
echo -n "📦 Python deps: "
source "$CLONE_DIR/unitree_ros2/venv/bin/activate" 2>/dev/null
if python -c "import torch, onnx, vuer" 2>/dev/null; then
    echo "✅ OK"
else
    echo "❌ MISSING"
    ERRORS=$((ERRORS+1))
fi

echo ""
echo "===================================="
if [ $ERRORS -eq 0 ]; then
    echo "✅ System ready for deployment!"
    echo ""
    echo "Next steps:"
    echo "1. ssh unitree@192.168.123.164"
    echo "   cd ~/onboard && ./localization_server.sh"
    echo "2. ./start_ngrok.sh"
    echo "3. ./start_lowcmd.sh"
    echo "4. ./start_g1_server.sh"
else
    echo "❌ Found $ERRORS error(s)"
    echo "Please fix the issues above before deployment"
fi
echo "===================================="
