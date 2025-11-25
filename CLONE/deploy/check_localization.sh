#!/bin/bash

# Script para verificar se a localização está sendo recebida

echo "🔍 Checking Localization Data Flow"
echo "===================================="
echo ""

# Verificar conectividade com G1
echo "1️⃣ Testing network connectivity to G1 PC2..."
if ping -c 1 -W 2 192.168.123.164 > /dev/null 2>&1; then
    echo "   ✅ G1 PC2 is reachable (192.168.123.164)"
else
    echo "   ❌ Cannot reach G1 PC2 (192.168.123.164)"
    echo "   → Check Ethernet connection"
    exit 1
fi
echo ""

# Verificar porta ZeroMQ
echo "2️⃣ Testing ZeroMQ port (6006)..."
if timeout 2 bash -c "cat < /dev/null > /dev/tcp/192.168.123.164/6006" 2>/dev/null; then
    echo "   ✅ Port 6006 is open"
else
    echo "   ⚠️  Port 6006 is not responding"
    echo "   → pos_server.py may not be running on G1"
fi
echo ""

# Verificar se pos_server está rodando no G1
echo "3️⃣ Checking if pos_server is running on G1..."
if ssh -o ConnectTimeout=2 unitree@192.168.123.164 'pgrep -f pos_server.py' > /dev/null 2>&1; then
    PID=$(ssh unitree@192.168.123.164 'pgrep -f pos_server.py')
    echo "   ✅ pos_server.py is running (PID: $PID)"
else
    echo "   ❌ pos_server.py is NOT running"
    echo "   → Start it with: ssh unitree@192.168.123.164 'cd ~/onboard && python3 pos_server.py &'"
fi
echo ""

# Verificar ROS topics no G1 (se possível)
echo "4️⃣ Checking ROS topics on G1..."
if ssh -o ConnectTimeout=2 unitree@192.168.123.164 'source /opt/ros/noetic/setup.bash && rostopic list' 2>/dev/null | grep -q "/localization"; then
    echo "   ✅ /localization topic exists on G1"
    RATE=$(ssh unitree@192.168.123.164 'source /opt/ros/noetic/setup.bash && timeout 2 rostopic hz /localization 2>&1' | grep "average rate" | awk '{print $3}')
    if [ ! -z "$RATE" ]; then
        echo "   📊 Publishing at: $RATE Hz"
    fi
else
    echo "   ⚠️  Cannot verify ROS topics (may need ROS environment)"
fi
echo ""

# Teste de recepção ZeroMQ
echo "5️⃣ Testing ZeroMQ reception (3 seconds)..."
cat > /tmp/test_zmq.py << 'EOF'
import zmq
import pickle
import sys

try:
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://192.168.123.164:6006")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.RCVTIMEO, 3000)

    message = socket.recv()
    position, quat = pickle.loads(message)

    print(f"   ✅ Received localization data!")
    print(f"   📍 Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
    print(f"   🧭 Quaternion: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")

    socket.close()
    context.term()
    sys.exit(0)
except zmq.Again:
    print("   ⚠️  Timeout - no data received in 3 seconds")
    print("   → Check if FAST_LIO_LOCALIZATION is running")
    sys.exit(1)
except Exception as e:
    print(f"   ❌ Error: {e}")
    sys.exit(1)
EOF

python3 /tmp/test_zmq.py
TEST_RESULT=$?
rm /tmp/test_zmq.py
echo ""

# Resumo
echo "===================================="
if [ $TEST_RESULT -eq 0 ]; then
    echo "✅ Localization data flow is working!"
    echo ""
    echo "You can now run: ./start_localization_viz.sh"
else
    echo "❌ Localization data flow has issues"
    echo ""
    echo "Troubleshooting steps:"
    echo "1. SSH to G1: ssh unitree@192.168.123.164"
    echo "2. Start localization: cd ~/onboard && ./localization_server.sh"
    echo "3. Verify pos_server is running: ps aux | grep pos_server"
    echo "4. Check for errors in pos_server output"
fi
