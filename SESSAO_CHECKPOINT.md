# CHECKPOINT DA SESSÃO - INSTALAÇÃO CLONE

**Data**: 2025-11-19
**Status**: PC2 100% CONFIGURADO E VERIFICADO ✅
**Próximo Passo**: Configurar Server PC

---

## 📋 RESUMO DO QUE FOI FEITO

### PC2 (Unitree G1 - 192.168.123.164)
- ✅ Livox-SDK2 compilado e configurado
- ✅ livox_ros_driver2 compilado para ROS1
- ✅ FAST_LIO compilado (headers gerados)
- ✅ FAST_LIO_LOCALIZATION compilado e funcionando
- ✅ Scripts Python convertidos de Python 2 para Python 3
- ✅ Calibração extrinsic corrigida (nuvem correta, seta apontando para frente)
- ✅ Script de inicialização `localization_server.sh` criado e testado
- ✅ Todos os tópicos ROS publicando a ~10Hz

### Server PC (192.168.123.99)
- ✅ Repositórios CLONE clonados
- ✅ Python venv criado em `/home/luizmarques/Documents/CLONE/unitree_ros2/venv/`
- ✅ VisionWrapper instalado (modo editable)
- ✅ Dependências básicas instaladas (grpcio-tools, protobuf, onnx, onnxruntime)
- ⏸️ pytorch3d não instalado ainda (opcional)
- ⏸️ Deployment scripts não testados ainda

---

## 🔧 CONFIGURAÇÕES CRÍTICAS

### 1. LiDAR Mid-360 IP Configuration
**Arquivo**: `~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json` (PC2)

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.123.164",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.123.164",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.123.164",
      "point_data_port": 56301,
      "imu_data_ip": "192.168.123.164",
      "imu_data_port": 56401,
      "log_data_ip": "192.168.123.164",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.123.120",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

### 2. FAST_LIO_LOCALIZATION Extrinsic Calibration
**Arquivo**: `~/catkin_ws/src/FAST_LIO_LOCALIZATION/config/mid360.yaml` (PC2)

**CONFIGURAÇÃO FINAL CORRETA**:
```yaml
mapping:
  extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
  extrinsic_R: [ 1,  0,  0,
                 0, -1,  0,
                 0,  0, -1]
```

**Nota**: Esta configuração corrige:
- Nuvem de pontos não aparece mais de cabeça para baixo
- Seta de localização aponta para frente do robô (não para trás)

### 3. Localization Server Script
**Arquivo**: `~/onboard/localization_server.sh` (PC2)

```bash
#!/bin/bash

# Carregar ambiente ROS
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "[1/3] Starting FAST_LIO_LOCALIZATION..."
roslaunch fast_lio_localization localization_mid360.launch &
LOCALIZATION_PID=$!

sleep 3

echo "[2/3] Starting Livox ROS Driver 2..."
roslaunch livox_ros_driver2 msg_MID360.launch &
LIDAR_PID=$!

sleep 3

echo "[3/3] Starting position server..."
cd ~/onboard
python3 pos_server.py &
POS_SERVER_PID=$!

echo "=========================================="
echo "All services started!"
echo "FAST_LIO_LOCALIZATION PID: $LOCALIZATION_PID"
echo "Livox Driver PID: $LIDAR_PID"
echo "Position Server PID: $POS_SERVER_PID"
echo "=========================================="
echo "Press Ctrl+C to stop all services"

trap "echo 'Stopping services...'; kill $LOCALIZATION_PID $LIDAR_PID $POS_SERVER_PID 2>/dev/null; exit" SIGINT SIGTERM

wait
```

**Dar permissão de execução**:
```bash
chmod +x ~/onboard/localization_server.sh
```

### 4. Server PC Config
**Arquivo**: `/home/luizmarques/Documents/CLONE/CLONE/deploy/config.py`

```python
VISION_WRAPPER_BACKEND = 'avp_stream'  # Para rede local
# VISION_WRAPPER_BACKEND = 'vuer'  # Para acesso via internet (requer ngrok)

VISION_PRO_IP = '192.168.123.8'
VISION_PRO_DELTA_H = -0.54
USE_DEX_HANDS = True
```

---

## ✅ CHECKLIST DE VERIFICAÇÃO DO PC2 (ANTES DE PROSSEGUIR)

Execute estes comandos no **PC2 via SSH** para verificar que tudo está funcionando:

### 1. Iniciar os serviços
```bash
ssh unitree@192.168.123.164
cd ~/onboard
./localization_server.sh
```

### 2. Em outro terminal SSH, verificar processos
```bash
ssh unitree@192.168.123.164
ps aux | grep -E "roslaunch|python3"
```

**Resultado esperado**: 3 processos rodando:
- `roslaunch fast_lio_localization localization_mid360.launch`
- `roslaunch livox_ros_driver2 msg_MID360.launch`
- `python3 pos_server.py`

### 3. Verificar tópicos ROS
```bash
# Verificar lista de tópicos
rostopic list

# Verificar frequência dos tópicos principais
rostopic hz /livox/lidar
rostopic hz /cloud_registered
rostopic hz /Odometry
```

**Resultado esperado**: Todos publicando a ~10 Hz
- `/livox/lidar` @ ~10 Hz
- `/cloud_registered` @ ~10 Hz
- `/Odometry` @ ~10 Hz

### 4. Verificar conectividade de rede
```bash
# Do PC2, pingar o Server PC
ping -c 3 192.168.123.99

# Do Server PC, pingar o PC2
ping -c 3 192.168.123.164

# Verificar se pos_server está escutando
netstat -tuln | grep 6006
```

### 5. (Opcional) Visualizar no RViz
Se tiver acesso GUI via NoMachine:
```bash
rviz -d ~/catkin_ws/src/FAST_LIO_LOCALIZATION/rviz_cfg/localization.rviz
```

Verificar:
- ✅ Nuvem de pontos aparece corretamente (não de cabeça para baixo)
- ✅ Seta de localização aponta para frente do robô
- ✅ Fixed Frame: "camera_init"

---

## 📝 PRÓXIMOS PASSOS (QUANDO RETOMAR)

### 1. Verificar PC2 novamente (conforme checklist acima)

### 2. Configurar Server PC para CLONE

#### A. Verificar ambiente Python
```bash
cd /home/luizmarques/Documents/CLONE
source unitree_ros2/venv/bin/activate
python --version  # Deve ser 3.10.12
pip list | grep -E "(torch|onnx|grpc|protobuf|vision|numpy|scipy)"
```

#### B. Instalar dependências faltantes (se necessário)
```bash
# Se pytorch3d for necessário (opcional por enquanto)
# pip install pytorch3d

# Verificar se VisionWrapper está instalado
pip show vision-wrapper
```

#### C. Configurar método de conexão
**Opção 1: Rede Local (mais simples)**
- Conectar Server PC e G1 ao mesmo roteador via ethernet
- Usar `VISION_WRAPPER_BACKEND = 'avp_stream'` em config.py
- Vision Pro conecta ao mesmo WiFi do roteador

**Opção 2: Via Internet (mais flexível)**
- Instalar ngrok no Server PC
- Usar `VISION_WRAPPER_BACKEND = 'vuer'` em config.py
- Criar túnel: `ngrok http --region=us --hostname=<seu-dominio>.ngrok-free.app 8012`
- Atualizar URLs no Vision Pro

#### D. Testar deployment scripts
```bash
cd /home/luizmarques/Documents/CLONE/CLONE/deploy
source ../../unitree_ros2/venv/bin/activate

# Terminal 1: Lowcmd publisher (relay de comandos a 1kHz)
python lowcmd_publisher.py

# Terminal 2: G1 Server (servidor principal CLONE)
python g1_server.py
```

### 3. Conectar Vision Pro
- Abrir Safari no Vision Pro
- Acessar URL conforme backend escolhido
- Testar controles de teleoperação

---

## 📚 DOCUMENTAÇÃO ADICIONAL

- **Troubleshooting completo**: `INSTALACAO_TROUBLESHOOTING.md`
- **README original CLONE**: `README.md`
- **Deployment README**: `deploy/README.md`

---

## 🔗 TOPOLOGIA DE REDE

```
┌─────────────────────────┐
│   Apple Vision Pro      │
│   192.168.123.8         │
└───────────┬─────────────┘
            │ WiFi
            │
┌───────────▼─────────────┐
│   Roteador              │
│   (192.168.123.x)       │
└───┬─────────────────┬───┘
    │ Ethernet        │ Ethernet
    │                 │
┌───▼───────────┐ ┌──▼──────────────┐
│  Server PC    │ │   Unitree G1    │
│  192.168.123. │ │   (PC2)         │
│  99           │ │   192.168.123.  │
│               │ │   164           │
│  - CLONE      │ │   - LiDAR       │
│    Policy     │ │   - FAST_LIO    │
│  - Vision     │ │   - pos_server  │
│    Wrapper    │ │                 │
└───────────────┘ └─────────────────┘
                      │
                      │ Ethernet
                      │
                  ┌───▼───────────┐
                  │  Livox Mid-360│
                  │  192.168.123. │
                  │  120          │
                  └───────────────┘
```

---

## ⚠️ PROBLEMAS CONHECIDOS E SOLUÇÕES

1. **"No device will be connected"** no Livox-SDK sample
   - ✅ **Solução**: Verificar IPs no `mid360_config.json` (deve ser 192.168.123.x)

2. **"ament_cmake_auto not found"** ao compilar livox_ros_driver2
   - ✅ **Solução**: Rodar `./build.sh ROS1` antes de catkin_make

3. **Conflito entre FAST_LIO e FAST_LIO_LOCALIZATION**
   - ✅ **Solução**: Compilação em duas etapas usando CATKIN_IGNORE

4. **Python scripts com erro "No module named open3d"**
   - ✅ **Solução**: Instalar `pip3 install numpy open3d scipy` e converter scripts para Python 3

5. **Nuvem de pontos de cabeça para baixo no RViz**
   - ✅ **Solução**: Ajustar `extrinsic_R` para `[1, 0, 0, 0, -1, 0, 0, 0, -1]`

6. **protobuf version conflicts**
   - ✅ **Solução**: `pip install grpcio-tools==1.48.0 protobuf==3.20.3`

---

## 🎯 OBJETIVOS DA PRÓXIMA SESSÃO

1. ✅ **Verificar** que PC2 ainda está funcionando corretamente
2. 🔄 **Configurar** Server PC para deployment
3. 🔄 **Testar** comunicação entre Server PC e PC2
4. 🔄 **Decidir** método de conexão (rede local vs internet)
5. 🔄 **Rodar** primeiro teste completo do CLONE

---

---

## 📅 ATUALIZAÇÃO - 2025-11-21

**Status**: ✅ SISTEMA CLONE COMPLETO FUNCIONANDO + VISUALIZAÇÃO RViz2 IMPLEMENTADA
**Próximo Passo**: Testar visualização completa (PointCloud no RViz2)

### ✅ O Que Foi Feito Hoje

#### 1. Correção Documentação - Debug Mode
- ❌ Removido referências a Sport Mode (não funciona)
- ✅ Esclarecido que Debug Mode usa **política neural de equilíbrio do CLONE** (MoE @ 52Hz)
- ✅ **NÃO** é "sem equilíbrio" - é equilíbrio controlado por rede neural treinada
- Arquivos atualizados:
  - `GUIA_RAPIDO_DEPLOYMENT.md`
  - `SUCESSO_DEPLOYMENT.md`
  - `INSTALACAO_TROUBLESHOOTING.md`

#### 2. Sistema de Visualização RViz2 Completo

**Arquivos Criados:**
- ✅ `localization_bridge.py` - Bridge ZeroMQ → ROS2 (localização + PointCloud)
- ✅ `cloud_server.py` - Servidor ZeroMQ para PointCloud (porta 6007, no G1)
- ✅ `localization_visualization.rviz2` - Config RViz2 completa
- ✅ `start_localization_viz.sh` - Script inicialização automática
- ✅ `check_localization.sh` - Script diagnóstico
- ✅ `LOCALIZATION_VISUALIZATION.md` - Documentação completa

**Arquitetura:**
```
G1 PC2 (ROS1):
  FAST_LIO_LOCALIZATION publica:
    - /localization (Odometry) → pos_server.py (porta 6006)
    - /cloud_registered (PointCloud2) → cloud_server.py (porta 6007)
         ↓ ZeroMQ TCP                      ↓ ZeroMQ TCP
Server PC (ROS2):
  localization_bridge.py (recebe ambos)
    - Publica /localization (Odometry)
    - Publica /localization_path (Path)
    - Publica /cloud_registered (PointCloud2)
    - Publica TF: map → body
         ↓
      RViz2
```

**Características:**
- Downsampling: 4x (25% dos pontos)
- Frequência: 10 Hz PointCloud, 50 Hz localização
- Sem ros1_bridge (ZeroMQ puro)
- Compressão: ~60-240 KB por frame

#### 3. Descoberta Técnica - Pipeline de Localização

**Pergunta respondida:** `/localization` já tem filtro de Kalman?

**Resposta:** Sim! É resultado FINAL do pipeline completo:

```
LiDAR + IMU
    ↓
FAST_LIO (C++)
  - IEKF (Iterated Extended Kalman Filter)
  - Odometria local precisa
    ↓
/Odometry topic
    ↓
global_localization.py (Python)
  - ICP (Iterative Closest Point) com Open3D
  - Registra scan vs mapa global
  - Corrige drift acumulado
    ↓
/map_to_odom topic
    ↓
transform_fusion.py (Python)
  - T_map_to_base = T_map_to_odom * T_odom_to_base
  - Publica a 50 Hz
    ↓
/localization topic ✅ (IEKF + ICP + Fusão)
```

### 🐛 Problemas Resolvidos Hoje

1. **RViz2 - Subscription duplicada**
   - Config tinha 2 displays para `/localization` (Pose + Odometry)
   - Removido display Pose (incorreto)

2. **Porta 6006 em uso**
   - `pos_server.py` antigo ainda rodando
   - Solução: `pkill -f pos_server` antes de reiniciar

3. **PointCloud com frame errado**
   - Vinha com frame `camera_init` mas RViz2 espera `map`
   - **Corrigido** `cloud_server.py` linha 87 para forçar frame `map`
   - ⚠️ **Precisa copiar para G1 e reiniciar**

### ⏸️ Pendente para Segunda-Feira

1. **Copiar cloud_server.py corrigido para G1:**
```bash
scp /home/luizmarques/Documents/CLONE/CLONE/deploy/onboard/cloud_server.py unitree@192.168.123.164:~/onboard/
```

2. **Testar Sistema Completo de Visualização:**

**No G1 PC2 - 2 terminais SSH:**
```bash
# Terminal 1
ssh unitree@192.168.123.164
cd ~/onboard
./localization_server.sh

# Terminal 2
ssh unitree@192.168.123.164
cd ~/onboard
python3 cloud_server.py
```

**No Server PC:**
```bash
cd /home/luizmarques/Documents/CLONE
./start_localization_viz.sh
```

**Verificar RViz2 mostra:**
- ✅ Seta verde (pose)
- ✅ Linha ciano (trajetória)
- ✅ Pontos amarelos (nuvem LiDAR) 🆕
- ✅ Logs: `Loc: 10.1 Hz | Cloud: 10.0 Hz (12543 pts)`
- ✅ Sem erros "dropping message"

---

**Última Atualização**: 2025-11-21
**Status PC2**: ✅ Funcionando
**Status Server PC**: ✅ Sistema CLONE operacional + Visualização implementada
**Status Visualização**: ⚠️ Aguardando teste final (cloud_server.py atualizado)
