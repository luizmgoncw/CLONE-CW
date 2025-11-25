# Troubleshooting - Instalação do CLONE no Unitree G1

Este documento lista todos os problemas encontrados durante a instalação do CLONE que não estavam documentados no README original, junto com suas soluções.

---

## 📋 Índice

### PC2 (Unitree G1) - Configuração Inicial
1. [Configuração do LiDAR Livox Mid-360](#1-configuração-do-lidar-livox-mid-360)
2. [Instalação Offline dos Drivers](#2-instalação-offline-dos-drivers)
3. [Compilação do Livox-SDK2](#3-compilação-do-livox-sdk2)
4. [Configuração do livox_ros_driver2](#4-configuração-do-livox_ros_driver2)
5. [Submodules do FAST_LIO_LOCALIZATION](#5-submodules-do-fast_lio_localization)
6. [Conflito FAST_LIO vs FAST_LIO_LOCALIZATION](#6-conflito-fast_lio-vs-fast_lio_localization)
7. [Ambiente Python para CLONE](#7-ambiente-python-para-clone)
8. [Configuração do bashrc no PC2](#8-configuração-do-bashrc-no-pc2)
9. [Dependências Python no PC2 (FAST_LIO_LOCALIZATION)](#9-dependências-python-no-pc2-fast_lio_localization)
10. [Scripts Python usando Python 2](#10-scripts-python-usando-python-2)
11. [RViz tentando abrir GUI via SSH](#11-rviz-tentando-abrir-gui-via-ssh)
12. [Correção do localization_server.sh](#12-correção-do-localization_serversh)

### Server PC - Deployment
13. [Problemas de Deployment no Server PC](#13-problemas-de-deployment-no-server-pc)
    - [ModuleNotFoundError: rclpy](#131-modulenotfounderror-no-module-named-rclpy)
    - [ModuleNotFoundError: zmq](#132-modulenotfounderror-no-module-named-zmq)
    - [AttributeError: sapien.core](#133-attributeerror-module-sapiencore-has-no-attribute-articulation)
    - [ModuleNotFoundError: VisionWrapper](#134-modulenotfounderror-no-module-named-visionwrapper)
    - [FileNotFoundError: paths hardcoded](#135-filenotfounderror-deployresourcesg1_fkurdf)
    - [TypeError: mjv_connector()](#136-typeerror-mjv_connector-incompatible-function-arguments)
    - [Interface CycloneDDS não encontrada](#137-interface-cyclonedds-não-encontrada)
14. [Image Server (Camera RealSense)](#14-image-server-camera-realsense)
    - [Device or resource busy](#141-device-or-resource-busy)
    - [Image Server é Obrigatório?](#142-image-server-é-obrigatório)
    - [Obter Serial Number da RealSense](#143-obter-serial-number-da-realsense)

---

## 1. Configuração do LiDAR Livox Mid-360

### Problema
Ao executar `./lidar_lvx_sample` do Livox-SDK, aparecia "No device will be connected", mesmo com ping funcionando para o LiDAR (192.168.123.120).

### Causa
- O Livox-SDK usa **broadcast UDP** para descobrir dispositivos
- A configuração padrão do SDK usa IPs da rede `192.168.1.x`
- O robô G1 usa a rede `192.168.123.x`

### Solução
Criar arquivo de configuração `mid360_config.json` com os IPs corretos:

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

**Localização:**
- Para Livox-SDK2 samples: `~/livox_drivers_offline/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json`
- Para livox_ros_driver2: `~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json`

---

## 2. Instalação Offline dos Drivers

### Problema
O PC2 do robô não tem acesso à internet, impedindo o clone direto dos repositórios Git.

### Solução
Processo em duas etapas:

**No PC com internet:**
```bash
cd ~/livox_drivers_offline

# Baixar todos os repositórios
git clone https://github.com/Livox-SDK/Livox-SDK2.git
git clone https://github.com/Livox-SDK/livox_ros_driver.git
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
git clone https://github.com/hku-mars/FAST_LIO.git
git clone https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION.git

# Inicializar submodules (importante!)
cd FAST_LIO
git submodule update --init --recursive
cd ../FAST_LIO_LOCALIZATION
git submodule update --init --recursive
```

**Transferir para PC2:**
```bash
scp -r livox_ros_driver unitree@192.168.123.164:~/catkin_ws/src/
scp -r livox_ros_driver2 unitree@192.168.123.164:~/catkin_ws/src/
scp -r FAST_LIO unitree@192.168.123.164:~/catkin_ws/src/
scp -r FAST_LIO_LOCALIZATION unitree@192.168.123.164:~/catkin_ws/src/
scp -r Livox-SDK2 unitree@192.168.123.164:~/livox_drivers_offline/
```

---

## 3. Compilação do Livox-SDK2

### Problema
O `livox_ros_driver2` precisa da biblioteca estática do Livox-SDK2 instalada no sistema.

### Causa
O CMakeLists.txt do livox_ros_driver2 procura por:
```cmake
find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_static.a /usr/local/lib)
```

### Solução
**No PC2:**
```bash
cd ~/livox_drivers_offline/Livox-SDK2
mkdir build && cd build
cmake ..
make
sudo make install
```

Isso instala:
- `/usr/local/lib/liblivox_lidar_sdk_static.a`
- `/usr/local/include/livox_lidar_*.h`

---

## 4. Configuração do livox_ros_driver2

### Problema
Erro durante `catkin_make`:
```
Could not find a package configuration file provided by "ament_cmake_auto"
```

### Causa
O `livox_ros_driver2` suporta tanto ROS1 quanto ROS2, mas precisa ser configurado explicitamente para ROS1.

### Solução
**No PC2, ANTES do catkin_make:**
```bash
cd ~/catkin_ws/src/livox_ros_driver2
./build.sh ROS1
```

O script `build.sh ROS1`:
- Copia `package_ROS1.xml` → `package.xml`
- Configura CMakeLists.txt para usar catkin (ROS1)
- Limpa builds anteriores

**⚠️ Importante:** Execute `./build.sh ROS1` sempre que:
- Clonar o repositório pela primeira vez
- Limpar o workspace (`rm -rf build devel`)

---

## 5. Submodules do FAST_LIO_LOCALIZATION

### Problema
Erro durante compilação:
```
Cannot find source file: include/ikd-Tree/ikd_Tree.cpp
```

### Causa
O repositório usa Git submodules que não são clonados automaticamente com `git clone`.

### Solução
**No PC com internet (antes de transferir):**
```bash
cd FAST_LIO_LOCALIZATION
git submodule update --init --recursive

# Verificar se ikd-Tree foi baixado
ls include/ikd-Tree/
# Deve mostrar: ikd_Tree.cpp, ikd_Tree.h, etc.
```

**Depois transferir para o PC2.**

---

## 6. Conflito FAST_LIO vs FAST_LIO_LOCALIZATION

### Problema
Erro durante compilação:
```
add_executable cannot create target "fastlio_mapping" because another
target with the same name already exists
```

### Causa
Ambos os pacotes (FAST_LIO e FAST_LIO_LOCALIZATION) tentam criar um executável com o mesmo nome: `fastlio_mapping`.

### Por que precisamos dos dois?
- **FAST_LIO_LOCALIZATION** depende dos headers do **FAST_LIO** (`fast_lio/Pose6D.h`)
- Mas não podemos compilar ambos ao mesmo tempo devido ao conflito de nomes

### Solução: Compilação em duas etapas

**No PC2:**
```bash
cd ~/catkin_ws

# Etapa 1: Compilar FAST_LIO (gera headers)
rm -f src/FAST_LIO/CATKIN_IGNORE
touch src/FAST_LIO_LOCALIZATION/CATKIN_IGNORE
catkin_make

# Etapa 2: Compilar FAST_LIO_LOCALIZATION (usa headers já gerados)
touch src/FAST_LIO/CATKIN_IGNORE
rm src/FAST_LIO_LOCALIZATION/CATKIN_IGNORE
catkin_make
```

O arquivo `CATKIN_IGNORE` faz o catkin pular o pacote durante a compilação, mas mantém os arquivos disponíveis para outros pacotes usarem.

**Verificar resultado:**
```bash
source devel/setup.bash
rospack list | grep -E "fast_lio|livox"
```

Deve mostrar:
- `livox_ros_driver`
- `livox_ros_driver2`
- `fast_lio_localization`

---

## 7. Ambiente Python para CLONE

### Problema
Erro ao instalar `requirements.txt` com Python 3.13:
```
AttributeError: module 'pkgutil' has no attribute 'ImpImporter'
ERROR: No matching distribution found for pytorch3d
```

### Causa
- Python 3.13 é muito novo
- `numpy==1.23.0` não suporta Python 3.13
- `pytorch3d` não está disponível via pip comum

### Solução

**Opção 1: Usar venv do unitree_ros2 (Python 3.10)**
```bash
cd ~/Documents/CLONE/CLONE
source ../unitree_ros2/venv/bin/activate

# Instalar dependências (exceto pytorch3d temporariamente)
grep -v "pytorch3d" requirements.txt > requirements_temp.txt
pip install -r requirements_temp.txt
```

**Opção 2: Criar novo ambiente conda**
```bash
conda create -n clone python=3.10 -y
conda activate clone
pip install -r requirements.txt
```

**Para pytorch3d:**
```bash
# Instalar PyTorch primeiro
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Compilar pytorch3d do source
pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable"
```

---

## 8. Configuração do bashrc no PC2

### Problema
Comandos ROS (`rostopic`, `roslaunch`, etc.) não são reconhecidos em novos terminais SSH.

### Causa
O ambiente ROS não é carregado automaticamente quando você abre um novo terminal.

### Solução
**No PC2:**
```bash
# Adicionar ao bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Aplicar imediatamente
source ~/.bashrc
```

**⚠️ Importante:**
- O `~/.bashrc` só é executado ao **abrir um novo terminal**
- Em terminais já abertos, execute `source ~/.bashrc` manualmente
- Sempre que compilar com `catkin_make`, execute `source devel/setup.bash`

---

## 📝 Resumo da Ordem de Instalação no PC2

```bash
# 1. Criar workspace ROS1
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make

# 2. Configurar bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# 3. Compilar e instalar Livox-SDK2
cd ~/livox_drivers_offline/Livox-SDK2/build
cmake .. && make && sudo make install

# 4. Configurar livox_ros_driver2 para ROS1
cd ~/catkin_ws/src/livox_ros_driver2
./build.sh ROS1

# 5. Criar configuração do Mid-360
mkdir -p ~/catkin_ws/src/livox_ros_driver2/config
nano ~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json
# (Colar configuração da seção 1)

# 6. Compilar em duas etapas
cd ~/catkin_ws

# Etapa 1: FAST_LIO e drivers
rm -f src/FAST_LIO/CATKIN_IGNORE
touch src/FAST_LIO_LOCALIZATION/CATKIN_IGNORE
catkin_make

# Etapa 2: FAST_LIO_LOCALIZATION
touch src/FAST_LIO/CATKIN_IGNORE
rm src/FAST_LIO_LOCALIZATION/CATKIN_IGNORE
catkin_make

# 7. Verificar
source devel/setup.bash
rospack list | grep -E "fast_lio|livox"
```

---

## 🧪 Testar LiDAR

**Terminal 1:**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

**Terminal 2:**
```bash
source ~/catkin_ws/devel/setup.bash
rostopic list | grep livox
rostopic hz /livox/lidar
```

Se aparecer a frequência de publicação (~10 Hz), está funcionando! ✅

---

## 9. Dependências Python no PC2 (FAST_LIO_LOCALIZATION)

### Problema
Ao executar `localization_mid360.launch`, aparecem erros:
```
ImportError: No module named open3d
ImportError: No module named numpy
```

### Causa
Os scripts Python do FAST_LIO_LOCALIZATION usam bibliotecas que não vêm instaladas por padrão no ROS.

### Solução
**No PC2:**
```bash
# Instalar pip3 se necessário
sudo apt update
sudo apt install -y python3-pip

# Instalar dependências
pip3 install numpy open3d scipy

# Verificar instalação
python3 -c "import numpy; import open3d; print('✓ Dependências OK')"
```

---

## 10. Scripts Python usando Python 2

### Problema
Mesmo após instalar as bibliotecas, os scripts continuam falhando porque estão configurados para Python 2:
```python
#!/usr/bin/env python2
```

### Causa
O FAST_LIO_LOCALIZATION foi originalmente escrito para Python 2, mas o ROS Noetic usa Python 3.

### Solução
**No PC2:**
```bash
# Corrigir shebang de todos os scripts
sed -i '1s|#!/usr/bin/env python2|#!/usr/bin/env python3|' ~/catkin_ws/src/FAST_LIO_LOCALIZATION/scripts/*.py

# Verificar se corrigiu
head -1 ~/catkin_ws/src/FAST_LIO_LOCALIZATION/scripts/*.py
```

Todos os scripts devem mostrar: `#!/usr/bin/env python3`

---

## 11. RViz tentando abrir GUI via SSH

### Problema
Erro ao executar via SSH:
```
qt.qpa.xcb: could not connect to display
This application failed to start because no Qt platform plugin could be initialized
[rviz-4] process has died
```

### Causa
O RViz tenta abrir interface gráfica, mas você está conectado via SSH sem X11 forwarding.

### Solução
Desabilitar RViz no launch file (você não precisa dele no PC2):

**No PC2:**
```bash
# Editar launch file
nano ~/catkin_ws/src/FAST_LIO_LOCALIZATION/launch/localization_mid360.launch
```

Procure esta linha:
```xml
<arg name="rviz" default="true"/>
```

Mude para:
```xml
<arg name="rviz" default="false"/>
```

Salve (Ctrl+O, Enter, Ctrl+X).

---

## 12. Correção do localization_server.sh

### Problema
O script original tem caminhos incorretos:
```bash
cd nav/rosws/fastlio_localization  # Não existe
source devel/setup.bash  # Está no lugar errado
cd ~/teleoperation  # Não existe
```

### Solução
**No PC2, edite o arquivo:**
```bash
nano ~/onboard/localization_server.sh
```

**Script corrigido:**
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

# Esperar e matar todos os processos ao sair
trap "echo 'Stopping services...'; kill $LOCALIZATION_PID $LIDAR_PID $POS_SERVER_PID 2>/dev/null; exit" SIGINT SIGTERM

wait
```

Tornar executável:
```bash
chmod +x ~/onboard/localization_server.sh
```

---

## 13. Problemas de Deployment no Server PC

### 13.1. ModuleNotFoundError: No module named 'rclpy'

#### Problema
Ao executar `python lowcmd_publisher.py` ou `g1_server.py` diretamente:
```
ModuleNotFoundError: No module named 'rclpy'
ModuleNotFoundError: No module named 'unitree_hg'
```

#### Causa
Scripts precisam do ambiente ROS2 carregado antes de rodar. `rclpy` e `unitree_hg` são módulos do ROS2, não do pip.

#### Solução
**NÃO execute** os scripts diretamente! Use os scripts de startup que carregam o ambiente completo:

```bash
cd /home/luizmarques/Documents/CLONE

# Terminal 1 - Lowcmd Publisher
./start_lowcmd.sh

# Terminal 2 - G1 Server
./start_g1_server.sh
```

Estes scripts automaticamente:
- Carregam ROS2 Humble
- Carregam Unitree ROS2 workspace
- Configuram CycloneDDS
- Ativam Python venv

---

### 13.2. ModuleNotFoundError: No module named 'zmq'

#### Problema
```
ModuleNotFoundError: No module named 'zmq'
```

#### Solução
```bash
source unitree_ros2/venv/bin/activate
pip install pyzmq
```

---

### 13.3. AttributeError: module 'sapien.core' has no attribute 'Articulation'

#### Problema
```
AttributeError: module 'sapien.core' has no attribute 'Articulation'
```

#### Causa
`dex_retargeting 0.1.1` requer `sapien 2.x`, mas por padrão instala `sapien 3.x` (API incompatível).

#### Solução
```bash
source unitree_ros2/venv/bin/activate
pip uninstall sapien -y
pip install "sapien<3.0"
```

Versão correta: `sapien 2.2.2`

---

### 13.4. ModuleNotFoundError: No module named 'VisionWrapper'

#### Problema
```
ModuleNotFoundError: No module named 'VisionWrapper'
```

#### Causa
O pacote foi instalado como `vision_wrapper` (minúsculo), mas o código importava como `VisionWrapper` (maiúsculo).

#### Solução
**Já corrigido no código.** O import correto é:
```python
from vision_wrapper.vision_wrapper import VisionWrapper
```

Se ainda encontrar este erro em outros arquivos, corrija o import para usar `vision_wrapper` (minúsculo).

---

### 13.5. FileNotFoundError: 'deploy/resources/g1_fk.urdf'

#### Problema
```
FileNotFoundError: [Errno 2] No such file or directory: 'deploy/resources/g1_fk.urdf'
```

#### Causa
Paths hardcoded relativos ao diretório de execução (não ao arquivo Python).

#### Solução
**Já corrigido no código.** Os arquivos foram alterados para usar paths relativos ao próprio arquivo Python:

- `teleop/local2word.py` - path do URDF
- `teleop/robot_control/hand_retargeting.py` - configs das mãos
- `g1_server.py` - path do XML do MuJoCo

Se encontrar outros erros similares, use:
```python
import os
_current_dir = os.path.dirname(os.path.abspath(__file__))
_file_path = os.path.join(_current_dir, "relative", "path", "to", "file")
```

---

### 13.6. TypeError: mjv_connector() incompatible function arguments

#### Problema
```
TypeError: mjv_connector(): incompatible function arguments
```

#### Causa
API do MuJoCo mudou:
- Antiga: `mjv_makeConnector(geom, type, radius, x1, y1, z1, x2, y2, z2)`
- Nova: `mjv_connector(geom, type, radius, from_array, to_array)`

#### Solução
**Já corrigido no código.** Em `g1_server.py`, a função `add_visual_capsule` foi atualizada:

```python
# Corrigido
mujoco.mjv_connector(scene.geoms[scene.ngeom-1],
                    mujoco.mjtGeom.mjGEOM_CAPSULE, radius,
                    point1.astype(np.float64), point2.astype(np.float64))
```

---

### 13.7. Interface CycloneDDS não encontrada

#### Problema
```
enx00e6021980f3: does not match an available interface.
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain
```

#### Causa
O robô G1 estava desligado, então a interface Ethernet estava inativa.

#### Solução
1. **Ligue o robô G1** e aguarde ~30 segundos
2. Verifique a interface:
```bash
ip link show enx00e6021980f3
ping 192.168.123.164
```

Se a interface for diferente, edite `unitree_ros2/setup.sh` e os scripts `start_*.sh`.

---

## 14. Image Server (Camera RealSense)

### 14.1. Device or resource busy

#### Problema
```
[Image Server] Failed to initialize RealSense camera: xioctl(VIDIOC_S_FMT) failed, errno=16
Last Error: Device or resource busy
```

#### Causa
Outro processo já está usando a câmera RealSense (pode ser um demo ou processo travado).

#### Solução

**No G1 PC2:**

```bash
# 1. Identificar processos usando a câmera
for d in /dev/video*; do
    echo "### $d"
    sudo fuser -v "$d" || true
done

# 2. Matar o processo (substitua <PID> pelo número encontrado)
sudo kill <PID>

# Ou matar todos os processos RealSense
pkill -9 -f realsense
pkill -9 -f rs-enumerate

# 3. Tentar novamente
cd ~/onboard/image_server
python3 image_server.py
```

Se necessário, resetar USB:
```bash
sudo sh -c "echo '1-1' > /sys/bus/usb/drivers/usb/unbind"
sleep 1
sudo sh -c "echo '1-1' > /sys/bus/usb/drivers/usb/bind"
```

---

### 14.2. Image Server é Obrigatório?

#### Resposta
**NÃO!** O image_server é **OPCIONAL**.

- **COM câmera:** Mostra feed de vídeo na interface web
- **SEM câmera:** Interface fica preta, mas teleoperação funciona perfeitamente

O tracking do Vision Pro (cabeça e mãos) funciona independentemente da câmera do robô.

---

### 14.3. Obter Serial Number da RealSense

Se precisar configurar o serial number da câmera:

```bash
python3 - << 'PY'
import pyrealsense2 as rs
ctx = rs.context()
devs = ctx.query_devices()
print("Found", len(devs), "device(s)")
for d in devs:
    print("Serial:", d.get_info(rs.camera_info.serial_number),
          "Name:", d.get_info(rs.camera_info.name))
PY
```

Edite o arquivo de configuração:
```bash
nano ~/onboard/image_server/config.yaml
# Adicione o serial number encontrado
```

---

## 15. Configuração AVP_stream (Vision Pro Tracking)

### 15.1. Arquitetura Invertida - Vision Pro como Servidor

#### Problema Comum
Confusão sobre quem é cliente e quem é servidor no avp_stream.

#### Arquitetura Correta
```
┌─────────────────────┐
│   Vision Pro        │  ← SERVIDOR gRPC (roda o servidor)
│   IP: 192.168.31.4  │
│   Porta: 12345      │
└──────────▲──────────┘
           │
           │ gRPC
           │
┌──────────┴──────────┐
│   Server PC         │  ← CLIENTE (conecta ao Vision Pro)
│   g1_server.py      │
└─────────────────────┘
```

**Pontos-chave:**
- Vision Pro **roda o servidor** gRPC
- g1_server **conecta** como cliente ao Vision Pro
- Porta padrão: **12345**

---

### 15.2. Configuração Correta

#### No Server PC (config.py):
```python
VISION_WRAPPER_BACKEND = 'avp_stream'
VISION_PRO_IP = '192.168.31.4'  # IP do Vision Pro na rede WiFi
```

#### No Vision Pro (Tracking Streamer App):
```
Python Server IP: 0.0.0.0  (ou 192.168.31.4)
```

**Importante:** O campo "Python Server IP" no app é o IP que o Vision Pro vai **escutar**, não o IP para conectar!

---

### 15.3. Connection Refused

#### Problema
```
Connection refused (111)
Failed to connect to remote host: ipv4:192.168.31.4:12345
```

#### Causa
O app Tracking Streamer **não iniciou o servidor** gRPC.

#### Solução
**No Tracking Streamer:**
1. Configurar "Python Server IP": `0.0.0.0`
2. **PRESSIONAR** botão "Start Server" ou "Start Streaming"
3. Status deve mudar para "Server: Running"
4. **DEPOIS** iniciar g1_server no Server PC

**Ordem correta:**
1. Vision Pro: Iniciar servidor
2. Server PC: Iniciar g1_server
3. Conexão estabelecida

---

### 15.4. Socket Closed Durante Inicialização

#### Problema
```
Socket closed
grpc_status:14
```

Logo após ver:
```
DEGREE tensor([-4.7171]) Head DEG tensor([1.5381])
---Initialized---
```

#### Causa
Handshake gRPC incompleto - Vision Pro envia dados iniciais mas conexão não se estabelece completamente.

#### Solução Temporária
- **Manter o app aberto** no Vision Pro
- **Não** fechar o Tracking Streamer
- Se fechar, reconectar

#### Verificar Tracking Funciona
Mesmo sem logs de "connected", se o **MuJoCo viewer** mostrar as bolas coloridas seguindo suas mãos/cabeça, o tracking **está funcionando**!

---

### 15.5. Rede Local vs Internet

#### AVP_stream Requer Rede Local
```
❌ Não funciona via ngrok
✅ Funciona em rede local (Vision Pro e Server PC na mesma WiFi)
```

#### Vuer Funciona em Ambos
```
✅ Funciona via ngrok (internet)
✅ Funciona em rede local
```

---

### 15.6. Verificar Tracking Sem Logs

#### Método Visual (MuJoCo Viewer)
Quando g1_server inicia, uma **janela MuJoCo** abre automaticamente mostrando:
- 🟢 Bolas verdes/cianas = Posição das mãos
- ⚪ Bola branca = Localização do robô

**Se as bolas seguem seus movimentos = ✅ Tracking OK!**

Mesmo sem prints no terminal, isso confirma que:
- Vision Pro está transmitindo
- g1_server está recebendo
- Pipeline funcionando

#### Método Numérico (lowcmd_publisher)
```bash
# Observe os logs do lowcmd_publisher

# SEM tracking (valores fixos):
[Lowcmd Publisher] Sample joint 0 position: -0.0998
[Lowcmd Publisher] Sample joint 0 position: -0.0998

# COM tracking (valores mudando):
[Lowcmd Publisher] Sample joint 0 position: -0.0998
[Lowcmd Publisher] Sample joint 0 position: 0.1523
[Lowcmd Publisher] Sample joint 0 position: 0.2841
```

---

### 15.7. IP Forwarding no Server PC

#### Por Que Funciona Acessar Ambos IPs?
```bash
# Funciona acessar:
192.168.31.3   (IP WiFi - mesma rede que Vision Pro)
192.168.123.99 (IP Ethernet - rede do robô)
```

#### Motivo
Server PC tem **IP forwarding habilitado**:
```bash
cat /proc/sys/net/ipv4/ip_forward
# Output: 1
```

Isso permite que o Server PC **rotee** pacotes entre:
- Rede WiFi (192.168.31.x) ← Vision Pro
- Rede Ethernet (192.168.123.x) ← G1

O servidor escuta em **todas as interfaces** (`0.0.0.0:8012`), então aceita conexões de qualquer uma.

---

### 15.8. Debug Mode e Política Neural de Equilíbrio

#### Características do Debug Mode
- ✅ Controle direto dos motores via ROS2
- ❌ Desabilita política de equilíbrio **nativa da Unitree**
- ✅ Habilita **política de equilíbrio neural do CLONE** (MoE)
- 🧠 Equilíbrio controlado por rede neural treinada (52 Hz)
- ⚙️ Policy recebe: tracking (Vision Pro) + localização (FAST_LIO) + estado do robô

#### Como Funciona
O robô **TEM equilíbrio**, mas é fornecido pela política neural do CLONE:
1. Vision Pro envia pose das mãos/cabeça do operador
2. Policy neural (MoE) processa e gera comandos de junta
3. Comandos incluem controle de equilíbrio aprendido por RL
4. lowcmd_publisher envia comandos a 1008 Hz para os motores

#### Recomendações Iniciais
- Calibração (R1/R2) é **crítica** para política funcionar corretamente
- Primeiros testes: robô apoiado/suspenso até validar tracking e calibração
- Após validação: robô pode operar livremente com política neural

---

## 🔗 Referências Úteis

- [Livox-SDK2 GitHub](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2 GitHub](https://github.com/Livox-SDK/livox_ros_driver2)
- [FAST_LIO GitHub](https://github.com/hku-mars/FAST_LIO)
- [FAST_LIO_LOCALIZATION GitHub](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
- [CLONE Deploy README](https://github.com/humanoid-clone/CLONE/blob/main/deploy/README.md)

---

**Última atualização:** 2025-11-21
**Testado em:** Unitree G1 EDU (PC2), Ubuntu 20.04/22.04, ROS Noetic/Humble
