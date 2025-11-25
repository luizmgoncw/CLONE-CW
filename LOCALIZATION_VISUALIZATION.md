# 📊 Visualização de Localização no Server PC

Este guia mostra como visualizar os dados de localização do G1 no Server PC usando RViz2.

---

## ⚡ Quick Start

### **No G1 PC2:**

```bash
# Terminal 1 - SSH
ssh unitree@192.168.123.164
cd ~/onboard
./localization_server.sh

# Terminal 2 - SSH (nova janela)
ssh unitree@192.168.123.164
cd ~/onboard
python3 cloud_server.py
```

### **No Server PC:**

```bash
cd /home/luizmarques/Documents/CLONE
./start_localization_viz.sh
```

**Pronto!** O RViz2 abrirá mostrando:
- 🎯 Seta verde = Pose do robô
- 🔵 Linha ciano = Trajetória
- 🟡 Pontos amarelos = LiDAR em tempo real

---

## 🎯 O Que Você Pode Visualizar

### ✅ Disponível Agora:
- **Localização do robô** (pose 3D: posição + orientação)
- **Trajetória** (path histórico de 500 poses)
- **Frames TF** (`map` → `body`)
- **Nuvem de pontos do LiDAR** (PointCloud2 em tempo real) 🆕

### 🔧 Futuro (Requer Configuração Adicional):
- Mapa 3D completo (PCD)
- Múltiplas nuvens (scan atual vs mapa)

---

## 🚀 Como Usar

### Pré-requisitos:
1. **G1 PC2** rodando:
   - `localization_server.sh` (FAST_LIO + pos_server na porta 6006)
   - `cloud_server.py` (publica PointCloud na porta 6007) 🆕
2. **Server PC** conectado ao G1 via Ethernet

### Passo 1: Iniciar Servidores no G1 PC2

**Terminal 1 - SSH no G1:**
```bash
ssh unitree@192.168.123.164
cd ~/onboard
./localization_server.sh
```

**Terminal 2 - SSH no G1 (Novo!):**
```bash
ssh unitree@192.168.123.164
cd ~/onboard
python3 cloud_server.py
```

### Passo 2: Iniciar Visualização no Server PC

```bash
cd /home/luizmarques/Documents/CLONE
./start_localization_viz.sh
```

O script automaticamente:
1. Carrega ambiente ROS2 Humble
2. Inicia `localization_bridge.py` (ZeroMQ → ROS2)
   - Conecta em `192.168.123.164:6006` (localização)
   - Conecta em `192.168.123.164:6007` (PointCloud)
3. Abre RViz2 com configuração pré-definida

---

## 📡 Arquitetura

```
┌───────────────────────────────────────────────────────┐
│ G1 PC2 (ROS1)                                         │
│                                                       │
│ FAST_LIO_LOCALIZATION publishes:                      │
│  ├─ /localization (Odometry)                          │
│  └─ /cloud_registered (PointCloud2)                   │
│           ↓                    ↓                       │
│    pos_server.py        cloud_server.py               │
│           ↓                    ↓                       │
│    ZeroMQ PUB (6006)    ZeroMQ PUB (6007)             │
└───────────────────────────────────────────────────────┘
              │                    │
         Ethernet             Ethernet
              │                    │
┌───────────────────────────────────────────────────────┐
│ Server PC (ROS2)                                      │
│              │                    │                   │
│    ZeroMQ SUB (6006)    ZeroMQ SUB (6007)             │
│              └──────────┬──────────┘                  │
│                  localization_bridge.py               │
│                         ↓                             │
│   ├─ /localization (Odometry)                         │
│   ├─ /localization_path (Path)                        │
│   ├─ /cloud_registered (PointCloud2) 🆕               │
│   └─ TF: map → body                                   │
│                         ↓                             │
│                      RViz2                            │
└───────────────────────────────────────────────────────┘
```

**Não há bridge ROS1/ROS2!** O `localization_bridge.py` recebe via ZeroMQ e republica como ROS2.

---

## 📋 Topics ROS2 Disponíveis

Quando o bridge está rodando, você pode listar:

```bash
# Em outro terminal com ROS2 carregado
source /opt/ros/humble/setup.bash
source ~/Documents/CLONE/unitree_ros2/install/setup.bash

# Listar topics
ros2 topic list

# Verificar frequência
ros2 topic hz /localization

# Ver dados
ros2 topic echo /localization

# Ver TF tree
ros2 run tf2_tools view_frames
```

Topics publicados:
- `/localization` - `nav_msgs/msg/Odometry` (pose do robô)
- `/localization_path` - `nav_msgs/msg/Path` (trajetória)
- `/cloud_registered` - `sensor_msgs/msg/PointCloud2` (nuvem de pontos) 🆕
- TF: `map` → `body`

---

## 🎨 RViz2 - Configuração

O arquivo `localization_visualization.rviz2` já está configurado com:

### Displays Ativos:
1. **Grid** - Grade XY no frame `map`
2. **Map Frame** - Eixos do frame `map`
3. **Body Frame** - Eixos do frame `body` (robô)
4. **Localization (Pose)** - Seta mostrando pose atual
5. **Odometry** - Histórico de 100 poses com setas
6. **Path** - Trajetória contínua (500 poses)
7. **PointCloud** - Nuvem de pontos do LiDAR (amarelo) 🆕

### Controles:
- **Mouse esquerdo + arrastar**: Rotacionar câmera
- **Mouse direito + arrastar**: Pan (mover câmera)
- **Scroll**: Zoom
- **Middle click**: Reset view

---

## 🐛 Troubleshooting

### Problema: "No messages received"

**Verificar se pos_server e cloud_server estão rodando no G1:**
```bash
ssh unitree@192.168.123.164
ps aux | grep -E "(pos_server|cloud_server)"
```

Deve mostrar 2 processos Python rodando. Se não:
```bash
# Terminal 1
cd ~/onboard
python3 pos_server.py &

# Terminal 2
cd ~/onboard
python3 cloud_server.py &
```

---

### Problema: "Address already in use" (porta 6006 ou 6007)

**Há um processo antigo rodando. Matar e reiniciar:**
```bash
ssh unitree@192.168.123.164

# Matar processos antigos
pkill -f "pos_server\|cloud_server"

# Aguardar 2 segundos
sleep 2

# Reiniciar
cd ~/onboard
./localization_server.sh

# Em outro terminal
python3 cloud_server.py
```

---

### Problema: "Message Filter dropping message: frame 'camera_init'"

**A nuvem de pontos não aparece e vê esse erro repetido.**

**Causa:** O frame_id da PointCloud é `camera_init`, mas RViz2 está configurado para `map`.

**Solução Permanente (Recomendada):**

O arquivo `cloud_server.py` **já foi corrigido** para publicar com frame `map`. Se você copiou a versão antiga, atualize:

```bash
# Copiar versão corrigida
scp /home/luizmarques/Documents/CLONE/CLONE/deploy/onboard/cloud_server.py unitree@192.168.123.164:~/onboard/

# Reiniciar cloud_server no G1
ssh unitree@192.168.123.164
pkill -f cloud_server
cd ~/onboard
python3 cloud_server.py
```

**Solução Temporária:**

No RViz2, mude "Fixed Frame" de `map` para `camera_init` no painel "Global Options".

---

### Problema: "Connection refused"

**Verificar conectividade de rede:**
```bash
ping 192.168.123.164
telnet 192.168.123.164 6006
```

Se ping funciona mas telnet não, pos_server não está rodando.

---

### Problema: "No transform from 'map' to 'body'"

**Verificar se bridge está publicando TF:**
```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo map body
```

Se não mostrar nada, verificar logs do bridge.

---

### Problema: Bridge não inicia (ModuleNotFoundError)

**Verificar dependências instaladas:**
```bash
source ~/Documents/CLONE/unitree_ros2/venv/bin/activate
pip list | grep -E "(rclpy|zmq)"
```

Deve mostrar:
- `pyzmq`
- Não mostra rclpy (vem do ROS2, não pip)

Se faltar pyzmq:
```bash
pip install pyzmq
```

---

## 🔧 Executar Manualmente (Debug)

Se quiser rodar os componentes separadamente:

### Terminal 1 - Bridge:
```bash
cd /home/luizmarques/Documents/CLONE
source /opt/ros/humble/setup.bash
source unitree_ros2/install/setup.bash
source unitree_ros2/venv/bin/activate
cd CLONE/deploy
python3 localization_bridge.py
```

### Terminal 2 - RViz2:
```bash
source /opt/ros/humble/setup.bash
rviz2 -d /home/luizmarques/Documents/CLONE/localization_visualization.rviz2
```

---

## 📊 Adicionar Pontos do LiDAR (Avançado)

Se quiser visualizar os pontos do LiDAR em tempo real, você precisa:

### Opção 1: Bridge ROS1/ROS2 Completo

Instalar ros1_bridge (complexo, requer compilação):
```bash
sudo apt install ros-humble-ros1-bridge
```

Iniciar bridge bidirecional entre ROS1 (PC2) e ROS2 (Server).

### Opção 2: ZeroMQ PointCloud Bridge (Recomendado)

Criar servidor ZeroMQ no PC2 similar ao `pos_server.py`, mas para PointCloud2:

**No G1 PC2 - `cloud_server.py`:**
```python
import rospy
import zmq
from sensor_msgs.msg import PointCloud2

def callback(msg):
    # Serializar PointCloud2
    # Enviar via ZeroMQ
    pass

rospy.Subscriber('/cloud_registered', PointCloud2, callback)
```

**No Server PC - Estender `localization_bridge.py`:**
```python
# Adicionar subscriber ZeroMQ para PointCloud
# Republicar como ROS2 PointCloud2
```

Se tiver interesse, posso criar esses scripts!

---

## 📈 Métricas Esperadas

Com o sistema funcionando:
- **Frequência de localização**: ~10 Hz (limitado pelo FAST_LIO)
- **Frequência PointCloud**: ~10 Hz (com downsampling 4x)
- **Pontos por frame**: ~5.000-20.000 (original: 20.000-80.000)
- **Tamanho por mensagem**: ~60-240 KB (após downsampling)
- **Latência ZeroMQ**: < 10 ms (rede local)
- **Taxa de publicação ROS2**: 50 Hz localization, 10 Hz cloud

**Logs esperados:**

**cloud_server.py (G1):**
```
[Cloud Server] Rate: 10.0 Hz | Points: 12543/50172 (4.0x) | Size: 147.2 KB
```

**localization_bridge.py (Server):**
```
Loc: 10.1 Hz | Cloud: 10.0 Hz (12543 pts)
```

---

## ⚙️ Configurações Avançadas

### Desabilitar PointCloud (só localização):

Edite `localization_bridge.py` linha 30:
```python
self.declare_parameter('enable_cloud', False)  # Era True
```

### Ajustar Downsampling (mais/menos pontos):

Edite `cloud_server.py` no G1, linha 96:
```python
'downsample_factor': 4,  # 4 = 25% dos pontos
                          # 2 = 50% dos pontos
                          # 8 = 12.5% dos pontos
```

### Ajustar Taxa de Publicação do PointCloud:

Edite `cloud_server.py` no G1, linha 95:
```python
'publish_rate': 10.0,  # Hz (10 Hz padrão)
                       # Reduzir para economizar banda
```

---

## 🛑 Parar Sistema

**Server PC:** Ctrl+C no terminal do RViz2 (mata bridge automaticamente)

**G1 PC2:**
```bash
ssh unitree@192.168.123.164
pkill -f "localization_server\|cloud_server\|pos_server"
```

---

## 🎯 Próximos Passos

1. ✅ Visualização básica (pose + trajetória + nuvem)
2. **Carregar mapa PCD** para comparação visual
3. **Adicionar marcadores customizados** (mãos do Vision Pro, etc.)
4. **Otimizar compressão** da nuvem se necessário

---

**Criado em**: 2025-11-21
**Última atualização**: 2025-11-21
