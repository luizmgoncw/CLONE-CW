# Tutorial: Configurar CLONE com Inspire FTP Hand

Este tutorial detalha todos os passos necessários para configurar o projeto CLONE para funcionar com as mãos Inspire FTP.

---

## Sumário

1. [Pré-requisitos](#1-pré-requisitos)
2. [Instalar Dependências](#2-instalar-dependências)
3. [Copiar Arquivos Necessários](#3-copiar-arquivos-necessários)
4. [Modificar Arquivos de Configuração](#4-modificar-arquivos-de-configuração)
5. [Modificar Código Python](#5-modificar-código-python)
6. [Configuração de Rede](#6-configuração-de-rede)
7. [Execução](#7-execução)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Pré-requisitos

### Hardware
- Robô G1 com mãos Inspire FTP instaladas
- Apple Vision Pro (ou outro dispositivo XR compatível)
- PC com Ubuntu 22.04 ou superior
- Conexão de rede com o robô (ethernet recomendado)

### Software Base
- Python 3.8+
- ROS2 Humble (ou versão compatível)
- CUDA (opcional, para aceleração GPU)

### IPs das Mãos Inspire FTP
| Mão | IP |
|-----|-----|
| Direita | `192.168.123.211` |
| Esquerda | `192.168.123.210` |

---

## 2. Instalar Dependências

### 2.1 Inspire Hand SDK

O SDK está localizado em: `~/Documents/Unitree/inspire_hand_ws/inspire_hand_sdk`

```bash
# Navegar até o diretório do SDK
cd ~/Documents/Unitree/inspire_hand_ws/inspire_hand_sdk

# Instalar o SDK
pip install -e .
```

**Dependências instaladas automaticamente:**
- `cyclonedds==0.10.2`
- `numpy`
- `PyQt5`
- `pyqtgraph`
- `colorcet`
- `pymodbus==3.6.9`
- `pyserial`

### 2.2 Unitree SDK2 Python (versão do inspire_hand_ws)

```bash
cd ~/Documents/Unitree/inspire_hand_ws/unitree_sdk2_python
pip install -e .
```

### 2.3 Dex Retargeting

```bash
pip install dex-retargeting
```

### 2.4 Outras dependências

```bash
pip install transforms3d pytorch-kinematics mujoco
```

---

## 3. Copiar Arquivos Necessários

### 3.1 Estrutura de diretórios esperada

```
CLONE/deploy/
├── config.py
├── g1_server.py
├── dex_server.py                    # MODIFICAR ou SUBSTITUIR
├── resources/
│   └── inspire_hand/
│       ├── inspire_hand.yml         # SUBSTITUIR
│       ├── inspire_hand_left.urdf
│       ├── inspire_hand_right.urdf
│       └── meshes/
└── teleop/
    └── robot_control/
        ├── hand_retargeting.py      # SUBSTITUIR
        ├── robot_hand_inspire.py    # ATUALIZAR
        └── robot_hand_inspire_ftp.py # NOVO - COPIAR
```

### 3.2 Copiar arquivos da versão funcional

```bash
# Criar backup dos arquivos originais
cd ~/Documents/CLONE/CLONE/deploy
cp teleop/robot_control/hand_retargeting.py teleop/robot_control/hand_retargeting.py.bak
cp resources/inspire_hand/inspire_hand.yml resources/inspire_hand/inspire_hand.yml.bak

# Copiar arquivos da versão funcional
cp ~/Documents/Unitree/xr_teleoperate/teleop/robot_control/hand_retargeting.py \
   teleop/robot_control/hand_retargeting.py

cp ~/Documents/Unitree/xr_teleoperate/teleop/robot_control/robot_hand_inspire_ftp.py \
   teleop/robot_control/robot_hand_inspire_ftp.py

cp ~/Documents/Unitree/xr_teleoperate/assets/inspire_hand/inspire_hand.yml \
   resources/inspire_hand/inspire_hand.yml

# Copiar logging_mp se não existir
cp ~/Documents/Unitree/xr_teleoperate/teleop/logging_mp.py \
   teleop/logging_mp.py 2>/dev/null || echo "Arquivo já existe ou não encontrado"
```

---

## 4. Modificar Arquivos de Configuração

### 4.1 config.py

Adicionar configuração para Inspire Hand:

```python
# ~/Documents/CLONE/CLONE/deploy/config.py

VISION_WRAPPER_BACKEND = 'avp_stream'  # ou 'vuer'
VISION_PRO_IP = '192.168.31.5'
VISION_PRO_DELTA_H = -0.54

# Tipo de mão: 'dex3' ou 'inspire'
HAND_TYPE = 'inspire'
USE_DEX_HANDS = True

# IPs das mãos Inspire FTP
INSPIRE_RIGHT_HAND_IP = '192.168.123.211'
INSPIRE_LEFT_HAND_IP = '192.168.123.210'
```

### 4.2 inspire_hand.yml (já copiado)

O arquivo deve ter `type: DexPilot` (não `vector`):

```yaml
left:
  type: DexPilot
  urdf_path: inspire_hand/inspire_hand_left.urdf
  target_joint_names:
    [
     'L_thumb_proximal_yaw_joint',
     'L_thumb_proximal_pitch_joint',
     'L_index_proximal_joint',
     'L_middle_proximal_joint',
     'L_ring_proximal_joint',
     'L_pinky_proximal_joint'
    ]
  wrist_link_name: "L_hand_base_link"
  finger_tip_link_names: [ "L_thumb_tip", "L_index_tip", "L_middle_tip", "L_ring_tip", "L_pinky_tip" ]
  target_link_human_indices_dexpilot: [[ 9, 14, 19, 24, 14, 19, 24, 19, 24, 24, 0, 0, 0, 0, 0], [ 4, 4, 4, 4, 9, 9, 9, 14, 14, 19, 4, 9, 14, 19, 24]]
  scaling_factor: 1.20
  low_pass_alpha: 0.2

right:
  type: DexPilot
  urdf_path: inspire_hand/inspire_hand_right.urdf
  # ... (similar ao left, com prefixo R_)
```

---

## 5. Modificar Código Python

### 5.1 Atualizar hand_retargeting.py

O arquivo copiado já deve ter os paths corretos. Verifique os caminhos:

```python
# ~/Documents/CLONE/CLONE/deploy/teleop/robot_control/hand_retargeting.py

class HandType(Enum):
    INSPIRE_HAND = "../resources/inspire_hand/inspire_hand.yml"  # Ajustar path
    INSPIRE_HAND_Unit_Test = "../../resources/inspire_hand/inspire_hand.yml"
    UNITREE_DEX3 = "../resources/unitree_hand/unitree_dex3.yml"
    UNITREE_DEX3_Unit_Test = "../../resources/unitree_hand/unitree_dex3.yml"
```

**Ajuste os caminhos** conforme a estrutura do CLONE:
- De `../assets/` para `../resources/`

### 5.2 Atualizar robot_hand_inspire_ftp.py

Ajustar o path do inspire_hand_sdk:

```python
# ~/Documents/CLONE/CLONE/deploy/teleop/robot_control/robot_hand_inspire_ftp.py

# Linha ~12: Ajustar path do SDK
inspire_hand_ws_path = os.path.expanduser("~/Documents/Unitree/inspire_hand_ws")
sys.path.insert(0, os.path.join(inspire_hand_ws_path, "inspire_hand_sdk"))
sys.path.insert(0, os.path.join(inspire_hand_ws_path, "unitree_sdk2_python"))
```

### 5.3 Criar/Modificar dex_server.py para Inspire

Você tem duas opções:

#### Opção A: Modificar dex_server.py existente

Alterar as seguintes linhas em `dex_server.py`:

```python
# Linha 4: Adicionar import
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType

# Linha 16: Alterar índices dos dedos
# DE: unitree_tip_indices = [4, 9, 14]  # 3 dedos DEX3
# PARA:
inspire_tip_indices = [4, 9, 14, 19, 24]  # 5 dedos Inspire

# Linha 17: Alterar número de motores
# DE: Dex3_Num_Motors = 7
# PARA:
Inspire_Num_Motors = 6

# Linha 109: Alterar tipo de mão
# DE: self.hand_retargeting = HandRetargeting(HandType.UNITREE_DEX3)
# PARA:
self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
```

#### Opção B: Usar Inspire_FTP_Controller diretamente (Recomendado)

Modificar `g1_server.py` para usar o controlador Inspire FTP em vez do dex_server:

```python
# Em g1_server.py, substituir:
# from dex_server import start_dex_server

# Por:
from teleop.robot_control.robot_hand_inspire_ftp import Inspire_FTP_Controller
```

---

## 6. Configuração de Rede

### 6.1 Verificar conectividade com as mãos

```bash
# Testar conexão com mão direita
ping 192.168.123.211

# Testar conexão com mão esquerda
ping 192.168.123.210
```

### 6.2 Configurar interface de rede

O DDS precisa da interface de rede correta:

```bash
# Verificar interfaces disponíveis
ip addr

# A interface para o robô geralmente é eth0 ou enp*
# Certifique-se que está na subnet 192.168.123.x
```

### 6.3 Inicializar DDS com interface correta

No código, o DDS deve ser inicializado com a interface:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# Inicializar com interface específica
ChannelFactoryInitialize(0, "eth0")  # ou a interface correta
```

---

## 7. Execução

### 7.1 Ordem de inicialização

1. **Ligar o robô G1** e aguardar inicialização
2. **Verificar conexão de rede** com as mãos
3. **Iniciar o Vision Pro** e conectar ao streaming
4. **Executar o servidor**

### 7.2 Comando de execução

```bash
cd ~/Documents/CLONE/CLONE/deploy

# Com Vision Pro
python g1_server.py

# OU se estiver usando xr_teleoperate diretamente:
cd ~/Documents/Unitree/xr_teleoperate
python teleop/teleop_hand_and_arm.py --ee inspire1
```

### 7.3 Parâmetros importantes

| Parâmetro | Descrição | Valores |
|-----------|-----------|---------|
| `--ee` | Tipo de end-effector | `inspire1`, `dex3`, `dex1` |
| `--sim` | Modo simulação | flag |
| `--iface` | Interface de rede | `eth0`, `enp0s31f6`, etc |

---

## 8. Troubleshooting

### 8.1 Erro: "ModbusTCP connection timeout"

**Causa:** Não consegue conectar às mãos via ModbusTCP

**Solução:**
1. Verificar IPs das mãos (`ping 192.168.123.211`)
2. Verificar se está na mesma subnet
3. Verificar se as mãos estão ligadas

### 8.2 Erro: "DDS publisher initialization failed"

**Causa:** Interface de rede incorreta para DDS

**Solução:**
```python
# Inicializar DDS com interface correta
ChannelFactoryInitialize(0, "eth0")
```

### 8.3 Erro: "Configuration file not found"

**Causa:** Caminhos incorretos no hand_retargeting.py

**Solução:**
Verificar se os caminhos em `HandType` apontam para os arquivos corretos:
```python
INSPIRE_HAND = "../resources/inspire_hand/inspire_hand.yml"
```

### 8.4 Erro: "inspire_sdkpy not found"

**Causa:** SDK não instalado

**Solução:**
```bash
cd ~/Documents/Unitree/inspire_hand_ws/inspire_hand_sdk
pip install -e .
```

### 8.5 Mãos não respondem aos comandos

**Verificar:**
1. Tópicos DDS estão sendo publicados:
   ```bash
   # Listar tópicos DDS ativos
   ros2 topic list | grep inspire
   ```
2. Valores sendo enviados estão no range correto [0-1000]
3. ModbusDataHandler está conectado

### 8.6 Movimentos invertidos ou incorretos

**Causa:** Mapeamento de juntas incorreto

**Solução:** Verificar ordem das juntas em `hand_retargeting.py`:
```python
self.left_inspire_api_joint_names = [
    'L_pinky_proximal_joint',      # 0
    'L_ring_proximal_joint',       # 1
    'L_middle_proximal_joint',     # 2
    'L_index_proximal_joint',      # 3
    'L_thumb_proximal_pitch_joint', # 4
    'L_thumb_proximal_yaw_joint'   # 5
]
```

---

## Resumo de Diferenças: DEX3 vs Inspire FTP

| Aspecto | Unitree DEX3 | Inspire FTP |
|---------|--------------|-------------|
| Dedos | 3 (polegar, índice, médio) | 5 (todos) |
| DOFs por mão | 7 | 6 |
| Comunicação | DDS apenas | ModbusTCP + DDS |
| Tópicos | `dex3/*/cmd` | `rt/inspire_hand/ctrl/*` |
| Tipo retargeting | `vector` | `DexPilot` |
| Range valores | Radianos | [0-1000] normalizado |
| IPs | N/A | 192.168.123.210/211 |

---

## Arquivos de Referência

| Arquivo | Localização | Descrição |
|---------|-------------|-----------|
| Versão funcional | `~/Documents/Unitree/xr_teleoperate/` | Implementação completa Inspire |
| Inspire SDK | `~/Documents/Unitree/inspire_hand_ws/inspire_hand_sdk/` | SDK de comunicação |
| CLONE atual | `~/Documents/CLONE/CLONE/deploy/` | Projeto a ser modificado |

---

## 9. Correções Adicionais Necessárias

### 9.1 dex-retargeting (versão customizada)

A versão do PyPI (0.1.1 - 0.5.0) **não suporta** o campo `target_link_human_indices_dexpilot`. É necessário instalar a versão customizada do xr_teleoperate:

```bash
source ~/Documents/CLONE/unitree_ros2/venv/bin/activate
pip install -e ~/Documents/Unitree/xr_teleoperate/teleop/robot_control/dex-retargeting
pip install pin==2.7.0  # Versão compatível
```

### 9.2 Correções de paths em hand_retargeting.py

Alterar de `../assets/` para `./resources/` (relativo ao diretório de execução `/deploy/`):

```python
class HandType(Enum):
    INSPIRE_HAND = "./resources/inspire_hand/inspire_hand.yml"  # NÃO ../

# E também:
RetargetingConfig.set_default_urdf_dir('./resources')  # NÃO ../
```

### 9.3 Correção de import em hand_retargeting.py

```python
# DE:
from dex_retargeting import RetargetingConfig
# PARA:
from dex_retargeting.retargeting_config import RetargetingConfig
```

### 9.4 logging_mp.py

Copiar para `/deploy/` (não apenas `/deploy/teleop/`):

```bash
cp ~/Documents/Unitree/xr_teleoperate/teleop/logging_mp.py \
   ~/Documents/CLONE/CLONE/deploy/logging_mp.py
```

### 9.5 config.py - adicionar NETWORK_INTERFACE

```python
# Interface de rede para DDS (Inspire hands)
NETWORK_INTERFACE = 'enx00e6021980f3'
```

### 9.6 g1_server.py - inicialização DDS

Adicionar `ChannelFactoryInitialize` antes de criar o Inspire controller:

```python
# No topo, após imports condicionais:
if HAND_TYPE == 'inspire':
    from teleop.robot_control.robot_hand_inspire_ftp import Inspire_FTP_Controller
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# Em launch_dex_hands():
if HAND_TYPE == 'inspire':
    ChannelFactoryInitialize(0, NETWORK_INTERFACE)  # ANTES de criar controller
    self.inspire_controller = Inspire_FTP_Controller(...)
```

---

## Contatos e Recursos

- [Unitree G1 Developer Guide](https://support.unitree.com/home/en/G1_developer)
- [Inspire FTP Hand Documentation](https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand)
- [xr_teleoperate Issues](https://github.com/unitreerobotics/xr_teleoperate/issues)
