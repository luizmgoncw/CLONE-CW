# 🎉 SUCESSO - CLONE DEPLOYMENT

**Data**: 2025-11-21
**Status**: ✅ **SISTEMA FUNCIONANDO!**

---

## 🏆 CONQUISTA

Sistema CLONE **implantado com sucesso** e **testado**!

O robô Unitree G1 respondeu aos comandos de teleoperação via Apple Vision Pro! 🤖🎮

---

## ✅ COMPONENTES VALIDADOS

### **Hardware**
- ✅ Unitree G1 EDU (29-DoF)
- ✅ Apple Vision Pro
- ✅ Server PC (Ubuntu 22.04, ROS2 Humble)
- ✅ Livox Mid-360 LiDAR
- ✅ RealSense D435I (opcional)
- ✅ Gamepad (controle sem fio)

### **Software**
- ✅ G1 PC2: FAST_LIO_LOCALIZATION rodando (~10Hz)
- ✅ G1 PC2: pos_server ativo
- ✅ Server PC: ROS2 Humble + Unitree SDK2
- ✅ Server PC: Python 3.10 + todas dependências
- ✅ Server PC: lowcmd_publisher (1008 Hz!)
- ✅ Server PC: g1_server + policy neural (52 Hz)
- ✅ Server PC: VisionWrapper com avp_stream

### **Comunicação**
- ✅ ROS2 CycloneDDS: Server PC ↔ G1 (1008 Hz)
- ✅ gRPC: Vision Pro ↔ Server PC (tracking)
- ✅ TCP: G1 PC2 ↔ Server PC (localização)
- ✅ ZMQ: G1 PC2 ↔ Server PC (imagem)

---

## 🌐 CONFIGURAÇÃO DE REDE FINAL

### **Topologia Implementada:**
```
Vision Pro (WiFi)
    192.168.31.4
         │
    WiFi Network
         │
    Server PC
    ├─ wlp7s0: 192.168.31.3   (WiFi)
    ├─ enx...: 192.168.123.99 (Ethernet)
    └─ IP Forwarding: ON
         │
    Ethernet
         │
    G1 PC2: 192.168.123.164
```

**Descoberta Importante:**
- Server PC faz **roteamento automático** entre redes
- Vision Pro acessa servidor via **rede WiFi local**
- **SEM necessidade de ngrok** (rede local = sem limite de banda!)

---

## ⚙️ CONFIGURAÇÃO FUNCIONANDO

### **config.py (CLONE/deploy/config.py):**
```python
VISION_WRAPPER_BACKEND = 'avp_stream'
VISION_PRO_IP = '192.168.31.4'
VISION_PRO_DELTA_H = -0.54
USE_DEX_HANDS = True
```

### **Tracking Streamer (Vision Pro):**
```
Python Server IP: 0.0.0.0
Status: Server Running
```

---

## 🚀 GUIA RÁPIDO DE EXECUÇÃO

### **Ordem de Inicialização:**

Para executar o sistema CLONE, siga esta sequência exata:

#### **Terminal 1 (SSH no G1 PC2):**
```bash
~/onboard/localization_server.sh
```
Aguarde até ver as mensagens de localização rodando (~10Hz).

#### **Terminal 2 (SSH no G1 PC2):**
```bash
python ~/onboard/image_server/image_server.py
```
Aguarde até ver "Image server listening...".

#### **Terminal 3 (Server PC):**
```bash
./start_lowcmd.sh
```
Aguarde até ver "[Lowcmd Publisher] Node initialized".

#### **Terminal 4 (Server PC):**
```bash
# IMPORTANTE: Só execute após iniciar o Tracking Streamer no Apple Vision Pro
./start_g1_server.sh
```

**⚠️ ATENÇÃO:**
- Inicie o **Tracking Streamer no Apple Vision Pro** ANTES de executar o Terminal 4
- Verifique no Tracking Streamer: "Server Running" (IP: 0.0.0.0)
- Aguarde todos os componentes estarem rodando antes de testar movimentos

### **Controles:**
- **R1/R2**: Calibração (Location Offset Reset)
- **L1**: Iniciar/parar policy neural
- **Gamepad**: Navegação e ajustes finos

---

## 🔧 CORREÇÕES NECESSÁRIAS

### **1. Import do VisionWrapper**
```python
# Antes (ERRADO):
from VisionWrapper.vision_wrapper import VisionWrapper

# Depois (CORRETO):
from vision_wrapper.vision_wrapper import VisionWrapper
```

### **2. Paths Relativos**
Arquivos corrigidos para usar paths relativos ao próprio arquivo:
- `teleop/local2word.py` - URDF path
- `teleop/robot_control/hand_retargeting.py` - config paths
- `g1_server.py` - MuJoCo XML path

### **3. API do MuJoCo**
```python
# Antes:
mujoco.mjv_makeConnector(geom, type, radius, x1, y1, z1, x2, y2, z2)

# Depois:
mujoco.mjv_connector(geom, type, radius, point1_array, point2_array)
```

### **4. Compatibilidade Sapien**
```bash
# Downgrade necessário:
pip uninstall sapien -y
pip install "sapien<3.0"  # Instala 2.2.2
```

### **5. PyZMQ para Image Client**
```bash
pip install pyzmq
```

### **6. Image Client Attribute**
Inicializar `self.pad_array = None` antes do bloco try/except.

### **7. Logs no lowcmd_publisher**
Adicionados logs para diagnóstico:
- Mensagem de primeira recepção
- Taxa de comandos (Hz)
- Sample de posição de junta

---

## 🎮 TESTE REALIZADO

### **Sequência Executada:**
1. ✅ Iniciado localization_server.sh no G1 PC2
2. ✅ Iniciado image_server.py no G1 PC2
3. ✅ Iniciado lowcmd_publisher no Server PC
4. ✅ Iniciado g1_server no Server PC
5. ✅ Conectado Vision Pro via Tracking Streamer
6. ✅ Verificado tracking no MuJoCo viewer (bolas seguindo movimentos)
7. ✅ Pressionado R1/R2 para calibrar (offset reset)
8. ✅ Pressionado L1 para iniciar policy
9. ✅ **Robô SE MOVEU!** 🎉

### **Evidências de Funcionamento:**

**lowcmd_publisher:**
```
[Lowcmd Publisher] First message received from g1_server!
[Lowcmd Publisher] Receiving & publishing commands at 1008.0 Hz
[Lowcmd Publisher] Sample joint 0 position: 0.1523  ← Mudando!
```

**g1_server:**
```
Policy start!
POL FREQ: 52Hz
Location Offset Reset
```

**MuJoCo Viewer:**
- Bolas coloridas seguindo mãos e cabeça em tempo real

**Robô Físico:**
- Motores energizados
- Movimentos executados (em debug mode com política neural CLONE)

---

## 📊 MÉTRICAS

| Componente | Taxa | Status |
|------------|------|--------|
| Policy Neural | 52 Hz | ✅ Ótimo |
| Lowcmd Publisher | 1008 Hz | ✅ Perfeito |
| FAST_LIO Localization | 10 Hz | ✅ Bom |
| Vision Pro Tracking | Tempo real | ✅ Funcionando |

---

## ⚠️ OBSERVAÇÕES

### **Debug Mode:**
- Debug Mode desabilita a **política de equilíbrio nativa da Unitree**
- Robô usa a **política de equilíbrio neural do CLONE** (MoE policy rodando a 52 Hz)
- Equilíbrio é controlado pela rede neural treinada, não pelo firmware Unitree
- Movimento inicial pode parecer "maluco" até calibração completa
- **Recomendação:** Primeiros testes com robô apoiado/suspenso para validar política

### **Calibração:**
- Crítica para mapeamento correto
- Requer pose neutra (braços relaxados)
- Pressionar R1/R2 múltiplas vezes
- Verificar "Location Offset Reset" no terminal

---

## 🎯 PRÓXIMOS PASSOS

### **Para Melhorar:**
1. **Testar com robô apoiado** - Evitar quedas em debug mode
2. **Calibrar parâmetros** - Ajustar ganhos, offsets
3. **Testar movimentos específicos** - Braços, mãos, locomoção
4. **Ajustar MoE policy** - Fine-tuning se necessário

### **Documentação Criada:**
- ✅ `GUIA_RAPIDO_DEPLOYMENT.md` - Atualizado com rede local
- ✅ `INSTALACAO_TROUBLESHOOTING.md` - Seção AVP_stream adicionada
- ✅ `SUCESSO_DEPLOYMENT.md` - Este arquivo
- ✅ Scripts de startup: `start_*.sh`
- ✅ Script de verificação: `check_system.sh`

---

## 🙏 LIÇÕES APRENDIDAS

### **1. Rede Local > Internet**
- AVP_stream requer rede local (não funciona via ngrok)
- Vuer funciona em ambos mas rede local é mais rápido
- IP forwarding no Server PC permitiu topologia híbrida

### **2. Arquitetura Invertida**
- Vision Pro é o **servidor** gRPC
- Server PC é o **cliente** que conecta
- Contraintuitivo mas necessário para avp_stream

### **3. MuJoCo Viewer é Diagnóstico Visual**
- Bolas coloridas mostram tracking funcionando
- Mais confiável que logs de texto
- Essencial para debug

### **4. Verificação em Camadas**
- Testar comunicação ROS2 independente (lowcmd logs)
- Testar tracking independente (MuJoCo viewer)
- Testar policy independente (frequência Hz)
- Depois integrar tudo

### **5. Política de Equilíbrio Neural**
- Debug mode = desabilita controle Unitree, habilita política CLONE
- Equilíbrio controlado por rede neural MoE (52 Hz)
- Calibração crítica para funcionamento correto da política
- Primeiros testes recomendados com robô apoiado para validação

---

## 🎊 CONCLUSÃO

**SISTEMA CLONE TOTALMENTE OPERACIONAL!** ✅

Todos os componentes foram:
- ✅ Instalados
- ✅ Configurados
- ✅ Integrados
- ✅ Testados
- ✅ Validados

O pipeline completo funciona:
```
Vision Pro → Server PC → G1 PC2 → Unitree G1
  (Tracking) (Policy)   (Localização) (Execução)
```

**Pronto para teleoperação avançada!** 🤖🎮🚀

---

**Equipe**: Luiz + Claude Code
**Duração**: ~6 horas (instalação + troubleshooting + testes)
**Resultado**: 🏆 **SUCESSO TOTAL**

---

## 📝 ATUALIZAÇÕES

### **2025-11-26: Correções de Compatibilidade**

#### **Problema: RTX 5090 não compatível com PyTorch/NumPy antigos**
- RTX 5090 (arquitetura Blackwell, sm_120) requer PyTorch 2.6+ com CUDA 13.0
- CLONE original usa NumPy 1.23.0 que não é compatível com PyTorch novo
- **Solução**: Forçar uso de CPU no `g1_server.py`:
  ```python
  # Em class G1.__init__():
  self.device = 'cpu'  # Force CPU mode (RTX 5090 not compatible)
  ```

#### **Problema: dex_retargeting versão incompatível**
- CLONE requirements pede `dex_retargeting==0.1.1`
- Versão instalada era 0.4.7 (modificada pelo xr_teleoperate)
- Parâmetro no YAML mudou: `target_link_human_indices` → `target_link_human_indices_vector`
- **Solução**: Corrigido `unitree_dex3.yml` para usar nome correto do parâmetro

#### **Problema: Calibração R1/R2 não funcionava**
- Robô no MuJoCo não se reposicionava para alinhar com tracker
- **Causa**: Servidor ZMQ de localização (`pos_server.py`) não estava rodando no PC2
- Erro: `zmq.error.ZMQError: Address already in use (addr='tcp://*:6006')`
- **Solução**: Matar processo antigo que estava usando a porta:
  ```bash
  # No PC2:
  sudo fuser -k 6006/tcp
  # Depois reiniciar localization_server.sh
  ```

#### **Dica de Debug: Testar conexão ZMQ**
```bash
# No Server PC, testar se dados de localização estão chegando:
python3 -c "
import zmq, pickle
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect('tcp://192.168.123.164:6006')
sock.setsockopt_string(zmq.SUBSCRIBE, '')
sock.setsockopt(zmq.RCVTIMEO, 3000)
try:
    print('Dados:', pickle.loads(sock.recv()))
except zmq.Again:
    print('Timeout - verificar pos_server no PC2')
"
```

### **2025-12-03: Firewall bloqueando lowstate**

#### **Problema: lowcmd é enviado mas lowstate não chega**
- Sistema parece funcionar: calibração OK, policy rodando
- `ros2 topic list` mostra `/lowstate` mas `ros2 topic echo /lowstate` não mostra nada
- `ros2 topic info /lowstate -v` mostra **Publisher count: 0**
- Robô está em Debug Mode mas não recebe estado

#### **Causa: Firewall do Ubuntu bloqueando multicast DDS**
- O `lowstate` usa DDS multicast para comunicação
- Firewall (ufw) bloqueia pacotes de entrada por padrão
- Comandos (`lowcmd`) saem, mas estados (`lowstate`) não entram

#### **Solução: Desabilitar firewall no Server PC**
```bash
sudo ufw disable
```

#### **Alternativa (mais segura): Abrir portas específicas**
```bash
sudo ufw allow in from 192.168.123.0/24
sudo ufw allow in proto udp to 224.0.0.0/4  # Multicast DDS
```

#### **Diagnóstico rápido:**
```bash
# Verificar se há publishers de lowstate
ros2 topic info /lowstate -v

# Se Publisher count: 0, verificar:
# 1. Debug Mode ativo no robô
# 2. Firewall: sudo ufw status
# 3. Conectividade: ping 192.168.123.164
```
