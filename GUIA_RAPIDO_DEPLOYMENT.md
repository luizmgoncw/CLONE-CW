# 🚀 GUIA RÁPIDO DE DEPLOYMENT - CLONE

**Data**: 2025-11-21
**Modos Suportados**:
- ✅ **Rede Local com AVP_stream** (RECOMENDADO - baixa latência)
- ⚠️ Rede Local com Vuer
- ⚠️ Internet com Vuer + Ngrok (limite de banda)

---

## ⚡ INÍCIO RÁPIDO

### Pré-requisitos
- [ ] G1 ligado e em Debug Mode
- [ ] Server PC conectado à rede via Ethernet (para G1) E WiFi (para Vision Pro)
- [ ] G1 conectado ao Server PC via Ethernet
- [ ] Vision Pro na mesma rede WiFi que o Server PC
- [ ] App "Tracking Streamer" instalado no Vision Pro (para avp_stream)

---

## 🌐 TOPOLOGIA DE REDE (IMPORTANTE!)

```
┌──────────────┐
│ Vision Pro   │  IP WiFi: 192.168.31.4
│ (WiFi)       │
└──────┬───────┘
       │
    WiFi Network (192.168.31.x)
       │
┌──────▼────────────────────┐
│    Server PC              │
│  wlp7s0: 192.168.31.3    │ ← Interface WiFi (Vision Pro conecta aqui)
│  enx...: 192.168.123.99  │ ← Interface Ethernet (G1 conecta aqui)
│                           │
│  IP Forwarding: ENABLED  │ ← Roteia entre as duas redes
└──────┬────────────────────┘
       │
   Ethernet (192.168.123.x)
       │
┌──────▼───────┐
│  G1 PC2      │  IP: 192.168.123.164
└──────────────┘
```

**Pontos-chave:**
- Server PC tem **duas interfaces** de rede
- Vision Pro e Server PC na **mesma rede WiFi**
- G1 e Server PC na **rede Ethernet dedicada**
- Server PC faz **roteamento** entre as duas redes

---

## 📋 ORDEM DE EXECUÇÃO

### **1️⃣ G1 PC2 - Localization Stack**

```bash
# SSH no G1
ssh unitree@192.168.123.164

# Iniciar serviços de localização
cd ~/onboard
./localization_server.sh
```

**Verificar**: Deve mostrar 3 processos iniciados (FAST_LIO, Livox Driver, pos_server)

---

### **1.1️⃣ G1 PC2 - Image Server (OPCIONAL)**

⚠️ **Este passo é OPCIONAL!** O sistema funciona perfeitamente sem câmera. A interface ficará preta, mas a teleoperação funciona normalmente.

Se tiver uma câmera RealSense D435I conectada ao G1:

```bash
# Ainda na sessão SSH do G1 (em outro terminal)
ssh unitree@192.168.123.164

# Se der erro "Device or resource busy", liberar a câmera:
for d in /dev/video*; do
    echo "### $d"
    sudo fuser -v "$d" || true
done
# Anotar o PID e matar:
sudo kill <PID>

# Iniciar image server
cd ~/onboard/image_server
python3 image_server.py
```

**Verificar**: Deve mostrar "Image server has started"

**Sem câmera?** Pule este passo! O tracking do Vision Pro (cabeça e mãos) funciona independentemente.

---

### **2️⃣ Server PC - Terminal 1: Ngrok**

```bash
cd /home/luizmarques/Documents/CLONE
./start_ngrok.sh
```

**⚠️ IMPORTANTE:** Anote a URL que aparecer!
Exemplo: `https://1a2b-3c4d-5e6f.ngrok-free.app`

---

### **3️⃣ Server PC - Terminal 2: Lowcmd Publisher**

```bash
cd /home/luizmarques/Documents/CLONE
./start_lowcmd.sh
```

**Verificar**: Deve mostrar "Lowcmd publisher started"

---

### **4️⃣ Server PC - Terminal 3: G1 Server**

```bash
cd /home/luizmarques/Documents/CLONE
./start_g1_server.sh
```

**Verificar**: Deve mostrar "G1 server started" e conectar ao pos_server

---

### **5️⃣ Vision Pro - Tracking Streamer App**

#### **Opção A: AVP_stream (RECOMENDADO)**

**Configuração no Server PC:**
```bash
# Editar config.py
nano CLONE/deploy/config.py

# Configurar:
VISION_WRAPPER_BACKEND = 'avp_stream'
VISION_PRO_IP = '192.168.31.4'  # IP do Vision Pro na WiFi
```

**No Vision Pro - App "Tracking Streamer":**
1. Abrir app "Tracking Streamer"
2. **Python Server IP**: `0.0.0.0` (ou `192.168.31.4`)
3. Pressionar botão "Start Server" ou iniciar streaming
4. Status deve mudar para "Server: Running"

**Verificar Tracking Funciona:**
- Abrir janela MuJoCo no Server PC
- Mover mãos e cabeça
- Ver se as "bolas coloridas" seguem seus movimentos

---

#### **Opção B: Vuer (Navegador)**

**Configuração no Server PC:**
```bash
# Editar config.py
nano CLONE/deploy/config.py

# Configurar:
VISION_WRAPPER_BACKEND = 'vuer'
VISION_PRO_IP = '192.168.31.4'
```

**No Vision Pro - Safari:**
```
http://192.168.31.3:8012
```

**Importante:**
- Use IP **WiFi** do Server PC (192.168.31.3)
- NÃO use ngrok se estiver na mesma rede
- Interface web deve carregar

---

## 🔧 Autenticar Ngrok (Primeira Vez)

Se ainda não autenticou o ngrok:

```bash
# 1. Acesse e faça login:
# https://dashboard.ngrok.com/signup

# 2. Pegue seu token em:
# https://dashboard.ngrok.com/get-started/your-authtoken

# 3. Configure:
ngrok config add-authtoken SEU_TOKEN_AQUI

# 4. Teste:
ngrok http 8012
```

---

## 🎮 Controles e Calibração

### **Gamepad (conectado ao Server PC):**

**R1/R2**: Calibrar alinhamento
- Pressione **3-5 vezes**
- Fique em **pose neutra** (braços relaxados, olhando para frente)
- Veja no terminal: `Location Offset Reset`

**L1**: Iniciar execução da policy
- Após calibrar, pressione L1
- Veja no terminal: `Policy start!` e `POL FREQ: 52Hz`
- lowcmd_publisher deve mostrar posições mudando

**L2**: Parada de emergência
- Para a execução imediatamente

---

### **Verificar Tracking:**

**No MuJoCo Viewer (janela que abre automaticamente):**
- 🟢 Bolas verdes/cianas = Posição das mãos
- ⚪ Bola branca = Localização do robô
- Se as bolas seguem seus movimentos = ✅ Tracking OK

**No Terminal lowcmd_publisher:**
```
[Lowcmd Publisher] Sample joint 0 position: -0.0998  ← Fixo (SEM tracking)
[Lowcmd Publisher] Sample joint 0 position: 0.1523   ← Mudando (COM tracking!)
```

---

### **⚠️ Robô em Debug Mode:**

- Debug Mode desabilita a **política de equilíbrio nativa da Unitree**
- Robô usa a **política de equilíbrio neural do CLONE** (MoE policy)
- Equilíbrio é controlado pela rede neural treinada, não pelo firmware Unitree
- **Recomendado para primeiros testes:** Robô apoiado/suspenso até validar política

---

## 🐛 Troubleshooting Rápido

### Problema: "No module named 'rclpy'"
**Solução**: Use os scripts `start_*.sh` - eles carregam o ambiente ROS2 automaticamente

### Problema: "No module named 'unitree_hg'"
**Solução**: Use os scripts `start_*.sh` - eles carregam o workspace unitree_ros2

### Problema: "connection refused" no g1_server
**Solução**: Verifique se lowcmd_publisher está rodando primeiro

### Problema: Vision Pro não conecta
**Solução**:
- Use a URL SEM a porta: `https://sua-url.ngrok-free.dev/?ws=wss://sua-url.ngrok-free.dev`
- NÃO adicione `:8012` na URL
- Verifique se g1_server está rodando
- Acesse `http://localhost:4040` no Server PC para ver o dashboard do ngrok
- Se aparecer tela de aviso do ngrok, clique em "Visit Site"

### Problema: Ngrok "authentication required"
**Solução**: Configure o authtoken (ver seção "Autenticar Ngrok")

---

## 📊 Verificar Status dos Serviços

### No G1 PC2:
```bash
# Verificar processos
ps aux | grep -E "(roslaunch|python3)"

# Verificar tópicos ROS
rostopic hz /livox/lidar
rostopic hz /Odometry
```

### No Server PC:
```bash
# Verificar processos Python
ps aux | grep python

# Verificar conectividade
ping 192.168.123.164  # G1 PC2

# Verificar portas
netstat -tuln | grep -E "(6006|8012)"
```

---

## 🛑 Parar Tudo

### Server PC:
```bash
# Ctrl+C em cada terminal (Ngrok, Lowcmd, G1 Server)
```

### G1 PC2:
```bash
# Ctrl+C no terminal com localization_server.sh
# Ou:
pkill -f "roslaunch|python3"
```

---

## 📊 Visualizar Localização (OPCIONAL)

Você pode visualizar a localização do robô em tempo real no RViz2:

```bash
# Em um novo terminal no Server PC
cd /home/luizmarques/Documents/CLONE
./start_localization_viz.sh
```

Isso abre o RViz2 mostrando:
- Pose do robô em 3D (seta)
- Trajetória (path)
- Frames TF (map → body)

**Ver documentação completa**: `LOCALIZATION_VISUALIZATION.md`

---

## 📞 Suporte

- **Documentação completa**: `SESSAO_CHECKPOINT.md`
- **Troubleshooting detalhado**: `INSTALACAO_TROUBLESHOOTING.md`
- **Visualização de localização**: `LOCALIZATION_VISUALIZATION.md`
- **README original**: `CLONE/README.md`

---

**Última atualização**: 2025-11-21
**Scripts criados**: start_ngrok.sh, start_lowcmd.sh, start_g1_server.sh, start_localization_viz.sh
