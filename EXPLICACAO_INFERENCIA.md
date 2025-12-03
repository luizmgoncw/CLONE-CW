# Explicação Detalhada da Pipeline de Inferência - Projeto CLONE

Este documento explica como os dados de rastreamento da cabeça e mãos são recebidos e processados até a inferência do modelo neural.

---

## Visão Geral do Fluxo

```
Apple Vision Pro → VisionWrapper → get_vp_data() → compute_observations() → policy() → motores
```

---

## ETAPA 1: Recebimento dos Dados de Rastreamento

### Arquivo: `VisionWrapper/vision_wrapper/vision_wrapper.py`
### Método: `get_data_avp_stream()` (linhas 193-216)

```python
def get_data_avp_stream(self):
    # Obtém os dados mais recentes do streamer do Apple Vision Pro
    data = self.tv.latest  # linha 201
    if data is None:
        return

    # Extrai as matrizes de transformação 4x4 (posição + rotação)
    head_mat = data['head'][0].copy()               # Matriz 4x4 da cabeça
    left_wrist_mat = data['left_wrist'][0].copy()   # Matriz 4x4 do pulso esquerdo
    right_wrist_mat = data['right_wrist'][0].copy() # Matriz 4x4 do pulso direito
```

### O que é uma matriz 4x4 de transformação?

```
┌                      ┐
│ R00  R01  R02  Tx    │   R = Matriz de Rotação 3x3
│ R10  R11  R12  Ty    │   T = Vetor de Translação (posição XYZ)
│ R20  R21  R22  Tz    │
│  0    0    0    1    │
└                      ┘
```

### Transformação de coordenadas (linhas 209-211):

```python
# Converte da convenção OpenXR (Y up, Z back) para convenção do robô (Z up, Y left)
# T_z_90 = matriz de rotação de 90° em torno do eixo Z
head_mat = T_z_90 @ head_mat @ T_z_90.T
left_wrist_mat = T_z_90 @ left_wrist_mat @ T_x_180.T
right_wrist_mat = T_z_90 @ right_wrist_mat @ T_z_180.T
```

### Extração dos landmarks das mãos (linhas 213-214):

```python
# Aplica transformação e extrai posições dos 25 pontos de cada mão
# einsum realiza multiplicação de matrizes em batch
left_hand_mat = np.einsum('bik,bkj->bij', T_flip_left, data['left_fingers'].copy())
right_hand_mat = np.einsum('bik,bkj->bij', T_flip_right, data['right_fingers'].copy())
```

### Output (linha 216):

```python
return head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat[:, :3, 3], right_hand_mat[:, :3, 3]
```

| Variável | Shape | Descrição |
|----------|-------|-----------|
| `head_mat` | (4, 4) | Posição + rotação da cabeça |
| `left_wrist_mat` | (4, 4) | Posição + rotação do pulso esquerdo |
| `right_wrist_mat` | (4, 4) | Posição + rotação do pulso direito |
| `left_hand_mat[:, :3, 3]` | (25, 3) | Posições XYZ dos 25 landmarks da mão esquerda |
| `right_hand_mat[:, :3, 3]` | (25, 3) | Posições XYZ dos 25 landmarks da mão direita |

---

## ETAPA 2: Processamento dos Dados de Rastreamento

### Arquivo: `CLONE/deploy/g1_server.py`
### Método: `get_vp_data()` (linhas 822-959)

### 2.1 Recebimento e ajuste de altura (linhas 828-833):

```python
@torch.inference_mode()
def get_vp_data(self):
    data = self.tv_wrapper.get_data_full()  # Chama o VisionWrapper

    if not data is None:
        head_mat, left_wrist, right_wrist, left_hand, right_hand = data

        # Adiciona offset de altura (VP_POS_DH vem do config.py)
        # Isso ajusta a diferença de altura entre o operador e o robô
        head_mat[2, 3] += VP_POS_DH      # Componente Z da cabeça
        left_wrist[2, 3] += VP_POS_DH    # Componente Z do pulso esquerdo
        right_wrist[2, 3] += VP_POS_DH   # Componente Z do pulso direito
```

### 2.2 Envio dos dados das mãos para as mãos robóticas (linhas 835-845):

```python
if USE_DEX:
    if HAND_TYPE == 'inspire':
        # Envia landmarks para controller Inspire via memória compartilhada
        with self.left_hand_pos_array.get_lock():
            self.left_hand_pos_array[:] = left_hand.astype(np.float64).flatten()  # 75 valores
        with self.right_hand_pos_array.get_lock():
            self.right_hand_pos_array[:] = right_hand.astype(np.float64).flatten()
    else:
        # Envia para DEX3 Server via shared memory
        self.dex_shm_send_data[0:75] = left_hand.flatten()    # 25 pontos * 3 coords = 75
        self.dex_shm_send_data[75:150] = right_hand.flatten()
```

### 2.3 Armazenamento em buffer para cálculo de velocidade (linhas 855-867):

```python
# Mantém histórico das últimas 5 posições (janela deslizante)
self.vp_head_seq.append(head_mat[:3, 3])     # Adiciona posição XYZ atual
self.vp_lhand_seq.append(left_wrist[:3, 3])
self.vp_rhand_seq.append(right_wrist[:3, 3])

# Limita a 5 valores
if len(self.vp_head_seq) > self.vp_d_seqlen:  # vp_d_seqlen = 5
    self.vp_head_seq = self.vp_head_seq[-self.vp_d_seqlen:]

# Converte para arrays numpy para cálculo do gradiente
head_seq = np.stack(self.vp_head_seq)    # Shape: (5, 3)
lhand_seq = np.stack(self.vp_lhand_seq)
rhand_seq = np.stack(self.vp_rhand_seq)
```

### 2.4 Cálculo das velocidades via gradiente numérico (linhas 869-871):

```python
# np.gradient calcula a derivada numérica usando diferenças finitas
# self.vp_dt = 1/50 = 0.02 segundos (frequência de 50 Hz)
# edge_order=2 usa polinômio de segunda ordem nas bordas

head_d = np.gradient(head_seq, self.vp_dt, axis=0, edge_order=2)   # Velocidade da cabeça
lhand_d = np.gradient(lhand_seq, self.vp_dt, axis=0, edge_order=2) # Velocidade mão esquerda
rhand_d = np.gradient(rhand_seq, self.vp_dt, axis=0, edge_order=2) # Velocidade mão direita

# Resultado: arrays (5, 3) com velocidade em cada timestep
```

### 2.5 Validação de tracking das mãos (linhas 873-885):

```python
# Se a mão está muito perto da cabeça (< 0.1m), considera tracking inválido
# Isso acontece quando o Vision Pro perde o tracking da mão
if ((left_wrist[:3, 3] - head_mat[:3, 3]) ** 2).sum() < 0.1:
    left_vel_factor = 0.  # Zera a velocidade
    self.invalid_hands[0] = True
elif self.invalid_hands[0]:
    left_vel_factor = 0.  # Mantém zerado por 1 frame após recuperar
    self.invalid_hands[0] = False

# Mesmo processo para mão direita...
```

### 2.6 Extração das velocidades finais (linhas 887-889):

```python
# Usa o 3º valor do gradiente [-3] pois é mais estável que as bordas
self.head_vel[:] = torch.from_numpy(head_d[-3]) * vel_factor
self.left_wrist_vel[:] = torch.from_numpy(lhand_d[-3]) * vel_factor * left_vel_factor
self.right_wrist_vel[:] = torch.from_numpy(rhand_d[-3]) * vel_factor * right_vel_factor
```

### 2.7 Conversão de matrizes de rotação para quaternions (linhas 899-901):

```python
# matrix_to_quaternion: converte matriz 3x3 → quaternion (w, x, y, z)
# roll(-1): ajusta ordem para (x, y, z, w) que é o formato esperado
self.head_rot[:] = matrix_to_quaternion(head_mat[:3, :3]).roll(-1, dims=-1)
self.left_hand_rot[:] = matrix_to_quaternion(left_wrist[:3, :3]).roll(-1, dims=-1)
self.right_hand_rot[:] = matrix_to_quaternion(right_wrist[:3, :3]).roll(-1, dims=-1)
```

### 2.8 Cálculo das posições finais (linhas 903-911):

```python
# Adiciona pequeno offset (6cm) na direção X baseado na orientação
# Isso compensa a diferença entre o ponto de tracking e o ponto de controle
head_delta = my_quat_rotate(self.head_rot[None, :], torch.tensor([[1., 0., 0.]]))[0] * -0.06
lhand_delta = my_quat_rotate(self.left_hand_rot[None, :], torch.tensor([[1., 0., 0.]]))[0] * 0.06
rhand_delta = my_quat_rotate(self.right_hand_rot[None, :], torch.tensor([[1., 0., 0.]]))[0] * 0.06

# Posições finais = posição base + delta de orientação + offset global
self.head_pos = head_mat[:3, 3] + head_delta + self.vp_delta_pos
self.left_wrist_pos = left_wrist[:3, 3] + lhand_delta + self.vp_delta_pos
self.right_wrist_pos = right_wrist[:3, 3] + rhand_delta + self.vp_delta_pos
```

### Output da Etapa 2:

| Atributo | Shape | Descrição |
|----------|-------|-----------|
| `self.head_pos` | (3,) | Posição XYZ da cabeça |
| `self.left_wrist_pos` | (3,) | Posição XYZ do pulso esquerdo |
| `self.right_wrist_pos` | (3,) | Posição XYZ do pulso direito |
| `self.head_vel` | (3,) | Velocidade XYZ da cabeça |
| `self.left_wrist_vel` | (3,) | Velocidade XYZ do pulso esquerdo |
| `self.right_wrist_vel` | (3,) | Velocidade XYZ do pulso direito |
| `self.head_rot` | (4,) | Quaternion de rotação da cabeça |
| `self.left_hand_rot` | (4,) | Quaternion de rotação da mão esquerda |
| `self.right_hand_rot` | (4,) | Quaternion de rotação da mão direita |

---

## ETAPA 3: Construção da Observação da Tarefa

### Arquivo: `CLONE/deploy/g1_server.py`
### Método: `compute_task_observation()` (linhas 981-1045)

Esta função converte os dados de tracking em features que o modelo entende.

### 3.1 Montagem das posições/velocidades de referência (linhas 988-989):

```python
def compute_task_observation(self) -> np.ndarray:
    quat_xyzw = torch.from_numpy(self.obs_quat.copy())[None, :]  # Quaternion do IMU do robô

    # Concatena posições: [mão_esq, mão_dir, cabeça]
    # Reshape para (Batch=1, TimeSteps=1, Joints=3, XYZ=3)
    ref_body_pos = torch.cat((
        self.left_wrist_pos,
        self.right_wrist_pos,
        self.head_pos
    )).reshape(1, 1, 3, 3).float()

    ref_body_vel = torch.cat((
        self.left_wrist_vel,
        self.right_wrist_vel,
        self.head_vel
    )).reshape(1, 1, 3, 3).float()
```

### 3.2 Posição atual do corpo do robô (linhas 991-993):

```python
# body_pos_extend vem do módulo de localização (forward kinematics)
# Contém a posição atual dos end-effectors do robô
body_pos_extend = self.body_pos_extend.clone()

# Normaliza altura (coloca o pé no chão)
body_pos_extend[..., 2] -= torch.min(body_pos_extend[..., 2], dim=0, keepdim=True).values

# Pega as 3 últimas posições: [mão_esq, mão_dir, cabeça] do robô
body_pos = body_pos_extend[-3:]
```

### 3.3 Cálculo do heading (orientação horizontal) (linhas 1011-1014):

```python
# O "heading" é a orientação do robô no plano horizontal (rotação em Z)
# calc_heading_quat_inv retorna o inverso - usado para converter coords globais → locais
heading_inv_rot = calc_heading_quat_inv(quat_xyzw)  # Shape: (1, 4)

# Expande para todos os joints
heading_inv_rot_expand = heading_inv_rot.unsqueeze(-2).repeat((1, 3, 1))  # Shape: (1, 3, 4)
```

### 3.4 Diferença de posição (erro de tracking) (linhas 1017-1019):

```python
# Calcula o ERRO entre onde o operador quer (ref) e onde o robô está (body_pos)
diff_global_body_pos = ref_body_pos - body_pos  # Em coordenadas globais

# Converte para coordenadas LOCAIS (relativas à orientação do robô)
# Isso é importante porque a política precisa saber "mova 10cm para FRENTE"
# não "mova 10cm na direção X do mundo"
diff_local_body_pos_flat = my_quat_rotate(
    heading_inv_rot_expand.view(-1, 4),  # Quaternion inverso do heading
    diff_global_body_pos.view(-1, 3)     # Diferença de posição
)
```

### 3.5 Rotações locais das mãos (linhas 1022-1023):

```python
# Concatena quaternions das mãos
wrist_rots = torch.cat((self.left_hand_rot, self.right_hand_rot))  # Shape: (8,)

# Converte para coordenadas locais (relativas ao heading do robô)
local_ref_body_rot = quat_mul(
    heading_inv_rot.repeat(1, 2, 1),  # Inverso do heading, repetido 2x
    wrist_rots.view(1, 2, 4)          # Rotações das mãos
)
```

### 3.6 Posições locais relativas à cabeça (linhas 1028-1031):

```python
# Usa a cabeça como ponto de referência (ancora o frame de referência)
ref_head_xy = ref_body_pos[:, :, -1].clone()  # Posição da cabeça
ref_head_xy[..., 2] = 0.6  # Altura fixa de referência

# Posição das mãos relativa à cabeça
local_ref_body_pos = ref_body_pos - ref_head_xy

# Rotaciona para coordenadas locais
local_ref_body_pos = my_quat_rotate(heading_inv_rot_expand.view(-1, 4),
                                     local_ref_body_pos.view(-1, 3))
```

### 3.7 Velocidades locais (linha 1035):

```python
# Também converte velocidades para coordenadas locais
local_ref_body_vel = my_quat_rotate(heading_inv_rot_expand.view(-1, 4),
                                     ref_body_vel.view(-1, 3))
```

### 3.8 Montagem do vetor de observação (linhas 1038-1044):

```python
obs = []

# 1. Diferença de posição local (erro entre VP e robô)
obs.append(diff_local_body_pos_flat.view(1, 1, -1))  # 9 valores (3 joints * 3 coords)

# 2. Posição local de referência (onde o operador quer)
obs.append(local_ref_body_pos.view(1, 1, -1))        # 9 valores

# 3. Velocidade local de referência
obs.append(local_ref_body_vel.view(1, 1, -1))        # 9 valores

# 4. Rotações das mãos (quaternions)
obs.append(local_ref_body_rot.view(1, 1, -1))        # 8 valores (2 mãos * 4 componentes)

# Concatena tudo
obs = torch.cat(obs, dim=-1).view(-1)
return obs.numpy()  # Total: 35 valores
```

### Output da Etapa 3:

| Componente | Tamanho | Descrição |
|------------|---------|-----------|
| diff_local_body_pos | 9 | Erro de posição (3 joints × 3 XYZ) |
| local_ref_body_pos | 9 | Posição desejada local |
| local_ref_body_vel | 9 | Velocidade desejada local |
| local_ref_body_rot | 8 | Rotações das mãos (2 × quaternion) |
| **Total** | **35** | |

---

## ETAPA 4: Montagem do Buffer de Observação Completo

### Arquivo: `CLONE/deploy/g1_server.py`
### Método: `compute_observations()` (linhas 1068-1098)

Esta função monta o vetor completo de observação que será a entrada do modelo.

```python
def compute_observations(self):
    # Calcula observação da tarefa (tracking do Vision Pro)
    task_obs = self.compute_task_observation()  # 35 valores

    # Histórico de estados anteriores (para contexto temporal)
    history_to_be_append = self.env.trajectories[0: self.env.obs_context_len * LOG_STEP_LEN].copy()

    # ========== CONCATENAÇÃO FINAL ==========
    obs_buf = torch.tensor(np.concatenate((
        self.obs_joint_pos,                    # 29: posição das juntas (rad)
        self.obs_joint_vel * OBS_VEL_SCALE,    # 29: velocidade das juntas (scaled)
        self.obs_ang_vel,                      # 3: velocidade angular do IMU
        self.projected_gravity,                # 3: vetor gravidade projetado
        task_obs,                              # 35: observação da tarefa
        self.prev_action,                      # 29: ação do timestep anterior
        history_to_be_append                   # N*128: histórico de trajetórias
    ), axis=-1), dtype=torch.float, device=self.device).unsqueeze(0)

    self.env.obs_buf = obs_buf  # Armazena para uso na inferência
```

### Estrutura completa do obs_buf:

| Índice | Conteúdo | Tamanho | Descrição |
|--------|----------|---------|-----------|
| 0-28 | `obs_joint_pos` | 29 | Posição atual das 29 juntas (radianos) |
| 29-57 | `obs_joint_vel * 0.005` | 29 | Velocidade das juntas (escalada) |
| 58-60 | `obs_ang_vel` | 3 | Velocidade angular do giroscópio (rad/s) |
| 61-63 | `projected_gravity` | 3 | Vetor gravidade projetado no frame do robô |
| 64-98 | `task_obs` | 35 | Observação da tarefa (tracking VP) |
| 99-127 | `prev_action` | 29 | Ação executada no timestep anterior |
| 128+ | `history` | HISTORY_LEN × 128 | Histórico de estados |

**Tamanho total: 3328 + (HISTORY_LEN - 25) × 128 valores**

---

## ETAPA 5: Carregamento do Modelo

### Arquivo: `CLONE/deploy/g1_server.py`
### Método: `init_policy()` (linhas 778-799)

```python
def init_policy(self):
    self.get_logger().info("Preparing policy")

    # Cria objeto de ambiente com configurações do robô G1
    self.env = G1(task='self.task')

    # Carrega o modelo TorchScript pré-treinado
    file_pth = os.path.dirname(os.path.realpath(__file__))
    self.policy = torch.jit.load(
        os.path.join(file_pth, POLICY_PATH),  # 'models/g1_student_moel.pt'
        map_location=self.env.device           # 'cuda' se disponível, senão 'cpu'
    )
    self.policy.to(self.env.device)

    # Warmup: primeira inferência é mais lenta devido à compilação JIT
    actions = self.policy(self.env.obs_buf.detach().reshape(1, -1))
```

### O que é o modelo?

- **Arquivo:** `models/g1_student_moel.pt`
- **Formato:** TorchScript (modelo PyTorch serializado)
- **Tipo:** Rede neural (provavelmente MLP ou Transformer)
- **Treinamento:** Reinforcement Learning (aprendizado por reforço)

---

## ETAPA 6: Inferência do Modelo (Loop Principal)

### Arquivo: `CLONE/deploy/g1_server.py`
### Método: `main_loop()` (linhas 1100-1227)

### 6.1 Loop de controle a 50 Hz (linhas 1178-1183):

```python
@torch.inference_mode()  # Desabilita cálculo de gradientes (mais rápido)
def main_loop(self):
    while rclpy.ok():
        # Sincronização de tempo para manter 50 Hz
        while self.control_dt > time.monotonic() - loop_start_time:
            time.sleep(max(0., self.control_dt - (time.monotonic() - loop_start_time) - 0.001))

        loop_start_time = time.monotonic()
        rclpy.spin_once(self, timeout_sec=0.001)  # Processa callbacks ROS
```

### 6.2 Inferência (linhas 1186-1187):

```python
        # ========== PONTO CENTRAL DA INFERÊNCIA ==========

        # 1. Monta o buffer de observação com todos os dados
        self.compute_observations()

        # 2. Passa pelo modelo neural - FORWARD PASS
        raw_actions = self.policy(self.env.obs_buf)

        # raw_actions shape: (1, 29) - uma ação para cada junta
```

### 6.3 Verificação de segurança (linhas 1190-1193):

```python
        # Verifica se o modelo produziu valores inválidos
        if torch.any(torch.isnan(raw_actions)):
            self.get_logger().info("Emergency stop due to NaN")
            self.set_motor_position(q=self.env.default_dof_pos_np)
            raise SystemExit
```

### 6.4 Conversão para ângulos das juntas (linhas 1195-1198):

```python
        # Converte tensor para numpy
        raw_actions_np = raw_actions.clone().detach().cpu().numpy().squeeze(0)

        # DESNORMALIZAÇÃO:
        # O modelo produz ações normalizadas (tipicamente entre -1 e 1)
        # Convertemos para ângulos reais (radianos):
        # angle = action * scale + default_position
        angles = raw_actions_np * self.env.scale_action + self.env.default_dof_pos_np
        #                         └─ 0.25 (linha 119)    └─ posição padrão em pé

        # Limita aos limites físicos das juntas
        angles = np.clip(angles, self.env.joint_limit_lo_, self.env.joint_limit_hi_)
```

### 6.5 Envio para os motores (linhas 1202-1205):

```python
        self.angles = angles
        self.set_motor_position(self.angles)  # Configura posição alvo

        if not NO_MOTOR and not NO_ROS:
            self.motor_pub.publish(self.cmd_msg)  # Publica comando via ROS2 DDS
```

---

## DIAGRAMA COMPLETO DO FLUXO DE DADOS

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         APPLE VISION PRO                                 │
│                  (tracking da cabeça e mãos do operador)                │
│                                                                          │
│  Dados brutos: matrizes 4x4 de transformação + landmarks dos dedos     │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  VisionWrapper.get_data_avp_stream()                                    │
│  Arquivo: vision_wrapper.py:193-216                                     │
│                                                                          │
│  • Extrai matrizes de cabeça e pulsos                                   │
│  • Converte coordenadas OpenXR → Robot                                  │
│  • Extrai posições dos 25 landmarks de cada mão                         │
│                                                                          │
│  Output: head_mat(4x4), wrist_mats(4x4), hand_landmarks(25x3)          │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  DeployNode.get_vp_data()                                               │
│  Arquivo: g1_server.py:822-959                                          │
│                                                                          │
│  • Ajusta altura (+VP_POS_DH)                                           │
│  • Envia landmarks para mãos robóticas (DEX3/Inspire)                   │
│  • Calcula velocidades via gradiente numérico                           │
│  • Valida tracking (detecta perda de tracking)                          │
│  • Converte rotações matriz → quaternion                                │
│                                                                          │
│  Output: pos/vel/rot de cabeça e mãos (9 tensores torch)               │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  DeployNode.compute_task_observation()                                  │
│  Arquivo: g1_server.py:981-1045                                         │
│                                                                          │
│  • Calcula erro de posição (VP - robô)                                  │
│  • Converte para coordenadas locais                                     │
│  • Monta posições e velocidades locais                                  │
│  • Extrai rotações das mãos                                             │
│                                                                          │
│  Output: np.array(35) - observação da tarefa                            │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  DeployNode.compute_observations()                                      │
│  Arquivo: g1_server.py:1068-1098                                        │
│                                                                          │
│  Concatena:                                                              │
│  ┌────────────────────────────────────────────────────────────────┐     │
│  │ joint_pos(29) │ joint_vel(29) │ ang_vel(3) │ gravity(3)        │     │
│  │ task_obs(35)  │ prev_action(29) │ history(N×128)               │     │
│  └────────────────────────────────────────────────────────────────┘     │
│                                                                          │
│  Output: obs_buf tensor(1, 3328+)                                       │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  self.policy(self.env.obs_buf)                                          │
│  Arquivo: g1_server.py:1187                                             │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                    REDE NEURAL (MLP/Transformer)                │    │
│  │                                                                  │    │
│  │   Input: (1, 3328+)  ──►  Hidden Layers  ──►  Output: (1, 29)   │    │
│  │                                                                  │    │
│  │   Treinada via Reinforcement Learning                           │    │
│  │   Modelo: models/g1_student_moel.pt                             │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
│  Output: raw_actions tensor(1, 29)                                      │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Pós-processamento                                                      │
│  Arquivo: g1_server.py:1195-1198                                        │
│                                                                          │
│  angles = raw_actions × 0.25 + default_pos                              │
│  angles = clip(angles, joint_limits)                                    │
│                                                                          │
│  Output: angles np.array(29) - ângulos em radianos                      │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  motor_pub.publish()                                                    │
│  Arquivo: g1_server.py:1205                                             │
│                                                                          │
│  Envia comandos para os 29 motores via ROS2 DDS                         │
│  Frequência: 50 Hz                                                       │
│                                                                          │
│  Cada motor recebe: (q=ângulo, kp=gain, kd=damping)                     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## ETAPA 7: Controle de Locomoção via `loc_offset`

### Arquivo: `CLONE/deploy/g1_server.py` e `CLONE/deploy/localization.py`

O `loc_offset` é um mecanismo que permite mover o robô durante a execução da política, seja via controle remoto ou via reset automático de alinhamento.

### 7.1 Arquitetura de Memória Compartilhada

```
┌─────────────────────────────────────────────────────────────────────┐
│                         g1_server.py                                 │
│                                                                      │
│  loc_send_data (memória compartilhada - ENVIO)                      │
│  ┌─────────────┬─────────────┬──────────────┬─────────────────────┐ │
│  │ loc_offset  │  obs_quat   │ loc_delta_rot│   obs_joint_pos     │ │
│  │   [0:3]     │    [3:7]    │    [7:11]    │      [11:]          │ │
│  │   XYZ       │    XYZW     │    XYZW      │     29 joints       │ │
│  └─────────────┴─────────────┴──────────────┴─────────────────────┘ │
└────────────────────────────────┬────────────────────────────────────┘
                                 │ Escrito via memória compartilhada
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       localization.py                                │
│                    (processo separado)                               │
│                                                                      │
│  pos_client.position_offset = send_shm_data[0:3]  ← APONTA para     │
│                                                     loc_offset!      │
└────────────────────────────────┬────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       pos_client.py:76                               │
│                                                                      │
│  self.position[:] = quat_rotate(delta_quat, raw_position)           │
│                     + self.position_offset    ← USA O OFFSET!       │
│                                                                      │
│  Posição final = SLAM_position + offset                             │
└────────────────────────────────┬────────────────────────────────────┘
                                 │ Escrito em recv_shm_data[0:3]
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         g1_server.py                                 │
│                                                                      │
│  self.location = self.loc_recv_data[0:3]  ← Posição FINAL com       │
│                                              offset aplicado         │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.2 Inicialização do loc_offset (linha 449):

```python
# loc_send_data é um array de memória compartilhada
self.loc_send_data = np.ndarray(shm_send_shape, dtype=np.float32, buffer=self.loc_shm_send.buf)

# loc_offset é uma VIEW dos primeiros 3 elementos (não uma cópia!)
self.loc_offset = self.loc_send_data[0:3]  # XYZ offset

# Modificar loc_offset automaticamente modifica loc_send_data[0:3]
# que é lido pelo processo de localização
```

### 7.3 Modificação via Controle Remoto (linhas 681-686):

```python
if self.move_by_wireless_remote:
    lx, ly = self.gamepad.lx, self.gamepad.ly  # Valores do joystick
    if abs(lx) > 0.05 or abs(ly) > 0.05:       # Deadzone de 5%
        # Movimento no frame LOCAL do robô (rotacionado pelo heading)
        self.loc_offset[0] += lx * 0.008 * np.cos(self.loc_delta_angle) \
                           + ly * 0.008 * np.sin(self.loc_delta_angle)
        self.loc_offset[1] += -lx * 0.008 * np.sin(self.loc_delta_angle) \
                           + ly * 0.008 * np.cos(self.loc_delta_angle)

        # 0.008 = velocidade de ~0.4 m/s a 50Hz (0.008 * 50 = 0.4)
```

### 7.4 Reset Automático de Alinhamento (linhas 554-565):

```python
# A cada 0.5 segundos, recalcula o offset para alinhar robô com o VP
if time.monotonic() - self.last_loc_reset_time > 0.5:
    rel_loc_offset = np.zeros((3))

    # Ajuste de altura: coloca o pé do robô no chão (+ 1.5cm de margem)
    rel_loc_offset[2:3] = -self.body_pos_extend[..., 2].min() + 0.015

    # Ajuste XY: alinha a localização do robô com a cabeça do VP
    rel_loc_offset[0:2] = -self.location[0:2] + self.head_pos[0:2].numpy()

    # Aplica o ajuste incremental
    self.loc_offset[:] += rel_loc_offset
```

### 7.5 Consumo pelo Position_Client (pos_client.py:76):

```python
# No processo de localização, position_offset aponta para loc_offset
pos_client.position_offset = send_shm_data[0:3]

# Quando recebe dados do SLAM, aplica o offset:
self.position[:] = quat_rotate(self.delta_quat, raw_slam_position) \
                   + self.position_offset  # ← ADICIONA O OFFSET

# Resultado: posição do robô = SLAM + offset do joystick/alinhamento
```

### 7.6 Uso na Observação da Política:

A `self.location` (que contém o offset) afeta a política através de:

```python
# localization.py:108 - body_pos_extend incorpora a location
body_pos_extend_buf[:] = body_pos_extend_buf[:] + location - (...)

# g1_server.py - compute_task_observation()
body_pos_extend = self.body_pos_extend.clone()
body_pos = body_pos_extend[-3:]  # Posição atual do robô

# O ERRO é calculado como:
diff_global_body_pos = ref_body_pos - body_pos
# onde ref_body_pos = posição desejada (VP)
#       body_pos    = posição atual do robô (COM offset aplicado)

# Este erro vai para a política neural!
```

### 7.7 Diagrama de Fluxo do Controle de Locomoção:

```
┌────────────────────┐
│   Joystick (lx,ly) │
└─────────┬──────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  loc_offset += velocidade × direção                      │
│  (g1_server.py:684-685)                                  │
└─────────────────────────┬───────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  position = SLAM_raw + position_offset                   │
│  (pos_client.py:76)                                      │
└─────────────────────────┬───────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  body_pos_extend = FK(joints) rotacionado + location     │
│  (localization.py:107-108)                               │
└─────────────────────────┬───────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  erro = ref_body_pos (VP) - body_pos (robô com offset)   │
│  (g1_server.py:1017)                                     │
│                                                          │
│  Quando loc_offset aumenta → body_pos aumenta            │
│  → erro DIMINUI → política gera menos movimento          │
│                                                          │
│  Efeito: robô "persegue" a posição virtual               │
└─────────────────────────┬───────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  policy(obs_buf) → raw_actions → motores                 │
└─────────────────────────────────────────────────────────┘
```

### 7.8 Efeito Prático do loc_offset:

| Ação no Joystick | Efeito no loc_offset | Efeito na Política |
|------------------|---------------------|-------------------|
| Joystick para frente (+ly) | loc_offset[0,1] aumenta | body_pos aumenta → erro diminui → robô anda para frente |
| Joystick para trás (-ly) | loc_offset[0,1] diminui | body_pos diminui → erro aumenta (invertido) → robô anda para trás |
| Joystick para esquerda (+lx) | loc_offset lateral muda | Robô anda lateralmente |

**Insight chave**: O controle remoto não comanda diretamente os motores. Ele **manipula a percepção de posição do robô**, criando um "erro virtual" que a política neural tenta corrigir, resultando em movimento.

---

## Glossário

| Termo | Significado |
|-------|-------------|
| **Vision Pro** | Apple Vision Pro - headset de realidade mista |
| **DOF** | Degrees of Freedom - graus de liberdade (29 juntas) |
| **Quaternion** | Representação de rotação com 4 componentes (x, y, z, w) |
| **Heading** | Orientação horizontal (rotação em torno do eixo Z) |
| **Forward Kinematics** | Cálculo da posição dos end-effectors a partir dos ângulos |
| **TorchScript** | Formato de modelo PyTorch serializado para produção |
| **IMU** | Inertial Measurement Unit - sensor de orientação/aceleração |
| **ROS2 DDS** | Sistema de comunicação para robótica |

---

## Arquivos Principais

| Arquivo | Função |
|---------|--------|
| `vision_wrapper.py` | Interface com o Apple Vision Pro |
| `g1_server.py` | Loop principal de controle e inferência |
| `models/g1_student_moel.pt` | Modelo neural treinado |
| `config.py` | Configurações (IP, alturas, etc.) |
| `localization.py` | Forward kinematics para posição do robô |
