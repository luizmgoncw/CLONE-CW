# Curiosidades Técnicas - CLONE

---

## 1. Controle Gamepad: Referencial Global vs Robô

**Data:** 2025-12-03

### Referencial Global Fixo (original)
```python
# loc_delta_angle = offset fixo definido na calibração (R1/R2)
offset_x = lx * cos(loc_delta_angle) + ly * sin(loc_delta_angle)
offset_y = -lx * sin(loc_delta_angle) + ly * cos(loc_delta_angle)
```

### Referencial do Robô (novo)
```python
# robot_heading = heading atual do robô (muda conforme ele gira)
offset_x = ly * cos(robot_heading) + lx * sin(robot_heading)
offset_y = ly * sin(robot_heading) - lx * cos(robot_heading)
```

### Por que a fórmula muda?
- `loc_delta_angle` é um **offset de calibração** (diferença robô-operador)
- `robot_heading` é o **heading absoluto** do robô

Para referencial do robô: frente = `[cos(θ), sin(θ)]`, direita = `[sin(θ), -cos(θ)]`

**Não basta trocar a variável** - `lx` e `ly` trocam de posição na fórmula.

**Arquivo:** `g1_server.py:688-694`

---

## 2. Heading vs Yaw

**Data:** 2025-12-03

| | `self.yaw` | `calc_heading(obs_quat)` |
|---|---|---|
| Fonte | IMU direto (`rpy[2]`) | Quaternion corrigido |
| Com robô nivelado | ≈ iguais | ≈ iguais |
| Com robô inclinado | Pode ter erro | Mais preciso |

`calc_heading` projeta o vetor frente no plano XY → "para onde o robô vai se andar reto"

**Arquivo:** `teleop/torch_utils.py:415-426`

---

## 3. Convenção de Eixos (ROS)

- **X** = frente do robô
- **Y** = esquerda do robô
- **Z** = cima

Por isso `quat_rotate(q, [1,0,0])` retorna a direção da frente do robô no mundo.

---

*Última atualização: 2025-12-03*
