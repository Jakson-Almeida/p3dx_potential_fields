# P3DX Multi-Waypoints Navigation

Este script implementa navegação autônoma do robô Pioneer P3-DX por múltiplos waypoints usando campos potenciais, com funcionalidade de parada de emergência.

## Funcionalidades

### 1. Navegação por Waypoints
- Navega automaticamente por uma lista predefinida de pontos
- Waypoints hard-coded: `[(-1, -3), (-1, 0), (1.5, 0)]`
- Para o robô quando chega a cada waypoint
- Continua para o próximo waypoint automaticamente

### 2. Botão de Emergência
- Pressione **SPACE** a qualquer momento para parada de emergência
- Para imediatamente o movimento do robô
- Interrompe a sequência de navegação
- Pressione **Ctrl+C** para sair do programa

### 3. Algoritmo de Campos Potenciais
- Força atrativa: atrai o robô para o waypoint atual
- Força repulsiva: afasta o robô dos obstáculos detectados
- Evita colisões usando dados do sensor laser

## Como Usar

### 1. Executar o Script
```bash
rosrun p3dx_potential_fields p3dx_multi_waypoints.py
```

### 2. Controles
- **SPACE**: Parada de emergência
- **Ctrl+C**: Sair do programa

### 3. Saída do Terminal
```
=== NAVEGAÇÃO POR WAYPOINTS ===
Waypoints definidos: [(-1, -3), (-1, 0), (1.5, 0)]
Pressione SPACE a qualquer momento para parada de emergência
Pressione Ctrl+C para sair
========================================

--- Navegando para waypoint 1/3: (-1, -3) ---
Objetivo alcançado: (-1, -3)

--- Navegando para waypoint 2/3: (-1, 0) ---
Objetivo alcançado: (-1, 0)

--- Navegando para waypoint 3/3: (1.5, 0) ---
Objetivo alcançado: (1.5, 0)

=== Navegação concluída! ===
```

## Parâmetros Ajustáveis

### Waypoints
Edite a lista `WAYPOINTS` no início do script:
```python
WAYPOINTS = [(-1, -3), (-1, 0), (1.5, 0)]
```

### Parâmetros do Algoritmo
Na função `potential_field()`:
- `K_att=5`: Ganho da força atrativa
- `K_rep=0.05`: Ganho da força repulsiva
- `epsilon_0=2.0`: Distância de influência dos obstáculos
- `v_max=0.3`: Velocidade linear máxima
- `omega_max=2*np.pi`: Velocidade angular máxima
- `tol=0.1`: Tolerância para considerar objetivo alcançado

## Diferenças do Script Original

### Estrutura Mantida
- Mesma lógica de campos potenciais
- Mesmos callbacks para odometria e laser
- Mesma detecção de obstáculos

### Novas Funcionalidades
1. **Navegação Sequencial**: Percorre lista de waypoints automaticamente
2. **Botão de Emergência**: Parada imediata com SPACE
3. **Feedback Melhorado**: Mostra progresso da navegação
4. **Controle de Interrupção**: Retorna False se interrompido por emergência

## Requisitos

- ROS Noetic
- Ubuntu 20.04
- Pacote `p3dx_gazebo` instalado
- Simulação Gazebo rodando

## Troubleshooting

### Problema: Script não responde ao SPACE
- Certifique-se de que o terminal tem foco
- Verifique se não há outros programas capturando teclas

### Problema: Robô não para na emergência
- Verifique se o publisher `/RosAria/cmd_vel` está funcionando
- Confirme se o robô está recebendo os comandos

### Problema: Navegação lenta
- Ajuste `v_max` para aumentar velocidade
- Reduza `tol` para tolerância menor
- Ajuste `K_att` e `K_rep` para melhor performance 