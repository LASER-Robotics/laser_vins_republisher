# VINS Republisher - Documentação

## Visão Geral
O VINS Republisher é um nó ROS 2 que reprocessa e republica mensagens de odometria do sistema VINS (Visual-Inertial Navigation System). Ele realiza transformações de coordenadas, filtragem de taxa e ajustes de referência para integrar corretamente a odometria VINS com outros sistemas.

## Instalação

### Dependências
Certifique-se de ter instalado:
- ROS 2 (versão Foxy ou superior)
- Pacotes ROS 2:
  ```bash
  sudo apt-get install ros-<distro>-nav-msgs ros-<distro>-tf2-ros ros-<distro>-geometry-msgs
  ```

### Compilação
1. Clone o repositório para seu workspace ROS 2:
   ```bash
   cd ~/ros2_ws/src
   git clone <seu-repositorio>
   ```
2. Compile o pacote:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select laser_vins_republisher
   source install/setup.bash
   ```

## Como Usar

### Execução
Para iniciar o nó:
```bash
ros2 run laser_vins_republisher vins_republisher_nodevins_republisher.launch.py
```

### Parâmetros Configuráveis

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `velocity_in_body_frame` | bool | false | Se true, a velocidade é expressa no quadro do corpo do UAV |
| `init_in_zero` | bool | false | Se true, inicializa a posição em zero |
| `compensate_initial_tilt` | bool | false | Compensa a inclinação inicial do UAV |
| `rate_limiter.enabled` | bool | false | Habilita limitador de taxa |
| `rate_limiter.max_rate` | double | 10.0 | Taxa máxima de publicação (Hz) |
| `UAV_NAME` | string | "uav" | Nome do UAV para namespaces |
| `transform.fcu_frame` | string | "fcu" | Quadro de referência do FCU |
| `transform.vins_imu_frame` | string | "vins" | Quadro IMU do VINS |
| `transform.vins_world_frame` | string | "vins_world" | Quadro mundial do VINS |
| `transform.translations.x/y/z` | double | 0.0 | Translação entre quadros (metros) |
| `transform.rotations.r/p/y` | double | 0.0 | Rotação entre quadros (radianos) |

### Tópicos

#### Subscritos
- `odometry_in` (`nav_msgs/msg/Odometry`): Odometria bruta do VINS

#### Publicados
- `odometry_out` (`nav_msgs/msg/Odometry`): Odometria processada
- `/tf` e `/tf_static`: Transformações entre quadros de referência

## Funcionamento Interno

### Fluxo de Dados
1. Recebe odometria bruta do VINS (`odometry_in`)
2. Aplica transformações de quadro de referência
3. Valida os dados numéricos
4. Aplica compensações de inclinação (se habilitado)
5. Publica odometria processada (`odometry_out`)

### Transformações Principais
O nó realiza as seguintes transformações de coordenadas:
1. Transformação estática entre `fcu_frame` e `vins_imu_frame`
2. Transformação dinâmica da odometria do quadro VINS para o quadro do UAV
3. Compensação de inclinação inicial (se habilitada)

### Validações
- Verifica valores NaN/Inf na odometria
- Valida normalização de quatérnios
- Verifica consistência de transformações TF

## Exemplos de Uso

### Configuração Básica
```bash
ros2 run laser_vins_republisher vins_republisher_node \
  --ros-args -p UAV_NAME:="uav1" \
  -p velocity_in_body_frame:=true \
  -p compensate_initial_tilt:=true
```

### Visualização
Para visualizar as transformações:
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

Para visualizar a odometria:
```bash
ros2 topic echo /odometry_out
```

## Troubleshooting

### Problemas Comuns
1. **Transformações faltando**:
   - Verifique se todos os frames estão corretamente definidos
   - Use `ros2 run tf2_ros tf2_echo <frame1> <frame2>` para debug

2. **Odometria inválida**:
   - Verifique os logs para mensagens de validação
   - Confira se o VINS está publicando dados válidos

3. **Problemas de taxa**:
   - Ajuste `rate_limiter.max_rate` conforme necessário
   - Verifique a taxa de publicação com `ros2 topic hz /odometry_in`

## Contribuição
Contribuições são bem-vindas! Por favor, abra issues ou pull requests no repositório do projeto.

## Licença
