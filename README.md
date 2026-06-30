# lar_fusao_ekf

Pacote ROS1 (Noetic) para **fusão sensorial com Filtro de Kalman Estendido (EKF)**
do robô **Husky** no ambiente simulado do **LAR/UFBA** (Gazebo), comparando três
configurações de localização contra o *ground truth* do Gazebo:

1. **odometria**
2. **odometria + IMU**
3. **odometria + IMU + GPS**

O GPS (`/fix`, lat/lon) é convertido para coordenadas locais `x/y` e publicado como
`nav_msgs/Odometry` em `/gps/odom` pelo `navsat_transform_node`. A pose fundida sai
em `/odometry/filtered` e é comparada com `/gt/odom` (referência de avaliação — nunca
entra no filtro).

Avaliação Individual – Parte 2 — Localização Robótica, PPGEEC/UFBA, 2026.1.

---

## 1. Pré-requisitos

- **ROS1 Noetic** (Ubuntu 20.04), de preferência no contêiner Docker do `lar_gazebo`.
- Pacotes ROS: `robot_localization`, `teleop_twist_keyboard`, e a simulação do Husky
  (`husky_gazebo`, `husky_control`).
- Ambiente de simulação do LAR: [`lar-deeufba/lar_gazebo`](https://github.com/lar-deeufba/lar_gazebo).

Instale o `robot_localization` se faltar:

```bash
sudo apt update
sudo apt install ros-noetic-robot-localization ros-noetic-teleop-twist-keyboard
```

---

## 2. Instalação

```bash
cd ~/catkin_ws/src
git clone https://github.com/HenriqueNTeixeira/fusao-ekf-husky-lar.git
# o lar_gazebo deve estar no mesmo src
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

Em **cada terminal novo**: `source ~/catkin_ws/devel/setup.bash`

---

## 3. Passo 0 — descobrir os tópicos reais (FAÇA ISSO PRIMEIRO)

Os tópicos sugeridos no enunciado (`/wheel/odom`, `/imu/data`, `/fix`) podem **não**
coincidir com os que a simulação publica de fato. Suba a simulação e verifique:

```bash
# Terminal A — sobe o mundo do LAR + Husky (aperte "play" no Gazebo)
roslaunch lar_gazebo lar_husky.launch

# Terminal B — lista os tópicos relevantes
rostopic list | grep -E "imu|fix|gps|odom"
```

Anote os nomes reais. No Husky padrão costumam ser algo como:

| Função          | Tópico sugerido | Tópico real provável no Husky        |
| --------------- | --------------- | ------------------------------------- |
| Odometria rodas | `/wheel/odom`   | `/husky_velocity_controller/odom` ou `/odometry/filtered` |
| IMU             | `/imu/data`     | `/imu/data` ou `/imu/imu`             |
| GPS             | `/fix`          | `/navsat/fix` ou `/gps/fix`           |

### GPS desligado?

O Husky padrão normalmente **não** publica GPS por default. Se `rostopic list` não
mostrar nenhum `fix`, habilite o sensor **antes** de subir a simulação:

```bash
export HUSKY_NAVSAT_ENABLED=1
# (em algumas versões: export HUSKY_GPS_ENABLED=true)
roslaunch lar_gazebo lar_husky.launch
```

Se ainda assim não aparecer, será preciso adicionar o plugin GPS (`libhector_gazebo_ros_gps`
ou o `navsat` do Husky) ao xacro do robô. Confirme com `rostopic echo -n1 /SEU_FIX`.

### Conflito com o EKF interno do Husky

O Husky já roda um `robot_localization` próprio que publica `/odometry/filtered` e a TF
`odom→base_link`. Para não brigar com o nosso EKF, **desative o do Husky**:

```bash
rosnode list | grep -i ekf          # descubra o nome (ex.: /ekf_localization)
rosnode kill /ekf_localization      # mate o nó interno
```

(Ou, se preferir usar a odometria das rodas crua, aponte `raw_odom` para
`/husky_velocity_controller/odom`.)

---

## 4. Rodando os três experimentos

Para **cada** configuração: suba a simulação, suba o launch correspondente, dirija o
robô pelo mesmo trajeto, e encerre o nó de avaliação com `Ctrl+C` para salvar os
resultados. Passe os tópicos reais via `arg` se forem diferentes dos defaults.

### 4.1. Configuração 1 — só odometria

```bash
# Terminal 1 — simulação (play no Gazebo)
roslaunch lar_gazebo lar_husky.launch

# Terminal 2 — EKF + ground truth + avaliação
roslaunch lar_fusao_ekf fusao_odom.launch raw_odom:=/husky_velocity_controller/odom

# Terminal 3 — teleop (dirija um trajeto de ~1-2 min)
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
#   i = frente | , = ré | j / l = girar

# Ao terminar o trajeto: Ctrl+C no Terminal 2 -> salva resultados/odom
```

### 4.2. Configuração 2 — odometria + IMU

```bash
roslaunch lar_gazebo lar_husky.launch
roslaunch lar_fusao_ekf fusao_odom_imu.launch \
    raw_odom:=/husky_velocity_controller/odom raw_imu:=/imu/data
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# Ctrl+C -> resultados/odom_imu
```

### 4.3. Configuração 3 — odometria + IMU + GPS

```bash
export HUSKY_NAVSAT_ENABLED=1
roslaunch lar_gazebo lar_husky.launch
roslaunch lar_fusao_ekf fusao_odom_imu_gps.launch \
    raw_odom:=/husky_velocity_controller/odom \
    raw_imu:=/imu/data raw_fix:=/navsat/fix
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# Ctrl+C -> resultados/odom_imu_gps
```

> **Dica:** faça um trajeto parecido nas três rodadas (mesma distância/forma) para a
> comparação ser justa. Cada run salva em `~/resultados_fusao/<config>/`
> (`erros.csv`, `metricas.csv`, `trajetoria.png`, `erro_tempo.png`).

Verifique se o `/gps/odom` está saindo na config 3:

```bash
rostopic echo -n1 /gps/odom
rostopic hz /odometry/filtered
```

---

## 5. Análise comparativa (fora do ROS)

Depois de rodar as três configurações:

```bash
python3 ~/catkin_ws/src/lar_fusao_ekf/analise/analisa_resultados.py ~/resultados_fusao
```

Gera, em `~/resultados_fusao/comparacao/`:

- `comp_trajetorias.png` — trajetórias das 3 configs sobre o ground truth
- `comp_erro_tempo.png` — erro de posição × tempo
- `comp_barras.png` — RMSE, erro final e RMSE de orientação por config
- `comp_cdf.png` — CDF do erro de posição
- `resumo_comparativo.csv` — tabela com todas as métricas

Copie a pasta `comparacao/` e os subdiretórios para `resultados/` do repositório
antes de entregar.

---

## 6. Métricas calculadas

O nó `fusion_error.py` compara `/odometry/filtered` com `/gt/odom` e calcula:

- **Erro de posição** — distância euclidiana por amostra
- **RMSE de posição** — `sqrt(mean(erro²))`
- **Erro final** — erro na última amostra
- **Erro máximo** e **percentil 95**
- **Erro de orientação (yaw)** — diferença angular normalizada, RMSE em rad e graus

As trajetórias são alinhadas pelo deslocamento a partir do ponto inicial (remove o
offset constante entre o frame `odom` e o frame do Gazebo; assume que o robô nasce
alinhado com a origem).

---

## 7. Estrutura do repositório

```
lar_fusao_ekf/
├── config/
│   ├── ekf_odom.yaml            # EKF config 1: só odometria
│   ├── ekf_odom_imu.yaml        # EKF config 2: odom + IMU
│   ├── ekf_odom_imu_gps.yaml    # EKF config 3: odom + IMU + GPS
│   └── navsat_transform.yaml    # GPS lat/lon -> /gps/odom
├── launch/
│   ├── fusao_odom.launch
│   ├── fusao_odom_imu.launch
│   └── fusao_odom_imu_gps.launch
├── scripts/
│   ├── gt_odom.py               # /gazebo/model_states -> /gt/odom
│   └── fusion_error.py          # métricas /odometry/filtered x /gt/odom
├── analise/
│   └── analisa_resultados.py    # gráficos comparativos das 3 configs
├── resultados/                  # CSVs + figuras entregues
├── package.xml
└── CMakeLists.txt
```

---

## 8. Discussão dos resultados

| Configuração      | RMSE pos (m) | Erro final (m) | RMSE yaw (graus) |
| ----------------- | ------------ | -------------- | ---------------- |
| Odometria         | 1.157        | 2.000          | 51.66            |
| Odom + IMU        | 0.431        | 0.680          | 0.065            |
| Odom + IMU + GPS  | 0.007        | 0.006          | 0.039            |

Os resultados confirmam o comportamento esperado. Com apenas odometria, o erro
cresce de forma acumulativa (deriva), já que o filtro integra velocidades sem
nenhuma correção absoluta; o erro de orientação chega a ~52 graus. Adicionar a
IMU corrige drasticamente a estimativa de yaw (de ~52 para ~0.07 graus) e, por
consequência, reduz a deriva de posição (RMSE de 1.16 para 0.43 m). A inclusão
do GPS fornece correção absoluta de posição, mantendo o erro limitado ao longo
de todo o trajeto (RMSE de 0.007 m e erro final de 0.006 m), sem crescimento
com o tempo. Gráficos completos em resultados/comparacao/.
