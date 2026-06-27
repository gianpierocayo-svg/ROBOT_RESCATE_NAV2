[README.md](https://github.com/user-attachments/files/29403719/README.md)
#  robot_rescue_nav

> Navegación autónoma reactiva para robot de búsqueda y rescate — ROS 2 Humble + Nav2 + Behavior Trees

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-green)](https://navigation.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-orange)](http://gazebosim.org/)
[![BehaviorTree](https://img.shields.io/badge/BehaviorTree.CPP-v3-purple)](https://www.behaviortree.dev/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

---

##  Descripción

`robot_rescue_nav` implementa un sistema autónomo de búsqueda y rescate sobre un **TurtleBot3 Waffle** simulado en Gazebo Classic. El robot navega de forma autónoma en un entorno multi-sala mientras evalúa en tiempo real **cuatro reglas de comportamiento reactivo** mediante un Behavior Tree personalizado integrado en Nav2.

El sistema incorpora una **arquitectura dual de seguridad**: el Behavior Tree gestiona la navegación reactiva cuando hay un goal activo, y un monitor de seguridad independiente (`rescue_safety_monitor.py`) garantiza la vigilancia continua de los sensores incluso cuando el robot está detenido.

---

## Reglas de Comportamiento Reactivo

| Prioridad | Regla | Condición | Acción |
|:---------:|-------|-----------|--------|
| 1 | **Gas Tóxico** | `/sensor_gas_critico` = `"CRITICO"` | `BackUp` — retrocede 0.5 m |
| 2 | **Pérdida de Heartbeat** | Sin señal en `/heartbeat` > 3 s | `Spin` — giro 90° para recuperar señal |
| 3 | **Batería Crítica** | `/battery_status` < 20% | `Spin` 360° + retorno automático a base |
| 4 | **Detección de Víctima** | Visual + térmica confirmadas | `Wait` 5 s + registro TF de coordenadas |

---

## Arquitectura del Sistema

```
sensor_simulator.py          rescue_safety_monitor.py
   (publica sensores)    →       (vigila SIEMPRE)
          ↓                            ↓
   Behavior Tree (Nav2)         /cmd_vel directo
   [solo con goal activo]       + cancela goals
          ↓
      Robot (Gazebo)
```

### Componentes principales

```
robot_rescue_nav/
├── behavior_trees/
│   └── arbol.xml                  # BT activo: ReactiveSequence + Inverter
├── config/
│   └── nav2_params.yaml           # Configuración Nav2 (plantilla dinámica)
├── launch/
│   └── rescue_navigation.launch.py
├── maps/
│   ├── multi_sala.pgm             # Mapa generado por SLAM
│   └── multi_sala.yaml
├── scripts/
│   ├── sensor_simulator.py        # Simulador geoespacial de sensores
│   └── rescue_safety_monitor.py   # Monitor de seguridad independiente
├── src/
│   ├── check_gas_condition.cpp        # Plugin Regla 1
│   ├── check_heartbeat_condition.cpp  # Plugin Regla 2
│   ├── check_battery_condition.cpp    # Plugin Regla 3
│   ├── check_visual_condition.cpp     # Plugin Regla 4a
│   ├── check_thermal_condition.cpp    # Plugin Regla 4b
│   └── log_victim_action.cpp          # Plugin Regla 4c (TF)
├── urdf/
│   └── waffle_custom.urdf.xacro   # TurtleBot3 con sensores personalizados
└── worlds/
    └── multi_sala.world           # Entorno Gazebo multi-sala
```

---

## ⚙️ Requisitos

| Dependencia | Versión |
|-------------|---------|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo Classic | 11 |
| Nav2 | Humble |
| BehaviorTree.CPP | v3 |
| TurtleBot3 packages | Humble |
| slam_toolbox | Humble |

---

## Instalación y Uso

### 1. Clonar el repositorio

```bash
cd ~/ros2_ws/src
git clone https://github.com/USUARIO/robot_rescue_nav.git
```

### 2. Instalar dependencias

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Compilar

```bash
colcon build --packages-select robot_rescue_nav
source install/setup.bash
```

### 4. Lanzar el sistema completo

```bash
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle

ros2 launch robot_rescue_nav rescue_navigation.launch.py
```

### 5. Inicializar localización en RViz2

Una vez que aparezca `Managed nodes are active` en el log:

1. En RViz2, usar **2D Pose Estimate** para indicar la posición inicial del robot
2. Usar **2D Goal Pose** para enviar un destino de navegación

---

## Prueba de las Reglas

### Regla 1 — Gas Tóxico
```bash
# El simulador publica CRITICO automaticamente cuando el robot
# se acerca a las coordenadas (1.5, 1.0). Tambien se puede forzar:
ros2 topic pub /sensor_gas_critico std_msgs/msg/String \
  "{data: 'CRITICO'}" --rate 5
```

### Regla 2 — Pérdida de Heartbeat
```bash
# Matar el simulador (unico publisher de /heartbeat)
pkill -f "sensor_simulator.py"
# Esperar 3 segundos -> el robot ejecuta Spin 90°
```

### Regla 3 — Batería Crítica
```bash
# La bateria baja automaticamente 0.1% por segundo (~200 s)
# Para acelerar, relanzar con parametro:
ros2 launch robot_rescue_nav rescue_navigation.launch.py \
  battery_drain_percent_per_sec:=1.0
```

### Regla 4 — Detección de Víctima
```bash
# El simulador publica la victima cuando el robot se acerca a (2.5, -2.0)
# Tambien se puede forzar:
ros2 topic pub /detected_people std_msgs/msg/Bool "{data: true}" --rate 5
```

---

##  Detalles de Implementación

### Fix crítico: spin_some() en cada plugin

Los plugins del BT no procesaban mensajes porque comparten el executor del `bt_navigator`. La solución fue que cada plugin cree su **propio nodo ROS 2** y llame `spin_some()` en cada `tick()`:

```cpp
// Constructor: nodo propio con ID unico
const auto id = instance_counter_++;
node_ = rclcpp::Node::make_shared(
  "check_gas_condition_bt_node_" + std::to_string(id));

// tick(): procesar mensajes antes de evaluar
BT::NodeStatus tick() override {
  rclcpp::spin_some(node_);   // <- clave del fix
  std::lock_guard<std::mutex> lock(mutex_);
  // ...
}
```

### Tópico de gas: String, no Float32

El simulador publica `"NORMAL"` o `"CRITICO"` en `/sensor_gas_critico` basándose en la distancia del robot al foco de fuga configurado. El plugin acepta múltiples formatos: `"CRITICO"`, `"ALTO"`, `"HIGH"`, `"1"`, o valores numéricos comparados contra el umbral de 50.0.

### Estructura del Behavior Tree

```
RecoveryNode (6 reintentos)
├── ReactiveSequence "NavigateWithEmergencyCheck"
│   ├── Inverter → CheckGasCondition        ← re-evaluado CADA tick
│   ├── Inverter → CheckHeartbeatCondition  ← re-evaluado CADA tick
│   ├── Inverter → CheckBatteryCondition    ← re-evaluado CADA tick
│   ├── ReactiveFallback (víctima | AlwaysSuccess)
│   └── PipelineSequence (ComputePath + FollowPath)
└── ReactiveFallback "RecuperacionYEmergencia"
    ├── GoalUpdated
    └── RoundRobin
        ├── Sequence → Gas → BackUp
        ├── Sequence → HB  → Spin 90°
        ├── Sequence → Bat → Spin 360°
        ├── LimpiezaCostmaps
        ├── Spin
        └── Wait
```

---

##  Tópicos del Sistema

| Tópico | Tipo | Dirección |
|--------|------|-----------|
| `/sensor_gas_critico` | `std_msgs/String` | Simulador → Plugins |
| `/heartbeat` | `std_msgs/String` | Simulador → Plugins |
| `/battery_status` | `sensor_msgs/BatteryState` | Simulador → Plugins |
| `/detected_people` | `std_msgs/Bool` | Simulador → Plugins |
| `/thermal_sensor` | `sensor_msgs/Image` | Simulador → Plugins |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → Nav2 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2/Monitor → Gazebo |
| `/rescue/alerts` | `std_msgs/String` | Monitor → Sistema |
| `/rescue/safety_state` | `std_msgs/String` | Monitor → Sistema |

---

##  Recursos adicionales

- [`README_COMANDOS_PRUEBA.md`](README_COMANDOS_PRUEBA.md) — Guía detallada de prueba de cada regla
- [`behavior_trees/arbol.xml`](behavior_trees/arbol.xml) — Árbol de comportamiento completo
- [`config/nav2_params.yaml`](config/nav2_params.yaml) — Configuración de Nav2

---

##  Autor

**Gianpiero Cayo Delgado**
Ingeniería Mecatrónica — Universidad Católica San Pablo
Curso: Robótica Móvil — 2026-I

