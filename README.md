# sim_bot

ROS 2 пакет для запуска дифференциального робота в симуляторе Gazebo Harmonic с поддержкой движущихся препятствий.

## Зависимости

| Пакет | Версия |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| ros-jazzy-ros-gz-sim | — |
| ros-jazzy-ros-gz-bridge | — |
| ros-jazzy-ros-gz-image | — |
| ros-jazzy-xacro | — |
| ros-jazzy-joy | — |
| ros-jazzy-teleop-twist-joy | — |

## Сборка

```bash
cd ~/ros2_ws
colcon build --packages-select sim_bot
source install/setup.bash
```

---

## Запуск симуляции

### Вариант 1 — только робот

```bash
ros2 launch sim_bot launch_sim.launch.py
```

С кастомным миром:

```bash
ros2 launch sim_bot launch_sim.launch.py world:=/path/to/world.sdf
```

---

### Вариант 2 — робот + препятствия (два терминала)

**Терминал 1:**

```bash
ros2 launch sim_bot launch_sim.launch.py
```

**Терминал 2** — после того как Gazebo полностью загрузился:

```bash
ros2 launch sim_bot obstacles.launch.py
```

С кастомным конфигом:

```bash
ros2 launch sim_bot obstacles.launch.py \
    obstacles_config:=/path/to/my_obstacles.yaml
```

---

### Вариант 3 — всё одной командой (рекомендуется)

```bash
ros2 launch sim_bot sim_with_obstacles.launch.py
```

Доступные аргументы:

| Аргумент | По умолчанию | Описание |
|---|---|---|
| `world` | `worlds/empty.world` | Путь к SDF-файлу мира |
| `obstacles_config` | `config/obstacles.yaml` | Конфиг препятствий |
| `obstacles_start_delay` | `8.0` | Задержка старта препятствий, секунды |

Пример с переопределением:

```bash
ros2 launch sim_bot sim_with_obstacles.launch.py \
    world:=/path/to/world.sdf \
    obstacles_config:=/path/to/obstacles.yaml \
    obstacles_start_delay:=12.0
```

---

## Запись видео симуляции

В `worlds/empty.world` уже встроен плагин **Video Recorder** и основной 3D-вид.

После запуска симуляции в правом верхнем углу окна Gazebo появится панель записи:

- **●** — начать запись
- **■** — остановить и сохранить

Видео сохраняется в домашнюю директорию (`~/`). Доступные форматы: `mp4`, `ogv`, покадровые `jpg`.

---

## Управление роботом

### Джойстик

Конфиг: `config/joystick.yaml`

| Действие | Элемент управления |
|---|---|
| Движение вперёд/назад | Ось 1 (левый стик) |
| Поворот | Ось 0 (левый стик) |
| Включить движение | Кнопка 6 (LT/L2) |
| Режим турбо | Кнопка 7 (RT/R2) |

Скорость в обычном режиме: `0.5 м/с` / `0.5 рад/с`.
Скорость в турбо-режиме: `1.0 м/с` / `1.0 рад/с`.

### Клавиатура

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Прямая публикация в топик

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.5}}"
```

---

## Конфигурация препятствий

Файл: `config/obstacles.yaml`

### Типы препятствий

| `type` | Форма | Параметры размера |
|---|---|---|
| `cylinder` | круглый цилиндр | `radius`, `height` |
| `elliptic_cylinder` | вытянутая "сосиска" | `radius_x`, `radius_y`, `height` |

### Типы траекторий

| `trajectory` | Поведение | Ключевые параметры |
|---|---|---|
| `static` | стоит на месте | — |
| `linear` | вперёд-назад по одной оси | `linear_vel`, `turn_distance` |
| `circular` | постоянная дуга | `linear_vel`, `angular_vel` |
| `straight_spin` | прямо + вращение вокруг оси | `linear_vel`, `angular_vel`, `world_heading` |
| `sequence` | произвольные сегменты по времени | `segments: [{linear_vel, angular_vel, duration}]` |

### Добавление нового препятствия

1. Добавить имя в список `obstacle_names`.
2. Добавить секцию с параметрами под тем же именем.

```yaml
obstacle_names:
  - my_obs

my_obs:
  type: elliptic_cylinder
  radius_x: 2.0
  radius_y: 0.7
  height: 0.7
  mass: 15.0
  init_x: 3.0
  init_y: 0.0
  init_yaw: 0.0
  trajectory: static
  color_r: 1.0
  color_g: 0.8
  color_b: 0.0
```

### Управление препятствием вручную

```bash
ros2 topic pub /model/my_obs/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

---

## Структура пакета

```
sim_bot/
├── config/
│   ├── obstacles.yaml       # параметры препятствий
│   ├── joystick.yaml        # оси и кнопки джойстика
│   ├── gz_bridge.yaml       # топики ROS ↔ Gazebo
│   └── nav2_params.yaml     # параметры Nav2
├── description/             # URDF/xacro описание робота
├── launch/
│   ├── launch_sim.launch.py           # только робот
│   ├── obstacles.launch.py            # только препятствия
│   └── sim_with_obstacles.launch.py   # всё вместе
├── scripts/
│   └── obstacle_controller.py         # нода управления препятствиями
└── worlds/
    └── empty.world                    # мир со встроенным Video Recorder
```
