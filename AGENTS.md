# sim_bot — руководство для агентов и разработчиков

Пакет **sim_bot** — это ROS 2 (ament_cmake) **ресурсный** пакет: в нём нет собственных исполняемых узлов на C++/Python, только установка в `share/sim_bot` директорий `launch`, `description`, `config`, `worlds`. Сборка через `ament_cmake`; зависимости в `package.xml` минимальны — фактические зависимости задаются окружением (см. `README.md`).

## Назначение

Запуск симуляции дифференциального робота в **Gazebo Harmonic** (Ignition Gazebo) с мостом **ros_gz_bridge** / **ros_gz_image**, публикацией URDF через **robot_state_publisher**, опционально джойстиком и стеком **Nav2** / **slam_toolbox**.

## Структура каталогов

| Каталог / файл | Роль |
|----------------|------|
| `description/` | Xacro-модули URDF: шасси, колёса, Gazebo-плагины, сенсоры. Точка входа — `robot.urdf.xacro`. |
| `launch/` | Сценарии запуска симуляции, RSP, телеопа, навигации, локализации, SLAM. |
| `config/` | YAML: мост Gazebo↔ROS, Nav2, SLAM, джойстик, RViz, контроллеры (для будущего ros2_control). |
| `worlds/` | SDF-миры для `gz_sim` (например `empty.world`, `obstacles.world`). |
| `CMakeLists.txt` | Устанавливает перечисленные директории в share пакета. |
| `package.xml` | Метаданные пакета. |

## Как устроена модель робота (`description/`)

- **`robot.urdf.xacro`** — собирает робота из включений:
  - **`robot_core.xacro`** — `base_link`, `base_footprint`, `chassis`, приводные колёса (`left_wheel_joint`, `right_wheel_joint`), неподвижные «кастеры».
  - **`gazebo_control.xacro`** — плагины Gazebo Sim: **DiffDrive** (`cmd_vel` → одометрия + TF `odom`→`base_link`), **JointStatePublisher** для указанных шарниров.
  - **`lidar.xacro`** — GPU lidar, топик в симе `scan`, кадр `lidar_frame`.
  - **`camera.xacro`** — камера, топик `camera/image_raw`, кадр `camera_optical_link`.
- Закомментированные включения (при необходимости включаются вручную): `ros2_control.xacro`, `tof_sensors.xacro`, `depth_camera.xacro`, `ultrasonics.xacro`.

**Режим привода:** сейчас основной путь — плагин **gz-sim-diff-drive-system** в URDF, а не `ros2_control` (в `launch_sim.launch.py` спавнеры `diff_cont` / `joint_broad` закомментированы; в `rsp.launch.py` передаётся `use_ros2_control:=false`).

## Главный сценарий симуляции (`launch/launch_sim.launch.py`)

Цепочка действий:

1. **`rsp.launch.py`** — xacro → строка URDF → узел `robot_state_publisher` с `use_sim_time:=true`.
2. **`joystick.launch.py`** — `joy_node` + `teleop_twist_joy` → публикация **`/cmd_vel`** (узел `twist_stamper` и ремапы на `diff_cont` закомментированы — это согласовано с режимом без ros2_control).
3. **`ros_gz_sim` `gz_sim.launch.py`** — аргумент `gz_args`: `-r -v4` + путь к миру; по умолчанию мир из аргумента `world` (дефолт — `worlds/empty.world` в пакете).
4. **`ros_gz_sim` `create`** — спавн сущности из топика `robot_description`, имя `my_bot`, начальная поза задаётся аргументами `-x/-y/-z/-Y`.
5. **`ros_gz_bridge` `parameter_bridge`** — конфиг `config/gz_bridge.yaml`: синхронизация `clock`, `joint_states`, `odom`, `tf`, `cmd_vel`, `scan` между ROS 2 и Gazebo.
6. **`ros_gz_image` `image_bridge`** — мост для `/camera/image_raw`.

Аргумент запуска: **`world`** — полный путь к `.world` файлу (можно из этого пакета или внешний).

## Другие launch-файлы

| Файл | Назначение |
|------|------------|
| `rsp.launch.py` | Только URDF + `robot_state_publisher`; аргументы `use_sim_time`, при необходимости расширяйте для `use_ros2_control`. |
| `joystick.launch.py` | Телеоп с `config/joystick.yaml`. |
| `navigation_launch.py` | Стек Nav2 (controller, planner, BT navigator, smoother, lifecycle и т.д.); `params_file` по умолчанию `config/nav2_params.yaml`; ремапы `cmd_vel` ↔ навигация согласованы с параметрами Nav2. |
| `localization_launch.py` | `map_server` + `amcl` + lifecycle; требуется аргумент **`map`** (путь к YAML карты). |
| `online_async_launch.py` | **slam_toolbox** async node; параметры по умолчанию из `config/mapper_params_online_async.yaml`. |

Nav2-лаунчи основаны на шаблонах Intel / Nav2 (`RewrittenYaml`, remapping `/tf` → `tf` и т.д.) — при правках сохраняйте ту же логику подстановки `use_sim_time` и путей к картам.

## Конфигурация моста (`config/gz_bridge.yaml`)

Направления: из Gazebo в ROS — `clock`, `joint_states`, `odom`, `tf`, `scan`; из ROS в Gazebo — **`cmd_vel`**. Имена ROS-топиков должны совпадать с тем, что ожидают узлы (навигация, телеоп, RViz).

## Миры (`worlds/`)

SDF с плагинами физики, сцены, сенсоров, освещения. Подставляются в `launch_sim` через аргумент `world`. Разные файлы могут отличаться физикой/объектами — смотрите конкретный `.world` перед изменением.

## Типичные задачи для агента

- **Сменить сенсор / кинематику** — править соответствующий `.xacro`, проверить топик в `gz_bridge.yaml` и при необходимости в Nav2/SLAM YAML.
- **Включить ros2_control** — раскомментировать `ros2_control.xacro` в `robot.urdf.xacro`, в `rsp` переключить `use_ros2_control`, в `launch_sim` включить спавнеры контроллеров и согласовать `joystick.launch.py` (ремапы/`twist_stamper` на `diff_cont`).
- **Навигация в симе** — поднять симуляцию с `use_sim_time`, затем localization/navigation с тем же `use_sim_time:=true` и валидной картой или SLAM.

## Зависимости (ориентир)

См. `README.md`: ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, `xacro`; для Nav2/SLAM — соответствующие пакеты стека.

## Соглашения

- После изменений в `description/` или `launch/` пересоберите пакет (`colcon build --packages-select sim_bot`), чтобы обновился install-space.
- Новые файлы, которые должны попасть в установку, добавляйте в `CMakeLists.txt` в список `install(DIRECTORY ...)`.
