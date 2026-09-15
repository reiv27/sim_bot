"""
Launch file for moving obstacles in Gazebo Harmonic.

Run this after the main simulation is up:

    ros2 launch sim_bot launch_sim.launch.py
    ros2 launch sim_bot obstacles.launch.py

Optional override:

    ros2 launch sim_bot obstacles.launch.py \\
        obstacles_config:=/path/to/my_obstacles.yaml

What happens:
  1. Reads obstacle parameters from config/obstacles.yaml.
  2. Generates an SDF model file for each obstacle in /tmp/.
  3. After 3 s, spawns every obstacle in Gazebo via ros_gz_sim create.
  4. After 1 s, starts a ros_gz_bridge that forwards
         /model/<name>/cmd_vel  (ROS) → /model/<name>/cmd_vel  (GZ)
     for every obstacle so the controller can drive them.
  5. After 5 s, starts the obstacle_controller node.

Timing note: delays assume Gazebo is already running.
If you include this launch inside launch_sim.launch.py, increase the
spawn delay to ≥ 8 s to give Gazebo time to initialise.

Obstacle SDF details:
  cylinder        — native <cylinder> geometry for both visual and collision.
  elliptic_cyl.   — <polyline> stadium contour (straight sides + semicircular
                    caps) extruded to 'height'; flat top/bottom Z faces;
                    box + cylinders for collision (DART has no polyline support).
  Both types have gravity disabled (<gravity>false</gravity>) so the obstacle
  remains at its spawn height. The gz-sim-velocity-control-system plugin lets
  the obstacle_controller set body-frame linear + angular velocity via cmd_vel.

Trajectory editing note:
  The controller reads the same YAML file directly, so obstacle motion can be
  changed via trajectory_profiles / segments without editing Python code.
"""

import math
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


# ── SDF helpers ───────────────────────────────────────────────────────────────

def _velocity_control_plugin() -> str:
    return (
        '<plugin filename="gz-sim-velocity-control-system"'
        ' name="gz::sim::systems::VelocityControl">'
        '</plugin>'
    )


def _material_xml(r: float, g: float, b: float) -> str:
    return (
        f'<material>'
        f'<ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>'
        f'<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>'
        f'<specular>0.1 0.1 0.1 1</specular>'
        f'</material>'
    )


def _inertia_xml(ixx: float, iyy: float, izz: float) -> str:
    return (
        f'<inertia>'
        f'<ixx>{ixx:.5f}</ixx><iyy>{iyy:.5f}</iyy><izz>{izz:.5f}</izz>'
        f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>'
        f'</inertia>'
    )



def generate_cylinder_sdf(name: str, p: dict) -> str:
    r = float(p['radius'])
    h = float(p['height'])
    m = float(p['mass'])
    ixx = (m / 12.0) * (3.0 * r**2 + h**2)
    iyy = ixx
    izz = 0.5 * m * r**2
    mat = _material_xml(
        float(p.get('color_r', 1.0)),
        float(p.get('color_g', 0.5)),
        float(p.get('color_b', 0.0)),
    )
    geom = f'<cylinder><radius>{r}</radius><length>{h}</length></cylinder>'
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.6">'
        f'<model name="{name}">'
        f'{_velocity_control_plugin()}'
        '<link name="link">'
        '<gravity>false</gravity>'
        f'<inertial><mass>{m}</mass>{_inertia_xml(ixx, iyy, izz)}</inertial>'
        f'<collision name="collision"><geometry>{geom}</geometry></collision>'
        f'<visual name="visual"><geometry>{geom}</geometry>{mat}</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    )


def _stadium_polyline_xml(rx: float, ry: float, h: float, n_cap: int = 16) -> str:
    """Stadium (discorectangle) contour extruded to height h.

    The shape has straight sides of length 2*(rx-ry) along X and
    semicircular caps of radius ry at ±rx along X.
    Total extent: 2*rx along X, 2*ry along Y.
    n_cap — number of points per semicircular cap (total points = 2*n_cap).
    """
    half_len = rx - ry   # half-length of the straight section
    pts = []
    # Right cap (+X side): angles from -pi/2 to +pi/2
    for i in range(n_cap + 1):
        a = -math.pi / 2.0 + math.pi * i / n_cap
        pts.append((half_len + ry * math.cos(a), ry * math.sin(a)))
    # Left cap (-X side): angles from +pi/2 to +3*pi/2
    for i in range(n_cap + 1):
        a = math.pi / 2.0 + math.pi * i / n_cap
        pts.append((-half_len + ry * math.cos(a), ry * math.sin(a)))
    points_xml = ''.join(f'<point>{x:.4f} {y:.4f}</point>' for x, y in pts)
    return f'<polyline>{points_xml}<height>{h}</height></polyline>'


def generate_elliptic_cylinder_sdf(name: str, p: dict) -> str:
    """Stadium-shaped extrusion ("sausage") in the XY plane with flat Z faces.

    radius_x — half total length along X (including semicircular caps)
    radius_y — half-width / cap radius along Y
    height   — vertical extent (flat top and bottom faces)

    A polygonal stadium contour is used for the visual. Collision uses the
    analytical stadium (box + two cylinders), supported by DART. The visual
    polyline extrudes from z=0 to z=h, so it is shifted by -h/2.
    """
    rx = float(p['radius_x'])
    ry = float(p['radius_y'])
    h  = float(p['height'])
    m  = float(p['mass'])
    if rx < ry or ry <= 0 or h <= 0:
        raise ValueError('Stadium needs radius_x >= radius_y > 0 and height > 0')

    # Inertia of a solid elliptic cylinder (close enough for a stadium shape)
    ixx    = (m / 12.0) * (3.0 * ry**2 + h**2)
    iyy    = (m / 12.0) * (3.0 * rx**2 + h**2)
    izz    = 0.25 * m * (rx**2 + ry**2)

    mat = _material_xml(
        float(p.get('color_r', 0.0)),
        float(p.get('color_g', 0.5)),
        float(p.get('color_b', 1.0)),
    )
    stadium_geom = _stadium_polyline_xml(rx, ry, h)
    # polyline starts at z=0; shift down so centre is at link origin
    geom_pose = f'<pose>0 0 {-h / 2.0:.4f} 0 0 0</pose>'
    # DART cannot construct a collision from SDF polyline. A box plus two
    # cylinders has exactly the same stadium boundary, on this same link.
    half_len = rx - ry
    collision_xml = ''
    if half_len > 0:
        collision_xml += (
            '<collision name="collision_middle"><geometry><box>'
            f'<size>{2*half_len} {2*ry} {h}</size>'
            '</box></geometry></collision>'
        )
    for index, cap_x in enumerate((-half_len, half_len) if half_len > 0 else (0.0,)):
        collision_xml += (
            f'<collision name="collision_cap_{index}">'
            f'<pose>{cap_x} 0 0 0 0 0</pose><geometry><cylinder>'
            f'<radius>{ry}</radius><length>{h}</length>'
            '</cylinder></geometry></collision>'
        )
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.6">'
        f'<model name="{name}">'
        f'{_velocity_control_plugin()}'
        '<link name="link">'
        '<gravity>false</gravity>'
        f'<inertial><mass>{m}</mass>{_inertia_xml(ixx, iyy, izz)}</inertial>'
        f'{collision_xml}'
        f'<visual name="visual">'
        f'{geom_pose}'
        f'<geometry>{stadium_geom}</geometry>'
        f'{mat}'
        f'</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    )


def _load_obstacle_params(config_path: str) -> dict:
    with open(config_path, 'r', encoding='utf-8') as fh:
        raw = yaml.safe_load(fh) or {}
    return raw.get('obstacle_controller', {}).get('ros__parameters', {})


def generate_rigid_formation_sdf(name: str, obstacles: dict) -> str:
    """One rigid link with separate collision/visual shapes; gaps stay empty.

    Offsets are expressed in the formation frame. Its origin is at ground
    level, with each member's centre at height/2 + 0.01. Aggregate inertia is
    expressed at the combined centre of mass in axes parallel to this frame.
    """
    if not obstacles:
        raise ValueError('A rigid formation needs at least one obstacle')
    root = ET.Element('sdf', version='1.6')
    model = ET.SubElement(root, 'model', name=name)
    model.append(ET.fromstring(_velocity_control_plugin()))
    link = ET.SubElement(model, 'link', name='formation_link')
    ET.SubElement(link, 'gravity').text = 'false'
    bodies = []
    for member, cfg in obstacles.items():
        if any(key in cfg for key in ('init_x', 'init_y', 'init_yaw')):
            raise ValueError(f'{member}: use offset_x/y/yaw inside a formation')
        generator = {
            'cylinder': generate_cylinder_sdf,
            'elliptic_cylinder': generate_elliptic_cylinder_sdf,
        }[cfg.get('type', 'cylinder')]
        source = ET.fromstring(generator(member, cfg)).find('model/link')
        x = float(cfg.get('offset_x', 0.0))
        y = float(cfg.get('offset_y', 0.0))
        z = float(cfg['height']) / 2.0 + 0.01
        yaw = float(cfg.get('offset_yaw', 0.0))
        mass = float(cfg['mass'])
        if mass <= 0:
            raise ValueError(f'{member}: mass must be positive')
        c, s = math.cos(yaw), math.sin(yaw)
        inertia = source.find('inertial/inertia')
        ix, iy, iz = [float(inertia.findtext(k)) for k in ('ixx', 'iyy', 'izz')]
        rotated = [[c*c*ix + s*s*iy, c*s*(ix-iy), 0.0],
                   [c*s*(ix-iy), s*s*ix + c*c*iy, 0.0],
                   [0.0, 0.0, iz]]
        bodies.append((mass, (x, y, z), rotated))
        for shape in source.findall('collision') + source.findall('visual'):
            shape.set('name', f"{member}_{shape.get('name')}")
            pose = shape.find('pose')
            old = list(map(float, pose.text.split())) if pose is not None else [0.0]*6
            if pose is None:
                pose = ET.SubElement(shape, 'pose')
            pose.text = f'{x+c*old[0]-s*old[1]} {y+s*old[0]+c*old[1]} {z+old[2]} 0 0 {yaw+old[5]}'
            link.append(shape)
    total_mass = sum(body[0] for body in bodies)
    centre = [sum(m * p[i] for m, p, _ in bodies) / total_mass for i in range(3)]
    tensor = [[0.0] * 3 for _ in range(3)]
    for mass, pos, rotated in bodies:
        d = [pos[i] - centre[i] for i in range(3)]
        d2 = sum(v*v for v in d)
        for i in range(3):
            for j in range(3):
                tensor[i][j] += rotated[i][j] + mass * (
                    (d2 if i == j else 0.0) - d[i]*d[j])
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = str(total_mass)
    ET.SubElement(inertial, 'pose').text = ' '.join(map(str, centre)) + ' 0 0 0'
    inertia = ET.SubElement(inertial, 'inertia')
    for key, i, j in [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
                      ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]:
        ET.SubElement(inertia, key).text = str(tensor[i][j])
    return ET.tostring(root, encoding='unicode')


def _resolve_obstacle_config(name: str, obstacle_cfg: dict, profiles: dict) -> dict:
    profile_name = obstacle_cfg.get('trajectory_profile')
    if not profile_name:
        return dict(obstacle_cfg)
    if profile_name not in profiles:
        raise ValueError(
            f"Obstacle '{name}' references unknown trajectory_profile "
            f"'{profile_name}'"
        )

    resolved = dict(profiles[profile_name])
    resolved.update(obstacle_cfg)
    return resolved


def _build_obstacle_actions(context):
    config_path = LaunchConfiguration('obstacles_config').perform(context)
    params = _load_obstacle_params(config_path)
    obstacle_names = params.get('obstacle_names', [])
    profiles = params.get('trajectory_profiles', {})
    actions = []
    bridge_entries: list = []

    obstacles = {
        name: _resolve_obstacle_config(name, params[name], profiles)
        for name in obstacle_names
    }
    formation = params.get('rigid_formation')
    formation_sdf = None
    if formation is not None:
        name = formation['name']
        formation_sdf = generate_rigid_formation_sdf(name, obstacles)
        obstacles = {name: formation}

    for name, obs in obstacles.items():
        obs_type = obs.get('type', 'cylinder')
        h = float(obs.get('height', 1.0))
        x = float(obs.get('init_x', 0.0))
        y = float(obs.get('init_y', 0.0))
        z = h / 2.0 + 0.01
        yaw = float(obs.get('init_yaw', 0.0))

        if formation_sdf is not None:
            sdf_content = formation_sdf
            z = 0.0
        elif obs_type == 'elliptic_cylinder':
            sdf_content = generate_elliptic_cylinder_sdf(name, obs)
        else:
            sdf_content = generate_cylinder_sdf(name, obs)

        sdf_path = os.path.join(tempfile.gettempdir(), f'sim_bot_{name}.sdf')
        with open(sdf_path, 'w', encoding='utf-8') as fh:
            fh.write(sdf_content)

        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', sdf_path,
                '-name', name,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-Y', str(yaw),
            ],
            output='screen',
        )
        actions.append(TimerAction(period=3.0, actions=[spawn]))

        bridge_entries.append({
            'ros_topic_name': f'/model/{name}/cmd_vel',
            'gz_topic_name': f'/model/{name}/cmd_vel',
            'ros_type_name': 'geometry_msgs/msg/Twist',
            'gz_type_name': 'gz.msgs.Twist',
            'direction': 'ROS_TO_GZ',
        })

    bridge_yaml_path = os.path.join(
        tempfile.gettempdir(), 'sim_bot_obstacles_bridge.yaml'
    )
    with open(bridge_yaml_path, 'w', encoding='utf-8') as fh:
        yaml.dump(bridge_entries, fh)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml_path}'],
        output='screen',
    )
    controller = Node(
        package='sim_bot',
        executable='obstacle_controller.py',
        name='obstacle_controller',
        parameters=[{
            'config_path': config_path,
            'use_sim_time': True,
        }],
        output='screen',
    )

    actions.append(TimerAction(period=1.0, actions=[bridge]))
    actions.append(TimerAction(period=5.0, actions=[controller]))
    return actions


# ── Launch description ────────────────────────────────────────────────────────

def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('sim_bot')
    default_config = os.path.join(pkg, 'config', 'obstacles.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacles_config',
            default_value=default_config,
            description='Path to obstacles YAML configuration file',
        ),
        OpaqueFunction(function=_build_obstacle_actions),
    ])
