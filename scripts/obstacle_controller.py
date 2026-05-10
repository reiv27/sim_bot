#!/usr/bin/env python3
"""
ROS 2 node that drives moving obstacles in Gazebo Harmonic via VelocityControl.

The node reads obstacle and trajectory configuration from a YAML file passed via
the 'config_path' parameter. The YAML may define reusable trajectory profiles
and obstacles can either reference a profile or specify motion inline.

Supported trajectories:
  linear   — moves forward at linear_vel, reverses after turn_distance metres.
  circular — constant linear_vel + angular_vel → circular arc with radius v / w.
  static   — zero twist; obstacle stays at its spawn pose.
  sequence — repeats a list of timed segments, each with linear_vel,
             angular_vel and duration.
  straight_spin — keeps moving along a fixed world heading while spinning
                  around its own axis.

Velocity commands are published as geometry_msgs/Twist to /model/<name>/cmd_vel.
"""

import math
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ObstacleController(Node):
    _CONTROL_HZ = 20.0

    def __init__(self):
        super().__init__('obstacle_controller')
        self._dt = 1.0 / self._CONTROL_HZ
        self._obstacles: dict = {}
        self._pubs: dict = {}
        self._start_time = None  # set lazily on first control tick (sim clock ready)

        for name, cfg in self._load_obstacle_configs().items():
            self._obstacles[name] = self._build_motion_state(name, cfg)
            self._pubs[name] = self.create_publisher(
                Twist, f'/model/{name}/cmd_vel', 10
            )
            self.get_logger().info(
                f"Controlling '{name}' — trajectory: "
                f"{self._obstacles[name]['trajectory']}"
            )

        self.create_timer(self._dt, self._control_loop)

    def _load_obstacle_configs(self) -> dict:
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if config_path:
            return self._load_obstacle_configs_from_yaml(config_path)
        return self._load_legacy_obstacle_configs()

    def _load_obstacle_configs_from_yaml(self, config_path: str) -> dict:
        with Path(config_path).open('r', encoding='utf-8') as fh:
            raw = yaml.safe_load(fh) or {}

        params = raw.get('obstacle_controller', {}).get('ros__parameters', {})
        obstacle_names = params.get('obstacle_names', [])
        profiles = params.get('trajectory_profiles', {})
        resolved = {}
        for name in obstacle_names:
            if name not in params:
                raise ValueError(f"Missing obstacle configuration for '{name}'")
            resolved[name] = self._resolve_obstacle_config(name, params[name], profiles)
        return resolved

    def _load_legacy_obstacle_configs(self) -> dict:
        self.declare_parameter('obstacle_names', ['cylinder_obs', 'elliptic_obs'])
        names = self.get_parameter('obstacle_names').value
        resolved = {}

        for name in names:
            def _p(key, default, _n=name):
                self.declare_parameter(f'{_n}.{key}', default)
                return self.get_parameter(f'{_n}.{key}').value

            resolved[name] = {
                'trajectory': _p('trajectory', 'linear'),
                'linear_vel': _p('linear_vel', 0.5),
                'angular_vel': _p('angular_vel', 0.0),
                'turn_distance': _p('turn_distance', 3.0),
            }

        return resolved

    def _resolve_obstacle_config(self, name: str, obstacle_cfg: dict, profiles: dict) -> dict:
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

    def _build_motion_state(self, name: str, cfg: dict) -> dict:
        trajectory = str(cfg.get('trajectory', 'linear'))
        state = {
            'trajectory': trajectory,
            'linear_vel': float(cfg.get('linear_vel', 0.5)),
            'angular_vel': float(cfg.get('angular_vel', 0.0)),
            'turn_distance': float(cfg.get('turn_distance', 3.0)),
            'world_heading': float(cfg.get('world_heading', cfg.get('init_yaw', 0.0))),
            'init_yaw': float(cfg.get('init_yaw', 0.0)),
            'dist': 0.0,
            'direction': 1.0,
            'segments': [],
            'segment_index': 0,
            'segment_elapsed': 0.0,
            'loop': bool(cfg.get('loop', True)),
            'finished': False,
        }

        if trajectory == 'sequence':
            raw_segments = cfg.get('segments', [])
            if not raw_segments:
                raise ValueError(
                    f"Obstacle '{name}' uses trajectory 'sequence' without segments"
                )
            state['segments'] = self._parse_segments(name, raw_segments)

        return state

    def _parse_segments(self, name: str, raw_segments: list) -> list:
        segments = []
        for index, segment in enumerate(raw_segments):
            duration = float(segment.get('duration', 0.0))
            if duration <= 0.0:
                raise ValueError(
                    f"Obstacle '{name}' has non-positive duration in segment {index}"
                )
            segments.append({
                'linear_vel': float(segment.get('linear_vel', 0.0)),
                'angular_vel': float(segment.get('angular_vel', 0.0)),
                'duration': duration,
            })
        return segments

    def _control_loop(self) -> None:
        if self._start_time is None:
            self._start_time = self.get_clock().now()
            return
        for name, obs in self._obstacles.items():
            msg = Twist()
            v = obs['linear_vel']
            w = obs['angular_vel']

            traj = obs['trajectory']
            if traj == 'linear':
                obs['dist'] += v * self._dt
                if obs['dist'] >= obs['turn_distance']:
                    obs['direction'] *= -1.0
                    obs['dist'] = 0.0
                msg.linear.x = v * obs['direction']

            elif traj == 'circular':
                msg.linear.x = v
                msg.angular.z = w

            elif traj == 'sequence':
                if not obs['finished']:
                    segment = obs['segments'][obs['segment_index']]
                    msg.linear.x = segment['linear_vel']
                    msg.angular.z = segment['angular_vel']
                    obs['segment_elapsed'] += self._dt

                    if obs['segment_elapsed'] >= segment['duration']:
                        obs['segment_elapsed'] = 0.0
                        next_index = obs['segment_index'] + 1
                        if next_index >= len(obs['segments']):
                            if obs['loop']:
                                next_index = 0
                            else:
                                next_index = len(obs['segments']) - 1
                                obs['finished'] = True
                        obs['segment_index'] = next_index

            elif traj == 'straight_spin':
                elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
                yaw = obs['init_yaw'] + w * elapsed
                heading_error = obs['world_heading'] - yaw
                msg.linear.x = v * math.cos(heading_error)
                msg.linear.y = v * math.sin(heading_error)
                msg.angular.z = w

            # 'static': zero Twist (default)
            self._pubs[name].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(ObstacleController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
