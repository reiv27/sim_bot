"""Run with ROS 2 sourced: python3 -m unittest discover -s src/sim_bot/test."""
import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


launch = load_module('obstacles_launch', ROOT / 'launch/obstacles.launch.py')
controller = load_module('obstacle_controller', ROOT / 'scripts/obstacle_controller.py')


class RigidFormationTests(unittest.TestCase):
    def setUp(self):
        self.path = ROOT / 'config/obstacles_rigid_formation.yaml'
        self.params = yaml.safe_load(self.path.read_text())['obstacle_controller']['ros__parameters']
        self.members = {n: self.params[n] for n in self.params['obstacle_names']}
        self.model = ET.fromstring(
            launch.generate_rigid_formation_sdf('rigid_group', self.members)).find('model')

    def test_one_body_with_separate_collision_shapes_and_one_drive(self):
        self.assertEqual(len(self.model.findall('link')), 1)
        self.assertEqual(len(self.model.findall('plugin')), 1)
        link = self.model.find('link')
        self.assertEqual(len(link.findall('collision')), 7)
        self.assertEqual(len(link.findall('visual')), 3)
        self.assertEqual(float(link.findtext('inertial/mass')), 45.0)
        for name, cfg in self.members.items():
            visual = link.find(f"visual[@name='{name}_visual']")
            pose = list(map(float, visual.findtext('pose').split()))
            self.assertAlmostEqual(pose[0], cfg['offset_x'])
            self.assertAlmostEqual(pose[1], cfg['offset_y'])
            self.assertAlmostEqual(pose[5], cfg['offset_yaw'])
            # Polyline extrusion starts at z=0, cylinders are centred at z=0.
            expected_z = 0.01 if cfg['type'] == 'elliptic_cylinder' else 0.36
            self.assertAlmostEqual(pose[2], expected_z)

    def test_stadium_collision_primitives_follow_member_rotation(self):
        link = self.model.find('link')
        middle = link.find("collision[@name='blue_obs_collision_middle']")
        for actual, expected in zip(map(float,middle.findtext('geometry/box/size').split()), (1.4,1.2,.7)):
            self.assertAlmostEqual(actual,expected)
        caps = [link.find(f"collision[@name='blue_obs_collision_cap_{i}']") for i in (0,1)]
        for cap, expected_y in zip(caps, (-2.7, -1.3)):
            pose = list(map(float,cap.findtext('pose').split()))
            self.assertAlmostEqual(pose[0],0.)
            self.assertAlmostEqual(pose[1],expected_y)
            self.assertAlmostEqual(pose[2],.36)
            self.assertAlmostEqual(float(cap.findtext('geometry/cylinder/radius')),.6)
        self.assertFalse(link.findall('collision/geometry/polyline'))

    def test_parallel_axis_inertia(self):
        cfg = dict(type='cylinder', radius=1.0, height=2.0, mass=3.0)
        model = ET.fromstring(launch.generate_rigid_formation_sdf('pair', {
            'a': dict(cfg, offset_x=-2.0), 'b': dict(cfg, offset_x=2.0),
        }))
        inertial = model.find('model/link/inertial')
        self.assertAlmostEqual(float(inertial.findtext('inertia/izz')), 27.0)
        self.assertAlmostEqual(float(inertial.findtext('inertia/ixx')), 3.5)
        self.assertAlmostEqual(float(inertial.findtext('inertia/iyy')), 27.5)

    def test_distances_and_relative_angles_under_group_rotation(self):
        poses = [list(map(float, v.findtext('pose').split()))
                 for v in self.model.findall('link/visual')]
        for theta in (0.0, 0.4, 1.7, math.pi):
            c, s = math.cos(theta), math.sin(theta)
            world = [(5+c*p[0]-s*p[1], -3+s*p[0]+c*p[1], theta+p[5]) for p in poses]
            for i in range(len(poses)):
                for j in range(i):
                    self.assertAlmostEqual(math.dist(world[i][:2], world[j][:2]),
                                           math.dist(poses[i][:2], poses[j][:2]))
                    self.assertAlmostEqual(world[i][2]-world[j][2], poses[i][5]-poses[j][5])

    def test_controller_targets_only_group_and_preserves_legacy_config(self):
        node = controller.ObstacleController.__new__(controller.ObstacleController)
        configs = node._load_obstacle_configs_from_yaml(str(self.path))
        self.assertEqual(list(configs), ['rigid_group'])
        self.assertEqual(configs['rigid_group']['angular_vel'], 0.1)
        old = node._load_obstacle_configs_from_yaml(str(ROOT / 'config/obstacles.yaml'))
        self.assertIn('elliptic_obs', old)

    def test_reject_empty_group_and_ambiguous_world_positions(self):
        with self.assertRaises(ValueError):
            launch.generate_rigid_formation_sdf('empty', {})
        self.members['blue_obs']['init_x'] = 1.0
        with self.assertRaises(ValueError):
            launch.generate_rigid_formation_sdf('bad', self.members)

    def test_static_reference_layout(self):
        path = ROOT / 'config/obstacles_static_layout.yaml'
        params = yaml.safe_load(path.read_text())['obstacle_controller']['ros__parameters']
        expected = {
            'blue_obs': (14.0 / 3.0, 7.0 / 3.0, 0.0),
            'orange_obs': (20.0 / 3.0, -2.0 / 3.0, math.pi / 2),
            'green_obs': (11.0 / 3.0, -5.0 / 3.0, 3 * math.pi / 4),
        }
        self.assertEqual(params['obstacle_names'], list(expected))
        self.assertAlmostEqual(
            sum(params[name]['init_x'] for name in expected) / len(expected), 5.0)
        self.assertAlmostEqual(
            sum(params[name]['init_y'] for name in expected) / len(expected), 0.0)
        for name, pose in expected.items():
            obstacle = params[name]
            self.assertEqual(obstacle['trajectory'], 'static')
            self.assertAlmostEqual(obstacle['init_x'], pose[0])
            self.assertAlmostEqual(obstacle['init_y'], pose[1])
            self.assertAlmostEqual(obstacle['init_yaw'], pose[2])


if __name__ == '__main__':
    unittest.main()
