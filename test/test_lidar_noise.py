import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('lidar_noise', ROOT / 'scripts/lidar_noise.py')
noise = importlib.util.module_from_spec(spec)
spec.loader.exec_module(noise)
rsp_spec = importlib.util.spec_from_file_location('rsp_launch', ROOT / 'launch/rsp.launch.py')
rsp = importlib.util.module_from_spec(rsp_spec)
rsp_spec.loader.exec_module(rsp)


class LidarNoiseTests(unittest.TestCase):
    def test_noise_flag_selects_dense_raw_scan(self):
        self.assertEqual(rsp._lidar_samples('ideal', 'false'), '360')
        self.assertEqual(rsp._lidar_samples('ideal', 'true'), '3601')
        self.assertEqual(rsp._lidar_samples('mid360_2d', 'false'), '3601')
        with self.assertRaises(ValueError):
            rsp._lidar_samples('ideal', 'sometimes')

        config = yaml.safe_load((ROOT / 'config/livox_mid360_noise.yaml').read_text())
        params = config['lidar_noise']['ros__parameters']
        self.assertEqual(params['input_topic'], '/scan_raw')
        self.assertEqual(params['output_topic'], '/scan')
        self.assertEqual(params['range_sigma_near'], .005)
        self.assertEqual(params['range_sigma_far'], .02)
        self.assertLess(params['range_sigma_near'], params['range_sigma_far'])

    def test_zero_noise_preserves_geometry_and_missing_returns(self):
        raw = np.linspace(1.0, 5.0, 3601)
        raw[100:500] = np.inf
        ref, measured, _, _ = noise.corrupt_scan(
            raw, .1, 12., np.random.default_rng(1), sigma_near=0., sigma_far=0., angular_sigma_deg=0.)
        # The two endpoints are the same bearing on a real full-circle scan.
        np.testing.assert_array_equal(ref[:-1], measured[:-1])
        self.assertTrue(np.isinf(measured).any())

    def test_measured_noise_matches_specification_assumptions(self):
        rng = np.random.default_rng(360)
        errors, angles = [], []
        for _ in range(400):
            _, measured, e, a = noise.corrupt_scan(np.full(3601, 2.0), .1, 12., rng)
            self.assertEqual(len(measured), 360)
            errors.extend(e)
            angles.extend(a)
        expected_sigma = .005 + (.02-.005)*(2.0-.2)/(10.-.2)
        self.assertLess(abs(np.mean(errors)), .0005)
        self.assertAlmostEqual(np.std(errors), expected_sigma, delta=.0005)
        self.assertAlmostEqual(np.std(angles), math.radians(.15), delta=.00003)

    def test_seed_and_angular_jitter_at_an_edge(self):
        raw = np.full(3601, 4.)
        raw[:1801] = 2.
        def run(seed, angular):
            return noise.corrupt_scan(raw, .1, 12., np.random.default_rng(seed),
                                      output_samples=3601, sigma_near=0., sigma_far=0., angular_sigma_deg=angular)[1]
        np.testing.assert_array_equal(run(360,.15), run(360,.15))
        self.assertTrue(np.any(run(360,.15) != run(360,0.)))
        self.assertTrue(set(run(360,.15)).issubset({2.,4.}))

    def test_no_returns_do_not_create_obstacles(self):
        _, measured, residual, _ = noise.corrupt_scan(
            np.full(3601,np.inf), .1, 12., np.random.default_rng(360))
        self.assertTrue(np.isinf(measured).all())
        self.assertEqual(len(residual), 0)

    def test_kobuki_dense_sensor_and_ideal_default(self):
        path = str(ROOT / 'description/kobuki.urdf.xacro')
        for mappings, expected in [({},360), ({'lidar_samples':'3601'},3601)]:
            xml = ET.fromstring(xacro.process_file(path,mappings=mappings).toxml())
            sensor = xml.find(".//sensor[@name='laser']")
            self.assertEqual(int(sensor.findtext('lidar/scan/horizontal/samples')),expected)
            self.assertEqual(float(sensor.findtext('update_rate')),10.)
            self.assertAlmostEqual(float(sensor.findtext('lidar/scan/horizontal/max_angle')),math.pi)

    def test_update_rate_can_be_changed_for_both_robots(self):
        for filename in ('robot.urdf.xacro', 'kobuki.urdf.xacro'):
            xml = ET.fromstring(xacro.process_file(
                str(ROOT / 'description' / filename),
                mappings={'lidar_update_rate': '30'}).toxml())
            self.assertEqual(float(xml.findtext(".//sensor[@name='laser']/update_rate")),30.)


if __name__ == '__main__':
    unittest.main()
