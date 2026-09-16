#!/usr/bin/env python3
"""Approximate Mid-360 measurement errors in a horizontal LaserScan.

Dense ideal rays (0.1 deg) let angular errors sample actual scene geometry
without interpolating across obstacle edges. Output is 360 nominal bearings.
The nominal unperturbed scan is published alongside for matched comparisons.
See config/livox_mid360_noise.yaml for sources and modeling assumptions.
"""
import copy
import csv
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def corrupt_scan(ranges, range_min, range_max, rng, output_samples=360,
                 sigma_near=0.005, sigma_far=0.02, angular_sigma_deg=0.15):
    """Input spans [-pi, pi] including duplicate endpoint, returns paired scans.

    Invalid/no-return rays remain no-return. Values pushed outside the sensing
    interval become infinity, rather than piling up at a clipped range limit.
    """
    raw = np.asarray(ranges, dtype=float)
    if len(raw) < 2 or output_samples < 2:
        raise ValueError('At least two input/output rays required')
    period = len(raw) - 1
    nominal = np.linspace(0, period, output_samples)
    angular_error = rng.normal(0.0, math.radians(angular_sigma_deg), output_samples)
    indices = np.rint((nominal + angular_error * period / (2*math.pi)) % period).astype(int) % period
    reference = raw[np.rint(nominal).astype(int)].copy()
    sampled = raw[indices].copy()
    for values in (reference, sampled):
        values[~np.isfinite(values) | (values < range_min) | (values > range_max)] = np.inf
    finite = np.isfinite(sampled)
    sigma = np.interp(sampled[finite], [0.2, 10.0], [sigma_near, sigma_far])
    residual = rng.normal(0.0, sigma)
    noisy = sampled.copy()
    noisy[finite] += residual
    noisy[(noisy < range_min) | (noisy > range_max)] = np.inf
    return reference, noisy, residual, angular_error


class LidarNoise(Node):
    def __init__(self):
        super().__init__('lidar_noise')
        defaults = dict(input_topic='/scan_raw', output_topic='/scan',
                        reference_topic='/scan_reference', output_samples=360,
                        range_sigma_near=0.005, range_sigma_far=0.02,
                        angular_sigma_deg=0.15, seed=360, audit_path='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        if len({p['input_topic'], p['output_topic'], p['reference_topic']}) != 3:
            raise ValueError('Input, noisy output and reference topics must be distinct')
        if p['output_samples'] < 2 or any(p[k] < 0 for k in (
                'range_sigma_near', 'range_sigma_far', 'angular_sigma_deg')):
            raise ValueError('Invalid sample count or negative noise standard deviation')
        self.p = p
        self.rng = np.random.default_rng(p['seed'])
        # Reliable output also serves the controller's default reliable subscribers.
        self.noisy_pub = self.create_publisher(LaserScan, p['output_topic'], 10)
        self.reference_pub = self.create_publisher(LaserScan, p['reference_topic'], 10)
        self.subscription = self.create_subscription(
            LaserScan, p['input_topic'], self.on_scan, qos_profile_sensor_data)
        self.audit_file = None
        if p['audit_path']:
            self.audit_file = Path(p['audit_path']).open('w', newline='')
            self.writer = csv.writer(self.audit_file)
            self.writer.writerow(['stamp_sim_s','clean_min_m','noisy_min_m',
                                  'range_noise_n','range_noise_sum','range_noise_sum_sq',
                                  'angle_noise_n','angle_noise_sum','angle_noise_sum_sq'])
        self.received = 0
        self.get_logger().info(
            f"Mid-360 2D approximation: {p['input_topic']} -> {p['output_topic']}; "
            f"seed={p['seed']}")

    def on_scan(self, scan):
        if len(scan.ranges) < 3600 or not math.isclose(
                scan.angle_max-scan.angle_min, 2*math.pi, abs_tol=1e-4):
            self.get_logger().error('Noise profile requires a dense full-circle raw scan', throttle_duration_sec=5)
            return
        ref, noisy, residual, angle = corrupt_scan(
            scan.ranges, scan.range_min, scan.range_max, self.rng,
            self.p['output_samples'], self.p['range_sigma_near'],
            self.p['range_sigma_far'], self.p['angular_sigma_deg'])
        msg = copy.deepcopy(scan)
        msg.angle_increment = (scan.angle_max-scan.angle_min)/(self.p['output_samples']-1)
        msg.time_increment = 0.0  # all raw GPU rays share one simulation timestamp
        msg.intensities = []
        msg.ranges = ref.tolist()
        self.reference_pub.publish(msg)
        msg.ranges = noisy.tolist()
        self.noisy_pub.publish(msg)
        if self.audit_file:
            self.writer.writerow([scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9,
                                  np.min(ref), np.min(noisy), len(residual),
                                  residual.sum(), np.square(residual).sum(),
                                  len(angle), angle.sum(), np.square(angle).sum()])
            if self.received % 10 == 0:
                self.audit_file.flush()
        self.received += 1
        if self.received == 1:
            self.get_logger().info('First noisy scan published: 360 rays with original simulation timestamp')

    def destroy_node(self):
        if self.audit_file:
            self.audit_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
