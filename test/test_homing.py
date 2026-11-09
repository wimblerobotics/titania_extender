#!/usr/bin/env python3
# Copyright 2026 titania_extender contributors
# SPDX-License-Identifier: Apache-2.0
"""Exercise the actual node with a serial emulator and test-only sensors."""

import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Trigger
from titania_extender.action import MoveToPosition

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from roboclaw_emulator import Emulator, WRITE_LENGTHS  # noqa: E402, I100

NODE = sys.argv.pop(1)
os.environ['ROS_DOMAIN_ID'] = str(20 + os.getpid() % 60)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'


class TestEmulator(Emulator):

    def __init__(self, link):
        super().__init__(link)
        self.offsets = [0.0, 0.0]
        self.events = []
        self.current = (0.42, 0.17)
        self.fault = 0
        self.lock = threading.Lock()
        self.running = True

    def handle_write(self, command):
        with self.lock:
            self.write_locked(command)

    def write_locked(self, command):
        physical = None
        if command in (22, 23):
            axis = command - 22
            self.motors[axis].update()
            physical = self.motors[axis].position + self.offsets[axis]
        super().handle_write(command)
        if physical is not None:
            self.offsets[axis] = physical - self.motors[axis].position
            self.events.append((command, physical))
        elif command in (35, 36):
            self.events.append((command, self.motors[command - 35].speed))

    def handle_read(self, command):
        import struct
        if command == 49:
            self.reply_read(command, struct.pack(
                '>hh', *(int(value * 100) for value in self.current)))
        elif command == 90:
            self.reply_read(command, struct.pack('>I', self.fault))
        else:
            super().handle_read(command)

    def run(self):
        while self.running:
            if not select.select([self.master], [], [], 0.1)[0]:
                continue
            if self.read_exact(1)[0] != 0x80:
                continue
            command = self.read_exact(1)[0]
            if command in WRITE_LENGTHS:
                self.handle_write(command)
            else:
                self.handle_read(command)


class HomingIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.emulator = TestEmulator(str(self.path / 'serial'))
        self.quality = 0
        self.imu_valid = True
        self.freeze_extension = False
        self.initial_length = 0.15
        self.initial_angle = 25.0
        self.length_noise = 0.0
        self.angle_noise = 0.0
        self.counts_per_meter = 10000
        self.keep_running = True
        self.write_sensors()
        self.sensor_thread = threading.Thread(target=self.sensor_loop)
        self.sensor_thread.start()
        self.serial_thread = threading.Thread(
            target=self.emulator.run, daemon=True)
        self.serial_thread.start()
        env = dict(os.environ)
        env['TITANIA_EXTENDER_TEST_SENSOR_STATE'] = str(self.path / 'sensors')
        self.log = open(self.path / 'node.log', 'w+')
        self.process = subprocess.Popen([
            NODE, '--ros-args', '--params-file',
            str(ROOT / 'config' / 'titania_extender.yaml'),
            '-p', f'device_name:={self.path / "serial"}',
            '-p', 'homing.timeout_seconds:=10.0',
        ], env=env, stdout=self.log, stderr=subprocess.STDOUT)
        self.node = rclpy.create_node('homing_integration')
        self.home = self.node.create_client(Trigger, '/home')
        self.cancel = self.node.create_client(Trigger, '/cancel_home')
        self.reset = self.node.create_client(Trigger, '/reset_encoders')
        self.addCleanup(self.cleanup_node)
        self.assertTrue(self.home.wait_for_service(timeout_sec=10))
        self.action = ActionClient(
            self.node, MoveToPosition, '/rotate/move_to_position')
        self.assertTrue(self.action.wait_for_server(timeout_sec=5))
        self.spin_for(0.5)

    def cleanup_node(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.keep_running = False
        self.sensor_thread.join(timeout=2)
        self.emulator.running = False
        self.serial_thread.join(timeout=2)
        os.close(self.emulator.master)
        os.close(self.emulator.slave)
        self.node.destroy_node()
        self.log.close()

    def physical_counts(self, axis):
        return self.emulator.motors[axis].position + self.emulator.offsets[axis]

    def write_sensors(self):
        with self.emulator.lock:
            length = self.initial_length
            if not self.freeze_extension:
                length += self.physical_counts(1) / self.counts_per_meter
            angle = self.initial_angle - self.physical_counts(0) / 20
        temporary = self.path / 'next_sensors'
        temporary.write_text(f'{length} {angle} {self.quality} '
                             f'{int(self.imu_valid)} {self.length_noise} '
                             f'{self.angle_noise}\n')
        temporary.replace(self.path / 'sensors')

    def sensor_loop(self):
        while self.keep_running:
            self.write_sensors()
            time.sleep(0.02)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def result(self, future, timeout=15):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            self.log.flush()
            self.log.seek(0)
            self.fail(f'service timed out:\n{self.log.read()}')
        return future.result()

    def assert_stopped(self):
        self.assertTrue(all(not motor.moving for motor in self.emulator.motors))

    def set_parameter(self, name, value):
        client = AsyncParameterClient(self.node, '/titania_extender_node')
        self.assertTrue(client.wait_for_services(timeout_sec=5))
        response = self.result(client.set_parameters([Parameter(name, value=value)]))
        self.assertTrue(response.results[0].successful)

    def test_complete_homing(self):
        readings = {}
        subscriptions = [
            self.node.create_subscription(
                Float32, f'/{topic}',
                lambda message, name=topic: readings.update({name: message.data}), 10)
            for topic in ('extension_length', 'rotation_angle')]
        response = self.result(self.home.call_async(Trigger.Request()))
        self.assertTrue(response.success, response.message)
        speeds = {cmd: max(abs(speed) for event, speed in self.emulator.events
                           if event == cmd) for cmd in (35, 36)}
        self.assertEqual(speeds, {35: 300, 36: 600})
        for cmd in (35, 36):
            self.assertTrue(any(event == cmd and 0 < abs(speed) < speeds[cmd]
                                for event, speed in self.emulator.events))
        resets = [(cmd, position) for cmd, position in self.emulator.events
                  if cmd in (22, 23)]
        self.assertEqual([cmd for cmd, _ in resets], [23, 22])
        self.assertAlmostEqual(0.15 + resets[0][1] / 10000, 0.1, delta=0.005)
        self.assertAlmostEqual(25 - resets[1][1] / 20, 0, delta=1)
        self.assertEqual([int(m.position) for m in self.emulator.motors], [0, 0])
        self.assert_stopped()
        self.spin_for(0.3)
        self.assertAlmostEqual(readings['extension_length'], 0.1, delta=0.005)
        self.assertAlmostEqual(readings['rotation_angle'], 0, delta=0.01)
        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
        # A repeat home must preserve physical references after encoder resets.
        response = self.result(self.home.call_async(Trigger.Request()))
        self.assertTrue(response.success, response.message)
        goal = MoveToPosition.Goal(position=100, speed=200)
        handle = self.result(self.action.send_goal_async(goal))
        self.assertTrue(handle.accepted)
        result = self.result(handle.get_result_async())
        self.assertTrue(result.result.success, result.result.message)

    def test_homing_from_full_extension(self):
        # A 13000-count stroke must exceed the old 10000-count homing cap.
        with self.emulator.lock:
            self.initial_length = 0.75
            self.counts_per_meter = 20000
            self.length_noise = 0.004
            self.angle_noise = 1.2
        self.set_parameter('homing.timeout_seconds', 30.0)
        self.spin_for(0.3)
        response = self.result(self.home.call_async(Trigger.Request()), timeout=40)
        self.assertTrue(response.success, response.message)
        extension_resets = [position for cmd, position in self.emulator.events
                            if cmd == 23]
        self.assertEqual(len(extension_resets), 1)
        self.assertGreater(abs(extension_resets[0]), 10000)
        self.assertAlmostEqual(
            self.initial_length + extension_resets[0] / self.counts_per_meter,
            0.1, delta=0.005)
        self.assertEqual([int(m.position) for m in self.emulator.motors], [0, 0])
        self.assert_stopped()

    def test_noisy_sensors_near_home_do_not_hunt(self):
        with self.emulator.lock:
            self.initial_length = 0.103
            self.initial_angle = 0.6
            self.length_noise = 0.004
            self.angle_noise = 1.2
        self.spin_for(1.5)
        response = self.result(self.home.call_async(Trigger.Request()), timeout=5)
        self.assertTrue(response.success, response.message)
        self.assertFalse(any(cmd in (35, 36) for cmd, _ in self.emulator.events),
                         self.emulator.events)
        self.assertEqual([cmd for cmd, _ in self.emulator.events], [23, 22])
        self.assert_stopped()

    def test_noisy_sensors_converge_without_excessive_reversals(self):
        self.length_noise = 0.004
        self.angle_noise = 1.2
        response = self.result(self.home.call_async(Trigger.Request()), timeout=12)
        self.assertTrue(response.success, response.message)
        self.assertAlmostEqual(
            self.initial_length + self.physical_counts(1) / self.counts_per_meter,
            0.1, delta=0.005)
        self.assertAlmostEqual(
            self.initial_angle - self.physical_counts(0) / 20, 0, delta=1.0)
        for cmd in (35, 36):
            directions = [speed > 0 for event, speed in self.emulator.events
                          if event == cmd]
            reversals = sum(a != b for a, b in zip(directions, directions[1:]))
            self.assertLessEqual(reversals, 2)
        self.assert_stopped()

    def test_filter_history_resets_after_invalid_or_stale_samples(self):
        self.quality = 1
        self.imu_valid = False
        self.spin_for(0.7)
        with self.emulator.lock:
            self.initial_length = 0.1
            self.initial_angle = 0.0
            self.quality = 0
            self.imu_valid = True
        self.spin_for(0.3)
        response = self.result(self.home.call_async(Trigger.Request()), timeout=5)
        self.assertTrue(response.success, response.message)
        self.assertFalse(any(cmd in (35, 36) for cmd, _ in self.emulator.events))
        self.assert_stopped()

    def test_invalid_smoothing_prevents_movement(self):
        self.set_parameter('homing.smoothing_alpha', 0.0)
        response = self.result(self.home.call_async(Trigger.Request()))
        self.assertFalse(response.success)
        self.assertIn('smoothing alpha', response.message)
        self.assertFalse(self.emulator.events)
        self.assert_stopped()

    def test_cancellation_and_command_exclusion(self):
        future = self.home.call_async(Trigger.Request())
        self.spin_for(0.5)
        self.assertFalse(self.result(self.reset.call_async(Trigger.Request())).success)
        handle = self.result(self.action.send_goal_async(MoveToPosition.Goal(position=100)))
        self.assertFalse(handle.accepted)
        pub = self.node.create_publisher(Int32, '/rotate/cmd_position', 10)
        self.spin_for(0.3)
        pub.publish(Int32(data=2000))
        self.spin_for(0.2)
        self.assertEqual(self.emulator.motors[0].target, 0)
        self.assertTrue(self.result(self.cancel.call_async(Trigger.Request())).success)
        response = self.result(future)
        self.assertFalse(response.success)
        self.assertIn('canceled', response.message)
        self.assert_stopped()
        self.assertFalse(any(cmd in (22, 23) for cmd, _ in self.emulator.events))

    def test_invalid_tof_stops_motion(self):
        future = self.home.call_async(Trigger.Request())
        self.spin_for(0.5)
        self.quality = 1
        response = self.result(future)
        self.assertFalse(response.success)
        self.assertIn('extension', response.message)
        self.assert_stopped()

    def test_active_action_prevents_homing(self):
        goal = MoveToPosition.Goal(position=1000, speed=100)
        handle = self.result(self.action.send_goal_async(goal))
        self.assertTrue(handle.accepted)
        response = self.result(self.home.call_async(Trigger.Request()))
        self.assertFalse(response.success)
        self.assertIn('active move', response.message)
        self.result(handle.cancel_goal_async())
        self.result(handle.get_result_async())
        self.assert_stopped()

    def test_stale_imu_stops_motion(self):
        future = self.home.call_async(Trigger.Request())
        deadline = time.monotonic() + 5
        while not any(cmd == 35 for cmd, _ in self.emulator.events):
            self.assertLess(time.monotonic(), deadline,
                            'rotation homing did not start')
            self.assertFalse(future.done(), 'homing finished before sensor failure')
            self.spin_for(0.02)
        self.imu_valid = False
        response = self.result(future)
        self.assertFalse(response.success)
        self.assertIn('gravity', response.message)
        self.assert_stopped()

    def test_controller_fault_stops_motion(self):
        future = self.home.call_async(Trigger.Request())
        self.spin_for(0.5)
        self.emulator.fault = 1
        response = self.result(future)
        self.assertFalse(response.success)
        self.assertIn('fault', response.message)
        self.assert_stopped()

    def test_overcurrent_stops_motion(self):
        future = self.home.call_async(Trigger.Request())
        self.spin_for(0.5)
        self.emulator.current = (0.42, 7.0)
        response = self.result(future)
        self.assertFalse(response.success)
        self.assertIn('over-current', response.message)
        self.assert_stopped()

    def test_timeout_without_sensor_progress(self):
        self.freeze_extension = True
        self.set_parameter('homing.timeout_seconds', 0.5)
        response = self.result(self.home.call_async(Trigger.Request()))
        self.assertFalse(response.success)
        self.assertIn('timed out', response.message)
        self.assert_stopped()

    def test_total_travel_cap(self):
        self.set_parameter('homing.extend_max_travel_counts', 40)
        response = self.result(self.home.call_async(Trigger.Request()))
        self.assertFalse(response.success)
        self.assertIn('travel limit', response.message)
        self.assert_stopped()

    def test_invalid_sensor_prevents_movement(self):
        self.quality = 1
        self.spin_for(0.3)
        response = self.result(self.home.call_async(Trigger.Request()))
        self.assertFalse(response.success)
        self.assertIn('extension', response.message)
        self.assertFalse(self.emulator.events)
        self.assert_stopped()


if __name__ == '__main__':
    unittest.main()
