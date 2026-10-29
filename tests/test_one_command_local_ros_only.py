"""Regression tests for the local ROS-only one-command startup mode."""

import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree as ET

import one_command_launch as launcher


class LocalRosOnlyTests(unittest.TestCase):
    def dry_run(self, flags):
        output = io.StringIO()
        with patch.object(sys, 'argv', ['one_command_launch.py', '--dry-run', *flags]), \
             contextlib.redirect_stdout(output):
            result = launcher.main()
        self.assertEqual(result, 0)
        return output.getvalue()

    def test_offline_dry_run_disables_every_network_service(self):
        output = self.dry_run(['--local-ros-only'])
        self.assertIn('LOCAL ROS ONLY: loopback DDS', output)
        for external in ('IMU WebSocket', 'Windows WebSocket bridge',
                         'Calibration manager', 'HTML dashboard', 'Orbbec camera'):
            self.assertNotIn(external, output)
        self.assertIn('CANopen controller', output)
        self.assertIn('RPY converter', output)

    def test_normal_dry_run_unchanged(self):
        output = self.dry_run([])
        self.assertIn('IMU WebSocket', output)
        self.assertIn('Windows WebSocket bridge', output)
        self.assertIn('Calibration manager', output)
        self.assertIn('HTML dashboard', output)

    def test_reject_remote_web_drive(self):
        with patch.object(sys, 'argv', ['one_command_launch.py',
                                      '--local-ros-only', '--web-drive']):
            with self.assertRaises(SystemExit) as raised, \
                 contextlib.redirect_stderr(io.StringIO()):
                launcher.main()
        self.assertEqual(raised.exception.code, 2)

    def test_reject_unmanaged_imu_driver(self):
        with patch.object(sys, 'argv', ['one_command_launch.py',
                                      '--local-ros-only', '--external-imu-driver']):
            with self.assertRaises(SystemExit) as raised, \
                 contextlib.redirect_stderr(io.StringIO()):
                launcher.main()
        self.assertEqual(raised.exception.code, 2)

    def test_default_rmw_fastdds_is_loopback(self):
        env = {'ROS_STATIC_PEERS': '192.168.0.15',
               'ROS_DISCOVERY_SERVER': '192.168.0.20:11811',
               'ROS_LOCALHOST_ONLY': '0',
               'CYCLONEDDS_URI': 'old'}
        with contextlib.redirect_stdout(io.StringIO()):
            launcher.configure_local_ros_network(env)
        self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp')
        self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        self.assertEqual(env['ROS_STATIC_PEERS'], '')
        self.assertNotIn('ROS_DISCOVERY_SERVER', env)
        self.assertNotIn('ROS_LOCALHOST_ONLY', env)
        self.assertNotIn('CYCLONEDDS_URI', env)
        self.assertEqual(env['FASTDDS_DEFAULT_PROFILES_FILE'],
                         str(launcher.FAST_DDS_LOOPBACK_PROFILE))

    def test_selected_cyclone_rmw_is_loopback(self):
        env = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
               'FASTDDS_DEFAULT_PROFILES_FILE': '/tmp/unwanted.xml'}
        with contextlib.redirect_stdout(io.StringIO()):
            launcher.configure_local_ros_network(env)
        self.assertEqual(env['CYCLONEDDS_URI'],
                         launcher.CYCLONE_DDS_LOOPBACK_PROFILE.as_uri())
        self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        self.assertNotIn('FASTDDS_DEFAULT_PROFILES_FILE', env)

    def test_unknown_middleware_fails_closed(self):
        env = {'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp'}
        with self.assertRaises(launcher.LaunchError):
            launcher.configure_local_ros_network(env)

    def test_loopback_profiles_only_advertise_local_interfaces(self):
        ns = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        fast = ET.parse(launcher.FAST_DDS_LOOPBACK_PROFILE).getroot()
        addresses = [v.text for v in fast.findall(
            './/f:interfaceWhiteList/f:address', ns)]
        self.assertEqual(addresses, ['127.0.0.1'])
        initial_peers = [v.text for v in fast.findall(
            './/f:initialPeersList//f:address', ns)]
        self.assertEqual(initial_peers, ['127.0.0.1'])
        self.assertEqual(fast.find('.//f:useBuiltinTransports', ns).text, 'false')

        cyclone = ET.parse(launcher.CYCLONE_DDS_LOOPBACK_PROFILE).getroot()
        interfaces = cyclone.findall(
            './/{https://cdds.io/config}NetworkInterface')
        self.assertEqual([i.attrib.get('name') for i in interfaces], ['lo'])

    def test_runtime_starts_only_local_ros_children(self):
        started, checked_ports, envs = [], [], []

        class FakeSupervisor:
            def __init__(self, env, *_args, **_kwargs):
                envs.append(dict(env))

            def request_stop(self, *_args, **_kwargs):
                pass

            def start(self, name, _argv, **_kwargs):
                started.append(name)
                return name

            def wait_topic(self, *_args, **_kwargs):
                pass

            def wait_log(self, *_args, **_kwargs):
                pass

            def check(self):
                raise KeyboardInterrupt

            def shutdown(self):
                pass

        def fake_port_open(host, port):
            checked_ports.append((host, port))
            return False

        argv = ['one_command_launch.py', '--local-ros-only', '--log-dir',
                '/virtual/amr_one_command_test_logs']
        output = io.StringIO()
        with patch.object(sys, 'argv', argv), \
             patch.object(launcher.Path, 'mkdir'), \
             patch.object(launcher.Path, 'write_text'), \
             patch.object(launcher, 'validate_inputs'), \
             patch.object(launcher, 'source_ros_environment',
                          return_value={'PATH': '/usr/bin'}), \
             patch.object(launcher.shutil, 'which', return_value='/usr/bin/python3'), \
             patch.object(launcher, 'choose_camera_python',
                          return_value='/usr/bin/python3'), \
             patch.object(launcher, 'ensure_motors_enabled'), \
             patch.object(launcher, 'port_open', side_effect=fake_port_open), \
             patch.object(launcher, 'Supervisor', FakeSupervisor), \
             patch.object(launcher.signal, 'signal'), \
             contextlib.redirect_stdout(output):
            with self.assertRaises(KeyboardInterrupt):
                launcher.main()
        self.assertEqual(started, ['imu_driver', 'imu_rpy', 'can'])
        self.assertEqual(checked_ports, [])
        self.assertEqual(envs[0]['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        self.assertIn('Local ROS only', output.getvalue())


if __name__ == '__main__':
    unittest.main()
