"""Regression checks for camera-specific Orbbec/ROS native library isolation."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import subprocess

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('amr_one_command_camera_test', ROOT / 'one_command_launch.py')
launcher = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = launcher
spec.loader.exec_module(launcher)


class CameraSdkEnvironmentTests(unittest.TestCase):
    def test_prepend_sdk_and_keep_ros_paths_without_mutating_parent(self):
        base = {'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib:/other/lib', 'PYTHONNOUSERSITE': '1'}
        lib = '/usr/local/lib/python3.12/dist-packages/pyorbbecsdk/libOrbbecSDK.so.2'
        seen = []

        def run(argv, **kwargs):
            seen.append((argv, kwargs['env'].copy()))
            if len(seen) == 1:
                return subprocess.CompletedProcess(argv, 0, lib + '\n', '')
            return subprocess.CompletedProcess(argv, 0, '', '')

        with patch.object(launcher.subprocess, 'run', side_effect=run), \
             patch.object(launcher.Path, 'is_file', return_value=True):
            actual = launcher.camera_slam_environment(base, '/usr/bin/python3')
        self.assertEqual(base['LD_LIBRARY_PATH'], '/opt/ros/jazzy/lib:/other/lib')
        self.assertEqual(actual['LD_LIBRARY_PATH'],
                         '/usr/local/lib/python3.12/dist-packages/pyorbbecsdk:/opt/ros/jazzy/lib:/other/lib')
        self.assertEqual(seen[1][1], actual)
        self.assertIn('import pyorbbecsdk, rclpy, numpy', seen[1][0][-1])

    def test_missing_bundle_fails_closed(self):
        with patch.object(launcher.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, '', '')):
            with self.assertRaisesRegex(launcher.LaunchError, 'bundled libOrbbecSDK'):
                launcher.camera_slam_environment({'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib'}, '/usr/bin/python3')

    def test_incompatible_import_fails_before_camera_start(self):
        lib = '/usr/local/lib/python3.12/dist-packages/pyorbbecsdk/libOrbbecSDK.so.2'
        responses = [subprocess.CompletedProcess([], 0, lib + '\n', ''),
                     subprocess.CompletedProcess([], 1, '', 'undefined symbol')]
        with patch.object(launcher.subprocess, 'run', side_effect=responses), \
             patch.object(launcher.Path, 'is_file', return_value=True):
            with self.assertRaisesRegex(launcher.LaunchError, 'undefined symbol'):
                launcher.camera_slam_environment({'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib'}, '/usr/bin/python3')


if __name__ == '__main__':
    unittest.main()
