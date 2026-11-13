"""ROS NumPy preflight: run before any motor-enable operation."""

from types import SimpleNamespace
import subprocess
import unittest
from unittest.mock import patch

import one_command_launch as launcher


class RosNumpyPreflightTests(unittest.TestCase):
    def test_success_prints_numpy_version_and_path(self):
        completed = SimpleNamespace(
            returncode=0, stdout='1.26.4 /usr/lib/python3/dist-packages/numpy/__init__.py\n',
            stderr='',
        )
        with patch.object(launcher.subprocess, 'run', return_value=completed) as run:
            launcher.verify_ros_numpy_environment({'PYTHONNOUSERSITE': '1'}, '/usr/bin/python3')
        cmd = run.call_args.args[0]
        self.assertEqual(cmd[:2], ['/usr/bin/python3', '-c'])
        self.assertIn('import numpy, rclpy', cmd[2])
        self.assertEqual(run.call_args.kwargs['env']['PYTHONNOUSERSITE'], '1')

    def test_incompatible_numpy_stops_startup(self):
        completed = SimpleNamespace(returncode=1, stdout='', stderr='numpy ABI mismatch')
        with patch.object(launcher.subprocess, 'run', return_value=completed):
            with self.assertRaisesRegex(launcher.LaunchError, 'numpy ABI mismatch'):
                launcher.verify_ros_numpy_environment({'PYTHONNOUSERSITE': '1'}, '/usr/bin/python3')

    def test_probe_timeout_stops_startup(self):
        with patch.object(launcher.subprocess, 'run',
                          side_effect=subprocess.TimeoutExpired('python3', 10)):
            with self.assertRaisesRegex(launcher.LaunchError, 'preflight could not run'):
                launcher.verify_ros_numpy_environment({}, '/usr/bin/python3')


if __name__ == '__main__':
    unittest.main()
