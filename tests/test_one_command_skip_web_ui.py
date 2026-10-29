"""Tests for optional HTML dashboard in the One Command launcher."""

import contextlib
import io
import sys
import unittest
from unittest.mock import patch

import one_command_launch as launcher


class SkipWebUITests(unittest.TestCase):
    def run_dry(self, flags):
        output = io.StringIO()
        with patch.object(sys, 'argv', ['one_command_launch.py', '--dry-run', *flags]):
            with contextlib.redirect_stdout(output):
                result = launcher.main()
        self.assertEqual(result, 0)
        return output.getvalue()

    def run_without_hardware(self, flags):
        started, probed, checked_ports = [], [], []

        class FakeSupervisor:
            def __init__(self, *_args, **_kwargs):
                pass

            def request_stop(self, *_args, **_kwargs):
                pass

            def start(self, name, _argv, **_kwargs):
                started.append(name)
                return name

            def wait_log(self, *_args, **_kwargs):
                pass

            def wait_http(self, name, url, *_args, **_kwargs):
                probed.append((name, url))

            def check(self):
                raise KeyboardInterrupt

            def shutdown(self):
                pass

        def fake_port_open(host, port):
            checked_ports.append((host, port))
            return False

        argv = ['one_command_launch.py', '--skip-imu', '--skip-camera',
                '--skip-bridge', '--log-dir', '/virtual/one_command_test_logs', *flags]
        output = io.StringIO()
        with patch.object(sys, 'argv', argv), \
             patch.object(launcher.Path, 'mkdir'), \
             patch.object(launcher.Path, 'write_text'), \
             patch.object(launcher, 'validate_inputs'), \
             patch.object(launcher, 'source_ros_environment',
                          return_value={'PATH': '/usr/bin'}), \
             patch.object(launcher.shutil, 'which', return_value='/usr/bin/python3'), \
             patch.object(launcher, 'choose_camera_python', return_value='/usr/bin/python3'), \
             patch.object(launcher, 'ensure_motors_enabled'), \
             patch.object(launcher, 'port_open', side_effect=fake_port_open), \
             patch.object(launcher, 'Supervisor', FakeSupervisor), \
             patch.object(launcher.signal, 'signal'), \
             contextlib.redirect_stdout(output):
            with self.assertRaises(KeyboardInterrupt):
                launcher.main()
        return started, probed, checked_ports, output.getvalue()

    def test_dry_run_hides_dashboard_when_skipped(self):
        output = self.run_dry(['--skip-web-ui'])
        self.assertNotIn('HTML dashboard -> :8000', output)

    def test_dry_run_shows_dashboard_by_default(self):
        output = self.run_dry([])
        self.assertIn('HTML dashboard -> :8000', output)

    def test_skip_web_ui_does_not_start_or_probe_8000(self):
        started, probed, ports, output = self.run_without_hardware(
            ['--skip-calibration', '--skip-web-ui'])
        self.assertEqual(started, ['can'])
        self.assertEqual(probed, [])
        self.assertEqual(ports, [])
        self.assertIn('Web UI disabled', output)

    def test_default_still_starts_dashboard(self):
        started, probed, ports, output = self.run_without_hardware(
            ['--skip-calibration'])
        self.assertEqual(started, ['can', 'web_ui'])
        self.assertEqual(probed, [('Web UI', 'http://127.0.0.1:8000/index.html')])
        self.assertIn(('127.0.0.1', 8000), ports)
        self.assertIn('http://192.168.0.35:8000', output)

    def test_skip_web_ui_does_not_disable_calibration(self):
        started, probed, ports, _ = self.run_without_hardware(['--skip-web-ui'])
        self.assertEqual(started, ['can', 'calibration'])
        self.assertEqual(probed, [('Calibration manager',
                                    'http://192.168.0.35:8877/status')])
        self.assertNotIn(('127.0.0.1', 8000), ports)


if __name__ == '__main__':
    unittest.main()
