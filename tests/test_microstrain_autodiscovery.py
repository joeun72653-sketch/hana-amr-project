"""Tests for MicroStrain discovery when USB serial suffix changes."""

import tempfile
from pathlib import Path
from unittest import TestCase
from unittest.mock import patch

import one_command_launch as launcher


class MicrostrainDiscoveryTests(TestCase):
    def test_new_serial_suffix_is_detected(self):
        device = Path('/dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000_6287.248360-if00')
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter([device])), \
             patch.object(Path, 'exists', return_value=True), \
             patch.object(launcher.os, 'access', return_value=True):
            self.assertEqual(launcher.resolve_microstrain_port(), device)

    def test_missing_device_fails_closed(self):
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter([])):
            with self.assertRaisesRegex(launcher.LaunchError, 'No MicroStrain'):
                launcher.resolve_microstrain_port()

    def test_multiple_devices_fail_closed(self):
        devices = [Path('/dev/serial/by-id/USB_MicroStrain_A'),
                   Path('/dev/serial/by-id/USB_MicroStrain_B')]
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter(devices)), \
             patch.object(Path, 'exists', return_value=True):
            with self.assertRaisesRegex(launcher.LaunchError, 'Multiple MicroStrain'):
                launcher.resolve_microstrain_port()

    def test_unreadable_device_fails_closed(self):
        device = Path('/dev/serial/by-id/USB_MicroStrain_A')
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter([device])), \
             patch.object(Path, 'exists', return_value=True), \
             patch.object(launcher.os, 'access', return_value=False):
            with self.assertRaisesRegex(launcher.LaunchError, 'not readable/writable'):
                launcher.resolve_microstrain_port()

    def test_runtime_yaml_preserves_original(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config = root / 'amr_imu/config/microstrain.yml'
            config.parent.mkdir(parents=True)
            original = ('microstrain_inertial_driver:\\n'
                        '  ros__parameters:\\n'
                        '    port: "/dev/serial/by-id/old"\\n'
                        '    baudrate: 115200\\n')
            original = original.replace('\\n', '\n')
            config.write_text(original)
            port = Path('/dev/serial/by-id/usb-Lord_Microstrain_248360-if00')
            with patch.object(launcher, 'ROOT', root), \
                 patch.object(launcher, 'resolve_microstrain_port', return_value=port):
                runtime = launcher.prepare_microstrain_config(root)
            self.assertEqual(config.read_text(), original)
            self.assertIn(str(port), runtime.read_text())
            self.assertIn('baudrate: 115200', runtime.read_text())
            self.assertNotIn('/old', runtime.read_text())


if __name__ == '__main__':
    import unittest
    unittest.main()
