"""CAN USB auto-discovery tests; no motor hardware is accessed."""

from pathlib import Path
import unittest
from unittest.mock import patch

import one_command_launch as launcher


class CanPortDiscoveryTests(unittest.TestCase):
    def test_changed_ftdi_serial_number(self):
        expected = Path('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_NEW123-if00-port0')
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter([expected])), \
             patch.object(Path, 'exists', return_value=True), \
             patch.object(launcher.os, 'access', return_value=True):
            self.assertEqual(launcher.can_serial_port(), str(expected))

    def test_missing_ftdi_fails(self):
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter([])):
            with self.assertRaisesRegex(launcher.LaunchError, 'No FTDI_FT232R'):
                launcher.can_serial_port()

    def test_multiple_ftdi_adapters_fail_without_guessing(self):
        ports = [Path('/dev/serial/by-id/usb-FTDI_FT232R_A'),
                 Path('/dev/serial/by-id/usb-FTDI_FT232R_B')]
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter(ports)), \
             patch.object(Path, 'exists', return_value=True):
            with self.assertRaisesRegex(launcher.LaunchError, 'Multiple FTDI_FT232R'):
                launcher.can_serial_port()

    def test_unreadable_ftdi_fails(self):
        port = Path('/dev/serial/by-id/usb-FTDI_FT232R_A')
        with patch.object(Path, 'is_dir', return_value=True), \
             patch.object(Path, 'iterdir', return_value=iter([port])), \
             patch.object(Path, 'exists', return_value=True), \
             patch.object(launcher.os, 'access', return_value=False):
            with self.assertRaisesRegex(launcher.LaunchError, 'not readable/writable'):
                launcher.can_serial_port()


if __name__ == '__main__':
    unittest.main()
