"""Keep user-installed native Python packages out of managed ROS processes."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import one_command_launch as launcher


class PythonEnvironmentTests(unittest.TestCase):
    def sourced_environment(self, inherited):
        payload = b'__AMR_ENV_START__\0' + b''.join(
            f'{key}={value}'.encode() + b'\0'
            for key, value in inherited.items())
        result = SimpleNamespace(returncode=0, stdout=payload, stderr=b'')
        with patch.object(Path, 'is_file', return_value=True), \
                patch.object(launcher.subprocess, 'run', return_value=result) as run:
            env = launcher.source_ros_environment()
        return env, run.call_args.args[0][2]

    def test_isolation_precedes_ros_setup_and_overrides_inherited_opt_out(self):
        env, bootstrap = self.sourced_environment({'PYTHONNOUSERSITE': '0'})
        self.assertLess(bootstrap.index('export PYTHONNOUSERSITE=1'),
                        bootstrap.index('source /opt/ros/jazzy/setup.bash'))
        self.assertEqual(env['PYTHONNOUSERSITE'], '1')
        self.assertEqual(env['ROS_DOMAIN_ID'], '31')

    def test_managed_python_ignores_poisoned_user_site(self):
        with tempfile.TemporaryDirectory() as directory:
            user_site = (Path(directory) / 'lib' /
                         f'python{sys.version_info.major}.{sys.version_info.minor}' /
                         'site-packages')
            user_site.mkdir(parents=True)
            (user_site / 'amr_user_site_poison.py').write_text(
                "raise RuntimeError('user package must not be imported')\n")
            inherited = dict(os.environ, PYTHONUSERBASE=directory,
                             PYTHONNOUSERSITE='0', PYTHONPATH='')
            env, _ = self.sourced_environment(inherited)
            probe = (
                "import importlib.util, site; "
                "assert site.ENABLE_USER_SITE is False; "
                "assert importlib.util.find_spec('amr_user_site_poison') is None"
            )
            result = subprocess.run([sys.executable, '-c', probe], env=env,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
