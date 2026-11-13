"""Camera AI-off behavior: no Ultralytics dependency or model path required."""

import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent.parent
CAMERA = ROOT / 'amr_websocket/amr_openvino_camera_depth.py'
LAUNCHER = ROOT / 'one_command_launch.py'


class SkipDetechTests(unittest.TestCase):
    def test_ai_import_is_only_inside_enabled_branch(self):
        source = CAMERA.read_text(encoding='utf-8')
        tree = ast.parse(source)
        top_imports = [node for node in tree.body
                       if isinstance(node, (ast.Import, ast.ImportFrom))]
        self.assertFalse(any(
            getattr(node, 'module', '') == 'ultralytics' or
            any(alias.name == 'ultralytics' for alias in getattr(node, 'names', []))
            for node in top_imports
        ))
        self.assertIn('if not skip_detech:', source)
        self.assertIn('from ultralytics import YOLO', source)

    def test_depth_capture_remains_enabled_in_ai_off_mode(self):
        source = CAMERA.read_text(encoding='utf-8')
        self.assertIn('config.enable_stream(depth_profile)', source)
        self.assertIn('config.enable_stream(color_profile)', source)
        self.assertIn('depth_frame.get_data()', source)
        self.assertIn('if skip_detech:\n                annotated = color', source)
        self.assertIn('kwargs={"skip_detech": args.skip_detech}', source)

    def test_launcher_passes_switch_and_skips_model_preflight(self):
        source = LAUNCHER.read_text(encoding='utf-8')
        self.assertIn("parser.add_argument('--skip-detech'", source)
        self.assertIn("not args.skip_camera and not args.skip_detech and not CAMERA_MODEL.is_dir()", source)
        self.assertIn("camera_argv.append('--skip-detech')", source)


if __name__ == '__main__':
    unittest.main()
