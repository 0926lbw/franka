# Copyright 2026 lbw
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# flake8: noqa: E402
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.modules.pop("franka_tracker_bridge", None)


class DebugCurveNodeStaticTest(unittest.TestCase):
    def test_debug_curve_node_entrypoints_are_installed_and_launchable(self):
        cmake = (ROOT / "CMakeLists.txt").read_text()
        launch = (ROOT / "launch" / "tracker_preview.launch.py").read_text()
        package = (ROOT / "package.xml").read_text()

        self.assertTrue((ROOT / "scripts" / "tracker_debug_curve_node").is_file())
        self.assertTrue(
            (ROOT / "franka_tracker_bridge" / "tracker_debug_curve_node.py").is_file()
        )
        self.assertIn("scripts/tracker_debug_curve_node", cmake)
        self.assertIn("debug_curves", launch)
        self.assertIn("default_value", launch)
        self.assertIn("false", launch)
        self.assertIn("tracker_debug_curve_node", launch)
        self.assertIn("condition=IfCondition(debug_curves)", launch)
        self.assertIn("<exec_depend>python3-matplotlib</exec_depend>", package)


if __name__ == "__main__":
    unittest.main()
