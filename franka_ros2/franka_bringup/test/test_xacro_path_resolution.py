#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Path-resolution contract tests for composition xacros.

Proves that every in-scope xacro entry point resolves all its $(find …) includes
and processes without xacro error. This catches stale package paths after migrations
(e.g. ros2_control xacro moving from franka_description to franka_hardware).

These tests run against the INSTALLED tree (same as launch files use at runtime).
"""

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


def _share(package: str, *path_parts: str) -> str:
    """Resolve a file path under a package's share directory."""
    import os
    return os.path.join(get_package_share_directory(package), *path_parts)


def _expand(xacro_file: str, mappings: dict = None) -> str:
    """Expand a xacro file and return the XML string. Raises on any xacro error."""
    return xacro.process_file(xacro_file, mappings=mappings or {}).toxml()


# ---------------------------------------------------------------------------
# Parametrized test cases: (description, package, path_parts, mappings)
# ---------------------------------------------------------------------------

XACRO_ENTRIES = [
    (
        'franka_arm_bringup',
        'franka_bringup',
        ('urdf', 'franka_arm.urdf.xacro'),
        {'robot_type': 'fr3', 'robot_ip': '192.168.1.1'},
    ),
]


@pytest.mark.parametrize(
    'description,package,path_parts,mappings',
    XACRO_ENTRIES,
    ids=[e[0] for e in XACRO_ENTRIES],
)
class TestXacroPathResolution:
    """Verify that composition xacros expand without errors."""

    def test_expands_without_error(self, description, package, path_parts, mappings):
        """xacro.process_file succeeds — no unresolved $(find) or missing includes."""
        xacro_file = _share(package, *path_parts)
        # This will raise xacro.XacroException on any resolution failure
        _expand(xacro_file, mappings)

    def test_produces_valid_xml(self, description, package, path_parts, mappings):
        """Expanded output is well-formed XML with a <robot> root element."""
        xacro_file = _share(package, *path_parts)
        xml_str = _expand(xacro_file, mappings)
        root = ET.fromstring(xml_str)
        assert root.tag == 'robot', f'Expected <robot> root, got <{root.tag}>'


if __name__ == '__main__':
    pytest.main([__file__])
