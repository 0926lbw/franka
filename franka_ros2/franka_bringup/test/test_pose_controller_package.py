import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


class PoseControllerPackageTest(unittest.TestCase):
    def test_pose_controller_package_is_present_and_trimmed_to_pose_controller(self):
        package_root = REPO_ROOT / 'franka_arm_controllers'

        self.assertTrue((package_root / 'package.xml').is_file())
        self.assertTrue((package_root / 'CMakeLists.txt').is_file())
        self.assertTrue(
            (package_root / 'src' / 'joint_impedance_pose_controller.cpp').is_file()
        )
        self.assertTrue(
            (
                package_root
                / 'include'
                / 'franka_arm_controllers'
                / 'joint_impedance_pose_controller.hpp'
            ).is_file()
        )
        self.assertFalse(
            (package_root / 'src' / 'joint_impedance_ik_controller.cpp').exists(),
            'SpaceMouse/Twist controller should not be included in this workspace',
        )

        plugin_xml = (package_root / 'franka_arm_controllers.xml').read_text()
        self.assertIn('franka_arm_controllers/JointImpedancePoseController', plugin_xml)
        self.assertNotIn('JointImpedanceIKController', plugin_xml)

        cmake = (package_root / 'CMakeLists.txt').read_text()
        self.assertIn('src/joint_impedance_pose_controller.cpp', cmake)
        self.assertNotIn('src/joint_impedance_ik_controller.cpp', cmake)

    def test_bringup_registers_pose_controller_for_tracker_target_pose(self):
        controllers_yaml = (REPO_ROOT / 'franka_bringup' / 'config' / 'controllers.yaml').read_text()

        self.assertIn('joint_impedance_pose_controller:', controllers_yaml)
        self.assertIn(
            'type: franka_arm_controllers/JointImpedancePoseController',
            controllers_yaml,
        )
        self.assertIn(
            'target_pose_topic: /franka_controller/target_cartesian_pose',
            controllers_yaml,
        )
        self.assertIn(
            'kdl_desired_joint_states_topic: /franka_controller/kdl_desired_joint_states',
            controllers_yaml,
        )
        self.assertIn('ik_backend: kdl', controllers_yaml)
        self.assertIn('base_link_name: base', controllers_yaml)
        self.assertIn('tcp_link_name: fr3_link8', controllers_yaml)

    def test_workspace_dependencies_include_pose_controller_package(self):
        meta_package = (REPO_ROOT / 'franka_ros2' / 'package.xml').read_text()
        bringup_package = (REPO_ROOT / 'franka_bringup' / 'package.xml').read_text()

        self.assertIn('<depend>franka_arm_controllers</depend>', meta_package)
        self.assertIn('<exec_depend>franka_arm_controllers</exec_depend>', bringup_package)

    def test_pose_controller_package_contains_single_workspace_launch_assets(self):
        package_root = REPO_ROOT / 'franka_arm_controllers'

        expected_files = [
            package_root / 'launch' / 'joint_impedance_pose_controller.launch.py',
            package_root / 'launch' / 'franka.launch.py',
            package_root / 'urdf' / 'franka_arm.urdf.xacro',
            package_root / 'config' / 'tracker_fake_fr3_config.yaml',
            package_root / 'config' / 'example_fr3_config.yaml',
            package_root / 'config' / 'example_fr3_duo_config.yaml',
        ]

        for path in expected_files:
            with self.subTest(path=path):
                self.assertTrue(path.is_file())

        launch_file = (
            package_root / 'launch' / 'joint_impedance_pose_controller.launch.py'
        ).read_text()
        self.assertIn('default_value="tracker_fake_fr3_config.yaml"', launch_file)
        self.assertIn('default_value="joint_impedance_pose_controller"', launch_file)


if __name__ == '__main__':
    unittest.main()
