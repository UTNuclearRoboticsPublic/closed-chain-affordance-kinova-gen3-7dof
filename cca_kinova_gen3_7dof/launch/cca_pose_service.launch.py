"""
Launch the custom CCA Cartesian pose service.

Reuses the robot description and CCA parameters from
cca_kinova_gen3_7dof_settings.py, just like the existing
CCA action-server launch file.
"""

import os
import importlib.util

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_settings_module(
    package='cca_kinova_gen3_7dof',
    module_name='cca_kinova_gen3_7dof_settings'
):
    settings_path = os.path.join(
        FindPackageShare(package).find(package),
        'launch',
        f'{module_name}.py'
    )

    spec = importlib.util.spec_from_file_location(
        module_name,
        settings_path
    )

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def generate_launch_description():

    # Load the existing robot-specific settings.
    cca_robot_settings = load_settings_module()

    declare_launch_args = cca_robot_settings.declare_launch_args
    define_robot_paths_and_settings = (
        cca_robot_settings.define_robot_paths_and_settings
    )
    generate_robot_description_content = (
        cca_robot_settings.generate_robot_description_content
    )

    # Get launch arguments and robot settings.
    args = declare_launch_args()

    robot_args = args["robot_args"]
    base_args = args["base_args"]

    settings = define_robot_paths_and_settings()

    # Generate the robot description.
    robot_description = generate_robot_description_content(
        package_name=settings["urdf_package"],
        urdf_rel_path=settings["urdf_xacro_path"],
        launch_args=robot_args,
    )

    # Load the CCA robot configuration.
    cca_robot_description_params = (
        settings["cca_robot_description_path"]
    )

    robot_name = settings["cca_robot_package"].removeprefix("cca_")

    # Parameters required by the CCA planner node.
    params = [
        {"robot_description": robot_description},
        cca_robot_description_params,
    ]

    return LaunchDescription(
        robot_args
        + base_args
        + [
            Node(
                package="cca_kinova_gen3_7dof",
                executable="cca_pose_service",

                # This name matches the planner node created in
                # cca_pose_service.cpp, so it receives these parameters.
                name="cc_affordance_planner_ros",

                output="screen",
                parameters=params,
                remappings=[
                    (
                        "/cca_ros_val_and_viz",
                        f"/{robot_name}/cca_ros_val_and_viz"
                    ),
                ],
            ),
        ]
    )