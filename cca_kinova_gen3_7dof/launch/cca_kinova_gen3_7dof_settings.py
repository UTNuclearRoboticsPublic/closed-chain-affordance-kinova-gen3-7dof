"""Robot-specific CCA settings for kinova_gen3_7dof.

Author: Crasun Jans

This module centralizes all robot-specific configuration for kinova_gen3_7dof
for usage with the Closed-Chain Affordance (CCA) planner. It provides:

1. Launch arguments for robot hardware configuration
2. File paths for URDF, SRDF, and CCA configuration files
3. Functions to generate robot_description and robot_description_semantic content

### To Customize for Your Robot:
Replace the robot-specific configuration in:
- declare_launch_args(): Define your robot's hardware/configuration options
- define_robot_paths_and_settings(): Specify your robot's package names and file paths

The generic functions at the bottom should work for any robot and typically don't
need modification.
"""

import os
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# 1️⃣ USER-EDITABLE: Declare Launch Arguments
# ---------------------------------------------------------------------------
def declare_launch_args():
    """Declare all launch arguments for robot configuration.

    ✅ Modify the ROBOT ARGUMENTS section for your robot's hardware options.
    ⚠️ The BASE ARGUMENTS section should NOT be modified (used by launch files).

    Returns:
        dict: Dictionary with 'robot_args' and 'base_args' keys containing
              lists of DeclareLaunchArgument objects
    """
    # ---- ROBOT-SPECIFIC ARGS (Modify as needed) ----
    # robot_args = [
    #     # Example: 
    #     # DeclareLaunchArgument(
    #     #     "has_arm",
    #     #     default_value="True",
    #     #     choices=["True", "False"],
    #     #     description="Include the Spot arm.",
    #     # ),
    #     DeclareLaunchArgument(
    #         "name",
    #         description="Name of the robot",
    #         choices=["kinova"],
    #         default_value="kinova",
    #     ),
    #     DeclareLaunchArgument(
    #         "arm",
    #         description="Type/series of robot.",
    #         choices=["gen3", "gen3_lite"],
    #         default_value="gen3",
    #     ),
    #     DeclareLaunchArgument(
    #         "gripper",
    #         default_value="robotiq_2f_85",
    #         description="Name of the gripper attached to the arm",
    #     ),
    #     DeclareLaunchArgument(
    #         "dof",
    #         default_value="7",
    #         description="Robot's dof",
    #     ),
    # ]
    # These are forwarded to the scrat2 xacro. They MUST match what
    # scrat2_dual_kinova_moveit_config/launch/robot.launch.py passes on the real robot,
    # otherwise CCA plans against a different model (xacro defaults are the Ignition sim).
    robot_args = [
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Must match robot.launch.py sim_ignition.",
        ),
        DeclareLaunchArgument(
            "vision",
            default_value="true",
            description="Must match robot.launch.py vision (camera frames in URDF).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Must match robot.launch.py use_fake_hardware.",
        ),
        DeclareLaunchArgument(
            "right_gripper",
            default_value="robotiq_2f_85",
            description="Must match robot.launch.py right_gripper.",
        ),
    ]

    # ---- BASE / SYSTEM ARGS (DO NOT MODIFY) ----
    # These are standard arguments used by launch files
    base_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation time (true for Gazebo or rosbag playback).",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            choices=["true", "false"],
            description="Whether to launch RViz for visualization.",
        ),
    ]

    return {"robot_args": robot_args, "base_args": base_args}


# ---------------------------------------------------------------------------
# 2️⃣ USER-EDITABLE: Define Robot Paths and Settings
# ---------------------------------------------------------------------------
def define_robot_paths_and_settings():
    """Define robot-specific package names, file paths, and configuration settings.

    ⚠️ Modify package names and file paths for your robot.
       Keep the key names unchanged so launch files can access them consistently.

    Returns:
        dict: Configuration dictionary with the following keys:
            - urdf_package: Package containing robot URDF
            - urdf_xacro_path: Relative path to URDF xacro file
            - srdf_package: Package containing robot SRDF
            - srdf_path: Relative path to SRDF file
            - srdf_arg_usage: How SRDF uses launch args ("all", "none", or "subset")
            - srdf_subset_args: List of arg names if using "subset" mode
            - cca_robot_package: Package containing CCA configuration files
            - cca_robot_description_path: Path to CCA robot description YAML
    """
    return {
        # ---- ROBOT DESCRIPTION - URDF (Modify as needed) ----
        "urdf_package": "scrat2_dual_kinova_moveit_config", # Example: kinova_gen3_7dof_description
        "urdf_xacro_path": "config/scrat2_dual_kinova.urdf.xacro", # Example: urdf/kinova_gen3_7dof.urdf.xacro
        
        # ---- ROBOT DESCRIPTION SEMANTIC  - SRDF (Modify as needed) ----
        "srdf_package": "scrat2_dual_kinova_moveit_config", # Example: kinova_gen3_7dof_moveit_config
        "srdf_path": "config/scrat2_dual_kinova.srdf", # Example: config/kinova_gen3_7dof.srdf.xacro
        
        # SRDF argument reuse options:
        # - "all": Pass all robot_args to SRDF xacro
        # - "none": Don't pass any arguments to SRDF
        # - "subset": Pass only the arguments listed in srdf_subset_args
        "srdf_arg_usage": "all",
        "srdf_subset_args": [],  # Used if srdf_arg_usage == "subset"
        
        # ---- CCA CONFIGURATION FILES (AUTOGENERATED) ----
        "cca_robot_package": "cca_kinova_gen3_7dof",
        "cca_robot_description_path": os.path.join(
            get_package_share_directory("cca_kinova_gen3_7dof"),
            "config",
            "cca_kinova_gen3_7dof_description.yaml"
        ),
    }


# ---------------------------------------------------------------------------
# GENERIC FUNCTIONS (Should work for any robot - rarely needs modification)
# ---------------------------------------------------------------------------
def generate_robot_description_content(
    package_name: str, urdf_rel_path: str, launch_args: Dict[str, List[DeclareLaunchArgument]]
):
    """Generate the robot_description parameter content.

    Automatically detects whether the file is xacro or static URDF and processes
    it accordingly. For xacro files, all launch arguments are passed through.

    Args:
        package_name: ROS 2 package containing the URDF
        urdf_rel_path: Relative path to URDF file within the package
        launch_args: List of DeclareLaunchArgument objects to pass to xacro

    Returns:
        ParameterValue or str: Robot description content
            - ParameterValue: If file is xacro (will be evaluated at launch time)
            - str: If file is static URDF (loaded immediately)

    Raises:
        RuntimeError: If URDF file is not found

    Example:
        >>> robot_desc = generate_robot_description_content(
        ...     "my_robot_description",
        ...     "urdf/robot.urdf.xacro",
        ...     robot_args
        ... )
    """
    urdf_full_path = os.path.join(
        get_package_share_directory(package_name), urdf_rel_path
    )

    # Handle xacro files - process with launch arguments
    if urdf_full_path.endswith(".xacro"):
        xacro_args = []
        for arg in launch_args:
            xacro_args.extend([f" {arg.name}:=", LaunchConfiguration(arg.name)])

        urdf_file = PathJoinSubstitution(
            [FindPackageShare(package_name), urdf_rel_path]
        )
        return ParameterValue(
            Command(["xacro ", urdf_file, *xacro_args]), value_type=str
        )

    # Handle static URDF files - load directly
    try:
        with open(urdf_full_path, "r", encoding="utf-8") as f:
            return f.read()
    except FileNotFoundError as e:
        raise RuntimeError(
            f"URDF file not found: {urdf_full_path}\n"
            f"Ensure '{package_name}' package is installed and contains '{urdf_rel_path}'"
        ) from e


def generate_robot_description_semantic_content(
    package_name: str,
    srdf_path: str,
    all_launch_args: List[DeclareLaunchArgument],
    srdf_arg_usage: str,
    srdf_subset_args: Optional[List[str]] = None
):
    """Generate the robot_description_semantic parameter content.

    Supports both static and xacro-based SRDF files with flexible argument passing.
    The srdf_arg_usage parameter controls which launch arguments are passed to xacro.

    Args:
        package_name: ROS 2 package containing the SRDF
        srdf_path: Relative path to SRDF file within the package
        all_launch_args: List of all available DeclareLaunchArgument objects
        srdf_arg_usage: Argument passing mode:
            - "all": Pass all launch arguments to SRDF xacro
            - "none": Don't pass any arguments (for static SRDF or parameterless xacro)
            - "subset": Pass only arguments listed in srdf_subset_args
        srdf_subset_args: List of argument names to use when srdf_arg_usage == "subset"

    Returns:
        ParameterValue or str: Semantic robot description content
            - ParameterValue: If file is xacro (will be evaluated at launch time)
            - str: If file is static SRDF (loaded immediately)

    Raises:
        RuntimeError: If SRDF file is not found

    Example:
        >>> robot_semantic = generate_robot_description_semantic_content(
        ...     "my_robot_moveit_config",
        ...     "config/robot.srdf.xacro",
        ...     robot_args,
        ...     "subset",
        ...     ["has_gripper", "arm_type"]
        ... )
    """
    srdf_full_path = os.path.join(get_package_share_directory(package_name), srdf_path)

    # Handle xacro files - process with selected arguments
    if srdf_full_path.endswith(".xacro"):
        # Select which arguments to pass based on usage mode
        if srdf_arg_usage == "none":
            selected_args = []
        elif srdf_arg_usage == "subset":
            selected_args = [a for a in all_launch_args if a.name in srdf_subset_args]
        else:  # "all"
            selected_args = all_launch_args

        # Build xacro command with selected arguments
        xacro_args = []
        for arg in selected_args:
            xacro_args.extend([f" {arg.name}:=", LaunchConfiguration(arg.name)])

        srdf_file = PathJoinSubstitution(
            [FindPackageShare(package_name), srdf_path]
        )
        return ParameterValue(
            Command(["xacro ", srdf_file, *xacro_args]), value_type=str
        )

    # Handle static SRDF files - load directly
    try:
        with open(srdf_full_path, "r", encoding="utf-8") as f:
            return f.read()
    except FileNotFoundError as e:
        raise RuntimeError(
            f"SRDF file not found: {srdf_full_path}\n"
            f"Ensure '{package_name}' package is installed and contains '{srdf_path}'"
        ) from e
