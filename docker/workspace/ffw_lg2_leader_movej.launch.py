#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun
#
# ---------------------------------------------------------------------------
# LOCAL COPY for cyclo_control `movej` teleop.
# Source: ffw_bringup/launch/ffw_lg2_leader_ai.launch.py (ai_worker 2.2.5)
# Changes vs stock:
#  1. joint_trajectory_command_broadcaster output topics remapped to
#     `.../raw_joint_trajectory` so the cyclo_control controller is the ONLY
#     publisher on the follower's `.../joint_trajectory` topics.
#  2. /cmd_vel remapped to /cmd_vel_disabled (base motion off for tabletop).
#  3. Controllers yaml (see robot_controllers below) dead-ends the joystick
#     head/lift output topics (head/lift teleop off for tabletop).
#
# Run (inside the ai_worker container, after follower bringup and
# `motion_controller controller_type:=movej`):
#   ros2 launch /workspace/ffw_lg2_leader_movej.launch.py
#
# Do NOT combine with `ffw_sg2_ai` (it launches the unmodified leader).
# ---------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')

    # Robot controllers config file path
    # movej variant: local copy with sync_threshold raised (see file header comment).
    # Original: FindPackageShare('ffw_bringup')/config/ffw_lg2_leader/ffw_lg2_leader_ai_hardware_controller.yaml
    robot_controllers = '/workspace/ffw_lg2_leader_movej_controllers.yaml'

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_lg2_leader', description_file]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            # --- movej remaps (added) ---
            '--controller-ros-args',
            '-r /leader/joint_trajectory_command_broadcaster_left/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory',
            '--controller-ros-args',
            '-r /leader/joint_trajectory_command_broadcaster_right/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory',
            # base motion DISABLED for tabletop manip: joystick_controller's
            # hardcoded /cmd_vel publisher is remapped to a dead-end topic so the
            # swerve base can never be driven from the leader (even if swerve
            # mode is toggled by accident). Remove this remap to restore.
            '--controller-ros-args',
            '-r /cmd_vel:=/cmd_vel_disabled',
            # --- end remaps ---
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller_left',
            'spring_actuator_controller_right',
            'joystick_controller',
            'joint_state_broadcaster',
        ],
        parameters=[robot_description],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'frame_prefix': 'leader_'}],
    )

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
        ]
    )

    # Return combined LaunchDescription
    return LaunchDescription(declared_arguments + [leader_with_namespace])
