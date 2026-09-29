# panda_mujoco.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    controllers = os.path.join(here, "my_controllers.yaml")   # absolute path

    mc = (MoveItConfigsBuilder("moveit_resources_panda")
          .robot_description(file_path="config/panda.urdf.xacro")
          .trajectory_execution(controllers)
          .to_moveit_configs())
    rviz_cfg = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "launch", "moveit.rviz")
    return LaunchDescription([
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[mc.robot_description]),
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[mc.to_dict()]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             parameters=[mc.robot_description, mc.robot_description_semantic,
                         mc.robot_description_kinematics, mc.planning_pipelines,
                         mc.joint_limits]),
    ])