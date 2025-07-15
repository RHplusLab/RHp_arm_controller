from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rhparm").to_dict()
    # MTC Demo node
    pick_place_demo = Node(
        package="rhparm_mtc_pick_and_place",
        executable="vision_cylinder_one",
        output="screen",
        parameters=[
            moveit_config
        ],
    )

    return LaunchDescription([
        pick_place_demo
    ])
