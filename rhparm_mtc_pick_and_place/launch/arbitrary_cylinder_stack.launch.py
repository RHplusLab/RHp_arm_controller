from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rhparm").to_dict()

    # Declare launch arguments for x_coord and y_coord
    x_coord_arg = DeclareLaunchArgument(
        'x_coord',
        default_value='0.160',  # 기본값 설정 (문자열로 전달)
        description='X coordinate for the object placement.'
    )

    y_coord_arg = DeclareLaunchArgument(
        'y_coord',
        default_value='0.0',    # 기본값 설정 (문자열로 전달)
        description='Y coordinate for the object placement.'
    )

    # MTC Demo node
    pick_place_demo = Node(
        package="rhparm_mtc_pick_and_place",
        executable="arbitrary_cylinder_stack",
        output="screen",
        parameters=[
            moveit_config,
            # 런치 인자를 파라미터로 노드에 전달
            {'x_coord': LaunchConfiguration('x_coord')},
            {'y_coord': LaunchConfiguration('y_coord')}
        ],
    )

    return LaunchDescription([
        x_coord_arg,
        y_coord_arg,
        pick_place_demo
    ])
