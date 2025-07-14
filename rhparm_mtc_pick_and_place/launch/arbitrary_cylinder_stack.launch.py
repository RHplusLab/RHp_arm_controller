from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rhparm").to_dict()

    # Declare launch arguments for x_coord and y_coord
    x1_coord_arg = DeclareLaunchArgument(
        'x1_coord',
        default_value='0.160',  # 기본값 설정 (문자열로 전달)
        description='X coordinate for the object placement.'
    )

    y1_coord_arg = DeclareLaunchArgument(
        'y1_coord',
        default_value='0.0',    # 기본값 설정 (문자열로 전달)
        description='Y coordinate for the object placement.'
    )

    x2_coord_arg = DeclareLaunchArgument(
        'x2_coord',
        default_value='0.160',  # 기본값 설정 (문자열로 전달)
        description='X coordinate for the object placement.'
    )

    y2_coord_arg = DeclareLaunchArgument(
        'y2_coord',
        default_value='0.0',    # 기본값 설정 (문자열로 전달)
        description='Y coordinate for the object placement.'
    )

    x3_coord_arg = DeclareLaunchArgument(
        'x3_coord',
        default_value='0.160',  # 기본값 설정 (문자열로 전달)
        description='X coordinate for the object placement.'
    )

    y3_coord_arg = DeclareLaunchArgument(
        'y3_coord',
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
            {'x1_coord': LaunchConfiguration('x1_coord')},
            {'y1_coord': LaunchConfiguration('y1_coord')},
            {'x2_coord': LaunchConfiguration('x2_coord')},
            {'y2_coord': LaunchConfiguration('y2_coord')},
            {'x3_coord': LaunchConfiguration('x3_coord')},
            {'y3_coord': LaunchConfiguration('y3_coord')}
        ],
    )

    return LaunchDescription([
        x1_coord_arg,
        y1_coord_arg,
        x2_coord_arg,
        y2_coord_arg,
        x3_coord_arg,
        y3_coord_arg,
        pick_place_demo
    ])
