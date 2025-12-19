from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare log level argument
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Log level for nodes'
    )

    declare_holonomic = DeclareLaunchArgument(
        'holonomic',
        default_value='false',
        description='Set to true for holonomic drive, false for differential drive'
    )

    # Launch configuration
    log_level = LaunchConfiguration('log_level')
    holonomic = LaunchConfiguration('holonomic')

    # Motion controller node
    motion_controller = Node(
        package="soar_rosbot_controller",
        executable="motion_controller",
        name="motion_controller",
        parameters=[
            {"holonomic": holonomic},
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'info', '--log-level', 'motion_controller:=debug'],
    )

    # Yaw observer node
    yaw_observer = Node(
        package="soar_rosbot_controller",
        executable="yaw_observer",
        name="yaw_observer",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription(
        [
            declare_log_level,
            declare_holonomic,
            motion_controller,
            yaw_observer,
        ]
    )
