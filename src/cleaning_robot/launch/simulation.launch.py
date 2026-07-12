from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('cleaning_robot')

    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
    room_path = os.path.join(pkg_dir, 'urdf', 'room.sdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', room_path],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            arguments=[
                # ROS -> Gazebo only
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

                # Gazebo -> ROS only
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'cleaning_robot',
                        '-file', urdf_path,
                        '-x', '0',
                        '-y', '0',
                        '-z', '0.1'
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='cleaning_robot',
                    executable='cleaning_node',
                    name='cleaning_robot',
                    output='screen'
                )
            ]
        ),
    ])
