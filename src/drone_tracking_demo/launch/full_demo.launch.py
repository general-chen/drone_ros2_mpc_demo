import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('drone_tracking_demo')
    wind_level_config = LaunchConfiguration('wind_level')
    trajectory_mode_config = LaunchConfiguration('trajectory_mode')

    urdf_path = os.path.join(pkg, 'urdf', 'simple_quadrotor.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Simulator
    sim_node = Node(
        package='drone_tracking_demo',
        executable='drone_simulator',
        output='screen',
        parameters=[{'wind_level': wind_level_config}]
    )

    # MPC controller
    mpc_node = Node(
        package='drone_tracking_demo',
        executable='mpc_tracker',
        output='screen',
        parameters=[{'trajectory_mode': trajectory_mode_config}]
    )

    # Robot state publisher (URDF → TF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'wind_level',
            default_value='strong'
        ),
        DeclareLaunchArgument(
            'trajectory_mode',
            default_value='circle'
        ),
        sim_node,
        mpc_node,
        rsp_node,
        rviz_node
    ])
