from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {
                'fcu_url': 'serial:///dev/ttyACM0:921600',
                'plugin_allowlist': ['imu', 'sys_status'],
            }
        ]
    )

    imu_splitter_node = Node(
        package='imu_splitter',
        executable='imu_splitter_node',
        name='imu_splitter',
        output='screen'
    )

    return LaunchDescription([
        mavros_node,
        imu_splitter_node
    ])
