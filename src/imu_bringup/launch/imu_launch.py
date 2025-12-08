from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    imu_desc_path = os.path.join(
        get_package_share_directory('imu_description'),
        'urdf/imu_model.urdf.xacro'
    )

    # PROCESS the xacro file
    robot_description = xacro.process_file(imu_desc_path).toxml()


    return LaunchDescription([
        # Publish TF + URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Run your IMU node (adjust package/executable)
        Node(
            package='imu_mahony',
            executable='imu_node',
            output='screen'
        )
    ])
