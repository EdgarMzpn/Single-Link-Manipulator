from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_model = os.path.join(get_package_share_directory('slm_sim'), 'urdf', 'pendulum.urdf')
    robot_description_config = {'robot_description': urdf_model}

    slm_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='slm_state_pub',
        output='screen',
        parameters=[robot_description_config]
    )

    # Uncomment for TESTING ONLY (COMMENT YOUR JOINT STATE PUBLISHER)
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    rviz_config = os.path.join(get_package_share_directory('slm_sim'), 'rviz', 'manipulator.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        slm_state_pub_node,
        # Uncomment for TESTING ONLY (COMMENT YOUR JOINT STATE PUBLISHER)
        # joint_state_publisher_gui_node,
        rviz_node
    ])
