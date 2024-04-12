from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    urdf_model_path = get_package_share_directory('slm_sim') + '/urdf/pendulum.urdf'
    rviz_config_path = get_package_share_directory('slm_sim') + '/rviz/manipulator.rviz'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_model_path).read()}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])


if __name__ == '__main__':
    generate_launch_description()
