import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
	
    pkg_share = get_package_share_directory('hunter_se_control')
    sim_time = DeclareLaunchArgument('use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time')
    
    robot_localization_node = Node(
	    package='robot_localization',
	    executable='ekf_node',
	    name='ekf_filter_node',
	    output='screen',
        namespace='/hunter_se',
	    parameters=[os.path.join(pkg_share,'config/ekf.yaml'),{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'hunter_se_control'), 'config', 'nav2.rviz')]
    )

    ld.add_action(sim_time)
    ld.add_action(rviz2_node)
    ld.add_action(robot_localization_node)

    return ld

