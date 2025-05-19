from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('xtsdk_ros')+'/rviz/xtsdk_ros2.rviz'
    param_config=get_package_share_directory('xtsdk_ros')+'/cfg/xtsdk_ros2.yaml'
    return LaunchDescription([
        Node(
            package='xtsdk_ros',
            executable='xtsdk_node',
            name='xtsdk_node',
            output='screen',
			emulate_tty=True,
            parameters=[]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='xtrviz2',
            arguments=['-d',rviz_config]
        ),
		Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure'
        )
    ])
