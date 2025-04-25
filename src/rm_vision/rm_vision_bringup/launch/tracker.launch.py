import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher#, armor_tracker_node, buff_tracker_node, auto_record_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription


    detector_node = Node(
    package='tracker',
    executable='tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    ros_arguments=['--log-level', 'light_tracker:='+launch_params['tracker_detector_log_level']],
)
    delay_detector_node = TimerAction(
        period=0.5,
        actions=[detector_node],
    )


    return LaunchDescription([
        # robot_state_publisher,
        PushRosNamespace('dart'),      
        delay_detector_node
    ])
