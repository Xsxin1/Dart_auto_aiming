import os
import sys
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

def generate_launch_description():

    from common import node_params, launch_params#, robot_state_publisher#, armor_tracker_node, buff_tracker_node, auto_record_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit

    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml","port:=8765"])

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        respawn=True,           # 节点崩溃自动重启
        respawn_delay=3,        # 重启延迟
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    hik_camera= Node(
        package="hik_camera",
        executable="hik_camera_node",
        parameters=[node_params],
        arguments=["--ros-args", "--log-level", 'hik_camera:='+launch_params['camera_log_level']],
        output="screen",
    )


    return LaunchDescription([
        # PushRosNamespace('xsx'),
        # serial_driver_node,
        hik_camera,
        foxglove_bridge,
    ])
