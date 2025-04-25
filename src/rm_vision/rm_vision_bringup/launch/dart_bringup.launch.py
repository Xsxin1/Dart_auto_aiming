import os
import sys
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))
from tracetools_launch.action import Trace

def generate_launch_description():  
    cmd = ExecuteProcess(
        cmd=['rqt','ls /dev/ | grep ttyACM','sudo chmod 777 /dev/ttyACM0','1'], 
        # 运行信息输出到屏幕上
        output='screen' 
    )

    from common import node_params, launch_params, robot_state_publisher, auto_record_node#, armor_tracker_node, buff_tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True,}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                # ComposableNode(
                #     package='detector',
                #     plugin='rm_auto_aim::LightDetectorNode',
                #     name='light_detector',
                #     parameters=[node_params],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'detector:='+launch_params['detector_log_level'],
                            ],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera_ros2_driver', 'hik_camera_ros2_driver::HikCameraRos2DriverNode')
    old_hik_camera_node=get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    if (launch_params['camera'] == 'hik'):
        cam = get_camera_detector_container(hik_camera_node)
    elif (launch_params['camera'] == 'mv'):
        cam = get_camera_detector_container(mv_camera_node)

    hik_camera= Node(
        name="hik_camera_ros2_driver",
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        parameters=[node_params],
        arguments=["--ros-args", "--log-level", 'hik_camera_ros2_driver:='+launch_params['camera_log_level']],
        output="screen",
    )

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    delay_serial_node = TimerAction(
        period=0.5,
        actions=[serial_driver_node],
    )

    detector_node = Node(
    package='detector',
    executable='detector_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    ros_arguments=['--log-level', 'light_detector:='+launch_params['detector_log_level']],
)
    
    tracker_node = Node(
    package='tracker',
    executable='tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    ros_arguments=['--log-level', 'light_tracker:='+launch_params['tracker_log_level']],
    )

    delay_detector_node = TimerAction(
        period=1.0,
        actions=[detector_node],
    )

    delay_tracker_node = TimerAction(
        period=1.5,
        actions=[tracker_node],
    )

    delay_auto_record_node = TimerAction(
        period=2.5,
        actions=[auto_record_node],
    )

    track=Trace(
            session_name='my_trace_session',
            events_kernel=[],
            events_ust=['ros2:*']
        )

    return LaunchDescription([
        # track,
        PushRosNamespace('dart'),
        robot_state_publisher,
        # hik_camera,   #lately low
        cam,            #fps high
        # delay_serial_node,     
        delay_detector_node,
        delay_tracker_node,
        # delay_auto_record_node

    ])
