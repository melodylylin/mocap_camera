from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

from datetime import date, datetime
date_str = f"{date.today().strftime('%Y-%m-%d')}-{datetime.now().time().strftime('%I%M%S')}"
from lienp.SO3 import SO3
import numpy as np
import json

calib_info_dir = get_package_share_directory('mocap_camera') + '/camera_info'

########################
cam_name = "camera0"
########################


params_path = f"{get_package_share_directory('mocap_camera')}/config/{cam_name}_param.yaml"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            name=f'{cam_name}_node',
            executable='usb_cam_node_exe',
            namespace=f'{cam_name}',
            parameters=[params_path],
        ),

        Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', get_package_share_directory('mocap_camera') + '/config/camera_calib.rviz']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'qualisys']
        ),

        Node(
            package='qualisys_mocap',
            executable='qualisys_node',
            parameters=[
                {'server': '192.168.123.2'},
            ]
        )
        
        ###################################################
        #### uncomment the block blew to record rosbag ####
        ###################################################
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'bag', 'record',
        #         '-o', f'rosbags/multicam-{date_str}',
        #         '/camera0/image_raw/compressed',
        #         '/camera1/image_raw/compressed',
        #         '/camera2/image_raw/compressed',
        #         '/camera3/image_raw/compressed',
        #         '/camera0/pose',
        #         '/camera1/pose',
        #         '/camera2/pose',
        #         '/camera3/pose',
        #         '/drone1/pose',
        #         ],
        #     output='screen'
        # )
    ])

