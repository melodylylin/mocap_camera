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

calib_info_dir = get_package_share_directory('visnet_calib') + '/camera_info'

########################
cam_name = "camera3"
########################
R_cv = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0, -1,  0]
])
with open(f"{calib_info_dir}/{cam_name}_mocap_calib.json") as fs:
    calib_info = json.load(fs)
    R = np.array(calib_info['R'])
    R = R_cv @ R
    c = np.array(calib_info['c'])
    q = SO3.to_quat(R)
    print(c, q)

params_path = f"{get_package_share_directory('mocap_camera')}/config/{cam_name}_param.yaml"

def generate_launch_description():
    return LaunchDescription([
        # SetEnvironmentVariable(name='GSCAM_CONFIG', value="v4l2src device=/dev/video0 ! image/jpeg,width=1600,height=1200,framerate=30/1 ! jpegdec ! videoconvert"),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('visnet_calib') + '/camera_info/camera0.yaml'),
            
        Node(
            package='usb_cam',
            name=f'{cam_name}_node',
            executable='usb_cam_node_exe',
            namespace=f'{cam_name}',
            parameters=[params_path],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', str(c[0]), '--y', str(c[1]), '--z', str(c[2]), 
                '--qw', str(q[0]), '--qx', str(q[1]), '--qy', str(q[2]), '--qz', str(q[3]),
                '--frame-id', f'{cam_name}', '--child-frame-id', f'{cam_name}/calibrated'
            ]
        ),
    ])

