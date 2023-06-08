from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

from datetime import date, datetime
date_str = f"{date.today().strftime('%Y-%m-%d')}-{datetime.now().time().strftime('%I%M%S')}"

def cam_remap(i):
    return[
        (f'/camera{i}/camera/camera_info', f'/camera{i}/camera_info'),
        (f'/camera{i}/camera/image_raw', f'/camera{i}/image'),
        (f'/camera{i}/camera/image_raw/compressed', f'/camera{i}/image/compressed'),
        (f'/camera{i}/camera/image_raw/compressedDepth', f'/camera{i}/image/compressedDepth'),
        (f'/camera{i}/camera/image_raw/theora', f'/camera{i}/image/theora'),
    ]

def generate_launch_description():
    return LaunchDescription([
        # SetEnvironmentVariable(name='GSCAM_CONFIG', value="v4l2src device=/dev/video0 ! image/jpeg,width=1600,height=1200,framerate=30/1 ! jpegdec ! videoconvert"),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('visnet_calib') + '/camera_info/camera0.yaml'),
            
        Node(
            package='gscam',
            executable='gscam_node',
            namespace='camera0',
            parameters=[
                {'gscam_config': 'v4l2src device=/dev/video0 ! image/jpeg,width=1600,height=1200,framerate=30/1 ! jpegdec ! videoconvert'},
                {'camera_info_url': 'package://visnet_calib/config/camera0.yaml'},
                {'frame_id': 'camera0'},
                {'sync_sink': False},
                {'use_gst_timestamps': True},
            ],
            remappings=cam_remap(0)
        ),

        Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', get_package_share_directory('visnet_calib') + '/config/camera_calib.rviz']
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
        #         '/camera_0/image/compressed',
        #         '/camera_1/image/compressed',
        #         '/camera_2/image/compressed',
        #         '/camera_3/image/compressed',
        #         '/camera_0/pose',
        #         '/camera_1/pose',
        #         '/camera_2/pose',
        #         '/camera_3/pose',
        #         '/uae05/pose',
        #         ],
        #     output='screen'
        # )
    ])

