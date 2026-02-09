import os
from launch import LaunchDescription
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 1-One LiDAR one topic (required for dual lidar)
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'  # base frame (will be changed by relay)
use_ros_time  = True

# Point cloud filter parameters
filter_azimuth_min   = -120.0  # horizontal angle min (deg)
filter_azimuth_max   = 120.0   # horizontal angle max (deg)
filter_elevation_min = -90.0   # vertical angle min (deg)
filter_elevation_max = 90.0    # vertical angle max (deg)
filter_dist_min      = 0.0     # minimum distance (m)
filter_dist_max      = 100.0   # maximum distance (m)

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_dual_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"user_config_path": user_config_path},
    {"use_ros_time": use_ros_time},
    {"filter_azimuth_min": filter_azimuth_min},
    {"filter_azimuth_max": filter_azimuth_max},
    {"filter_elevation_min": filter_elevation_min},
    {"filter_elevation_max": filter_elevation_max},
    {"filter_dist_min": filter_dist_min},
    {"filter_dist_max": filter_dist_max}
]


def generate_launch_description():
    # Main livox driver (handles both lidars)
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # Relay: 192.168.0.100 -> lidar_front with frame_id change
    relay_front = Node(
        package='livox_ros_driver2',
        executable='pointcloud_relay.py',
        name='relay_lidar_front',
        output='screen',
        parameters=[
            {'input_topic': '/livox/lidar_192_168_0_100'},
            {'output_topic': '/livox/lidar_front'},
            {'frame_id': 'livox_frame_front'},
            {'msg_type': 'pointcloud'}
        ]
    )

    # Relay: 192.168.0.109 -> lidar_rear with frame_id change
    relay_rear = Node(
        package='livox_ros_driver2',
        executable='pointcloud_relay.py',
        name='relay_lidar_rear',
        output='screen',
        parameters=[
            {'input_topic': '/livox/lidar_192_168_0_109'},
            {'output_topic': '/livox/lidar_rear'},
            {'frame_id': 'livox_frame_rear'},
            {'msg_type': 'pointcloud'}
        ]
    )

    # Relay IMU: 192.168.0.100 -> imu_front
    relay_imu_front = Node(
        package='livox_ros_driver2',
        executable='pointcloud_relay.py',
        name='relay_imu_front',
        output='screen',
        parameters=[
            {'input_topic': '/livox/imu_192_168_0_100'},
            {'output_topic': '/livox/imu_front'},
            {'frame_id': 'livox_frame_front'},
            {'msg_type': 'imu'}
        ]
    )

    # Relay IMU: 192.168.0.109 -> imu_rear
    relay_imu_rear = Node(
        package='livox_ros_driver2',
        executable='pointcloud_relay.py',
        name='relay_imu_rear',
        output='screen',
        parameters=[
            {'input_topic': '/livox/imu_192_168_0_109'},
            {'output_topic': '/livox/imu_rear'},
            {'frame_id': 'livox_frame_rear'},
            {'msg_type': 'imu'}
        ]
    )

    return LaunchDescription([
        livox_driver,
        relay_front,
        relay_rear,
        relay_imu_front,
        relay_imu_rear
    ])
