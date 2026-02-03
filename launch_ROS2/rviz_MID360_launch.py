import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'
use_ros_time  = True  # True-Use ROS system time, False-Use sensor hardware time

# Point cloud filter parameters
# Azimuth (horizontal angle): -180 to 180 deg, 0=front(+X), 90=left(+Y), -90=right(-Y)
# Elevation (vertical angle): -90 to 90 deg, 0=horizontal, positive=up
# filter_azimuth_min   = -180.0  # horizontal angle min (deg)
# filter_azimuth_max   = 180.0   # horizontal angle max (deg)
filter_azimuth_min   = -120.0  # horizontal angle min (deg)
filter_azimuth_max   = 120.0   # horizontal angle max (deg)
filter_elevation_min = -90.0   # vertical angle min (deg)
filter_elevation_max = 90.0    # vertical angle max (deg)
filter_dist_min      = 0.0     # minimum distance (m)
filter_dist_max      = 100.0   # maximum distance (m)


cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"use_ros_time": use_ros_time},
    # Point cloud filter parameters
    {"filter_azimuth_min": filter_azimuth_min},
    {"filter_azimuth_max": filter_azimuth_max},
    {"filter_elevation_min": filter_elevation_min},
    {"filter_elevation_max": filter_elevation_max},
    {"filter_dist_min": filter_dist_min},
    {"filter_dist_max": filter_dist_max}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    #livox_rviz = Node(
    #        package='rviz2',
    #        executable='rviz2',
    #        output='screen',
    #        arguments=['--display-config', rviz_config_path]
    #    )

    return LaunchDescription([
        livox_driver,
        #livox_rviz,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
