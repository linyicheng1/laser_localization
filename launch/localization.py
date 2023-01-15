from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_localization',
            node_executable='laser_localization',
            node_name='laser_localization',
            parameters=[
                {"localization/mode": 2},
                {"bbs/max_range": 15.0},
                {"bbs/min_tx": -50.0},
                {"bbs/max_tx": 50.0},
                {"bbs/min_ty": -50.0},
                {"bbs/max_ty": 50.0},
                {"bbs/min_theta": -3.14},
                {"bbs/max_theta": 3.14},
                {"bbs/map_min_z": -0.2},
                {"bbs/map_max_z": 1.2},
                {"bbs/map_width": 512},
                {"bbs/map_height": 512},
                {"bbs/map_resolution": 0.5},
                {"bbs/max_points_pre_cell": 5},
                {"bbs/map_pyramid_level": 6},
                {"bbs/scan_min_z": -0.2},
                {"bbs/scan_max_z": 1.2},
                {"bbs/map_filter_resolution": 0.5},
                {"bbs/scan_filter_resolution": 0.5},
                {"global_map_width": 100.0},
                {"global_map_height": 100.0},
                {"ndt/resolution": 0.8},
                {"ndt/step_size": 0.2},
                {"ndt/epsilon": 0.01},
                {"ndt/max_iterations": 30.0},
                {"ndt/frame_resolution": 0.5},
                {"odom/kf_distance": 2.0},
                {"odom/local_map_size": 10},
                {"ndt/local_map_resolution": 0.5},
                {"global_map": "../map/map.pcd"},
                {"odom_frame": "odom"},
                {"base_link_frame": "base_link"},
                {"laser_frame": "os_sensor"},
                {"laser_topic": "/points"},
                {"pose_save": "../map/pos.txt"},
                {"correct_count": 5},
                {"global_resolution": 1.3},
                {"global_frame_resolution": 1},
                {"global_view_resolution": 3},
                {"local_map_size": 100},
                {"localization/ndt_resolution": 1},
                {"localization/ndt_step_size": 0.1},
                {"localization/ndt_epsilon": 0.01},
                {"initial_pose_save": "../map/init_pos.txt"},
            ]
        ),
    ])