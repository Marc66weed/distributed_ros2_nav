from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        # 【核彈級靜音】這兩行會強制殺掉所有 WARN 和 INFO，只留 ERROR
        AppendEnvironmentVariable('RCUTILS_LOGGING_MIN_SEVERITY_LEVEL', 'ERROR'),
        AppendEnvironmentVariable('ROS_LOG_LEVEL', 'ERROR'),

        GroupAction(
            actions=[
                #SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                    ]),
                    launch_arguments={
                        'use_sim_time': 'True',
                        'map': '/home/marc4/maps/my_map.yaml',
                        'params_file': '/home/marc4/test_nav.yaml',
                        #'params_file': '/home/marc4/my_nav_jazzy.yaml',
                        'use_rviz': 'False',
                        'use_composition': 'False',
                        'use_respawn': 'True',
                        'autostart': 'True',
                    }.items()
                )
            ]
        )
    ])
