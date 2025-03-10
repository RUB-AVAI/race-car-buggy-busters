
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    f110_car_pkg_share = get_package_share_directory('f110_car')
    slam_toolbox_config = PathJoinSubstitution([f110_car_pkg_share, "mapper_params_online_async.yaml"])
    print(f110_car_pkg_share)
   
    m2p_node = Node(
            package="f110_car",
            namespace="f110",
            executable="move_to_point",
            name="move_to_point",
            parameters=[{"use_stim_time": True}]

        )
    
    transform_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="transform_pose",
        name="transform_pose",
        parameters=[{"use_stim_time": True}]

    )
    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory("slam_toolbox"),
                                                                "launch", "online_async_launch.py"])),
            launch_arguments={
                "use_sim_time": 'true',
                "slam_params_file": slam_toolbox_config,
                }.items()

            )
    rviz = Node(
            package="rviz2",
            namespace="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{"use_sim_time": True}],
            arguments=["-d", PathJoinSubstitution([f110_car_pkg_share, "rviz_config.rviz"])],
            )
    transforms = GroupAction(
            actions = [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='laser_camera_tf',
                    output='screen',
                    arguments=[
                        '0', '0', '0', '0.0', '0.0', '0.0',
                        'laser', 'camera_link'
                        ],
                    parameters=[{'use_sim_time': True}],
                    ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_scan_tf',
                    output='screen',
                    arguments=[
                        '0', '0', '0', '0.0', '0.0', '0.0',
                        'base_link', 'laser'
                       ],
                    parameters=[{'use_sim_time': True}],
                    ),
                ])
    return LaunchDescription([
        #m2p_node,
        #transforms,
        #transform_node,
        slam_launch,
        rviz
    ])
