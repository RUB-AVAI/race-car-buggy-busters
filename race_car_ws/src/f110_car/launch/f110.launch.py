
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    f110_car_pkg_share = get_package_share_directory('f110_car')
    slam_toolbox_config = PathJoinSubstitution([f110_car_pkg_share, "mapper_params_online_async.yaml"])

    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    start_rviz = LaunchConfiguration("start_rviz", default=False)
   
    m2p_node = Node(
            package="f110_car",
            namespace="f110",
            executable="move_to_point",
            name="move_to_point",
            parameters=[{"use_stim_time": use_sim_time}]

        )
    exploration_node = Node(
            package="f110_car",
            namespace="f110",
            executable="exploration_node",
            name="exploration_node",
            parameters=[{'use_sim_time': use_sim_time}],
            )
    exploration_vis_node = Node(
            package="f110_car",
            namespace="f110",
            executable="exploration_vis_node",
            name="exploration_vis_node",
            parameters=[{'use_sim_time': use_sim_time}],
            )
    yolo_node = Node(
            package="test_package",
            namespace="f110",
            executable="yolo_node",
            name="yolo_node",
            parameters=[{'use_sim_time': use_sim_time}],
            )
    semantic_mapping_node = Node(
            package="test_package",
            namespace="f110",
            executable="semantic_mapping_node",
            name="semantic_mapping_node",
            parameters=[{'use_sim_time': use_sim_time}],
            )
    cone_marker_node = Node(
            package="test_package",
            namespace="f110",
            executable="cone_marker_node",
            name="cone_marker_node",
            parameters=[{'use_sim_time': use_sim_time}],
            )
    semantic_grid_visualizer_node = Node(
            package="test_package",
            namespace="f110",
            executable="semantic_grid_visualizer_node",
            name="semantic_grid_visualizer_node",
            parameters=[{'use_sim_time': use_sim_time}],
            )
    
    transform_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="transform_pose",
        name="transform_pose",
        parameters=[{"use_stim_time": use_sim_time}]

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
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", PathJoinSubstitution([f110_car_pkg_share, "rviz_config.rviz"])],
            condition=IfCondition(start_rviz)
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
                    parameters=[{'use_sim_time': use_sim_time}],
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
                    parameters=[{'use_sim_time': use_sim_time}],
                    ),
                ])
    return LaunchDescription([
        m2p_node,
        transforms,
        exploration_node,
        exploration_vis_node,
        yolo_node,
        cone_marker_node,
        semantic_mapping_node,
        semantic_grid_visualizer_node,
        #transform_node,
        slam_launch,
        rviz
    ])
