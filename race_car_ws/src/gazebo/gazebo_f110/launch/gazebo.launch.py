from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                            IncludeLaunchDescription, SetLaunchConfiguration, TimerAction)
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    pkg_gazebo_f110 = get_package_share_directory('gazebo_f110')
    slam_toolbox_config = PathJoinSubstitution([pkg_gazebo_f110, "mapper_params_online_async.yaml"])
    
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    start_rviz = LaunchConfiguration("start_rviz", default=True)
    
    wasd_node = Node(
            package="test_package",
            namespace="f110",
            executable="wasd_control_node",
            name="wasd_control",
            parameters=[{'use_sim_time': True}],
            )
    move_to_point = Node(
            package="f110_car",
            namespace="f110",
            executable="move_to_point",
            name="move_to_point",
            parameters=[{'use_sim_time': True}],
            )
    exploration_node = Node(
            package="f110_car",
            namespace="f110",
            executable="exploration_node",
            name="exploration_node",
            parameters=[{'use_sim_time': True}],
            )
    exploration_vis_node = Node(
            package="f110_car",
            namespace="f110",
            executable="exploration_vis_node",
            name="exploration_vis_node",
            parameters=[{'use_sim_time': True}],
            )
    yolo_node = Node(
            package="test_package",
            namespace="f110",
            executable="yolo_node",
            name="yolo_node",
            parameters=[{'use_sim_time': True}],
            )
    semantic_mapping_node = Node(
            package="test_package",
            namespace="f110",
            executable="semantic_mapping_node",
            name="semantic_mapping_node",
            parameters=[{'use_sim_time': True}],
            )
    cone_marker_node = Node(
            package="test_package",
            namespace="f110",
            executable="cone_marker_node",
            name="cone_marker_node",
            parameters=[{'use_sim_time': True}],
            )
    semantic_grid_visualizer_node = Node(
            package="test_package",
            namespace="f110",
            executable="semantic_grid_visualizer_node",
            name="semantic_grid_visualizer_node",
            parameters=[{'use_sim_time': True}],
            )
    ackermann_to_twist_node = Node(
            package="gazebo_f110",
            namespace="gazebo",
            executable="ackermann_to_twist",
            name="ackermann_to_twist",
            parameters=[{'use_sim_time': True}],
            )
    transform_node = Node(
        package="gazebo_f110",
        namespace="gazebo",
        executable="transform_pose",
        name="transform_pose",
        parameters=[{'use_sim_time': True}],
            )
    track_polygon_node = Node(
            package="f110_car",
            namespace="f110",
            executable="track_polygon_node",
            name="track_polygon_node",
            parameters=[{'use_sim_time': True}],
            )

    rviz = Node(
            package="rviz2",
            namespace="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg_gazebo_f110, "rviz_config.rviz"])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(start_rviz)
            )
    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory("slam_toolbox"),
                                                                "launch", "online_async_launch.py"])),
            launch_arguments={
                "slam_params_file": slam_toolbox_config,
                "use_sim_time": "true"
                }.items()
            )

    ros_gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist", 
                "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
                "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
                "/model/base_link/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                "/world/car_world/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V",
                "/model/base_link/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                "/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                ],
            remappings=[
                ("/rgbd_camera/camera_info", "/camera/realsense2_camera/color/camera_info"),
                ("/rgbd_camera/image", "/camera/realsense2_camera/color/image_raw"),
                ("/rgbd_camera/depth_image", "/camera/realsense2_camera/depth/image_rect_raw"),
                ("/model/base_link/odometry", "/odom"),
                ("/model/base_link/tf", "/tf"),
                ],
            output='screen',
            parameters=[{'use_sim_time': True}],
            )

    gazebo_launch_group = GroupAction(
            actions = [
                DeclareLaunchArgument(
                    'world',
                    default_value='plane',
                    choices=['plane', 'circle'],
                    description='World to load into Gazebo'
                    ),
                SetLaunchConfiguration(name='world_file', 
                                       value=[LaunchConfiguration('world'), 
                                              TextSubstitution(text='.sdf')]),
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource(gz_launch_path),
                   launch_arguments={
                       'ign_args': [PathJoinSubstitution([pkg_gazebo_f110, "world", LaunchConfiguration('world_file')])],
                       'on_exit_shutdown': 'True',
                       'use_sim_time': 'True'
                       }.items(),
                   )
               ]
            )
    #Kind of hacky way to make sure that this transform is the latest transform (after the car model transforms) and the camera->base_link coordinate transform works
    static_transform_publisher = TimerAction(
        period=5.0,
        actions=[Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_tf',
                    arguments=[
                        '0.12', '0', '0.155',
                        '-1.57', '0', '-1.57',
                        'base_link', 'camera_link'
                    ],
                    parameters=[{'use_sim_time': True}],
                )]
    )
    transforms = GroupAction(
            actions = [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='gpu_lidar_tf',
                    output='screen',
                    arguments=[
                        '0', '0', '0', '0.0', '0.0', '0.0',
                        'lidar_link', 'gpu_lidar'
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
                        'map', 'scan'
                       ],
                    parameters=[{'use_sim_time': True}],
                    ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_base_link',
                    output='screen',
                    arguments=[
                        '0', '0', '0', '0.0', '0.0', '0.0',
                        'map', 'odom'
                       ],
                    parameters=[{'use_sim_time': True}],
                    ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom_base_link',
                    output='screen',
                    arguments=[
                        '0', '0', '0', '0.0', '0.0', '0.0',
                        'odom', 'base_link'
                       ],
                    parameters=[{'use_sim_time': True}],
                    ),
                ])
    robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             arguments=[PathJoinSubstitution([pkg_gazebo_f110, "model", "car", "f110_car.sdf"])])
    return LaunchDescription([
        gazebo_launch_group,
        move_to_point,
        exploration_node,
        exploration_vis_node,
        #wasd_node,
        yolo_node,
        cone_marker_node,
        semantic_mapping_node,
        semantic_grid_visualizer_node,
        transforms,
        transform_node,
        ackermann_to_twist_node,
        ros_gz_bridge_node,
        slam_launch,
        track_polygon_node,
        rviz,
        robot_state_publisher,
        static_transform_publisher
        ])
