#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for cylinder landing mission."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
        
    # Set Gazebo model and resource paths
    gz_model_path = os.path.join(pkg_share, 'models')

    # # Set initial drone pose
    os.environ['PX4_GZ_MODEL_POSE'] = "0,0,0.1,0,0,0"
    
    # Add launch argument for PX4-Autopilot path
    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')
    
    # Launch PX4 SITL with x500_depth
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth_mono'],
        cwd=px4_autopilot_path,
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_00 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_00',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '-5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_01 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_01',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '-10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_02 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_02',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_03 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_03',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_04 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_04',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_10 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_10',
            '-x', '10',     # 5 meters in front of the drone
            '-y', '-5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_11 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_11',
            '-x', '10',     # 5 meters in front of the drone
            '-y', '-10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_12 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_12',
            '-x', '10',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_13 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_13',
            '-x', '10',     # 5 meters in front of the drone
            '-y', '5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_14 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_14',
            '-x', '10',     # 5 meters in front of the drone
            '-y', '10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_20 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_20',
            '-x', '15',     # 5 meters in front of the drone
            '-y', '-5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_21 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_21',
            '-x', '15',     # 5 meters in front of the drone
            '-y', '-10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_22 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_22',
            '-x', '15',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_23 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_23',
            '-x', '15',     # 5 meters in front of the drone
            '-y', '5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_24 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_24',
            '-x', '15',     # 5 meters in front of the drone
            '-y', '10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_30 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_30',
            '-x', '20',     # 5 meters in front of the drone
            '-y', '-5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_31 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_31',
            '-x', '20',     # 5 meters in front of the drone
            '-y', '-10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_32 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_32',
            '-x', '20',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_33 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_33',
            '-x', '20',     # 5 meters in front of the drone
            '-y', '5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_34 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_34',
            '-x', '20',     # 5 meters in front of the drone
            '-y', '10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_40 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_40',
            '-x', '25',     # 5 meters in front of the drone
            '-y', '-5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_41 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_41',
            '-x', '25',     # 5 meters in front of the drone
            '-y', '-10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_42 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_42',
            '-x', '25',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_43 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_43',
            '-x', '25',     # 5 meters in front of the drone
            '-y', '5',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front_44 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_front_44',
            '-x', '25',     # 5 meters in front of the drone
            '-y', '10',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )
 


       
    # Bridge node for camera and odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            # Front RGB Camera
            '/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Front Depth Camera
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Down Mono Camera
            '/mono_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Clock and Odometry
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # '/model/x500_depth_mono_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
        ],
        remappings=[
            # Front RGB Camera remappings
            ('/rgb_camera', '/drone/front_rgb'),
            ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            
            # Front Depth Camera remappings
            ('/depth_camera', '/drone/front_depth'),
            # ('/depth_camera/depth_image', '/drone/front_depth/depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/front_depth/camera_info'),
            
            # Down Mono Camera remappings
            ('/mono_camera', '/drone/down_mono'),
            ('/mono_camera/camera_info', '/drone/down_mono/camera_info'),
            
            # Odometry remapping
            # ('/model/x500_depth_mono_0/odometry_with_covariance', '/fmu/out/vehicle_odometry'),
        ],
        output='screen'
    )

    # Static transform for OakD-Lite camera
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_d_lite_tf',
        arguments=['0', '0', '0', '0', '0', '-1.570', 'base_link', 'OakD-Lite-Modify/base_link'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
        px4_sitl,
        camera_transform,
        TimerAction(
            period=2.0,
            actions=[spawn_cylinder_front_00]
        ),
        TimerAction(
            period=2.5,
            actions=[spawn_cylinder_front_01]
        ),
        TimerAction(
            period=3.0,
            actions=[spawn_cylinder_front_02]
        ),
        TimerAction(
            period=3.5,
            actions=[spawn_cylinder_front_03]
        ),
        TimerAction(
            period=4.0,
            actions=[spawn_cylinder_front_04]
        ),
        TimerAction(
            period=4.5,
            actions=[spawn_cylinder_front_10]
        ),
        TimerAction(
            period=5.0,
            actions=[spawn_cylinder_front_11]
        ),
        TimerAction(
            period=6.0,
            actions=[spawn_cylinder_front_12]
        ),
        TimerAction(
            period=7.0,
            actions=[spawn_cylinder_front_13]
        ),
        TimerAction(
            period=7.5,
            actions=[spawn_cylinder_front_14]
        ),
        TimerAction(
            period=8.0,
            actions=[spawn_cylinder_front_20]
        ),
        TimerAction(
            period=8.5,
            actions=[spawn_cylinder_front_21]
        ),
        TimerAction(
            period=9.0,
            actions=[spawn_cylinder_front_22]
        ),
        TimerAction(
            period=9.5,
            actions=[spawn_cylinder_front_23]
        ),
        TimerAction(
            period=10.0,
            actions=[spawn_cylinder_front_24]
        ),
        TimerAction(
            period=10.5,
            actions=[spawn_cylinder_front_30]
        ),
        TimerAction(
            period=11.0,
            actions=[spawn_cylinder_front_31]
        ),
        TimerAction(
            period=11.5,
            actions=[spawn_cylinder_front_32]
        ),
        TimerAction(
            period=12.0,
            actions=[spawn_cylinder_front_33]
        ),
        TimerAction(
            period=12.5,
            actions=[spawn_cylinder_front_34]
        ),
        TimerAction(
            period=13.0,
            actions=[spawn_cylinder_front_40]
        ),
        TimerAction(
            period=13.5,
            actions=[spawn_cylinder_front_41]
        ),
        TimerAction(
            period=14.0,
            actions=[spawn_cylinder_front_42]
        ),
        TimerAction(
            period=14.5,
            actions=[spawn_cylinder_front_43]
        ),
        TimerAction(
            period=15.0,
            actions=[spawn_cylinder_front_44]
        ),
        # TimerAction(
        #     period=2.5,
        #     actions=[spawn_cylinder_back]
        # ),
        TimerAction(
            period=3.0,
            actions=[bridge]
        )
    ]) 