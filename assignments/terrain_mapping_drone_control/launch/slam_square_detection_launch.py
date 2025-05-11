from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('ir_slam_square_detection')
    
    rtabmap_slam_executable = FindExecutable(name='rtabmap_slam')
    
    gazebo_world = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
             PathJoinSubstitution([pkg_share, 'models', 'x500_ir', 'model.sdf'])],
        output='screen'
    )
    
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap_slam',
        name='rtabmap_slam',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'rtabmap.yaml'])],
        output='screen',
        arguments=['-d'],
        condition=UnlessCondition(Command(['test -x ', rtabmap_slam_executable, ' && echo true || echo false']))
    )
    
    rtabmap_error_log = LogInfo(
        msg="ERROR: rtabmap_slam executable not found. Please install ros-humble-rtabmap-ros or build rtabmap_ros from source.",
        condition=UnlessCondition(Command(['test -x ', rtabmap_slam_executable, ' && echo true || echo false']))
    )
    
    square_detector_node = Node(
        package='ir_slam_square_detection',
        executable='square_detector',
        name='square_detector',
        output='screen'
    )
    
    return LaunchDescription([
        LogInfo(msg="Starting infrared SLAM and square detection..."),
        gazebo_world,
        rtabmap_node,
        rtabmap_error_log,
        square_detector_node
    ])