"""
Spawn Robot Description
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    test_robot_description_share = FindPackageShare(package='robot_urdf').find('robot_urdf')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot4.xacro')
    world5Box_path = os.path.join(test_robot_description_share, 'worlds/newWorld_5.world')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    camera_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['camera_joint_z_axis_controller'],
	)
	
    aruco_node = Node(
    	package='ros2_aruco',
	executable='aruco_node_moving_robot',
	name ='aruco_node_moving_robot'
    )	
    
    moving_robot = Node(
    	package='ros2_aruco',
    	executable = 'moving_robot',
    	name = 'moving_robot'
    
    )


    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description'],
                        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        camera_controller,
        moving_robot,
        aruco_node,
        ExecuteProcess(
            cmd=['gazebo', '--verbose',world5Box_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'),
    ])
