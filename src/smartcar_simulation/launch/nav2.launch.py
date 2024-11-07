from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    smartcar_sim_path = FindPackageShare('smartcar_simulation')
    # gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_model_path = PathJoinSubstitution([smartcar_sim_path, 'urdf', 'smartcar.urdf'])
    default_rviz_config_path = PathJoinSubstitution([smartcar_sim_path, 'rviz', 'urdf_config.rviz'])
    default_config_config_path = PathJoinSubstitution([smartcar_sim_path, 'config', 'ekf.yaml'])

    default_world_path = PathJoinSubstitution([smartcar_sim_path, 'world', 'smalltown.world'])
    default_map_path = PathJoinSubstitution([smartcar_sim_path, 'map', 'smalltown_world.yaml'])

    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    ld.add_action(DeclareLaunchArgument(name='smartcar_model', default_value=default_model_path,
                                        description='Path to smartcar urdf file relative to urdf_tutorial package'))

    ld.add_action(DeclareLaunchArgument(name='world', default_value=default_world_path,
                                        description='Path to the Gazebo world file'))
    
    ld.add_action(DeclareLaunchArgument(name='ekf_config', default_value=default_config_config_path,
                                        description='Path to the ekf config file'))
    
    ld.add_action(DeclareLaunchArgument(name='map_directory', default_value=default_map_path,
                                        description='Path to the ekf map file'))


    # nav2_bringup_path = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])]
        ),
        launch_arguments={'map': LaunchConfiguration('map_directory')}.items()
    ))

    ld.add_action(Node(
        package="smartcar_simulation",
        executable="Odometry",
        name='odometry',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))
    
    ld.add_action(Node(
        package="smartcar_simulation",
        executable="JointStatePublisher",
        name='JointStatePublisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))
    
    ld.add_action(Node(
        package="robot_localization",
        executable="ekf_node",
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config'), {'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_smartcar',
        output='screen',
        arguments=[LaunchConfiguration('smartcar_model')],
        parameters=[{'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}]
    ))
    
    ld.add_action(DeclareLaunchArgument(name='world', default_value=default_world_path,
                                        description='Path to the Gazebo world file'))

    # Include the gazebo launch description
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={'world': LaunchConfiguration('world'),'extra_args': '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so'}.items()
    ))

    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', 'smartcar', '-file', LaunchConfiguration('smartcar_model')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))
    
    return ld