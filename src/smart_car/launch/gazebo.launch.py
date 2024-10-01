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

    smartcar_sim_path = FindPackageShare('smart_car')
    # gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_model_path = PathJoinSubstitution([smartcar_sim_path, 'urdf', 'smartcar.urdf'])
    default_rviz_config_path = PathJoinSubstitution([smartcar_sim_path, 'rviz', 'urdf_config.rviz'])

    default_world_path = PathJoinSubstitution([smartcar_sim_path, 'world', 'smalltown.world'])

    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    ld.add_action(DeclareLaunchArgument(name='smartcar_model', default_value=default_model_path,
                                        description='Path to smartcar urdf file relative to urdf_tutorial package'))

    ld.add_action(DeclareLaunchArgument(name='world', default_value=default_world_path,
                                        description='Path to the Gazebo world file'))



    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_smartcar',
        output='screen',
        arguments=[LaunchConfiguration('smartcar_model')]
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    ))

    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
    #     ),
    #     launch_arguments={'world': LaunchConfiguration('world')}.items()
    # ))
    
    # # Optionally, provide a world file (can be empty world or any custom world)
    # world_path = os.path.join(gazebo_ros_share, 'worlds', 'empty.world')
    
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
        output='screen'
    ))

    return ld