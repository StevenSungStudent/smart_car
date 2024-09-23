from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_sim_path = FindPackageShare('robot_arm_sim')
    default_robot_model_path = PathJoinSubstitution([robot_sim_path, 'urdf', 'lynxmotion_arm.urdf'])
    default_cup_model_path = PathJoinSubstitution([robot_sim_path, 'urdf', 'cup.urdf'])
    default_rviz_config_path = PathJoinSubstitution([robot_sim_path,'rviz', 'urdf_config.rviz'])

    ld.add_action(DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui'))
    
    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file'))

    ld.add_action(DeclareLaunchArgument(name='robot_arm_model', default_value=default_robot_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))
    
    ld.add_action(DeclareLaunchArgument(name='cup_model', default_value=default_cup_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_robotarm',
		output='screen',
		arguments=[LaunchConfiguration('robot_arm_model')]        
	))

    ld.add_action(Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_robotarm',
		output='screen',
		arguments=[LaunchConfiguration('cup_model')],
        remappings=[('/robot_description', '/cup_description')]
	))


    ld.add_action(Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', LaunchConfiguration('rvizconfig')]
	))

    ld.add_action(Node(
        package="robot_arm_sim",
        executable="robot_arm_sim",
        name='robot_arm_sim',
        output='screen',
        parameters=[]
    ))

    ld.add_action(Node(
        package="robot_arm_sim",
        executable="cup_pose_publisher",
        name='cup_pose_publisher',
        output='screen',
        parameters=[]
    ))
    

    return ld
