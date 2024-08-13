import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nav_project')
    default_model_path = os.path.join(pkg_share, 'models/model.sdf')
    world_path = os.path.join(pkg_share, 'worlds/world4.world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    declare_model_argument = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to robot sdf file'
    )

    # gzserver_cmd = ExecuteProcess(
    #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
    #     output='screen'
    # )

    spawn_robot_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        output='screen',
        arguments=['-file', LaunchConfiguration('model'), '-entity', 'my_robot']
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    LDs = LaunchDescription()

    LDs.add_action(declare_model_argument)
    LDs.add_action(gzserver_cmd)
    LDs.add_action(gzclient_cmd)
    LDs.add_action(spawn_robot_cmd)


    return LDs
            
    