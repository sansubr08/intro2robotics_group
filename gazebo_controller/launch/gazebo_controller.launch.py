import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_controller').find('gazebo_controller')
    bridge_yaml = os.path.join(pkg_share,'config','ROS_to_GAZEBO.yaml')
    sdf_file = os.path.join(pkg_share,'models','basic_robot','model.sdf')
    world_file = os.path.join(pkg_share, 'world','empty_world.sdf')
    
    with open(sdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',os.path.join(pkg_share,'models')),
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),
        RosGzBridge(
            bridge_name='bridge',
            config_file=bridge_yaml
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name = 'robot_state_publisher',
            output='both',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True}]
        ),
        Node(
            package='gazebo_controller',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])