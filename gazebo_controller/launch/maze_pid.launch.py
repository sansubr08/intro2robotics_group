"""Launch Gazebo (maze world + robot), the ROS<->Gazebo bridge, the PID controller, and RViz.

    ros2 launch gazebo_controller maze_pid.launch.py
    ros2 launch gazebo_controller maze_pid.launch.py use_rviz:=false
    ros2 launch gazebo_controller maze_pid.launch.py teleop:=true   (then run teleop in another terminal)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gazebo_controller')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg, 'worlds', 'maze.sdf')
    urdf = os.path.join(pkg, 'urdf', 'basic_robot.urdf')
    rviz_cfg = os.path.join(pkg, 'rviz', 'gazebo_controller.rviz')
    with open(urdf, 'r') as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration('use_rviz')
    run_pid = LaunchConfiguration('run_pid')

    # Lets the world's <include><uri>model://diff_robot</uri> find our model.
    model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),   # -r: start running immediately
    )

    # Direction: '[' = Gazebo -> ROS, ']' = ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/maze_world/model/diff_robot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/world/maze_world/model/diff_robot/joint_state', '/joint_states')],
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    pid = Node(
        package='gazebo_controller', executable='pid_controller', name='pid_controller',
        output='screen', parameters=[{'use_sim_time': True}], condition=IfCondition(run_pid),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], output='screen',
        parameters=[{'use_sim_time': True}], condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_pid', default_value='true',
                              description='set false to drive with teleop_twist_keyboard instead'),
        model_path, gz_sim, bridge, rsp, pid, rviz,
    ])
