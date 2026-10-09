# gazebo_controller

Diff-drive robot (midterm dimensions) in a Gazebo maze, bridged to ROS 2, driven to RViz goals by the midterm PD controller.

## Build
    mkdir -p ~/gz_ws/src && cp -r gazebo_controller ~/gz_ws/src/
    cd ~/gz_ws && colcon build --packages-select gazebo_controller && source install/setup.bash

## Run
    ros2 launch gazebo_controller maze_pid.launch.py
Then in RViz click **2D Goal Pose**. Waypoints that stay clear of the walls (world == odom frame):
(6,-1) -> (-1,0) -> (-1,3) -> (7,3) -> (7,7) -> (0,7)

## Teleop check (task before the PID part)
    ros2 launch gazebo_controller maze_pid.launch.py run_pid:=false
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Quick debugging
    gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}"   # Gazebo alone, no ROS
    ros2 topic echo /odom --once          # bridge works?
    ros2 run tf2_ros tf2_echo odom chassis # TF works?
