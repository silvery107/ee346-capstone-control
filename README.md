# EE346 Capstone Control

## Lane Following

1. Remote PC
    ```
    roscore
    ```
2. SBC
    ```bash
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
    ```

3. Remote PC
    ```bash
    export AUTO_IN_CALIB=action
    export GAZEBO_MODE=false
    roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
    ```

   - View calibrated images
       ```
       rqt_image_view
       ```

   - Change ISO if the frame is shaking

       ```
       rosrun rqt_reconfigure rqt_reconfigure
       ```

4. Run the lane follower node
    ```bash
    cd ~/catkin_ws/src/EE346-Lab6/src
    python racetrack_control.py
    ```
## Autonomous Navigation
1. Remote PC
    ```
    roscore
    ```

2. SBC
    ```bash
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```

3. Remote PC
    ```bash
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_lab.yaml
    python navigation_control.py
    ```

## Integrated System
1. Remote PC
    ```
    roscore
    ```

1. SBC
    ```bash
    sudo /etc/init.d/ntp restart
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
    ```

2. Remote PC
    ```bash
    sudo /etc/init.d/ntp restart
    roslaunch ee346-capstone-control capstone_start.launch
    python cmd_sys.py
    ```


