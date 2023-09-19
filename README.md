# Turtlebot3 Lane Following and Navigation Capstone

We achieved a multifunctional and integrated system that includes autonomous navigation, lane following and Aruco detection using TurtleBot3 Burger. Our system is designed for the competition of finishing a specified task sequence.

For more details, please check our project ⭐[website](https://sites.google.com/view/ee346-capstone-22s-cmdzyl/home)!

<img src="https://github.com/silvery107/ee346-capstone-control/assets/44640904/125b46d0-f42e-44a4-899e-8aafb4d5f622" width="450"/>

<img src="https://github.com/silvery107/ee346-capstone-control/assets/44640904/480fc79c-ae49-4237-aa38-5725d4918bd1" width="350"/>

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

    First you need to generate a corresponding map in the $HOME
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


