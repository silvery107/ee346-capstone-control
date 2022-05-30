# EE346 Capstone Control

## Part I Lane Following

### TODO add timer and out of time exit, initialization func
1. Launch roscore on Remote PC.
    ```
    roscore
    ```
2. Trigger the camera and bringup the turtlebot on SBC.
    ```bash
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
    ```

3. Run a intrinsic camera calibration launch file on Remote PC.
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
## Part II Navigation

1. SBC
    ```bash
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```
2. PC
    ```bash
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_lab.yaml
    python navigation_control.py
    ```
