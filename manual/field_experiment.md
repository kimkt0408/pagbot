## Field experiment manual

### 1. Materials to bring for experiments

* Jackal
* Fully-charged battery
* Ethernet cable
* Jackal router
* Extension cord
* Recording & Ground truth measurements: ruler, tape, camera tripod, smartphone

### 2. Scenario

* Construct the experimental environment with corns/sorghums
  * What spacing should the plants be at?
  * How many rows ideally?
  * How many plants do we need to be in a row?
* Mark the initial robot pose and the plants with the tape
  * Take pictures of the setup
* Measure some values of the environment
  * Distance between crops in the same row
  * Distance between rows
  * Total driving distance
  * The number of corns/sorghums that the robot drives through
  * Growth stage
* Measure the ground truth 
  * Diameters/radii
  * Heights
  * Robot trajectories
* Begin to record all topics in a rosbag file
* Roslaunch to achieve the experimental goals
  * Drive autonomously to the goal locations
  * Cut the leaf with the nichrome wire  
  * Repeat the modules
* Stop recording

### 3. Code
(1) SSH connection
  ```
  sudo gedit ~/.bashrc
  ```
  
  * Update (https://www.clearpathrobotics.com/assets/guides/noetic/jackal/network.html)
    ```
    sudo netplan apply jackal_router.yaml  # Jackal Router
    sudo netplan apply msral.yaml          # MSRAL Highbay
    ```
      * MSRAL-highbay5G: `192.168.0.180`
      * Jackal_Router: `192.168.0.179`  

(2) Grid map creation
  
  * Conversion: PointCloud2 -> LaserScan ([https://github.com/kimkt0408/pagbot/tree/main/source/pc2_2_ls](https://github.com/kimkt0408/pagbot/tree/main/source/pc2_2_ls))
  ```
  roslaunch pc2_2_ls conversion.launch
  ```
  
  * Begin gmapping
  <pre>
  roslaunch gmapping gmapping_jackal.launch</pre>
  
  * Control Jackal by keyboard
  <pre>
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py</pre>

  * Save the grid map
  <pre>
  rosrun map_server map_saver –f MAP_NAME</pre>
  
(3) Sensor data subscription
  * [Activate the ekf localizer with fused sensor data](https://github.com/kimkt0408/pagbot/blob/main/manual/ekf_localization.md)
    ```
    roslaunch realsense2_camera rs_t265_localizer.launch
    ```
  
  * [Activate multi 3D LiDARs](https://github.com/kimkt0408/pagbot/blob/main/manual/multi_lidar_operation.md)
    ```
    roslaunch velodyne_pointcloud VLP16_points_multi.launch
    roslaunch pc2_2_ls conversion.launch
    ```
  
(4) AMCL localization
  * Apply the grid map into AMCL localizer (change the map file (.yaml))
    ```
    roscd jackal_navigation/launch
    sudo gedit amcl_test.launch
    ```

  * Begin AMCL localizer
    ```
    roscd /opt/ros/noetic/share/jackal_navigation/launch
    roslaunch amcl_test.launch
    ```
  
  * Change the global coordinate from `odom` to `map`
  
  * Initialize the robot pose with `2D estimated pose` in RVIZ
 
(5) Record all topics
  ```
  rosbag record -a
  ```

(6) Row tracer: autonomous navigation module

  * Change the goal location based on the map
    ```
    roscd navigation_goals/param
    sudo gedit navigation_goals_param.yaml
    ```
  
  * Launch the module
    ```
    roslaunch navigation_goals navigation_goals.launch
    ```
  
(7) Crop sampling

(8) Diameter measurement module (with the recorded rosbag file)
  
  * Tune the parameters
  ```
  roscd diameter_measurement/src
  code diameter_measurement3.py
  ```
  
  * Launch the file
  ```
  roslaunch diameter_measurement diameter_measurement.launch
  ```
  
  * Record all results in .txt file by uncommenting some lines in the src file

(9) Height measurement module (with the recorded rosbag file)
  
  * Launch the file

    * Real 
    
      ```
      roslaunch height_measurement height_measurement.launch
      ```

    * Simulation
    
      ```
      roslaunch p_agbot crop_height.launch
      ```
    
  * Record all results in .txt file by uncommenting some lines in the src file

### 4. etc

* Publish the constant velocity
  ```
  rostopic pub -r 60 /cmd_vel geometry_msgs/Twist "linear:
  ```

* How to copy rosbag file from jackal to the laptop by SSH
  ```
  scp -r administrator@192.168.131.1:/home/administrator/2021-09-22-14-15-20.bag /home/kimkt0408/
  ```
