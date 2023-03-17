### EKF localization with fused odometry

**0. Odometry sources to be fused**
* Wheel odometry: `/jackal_velocity_controller/odom` (rostopic type: `nav_msgs/Odometry`)
* Internal IMU data: `/imu/data` (rostopic type: `sensor_msgs/Imu`) 
* Visual-Inertial-Odometry(VIO) by [Realsense T265](https://github.com/IntelRealSense/realsense-ros#using-t265): `/camera/odom/sample` (rostopic type: `nav_msgs/Odometry`)


**1. TF setting between Jackal and T265**
  <pre>
  roscd realsense2_camera/launch
  sudo gedit rs_t265.launch</pre>

  * Fix config: `<arg name="publish_odom_tf"  default="true"/>` → `<arg name="publish_odom_tf"  default="false"/>` 
  * Put a static TF transform: `<node pkg="tf2_ros" type="static_transform_publisher" name="base_to_camera_pose_frame" args="-0.22 0 0.30 3.1415926535897931 0 0 base_link camera_pose_frame" />` 
  
  <pre>
  roscd realsense2_camera/launch/includes/
  sudo gedit nodelet.launch.xml</pre>
  
  * `odom_frame_id`: change into `odom`
  
  
**2. `robot_localization.yaml` update**: To fuse VIO into the existing odometry `/odometry/filtered`
  <pre>
  roscd jackal_control/config
  sudo gedit rovbot_localization_t265.yaml</pre>

  * Fix configurations for each sensor
  ```
  #Configuation for robot odometry EKF
  #
  frequency: 50

  odom0: /jackal_velocity_controller/odom
  odom0_config: [false, false, false,
                 false, false, false,
                 true, true, true,
                 false, false, true,
                 false, false, false]
  odom0_differential: false

  imu0: /imu/data
  imu0_config: [false, false, false,
                true, true, true,
                false, false, false,
                true, true, true,
                false, false, false]
  imu0_differential: false

  # Add T265 odometry (VIO)
  odom1: /camera/odom/sample
  odom1_config: [false, false, true,
             false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
  odom1_differential: false

  odom_frame: odom
  base_link_frame: base_link
  world_frame: odom

  predict_to_current_time: true
  ```


**3. New roslaunch file (`~/catkin_ws/src/realsense-ros/realsense2_camera/launch/rs_t265_localizer.launch`)**: this launch file can activate the Realsense camera and fuse the VIO sensor information into the existing odometry information
  ```
  <launch>
    <!-- Activate the Realsense camera -->
    <include file="$(find realsense2_camera)/launch/rs_t265.launch">
    </include>

    <!-- Start the robot_localization node to fuse odometry -->
    <node name="ekf_localization" pkg="robot_localization" type="ekf_localization_node" output="screen">
      <rosparam command="load" file="$(find jackal_control)/config/robot_localization_t265.yaml"/>
    </node>
  </launch>
  ```
  

**4. Run the launch file for odometry fusion & EKF localization update**
  <pre>
  roslaunch realsense2_camera rs_t265_localizer.launch</pre>
  
<br></br>
**RESULT**

<!--
* Fully-charged battery
* Ethernet cable
* Jackal router
* Extension cord
* Recording & Ground truth measurements: ruler, tape, camera tripod, smartphone

**2. Scenario**
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

**3. Code**

(1) SSH connection
<pre>
sudo gedit ~/.bashrc</pre>

(2) Grid map creation
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
  * Initialize 3D LiDAR(Ouster ver.)
  <pre>
  sudo ./init_ouster.sh</pre>
  
  * Connect 2D LiDAR with USB port
  <pre>
  sudo chmod a+rw /dev/ttyUSB0</pre>
  
  * Subscribe all sensor data
  <pre>
  roslaunch sensor_integration jackal_os_lds_t265.launch</pre>
  
(4) AMCL localization
  * Apply the grid map into AMCL localizer (change the map file (.yaml))
  <pre>
  roscd jackal_navigation/launch
  sudo gedit amcl_test.launch</pre>

  * Begin AMCL localizer
  <pre>
  roscd /opt/ros/melodic/share/jackal_navigation/launch
  roslaunch amcl_test.launch</pre>
  
  * Change the global coordinate from `odom` to `map`
  
  * Initialize the robot pose with `2D estimated pose` in RVIZ
 
(5) Record all topics
<pre>
rosbag record -a</pre>

(6) Row tracer: autonomous navigation module
  * Change the goal location based on the map
  <pre>
  roscd navigation_goals/param
  sudo gedit navigation_goals_param.yaml</pre>
  
  * Launch the module
  <pre>
  roslaunch navigation_goals navigation_goals.launch</pre>
  
(7) Crop sampling

(8) Diameter measurement module (with the recorded rosbag file)
  * Tune the parameters
  <pre>
  roscd diameter_measurement/src
  code diameter_measurement3.py</pre>
  
  * Launch the file
  <pre>
  roslaunch diameter_measurement diameter_measurement.launch</pre>
  
  * Record all results in .txt file by uncommenting some lines in the src file

(9) Height measurement module (with the recorded rosbag file)
  * Launch the file
  <pre>
  roslaunch height_measurement height_measurement.launch</pre>
  
  * Record all results in .txt file by uncommenting some lines in the src file

**4. Etc**
* Publish the constant velocity
<pre>
rostopic pub -r 60 /cmd_vel geometry_msgs/Twist "linear:</pre>

* How to copy rosbag file from jackal to the laptop by SSH
<pre>
scp -r administrator@192.168.131.1:/home/administrator/2021-09-22-14-15-20.bag /home/kimkt0408/</pre>
-->
