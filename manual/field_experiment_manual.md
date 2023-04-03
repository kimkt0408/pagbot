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

### 4. etc
* Publish the constant velocity
<pre>
rostopic pub -r 60 /cmd_vel geometry_msgs/Twist "linear:</pre>

* How to copy rosbag file from jackal to the laptop by SSH
<pre>
scp -r administrator@192.168.131.1:/home/administrator/2021-09-22-14-15-20.bag /home/kimkt0408/</pre>
