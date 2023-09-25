## P-AgSLAM ver. 2 (with optional RTK GPS)

This is the 2nd version of P-AgSLAM which optionally fuses RTK GPS data to update the robot pose in the global coordinate system.

### 1. RTK GPS Setting before field tests

(Edit `gps/launch/gps_localization.launch`)

  * The origin of the global coordinate
    * Check `latitude` & `longitude`: `rostopic echo /gps/fix`
    * Change values in `<rosparam param="datum">[40.49544639166667, -87.00096109166667, 0.0, world, base_link]</rosparam>`.

  * Yaw offset
    * Initialize `/odometry/filtered` by `roslaunch realsense2_camera rs_t265_localizer.launch`
   
    * Measure values in `/imu/data/orientation`.
    * Convert quaternion into [RPY](https://www.andre-gaschler.com/rotationconverter/).
    * Change values of `<param name="yaw_offset" value="2.4">`.
    * Put the value: `-Yaw (Z)`

  * Do `catkin_make`

  * Check whether the direction of RTK GPS trajectory is aligned well with the robot trajectory.

### 2. How to operate P-AgSLAM ver.2

  * Multi-lidar operation
  ```
	sudo ip addr flush dev br0
	sudo ifconfig br0 192.168.1.77
	sudo route add 192.168.1.201 br0
	sudo route add 192.168.1.202 br0
  ```

  ```
	ping 192.168.1.201
	ping 192.168.1.202
  ```

  ```
	roslaunch velodyne_pointcloud VLP16_points_multi.launch
  ```

  * RTK GPS operation
    (If GPS is not connected, unplug and replug the antenna with the GPS module.)
   ```
	 telnet 192.168.1.93 9001
	 roslaunch reach_ros_node nmea_tcp_driver.launch
   ```

  * Do RTK GPS setting before field tests (1.)
  
  * Initialize `/odometry/filtered` (Do twice: at the first launch, `ekf_localization_node` stops to publish rostopic
    
   ```
   roslaunch realsense2_camera rs_t265_localizer.launch
   ```
    
   ```
   roslaunch realsense2_camera rs_t265_localizer.launch
   ```

  * Ready to record rosbag files (If it does not work, `source /opt/ros/noetic/setup.bash`)
    
   ```
   rosbag record -a
   ```

  * P-AgSLAM / GTSAM (Check: `boolCloud_ = false`. If it is `true`, the rosbag file size should increase exponentially.)
    
   ```
   roslaunch gtsam factorGraph.launch
   ```


<!-- ### 3. Test scenarios
  * Long distance: 3 sections
  * Medium distance: 2 sections
  * Short distance: 1 section
  * Multiple rows: 6 rows - 1 section
  * Multiple rows: 5 rows - 2 sections
-->
