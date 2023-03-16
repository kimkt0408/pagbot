### 1. Internal IMU integration 
* [Calibration troubleshooting](https://github.com/husky/husky/issues/182)

### 2. IMU(3DM-GX5-25) integration 

* [3DM-GX5-25 User manual](https://cdn.shopify.com/s/files/1/1750/5061/files/3dm-gx5-25_user_manual.pdf?16138305523735781123)
* [GX5 description by Clearpath robotics](https://clearpathrobotics.com/blog/2019/05/clearpath-updates-lord-microstrain-imu-ros-driver-with-key-features/)
* [GX5 ROS driver from Clearpath](https://github.com/ros-drivers/microstrain_mips)
* [GX5 ROS driver from LORD-MicroStrain](https://github.com/LORD-MicroStrain/microstrain_inertial)

**(1) ROS driver/package installation**
  <pre>
  sudo apt-get update && sudo apt-get install ros-ROS_DISTRO-microstrain-inertial-driver</pre>
  <pre>
  cd catkin_ws/src
  git clone https://github.com/ros-drivers/microstrain_mips.git</pre>

**(2) Change usb port configuration**
  <pre>
  roscd microstrain_mips/launch
  gedit microstrain.launch</pre>
  
  <code>&lt;arg name="port" default="/dev/microstrain" /&gt;</code> 
  
  -> <code>&lt;arg name="port" default="/dev/ttyACM0" /&gt;</code> or  <code>&lt;arg name="port" default="/dev/ttyACM1" /&gt;</code>

**(3) Launch**
  <pre>
  roslaunch microstrain_mips microstrain.launch</pre>
  <!-- * Horizontal velodyne (H)
    * IP Address: 192.168.1.201
    * Data Port: 2368
    * Telemetry Port: 8308

  * Vertical velodyne (V)
    * IP Address: 192.168.1.202
    * Data Port: 2369
    * Telemetry Port: 8309
    
  * Informations for changing LiDAR ip configurations:
    * [Github about multi velodyne operation: https://github.com/JeongJae0815/Multi_Velodyne](https://github.com/JeongJae0815/Multi_Velodyne)
    * [VLP16 User guide: https://velodynelidar.com/wp-content/uploads/2019/09/63-9266-REV-A-WEBSERVER-USER-GUIDEHDL-32EVLP-16.pdf](https://velodynelidar.com/wp-content/uploads/2019/09/63-9266-REV-A-WEBSERVER-USER-GUIDEHDL-32EVLP-16.pdf)
    * [VLP16 tutorial: http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)


<br></br>
**2. Network configuration**: Each LiDAR requires a unique IP address. Assign a static IP address to each LiDAR using the device's configuration interface or web GUI. Ensure that these IP addresses are on the same subnet as the Jackal's onboard computer.

  * Name of an ethernet port which Jackal uses for LiDARs: br0

  * IP configuration command
  <pre>
  ip addr flush dev br0
  sudo ifconfig br0 192.168.1.77
  sudo route add 192.168.1.201 br0
  sudo route add 192.168.1.202 br0</pre>
  
  * To check ip connection
  <pre>
  ping 192.168.1.201
  ping 192.168.1.202</pre>
  
  * If you want to check the ip configurations of Jackal computer:
  <pre>
  ifconfig</pre>
  

<br></br>
**3. Roslaunch a launch file**: Two LiDARs are operated simultaneously with the following command.

  <pre>
  roslaunch velodyne_pointcloud VLP16_points_multi.launch</pre>
  
  * frame_id
    * Horizontal LiDAR (H): velodyne1
    * Vertical LiDAR (V): velodyne2

  * TF setting between base_link and LiDARs
  <pre><code>&lt;node pkg="tf2_ros" type="static_transform_publisher" name="base_to_velodyne1" args="-0.14 0 0.55 3.14159265359 0 0 base_link velodyne1" /&gt;</code></pre>
  <pre><code>&lt;node pkg="tf2_ros" type="static_transform_publisher" name="base_to_velodyne2" args="-0.26 0 0.41 1.570796327 0 -1.570796327 base_link velodyne2" /&gt;</code></pre>
  
 
<br></br>
**Implementation result**

  * Horizontal velodyne (H): Point cloud with colors
  * Vertical velodyne (V): Point cloud with white

![two_lidar_test_real](https://user-images.githubusercontent.com/42059549/225409163-1d7fb0db-9854-46d1-a707-9a8629e13892.png)
-->
