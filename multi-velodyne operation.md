### Multi-velodyne operation

**1. Assign static IP addresses to LiDARs**: Each LiDAR should have a unique IP address. You can assign a static IP address to each LiDAR using its configuration interface or web GUI. Make sure that these IP addresses are on the same subnet as the Jackal's onboard computer. For example, you can assign the IP addresses 192.168.1.201, 192.168.1.202, and so on.

  * Horizontal velodyne (H)
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

