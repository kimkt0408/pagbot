<RTK GPS setting before field tests (Edit gps/launch/gps_localization.launch)>

(1) Global Coordinate origin
- rostopic echo /gps/fix -> Check "latitude / longitude"
- Change values in <rosparam param="datum">[40.49544639166667, -87.00096109166667, 0.0, world, base_link]</rosparam>
        
(2) Yaw offset  
- Reset "odometry/filtered" by "roslaunch realsense2_camera rs_t265_localizer.launch"

- Measure values of "imu/data/orientation"
- Convert quaternion into RPY in https://www.andre-gaschler.com/rotationconverter/
- Change values of <param name="yaw_offset" value="2.4">
- Put the value of the minus on Yaw (Z) 

(3) "DO" catkin_make

(4) Check whether the direction of the RTK GPS is aligned well with robot trajectory.


<What to roslaunch>
(1) multi-lidar operation
sudo ip addr flush dev br0
sudo ifconfig br0 192.168.1.77
sudo route add 192.168.1.201 br0
sudo route add 192.168.1.202 br0

ping 192.168.1.201
ping 192.168.1.202

roslaunch velodyne_pointcloud VLP16_points_multi.launch

(2) RTK GPS operation
telnet 192.168.1.93 9001
roslaunch reach_ros_node nmea_tcp_driver.launch

* If it is not connected, unplug and replug the antenna with the GPS module.

"DO" RTK GPS setting before field tests

(3) Initialize "odometry/filtered" (DO twice: at the first time, ekf_localization stops to publish rostopic)
"roslaunch realsense2_camera rs_t265_localizer.launch"
"roslaunch realsense2_camera rs_t265_localizer.launch"

(4) Ready to record rosbag files
rosbag record -a

If it does not work,
source /opt/ros/noetic/setup.bash

(5) P-AgSLAM / GTSAM
roslaunch gtsam factorGraph.launch
check "boolCloud_" is false. If it is true, the rosbag file size should increase exponentially.

* Scenario
long
medium
short
multiple rows (6 rows)
multiple rows (5+5 rows)
