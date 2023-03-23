### Autonomous navigation in rows and under canopies


**0. Odometry sources to be fused**
* Wheel odometry: `/jackal_velocity_controller/odom` (type: `nav_msgs/Odometry`)
* Internal IMU data: `/imu/data` (type: `sensor_msgs/Imu`) 
* Visual-Inertial-Odometry(VIO) by [Realsense T265](https://github.com/IntelRealSense/realsense-ros#using-t265): `/camera/odom/sample` (type: `nav_msgs/Odometry`)


**1. TF setting between Jackal and T265** (Ref: [https://msadowski.github.io/Realsense-T265-First-Impressions/](
https://msadowski.github.io/Realsense-T265-First-Impressions/))
  <pre>
  roscd 
