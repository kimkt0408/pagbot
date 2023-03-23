### IMU integration

**1. Internal IMU activation**

  * [IMU calibration](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/calibration.html): Follow the instuctions in the link above. With this process, the imu is calibrated and corrected data can be published.
  * [Calibration troubleshooting](https://github.com/husky/husky/issues/182)

**2. External IMU(3DM-GX5-25) integration**

* [3DM-GX5-25 User manual](https://cdn.shopify.com/s/files/1/1750/5061/files/3dm-gx5-25_user_manual.pdf?16138305523735781123)
* [GX5 description by Clearpath robotics](https://clearpathrobotics.com/blog/2019/05/clearpath-updates-lord-microstrain-imu-ros-driver-with-key-features/)
* [GX5 ROS driver from Clearpath](https://github.com/ros-drivers/microstrain_mips)
* [GX5 ROS driver from LORD-MicroStrain](https://github.com/LORD-MicroStrain/microstrain_inertial)

(1) ROS driver/package installation
  <pre>
  sudo apt-get update && sudo apt-get install ros-ROS_DISTRO-microstrain-inertial-driver</pre>
  <pre>
  cd catkin_ws/src
  git clone https://github.com/ros-drivers/microstrain_mips.git</pre>

(2) Change usb port configuration
  <pre>
  roscd microstrain_mips/launch
  gedit microstrain.launch</pre>
  
  <code>&lt;arg name="port" default="/dev/microstrain" /&gt;</code> 
  
  → <code>&lt;arg name="port" default="/dev/ttyACM0" /&gt;</code> or  <code>&lt;arg name="port" default="/dev/ttyACM1" /&gt;</code>

(3) Launch
  <pre>
  roslaunch microstrain_mips microstrain.launch</pre>
  
<br></br>
#### RESULT
* IMU data before/after calibration
<br></br>
![imu_calibration_result](https://user-images.githubusercontent.com/42059549/226056731-685c3503-aff9-4a57-96a7-cfad42e717da.png)
