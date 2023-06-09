## [EKF](https://automaticaddison.com/extended-kalman-filter-ekf-with-python-code-example/) localization with fused odometry

### 0. Odometry sources to be fused
* Wheel odometry: `/jackal_velocity_controller/odom` (type: `nav_msgs/Odometry`)
* Internal IMU data: `/imu/data` (type: `sensor_msgs/Imu`) 
* Visual-Inertial-Odometry(VIO) by [Realsense T265](https://github.com/IntelRealSense/realsense-ros#using-t265): `/camera/odom/sample` (type: `nav_msgs/Odometry`)


### 1. TF setting between Jackal and T265 
(Ref: [https://msadowski.github.io/Realsense-T265-First-Impressions/](
https://msadowski.github.io/Realsense-T265-First-Impressions/))
  <pre>
  roscd realsense2_camera/launch
  sudo gedit rs_t265.launch</pre>
  
  * Fix config: `<arg name="publish_odom_tf"  default="true"/>` → `<arg name="publish_odom_tf"  default="false"/>` 
  * Put a static TF transform: `<node pkg="tf2_ros" type="static_transform_publisher" name="base_to_camera_pose_frame" args="-0.22 0 0.30 3.1415926535897931 0 0 base_link camera_pose_frame" />` 
  
  <pre>
  roscd realsense2_camera/launch/includes/
  sudo gedit nodelet.launch.xml</pre>
  
  * `odom_frame_id`: change into `odom`
  
  
### 2. `robot_localization.yaml` update
To fuse VIO into the existing odometry `/odometry/filtered` (Ref: [https://github.com/IntelRealSense/realsense-ros/issues/2400](https://github.com/IntelRealSense/realsense-ros/issues/2400))
  <pre>
  roscd jackal_control/config
  sudo gedit robot_localization_t265.yaml</pre>

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
                false, false, true,
                false, false, false,
                false, false, true,
                true, false, false]
  imu0_differential: false
  # imu0: /imu/data
  # imu0_config: [false, false, false,
  #               true, true, true,
  #               false, false, false,
  #               true, true, true,
  #               false, false, false]
  # imu0_differential: false

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


### 3. New roslaunch file 
`~/catkin_ws/src/realsense-ros/realsense2_camera/launch/rs_t265_localizer.launch`

This launch file can activate the Realsense camera and fuse the VIO sensor information into the existing odometry information
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
  

### 4. Run the launch file for odometry fusion & EKF localization update
  <pre>
  roslaunch realsense2_camera rs_t265_localizer.launch</pre>
  
-------------------
#### RESULT
**1. Rqt_graph**
<br></br>
<img src="https://user-images.githubusercontent.com/42059549/226077387-083d6749-cb8a-4ade-a4f2-1ddfb0b7e64c.png" alt="rqt_graph_ekf_localization_edited" width="400" />

**2. [Robot driving test](https://purdue0-my.sharepoint.com/:v:/g/personal/kim3686_purdue_edu/EZhB9CmXWBdKhuYYHdRuyNwBkB1wRhRyqMuNH8BLgIiEAQ?e=KYJOqz)**
