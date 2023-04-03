## P-AgSLAM baseline

### 1. LOAM
<pre>
roslaunch loam_ouster loam_ouster.launch</pre>

* LiDAR topic: `velodyne1_points`

<pre>
rosbag play –-clock ROSBAG_NAME.bag</pre>

* Rostopics

  * Keyframe poses: `/integrated_to_init`
  * Map: `/laser_cloud_surround`


### 2. LeGO-LOAM
* Change the parameters in `/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h`
  * PointCloud2 topic: `/velodyne1_points`
 
<pre>
roslaunch lego_loam run.launch</pre>

<pre>
rosbag play –-clock ROSBAG_NAME.bag</pre>

* Rostopics

  * Keyframe poses: `/integrated_to_init`
  * Map: `/registered_cloud`


### 3. LIO-SAM
* Change the parameters in `/catkin_ws/src/lio_sam/LeGO-LOAM/include/utility.h`
 
<pre>
roslaunch lio_sam run.launch</pre>

<pre>
rosbag play –-clock ROSBAG_NAME.bag</pre>

