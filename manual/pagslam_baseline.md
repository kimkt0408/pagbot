### P-AgSLAM baseline

**1. LOAM**
<pre>
roslaunch loam_ouster loam_ouster.launch</pre>

* Change the LiDAR topic depending on the rosbag file
  * Simulation: `os1_cloud_node/points`
  * Real: `os_cloud_node/points`

<pre>
rosbag play –clock ROSBAG_NAME.bag</pre>

* Rostopics
  * Keyframe poses: `/integrated_to_init`
  * Map: `/laser_cloud_surround`
  
**2. LeGO-LOAM**
* Change the parameters in `/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h`
  * PointCloud2 topic: `/os1_cloud_node/points` (simulation) → `/os_cloud_node/points` (ACRE)
 
<pre>
roslaunch lego_loam run.launch</pre>

<pre>
rosbag play –clock ROSBAG_NAME.bag</pre>

* Rostopics
  * Keyframe poses: `/integrated_to_init`
  * Map: `/registered_cloud`
