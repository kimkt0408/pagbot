## P-AgSLAM baseline

### 1. LOAM
```
roslaunch loam_ouster loam_ouster.launch
```

* LiDAR topic: `velodyne1_points`

```
rosbag play –-clock ROSBAG_NAME.bag
```

* Rostopics

  * Keyframe poses: `/integrated_to_init`
  * Map: `/laser_cloud_surround`


### 2. LeGO-LOAM
* Change the parameters in `/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h`
  * PointCloud2 topic: `/velodyne1_points`
 
```
roslaunch lego_loam run.launch
```

```
rosbag play –-clock ROSBAG_NAME.bag
```

* Rostopics

  * Keyframe poses: `/integrated_to_init`
  * Map: `/registered_cloud`


### 3. LIO-SAM
* Change the parameters in `/catkin_ws/src/lio_sam/LeGO-LOAM/include/utility.h`
 
```
roslaunch lio_sam run.launch
```

```
rosbag play –-clock ROSBAG_NAME.bag
```

