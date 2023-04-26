## P-AgSLAM baseline

* **Info**: Change 3D LiDAR topics if you use past rosbags

### 0. P-AgSLAM
```
roslaunch pagslam pagslam.launch
```

### 1. Full-ICP
```
roslaunch icp_slam icp_slam.launch
```
* Rostopics

  * Keyframe poses: `/odometry/icp`
  * Map: `/map_points`


### 2. [LOAM](https://github.com/laboshinl/loam_velodyne)
```
roslaunch loam_ouster loam_ouster.launch
```

* LiDAR topic: `velodyne1_points`

```
rosbag play –-clock ROSBAG_NAME.bag
```
* Rostopics

  * Keyframe poses: `/integrated_to_init/loam` (in `/home/kimkt0408/catkin_ws/src/loam_ouster/src/lib/TransformMaintenance.cpp`)
  * Map: `/laser_cloud_surround`


### 3. [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
* Change the parameters in `/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h`
  * PointCloud2 topic: `/velodyne1_points`
 
```
roslaunch lego_loam run.launch
```

```
rosbag play –-clock ROSBAG_NAME.bag
```
* Rostopics

  * Keyframe poses: `/integrated_to_init/legoloam` (in `/home/kimkt0408/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/src/transformFusion.cpp`)
  * Map: `/registered_cloud`


### 4. [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
* Change the parameters in `/catkin_ws/src/LIO-SAM/config/params.yaml`
 
```
roslaunch lio_sam run.launch
```

```
rosbag play –-clock ROSBAG_NAME.bag
```
* Rostopics

  * Keyframe poses: `/lio_sam/mapping/odometry`
  * Map: `/lio_sam/mapping/cloud_registered`
  
  
### Rosbag record
```
rosbag record /debug/gt_trajectory /pagslam/debug/original_trajectory /pagslam/debug/trajectory /pagslam/debug/map_total_cloud /pagslam/debug/map_cloud_marker /odometry/icp /map_points /integrated_to_init/loam /laser_cloud_surround /integrated_to_init/legoloam /registered_cloud /lio_sam/mapping/odometry /lio_sam/mapping/cloud_registered
```

-------------------
#### RESULT
**1. [Driving with a loop (LIO-SAM)](https://purdue0-my.sharepoint.com/:v:/g/personal/kim3686_purdue_edu/EU-KiY4CWeBJp-uaaAm6w1EB0xYqrBLdgcVhEzGRYYy8dg?e=6gBQ95)**

* Scenario

https://user-images.githubusercontent.com/42059549/229541953-5fffe994-7ef4-40a8-9361-8330a0abb765.mp4

