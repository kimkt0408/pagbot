## P-AgSLAM baseline

### 1. [LOAM](https://github.com/laboshinl/loam_velodyne)
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


### 2. [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
* Change the parameters in `/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h`
  * PointCloud2 topic: `/velodyne1_points`
 
```
roslaunch lego_loam run.launch
```

```
rosbag play –-clock ROSBAG_NAME.bag
```

* Rostopics

  * Keyframe poses: `/integrated_to_init/legoloam`
  * Map: `/registered_cloud`


### 3. [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
* Change the parameters in `/catkin_ws/src/LIO-SAM/config/params.yaml`
 
```
roslaunch lio_sam run.launch
```

```
rosbag play –-clock ROSBAG_NAME.bag
```

-------------------
#### RESULT
**1. [Driving with a loop](https://purdue0-my.sharepoint.com/:v:/g/personal/kim3686_purdue_edu/EU-KiY4CWeBJp-uaaAm6w1EB0xYqrBLdgcVhEzGRYYy8dg?e=6gBQ95)**

* Scenario

https://user-images.githubusercontent.com/42059549/229541953-5fffe994-7ef4-40a8-9361-8330a0abb765.mp4

