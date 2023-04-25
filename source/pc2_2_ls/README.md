## 3D LiDAR data conversion for Gmapping

This module converts 3D LiDAR sensor data `PointCloud2` to 2D LiDAR data type `LaserScan` to use in Gmapping module which is for autonomous navigation.

### 1. How to run  
  * Edit parameters: `TARGET_HEIGHT_MIN`, `TARGET_HEIGHT_MAX` in `/src/conversion.py`
  
  * Roslaunch
   ```
   roslaunch pc2_2_ls conversion.launch
   ```

-------------------
#### [RESULT](https://purdue0-my.sharepoint.com/:v:/g/personal/kim3686_purdue_edu/EW4aNzL6hJJCmrjP6RbLttQB6etdVL1M4PaMnTn8YVl3RQ?e=lnwIPF)
