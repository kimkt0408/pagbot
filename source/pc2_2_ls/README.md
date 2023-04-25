## 3D LiDAR data conversion for Gmapping

This module converts 3D LiDAR sensor datatype `PointCloud2` to 2D LiDAR datatype `LaserScan` to use in Gmapping module which is for autonomous navigation.

### 1. How to run  
  * Edit parameters: `TARGET_HEIGHT_MIN`, `TARGET_HEIGHT_MAX` in `/src/conversion.py`
  
  * Roslaunch
   
    ```
    roslaunch pc2_2_ls conversion.launch
    ```

-------------------
#### RESULT

https://user-images.githubusercontent.com/42059549/234399722-05137cc0-f333-4e8f-9a52-a065c3fe3ded.mp4

