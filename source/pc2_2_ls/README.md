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

* Simulation

https://user-images.githubusercontent.com/42059549/234588800-d225e990-db6a-433b-937f-7a6a7e80b335.mp4

* Lab environment

https://user-images.githubusercontent.com/42059549/234586851-9e6ac8a0-96b5-4501-902f-04663117782a.mp4
