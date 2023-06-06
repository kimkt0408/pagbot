## Autonomous navigation in rows and under canopies

This module is for the autonomous navigation by using 3D point cloud from a 3D LiDAR which is horizontally mounted on P-AgBot.

### 1. Simulation
  * Edit parameters of goal location and direction 
   ```
   roscd autonomous_navigation
   cd param
   sudo gedit navigation_param.yaml
   ```
  
  * Create a Gazebo world
   ```
   roslaunch jackal_gazebo test_world.launch
   ```
   ```
   roslaunch pagslam pagslam_gazebo.launch
   ```
  
  * Launch the autonomous navigation module
   ```
   roslaunch autonomous_navigation navigation.launch
   ```

-------------------
#### [RESULT](https://purdue0-my.sharepoint.com/:v:/g/personal/kim3686_purdue_edu/EW4aNzL6hJJCmrjP6RbLttQB6etdVL1M4PaMnTn8YVl3RQ?e=lnwIPF)

-------------------
#### Reference paper
```
@article{kim2022p,
  title={P-AgBot: In-Row \& Under-Canopy Agricultural Robot for Monitoring and Physical Sampling},
  author={Kim, Kitae and Deb, Aarya and Cappelleri, David J},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={3},
  pages={7942--7949},
  year={2022},
  publisher={IEEE}
}
```
