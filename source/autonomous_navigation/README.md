## Autonomous navigation in rows and under canopies

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
  
  * Launch the autonomous navigation module
  ```
  roslaunch autonomous navigation navigation.launch
  ```

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
