## Simulation with GPU

### 1. GPU setting

* How to activate GPU for Gazebo
  
  * Single GPU

    ```
    optirun gazebo
    ```

  * Multiple GPUs

    ```
    CUDA_VISIBLE_DEVICES=0,1,2 optirun gazebo
    ```
