## RTK GPS utilization ([Emlid Reach M2](https://emlid.com/reach/))

* About RTK GPS data format: https://anavs.com/knowledgebase/nmea-format/

* How to check whether Emlid M2 is connected to computer
  ```
  telnet 192.168.1.93 9001
  ```
  
  * `192.168.1.93`: Emlid M2 IP address (Check the app: (example: MSRAL_GPS))
  * `9001`: port of position output(position streaming) (Check the app)
 
### 1. How to get RTK GPS data in ROS

* [ROS driver(reach_ros_node)](https://github.com/rpng/reach_ros_node)
  
  ```
  git clone https://github.com/rpng/reach_ros_node.git
  ```
  
  * I modified some lines on the code. Check [here](https://github.com/kimkt0408/pagbot/tree/main/source/reach_ros_node).

* Run the driver: Connect GPS module in PC physically

  ```
  roslaunch reach_ros_node nmea_tcp_driver.launch
  ```
  
  * Output: `/tcpfix` (rostopic: `sensor_msgs/NavSatFix`)
  
  
### 2. How to get GPS data without RTK in ROS

* [ROS driver(nmea_navsat_driver)](http://wiki.ros.org/nmea_navsat_driver)
  ```
  sudo apt-get install ros-noetic-nmea-navsat-driver
  ```
  
* Find the serial port corresponding to the Emlid GPS device.
  ```
  dmesg | grep tty
  ```
   
* Run the driver: Connect GPS module in PC physically (in Jackal, use /dev/ttyACM1 instead of /dev/ttyACM0)
  ```
  roscore
  sudo chmod 666 /dev/ttyACM0
  rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyACM0 _baud:=38400
  ```
  
  (Check: Position streaming 2 (Serial, USB to PC, baud_rate: 38400, NMEA))
  
  * Output: `/fix` (rostopic: `sensor_msgs/NavSatFix`) (In Jackal, `/navsat/fix`)
  
    https://user-images.githubusercontent.com/42059549/234640369-64e1e5d1-3a3d-4f89-a87c-29d94a262e0f.mp4

    ```
    header: 
      seq: 314
      stamp: 
        secs: 1682525139
        nsecs: 691549777
      frame_id: "gps"
    status: 
      status: 2
      service: 1
    latitude: 40.421845121666664
    longitude: -86.92013239833334
    altitude: 156.23499999999999
    position_covariance: [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0011560000000000001]
    position_covariance_type: 1
    ```



### 3. How to visualize RTK GPS data
  
  * [Emlid studio](https://docs.emlid.com/emlid-studio/kinematic-processing-workflow/?_gl=1*1a2vkm3*_ga*MTY5MzgyODgwMy4xNjgxNDE1MTY4*_ga_958NJK16DK*MTY4MjA4MDU5NC42LjEuMTY4MjA4MjExMS4wLjAuMA..) 
  
    * Result
  
      <img src="https://user-images.githubusercontent.com/42059549/233645675-559a67e9-482b-44dc-8f5d-57aa26619905.JPG" alt="gps_result_example" width="600" />
  
  * Google maps (rktpost.exe/KML_GPX_converter)
  
  * [Mapviz (ROS)](https://medium.com/@hitlx916/visualize-the-gnss-messages-in-mapviz-ros-4ae7eec19936) 
    (Ref: [https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite)) 
    
    ```
    sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
    roslaunch mapviz mapviz.launch
    ```

    * Result
    
      <img src="https://user-images.githubusercontent.com/42059549/234695130-fdeb13d6-7207-4255-943a-2382f439369c.png" alt="gps_ros_mapviz" width="600" /> 

  
    
