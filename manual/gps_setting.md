## GPS setting (Emlid Reach M2)

* [ROS driver(nmea_navsat_driver)](http://wiki.ros.org/nmea_navsat_driver)
  ```
  sudo apt-get install ros-noetic-nmea-navsat-driver
  ```

* How to check whether Emlid M2 is connected to computer
  ```
  telnet 192.168.1.93 9001
  ```
  
  * `192.168.1.93`: Emlid M2 IP address (Check the app: (example: MSRAL_GPS))
  * `9001`: port of position output(position streaming) (Check the app)
  
* Find the serial port corresponding to the Emlid GPS device.
  ```
  dmesg | grep tty
  ```
   
* Run the driver: Connect GPS module in PC physically
  ```
  roscore
  sudo chmod 666 /dev/ttyACM0
  rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyACM0 _baud:=38400
  ```
  
  (Check: Position streaming 2 (Serial, USB to PC, baud_rate: 38400, NMEA))
  
  * Output: `rostopic echo /fix`
  
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

* How to visualize the log data with NTRIP
  
  (1) [Emlid studio](https://docs.emlid.com/emlid-studio/kinematic-processing-workflow/?_gl=1*1a2vkm3*_ga*MTY5MzgyODgwMy4xNjgxNDE1MTY4*_ga_958NJK16DK*MTY4MjA4MDU5NC42LjEuMTY4MjA4MjExMS4wLjAuMA..) 
  
  * Example
  
    <img src="https://user-images.githubusercontent.com/42059549/233645675-559a67e9-482b-44dc-8f5d-57aa26619905.JPG" alt="gps_result_example" width="600" />
  
  (2) Google maps
  
