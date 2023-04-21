## GPS setting (Emlid Reach M2)

* [ROS driver(Reach RTK ROS Node)](https://github.com/rpng/reach_ros_node): Follow the instructions

* How to check whether Emlid M2 is connected to computer
  ```
  telnet 192.168.0.104 9001
  ```
  
  * `192.168.0.104`: Emlid M2 IP address (Check the app: (example: MSRAL_highbay))
  * `9001`: port of position output(position streaming) (Check the app)
  * Output
  
    ```
    Trying 192.168.0.104...
    Connected to 192.168.0.104.
    Escape character is '^]'.
    $GNRMC,,,,,,,,,,,,*55
    $GNGGA,,,,,,,,,,,,,,*48
    $GPGSA,A,1,,,,,,,,,,,,,,,*1E
    ```
    
* Run the driver
  ```
  roscore
  rosrun reach_ros_node nmea_tcp_driver _host:=192.168.0.104 _port:=9001
  ```
  * Output
  
    ```
    [INFO] [1679501063.264807]: Connecting to Reach RTK 192.168.0.104 on port 9001
    [INFO] [1679501063.274310]: Successfully connected to device, starting publishing!
    [ERROR] [1679501064.055678]: Value error, likely due to missing fields in the NMEA message. Error was: invalid literal for int() with base 10: ''.
    [WARN] [1679501064.059864]: Sentence type 'ZDA' not in parse map, ignoring.
    [WARN] [1679501064.062766]: Failed to parse NMEA sentence. Sentence was: $GNZDA,,,,,,*56
    [WARN] [1679501064.065637]: Sentence type 'EBP' not in parse map, ignoring.
    [WARN] [1679501064.068228]: Failed to parse NMEA sentence. Sentence was: $GNEBP,,,,,,M*13
    [ERROR] [1679501065.049109]: Value error, likely due to missing fields in the NMEA message. Error was: invalid literal for int() with base 10: ''.
    ```

* How to visualize the log data with NTRIP: [Emlid studio](https://docs.emlid.com/emlid-studio/kinematic-processing-workflow/?_gl=1*1a2vkm3*_ga*MTY5MzgyODgwMy4xNjgxNDE1MTY4*_ga_958NJK16DK*MTY4MjA4MDU5NC42LjEuMTY4MjA4MjExMS4wLjAuMA..)
  * Example
  
    <img src="https://user-images.githubusercontent.com/42059549/233645675-559a67e9-482b-44dc-8f5d-57aa26619905.JPG" alt="gps_result_example" width="600" />

