Here's more information.
[swri-robotics/novatel_gps_driver](https://github.com/swri-robotics/novatel_gps_driver)
----------

#### Added Novatel GPS receiver linux driver and ROS Novatel GPS Driver install script

```bash
./ros_novatel_gps_driver_install.sh
```

#### Added example bag file
```bash
rosbag play example_bags_2018-12-06-17-48-12.bag
```

#### Added some new logs

- `/dualantennaheading` *(novatel_gps_msgs/DualantennaHeading)*: [DUALANTENNAHEADING](https://docs.novatel.com/OEM7/Content/Logs/DUALANTENNAHEADING.htm) logs
- `/GRS80Coordinate` *(novatel_gps_msgs/GRS80Coordinate)*: It has converted longitude & latitude to x & y coordinates based on GRS80
- `/PolynomialCoefficient` *(novatel_gps_msgs/PolynomialCoefficient)*: lateral lane center offset, heading angle error, and curvature rate derived from coordinates

#### Command

```bash
$mkdir novatel
```
```bash
$mkdir novatel/src
```
```bash
$cd novatel/src
```
```bash
$git clone https://github.com/jiwoong7813/novatel_gps_driver
```
```bash
$cd ..
```
```bash
$catkin_make
```
```bash
$catkin_make install
```

