# Laser Scan Matcher for ROS2
Ported to ros2 version of laser-scan-matcher by [hkuwenjian](https://github.com/nkuwenjian/laser_scan_matcher).


## Installation
* Install modified version of [csmlib](https://github.com/AlexKaravaev/csm])
## Topics

### Subscribed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))
### Published topics
- `/pose`([geometry_msgs/Pose2D](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))
