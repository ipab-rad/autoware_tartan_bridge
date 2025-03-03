# autoware_tartan_bridge

A collection of ROS2 pkgs to interface tartan carpet vehicle's sensors to autoware


## Play a Rosbag with point type conversion

```bash
ros2 launch tartan_rosbag_launcher play_bag_perception.launch.xml rosbag:=/path/to/your/rosbag.mcap
```
By default, the rosbag will play in a loop.

**Note**: Only point clouds whose `PointFields` match the [ouster-ros](https://github.com/ouster-lidar/ouster-ros/tree/ros2) `original` [point-type](https://github.com/ouster-lidar/ouster-ros/blob/80b37a0e63d56861d22ac70c730ee981723631cd/include/ouster_ros/os_point.h#L22-L86) are supported.