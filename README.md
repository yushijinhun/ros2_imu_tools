# IMU tools
This repository provides packages for computing and visualizing the orieantion of, e.g., a robot.

## Components

### `imu_fusion_madgwick`

This package implements a filte for fusing accelerometer and gyrometer readings to orienation. It subscribes to the topic `/Imu/data_raw` and publishes the comuted orientation to `/Imu/data`(including the raw accelerometer and the gyrometer readings)

### `imu_viz`

This package visualizes the orientation of the robot with three arrows representing the axes X (roll), Y (pitch) and Z (yaw). The axes (XYZ) are mapped to the colors red, green and blue (RGB).

For seeing the axes in RViz, you need to add the add a Display of type `MarkerArray` and subscibe to the topic `/Imu/viz`.

![The initial axes of the IMU visualization](https://gitlab.com/boldhearts/ros2_imu_tools/raw/master/imu_viz_marker_array.png)