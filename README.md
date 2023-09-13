# RR Estimator
This package was developed for ABU Robocon2023.
Detect ArUco Marker equipped with RR, estimate the position of RR from ER.
## Subscribe
- `image_raw` : Image data, from camera sensor. (sensor_msgs/msg/Image)
- `camera_info` : Camera infomation (sensor_msgs/msg/CameraInfo)

## Publish
RR position is broadcasted as tf data.
