# RVIZ Textured Quads
Plugin: Displays textured images (realtime or static sensor_msgs/Image topics) in 3D space

![Demo](https://github.com/MohitShridhar/rviz_textured_quads/blob/master/gifs/rviz_demo.gif)

## Usage

This uses rviz_camera_stream (https://github.com/lucasw/rviz_camera_stream),
if not available set up another image topic (like from a webcam) and set the MeshDisplayCustom
image property to that topic:

    roslaunch rviz_textured_quads demo.launch
