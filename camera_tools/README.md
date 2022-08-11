# Camera Tools

This ROS package is a collection of tools for manipulating images and camera_info messages.

---

## split_image

Split up or extract regions from an image, republishing them as new image + camera_info streams.

Usage:
```bash
roslaunch camera_tools split_image.launch image_topic:=<image_topic> config_file:=<config_file> node_name:=<node_name>
```

Alternatively create a new launch file:
```xml
<launch>
    <include file="$(find camera_tools)/launch/split_image.launch">
        <arg name="image_topic"     value="/my_camera/image"/>
        <arg name="config_file"     value="$(find my_camera)/config/camera_tools/split_my_camera.yaml"/>
        <arg name="node_name"       value="split_image_my_camera"/>
    </include>
</launch>
```

Look at `camera_tools/config/split_image_example.yaml` for an example config file.

---

## publish_info

Publish camera_info for an image stream.

Usage:
```bash
roslaunch camera_tools publish_info.launch input_image_topic:=<input_image_topic> output_info_topic:=<output_info_topic> config_file:=<config_file> node_name:=<node_name>
```

Alternatively create a new launch file:
```xml
<launch>
    <include file="$(find camera_tools)/launch/publish_info.launch">
        <arg name="input_image_topic"   value="/my_camera/image"/>
        <arg name="output_info_topic"   value="/my_camera/camera_info"/>
        <arg name="config_file"         value="$(find my_camera)/config/camera_tools/info_my_camera.yaml"/>
        <arg name="node_name"           value="publish_info_my_camera"/>
    </include>
</launch>
```

Look at `camera_tools/config/publish_info_example.yaml` for an example config file.

---
