# Pointcloud Tools

This is a package containing tools and libraries for manipulating and analysing pointclouds.

## Pointcloud Analyser

Print out useful metadata about an incoming pointcloud stream.

## Pointcloud File Converter

Load or save pointclouds from or to files.

```bash
rosrun pointcloud_tools pointcloud_file_converter
```

Save example:
```bash
rosservice call /pointcloud_file_converter/save_pointcloud "filepath: 'my_pointcloud.pcd'
topic: '/points'
timeout: 0.0
is_pcl_type: false"
```

Load example:
```bash
rosservice call /pointcloud_file_converter/load_pointcloud "filepath: 'my_pointcloud.pcd'
topic: '/points'
frame_id: 'lidar'
latch: true
as_pcl_type: false"
```
