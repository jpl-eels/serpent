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
timeout: 0.0"
```

Load example:
```bash
rosservice call /pointcloud_file_converter/load_pointcloud "filepath: 'my_pointcloud.pcd'
topic: '/points'
frame_id: 'lidar'
latch: true"
```

## Pointcloud Range Filter

Filter out points within a min range and beyond a max range.

### Known Issues

Currently only the xyz, rgb, normals and covariances fields are kept by the range filter. This be because the standard Open3D Pointcloud data structure is used which does not permit custom fields. The Open3D tensor pointcloud would be used if not for issues in the current Open3D tensor pointcloud conversion tools in `open3d_conversions` (https://github.com/ros-perception/perception_open3d/issues/21), and the lack of filtering functions in the tensor version, e.g. `SelectByIndex()`. PCL's templated pointcloud type cannot be used since the type must be known at compile time. The `pcl::PCLPointCloud2` type could be used, however this might require writing our own filter function. Until issues with the `Open3D` tensor pointcloud are resolved, this is likely the best option to resolve this issue.
