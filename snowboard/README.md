# Snowboard

## LIO-SAM

### Dependencies

* GTSAM (LIO-SAM recommends >= 4.0.2 with `-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF` however I was fine with `sudo apt install libgtsam-dev libgtsam-unstable-dev`)
* OpenCV (`sudo apt install libopencv-dev`)
* ROS packages: `sudo apt install ros-noetic-navigation ros-noetic-robot-localization`

### Build

If running on `noetic`, you may need to patch files by following these instructions:
1. In `include/utility.h`, replace `#include <opencv/cv.h>` with `#include <opencv2/opencv.hpp>`. Move the `#include <opencv2/opencv.hpp>` header after the pcl headers due to this issue: https://github.com/flann-lib/flann/issues/214, https://stackoverflow.com/questions/42504592/flann-util-serialization-h-class-stdunordered-mapunsigned-int-stdvectorun
2. In `CMakeLists.txt`, replace `set(CMAKE_CXX_FLAGS "-std=c++11")` with `set(CMAKE_CXX_FLAGS "-std=c++14")`

You may also need a recent GTSAM version, as described at https://github.com/TixiaoShan/LIO-SAM/issues/206 but for `noetic` the system GTSAM was sufficient.

```bash
cd ~/catkin_ws/src
git clone git@github.com:TixiaoShan/LIO-SAM.git
cd ..
catkin build
```

### Run

```bash
roslaunch snowboard snowboard_liosam.launch
```

```bash
rosbag play ...
```

## LOCUS

### Dependencies

* ROS packages: `sudo apt install ros-noetic-tf2-sensor_msgs`

### Build

```bash
cd ~/catkin_ws/src
git clone git@github.com:NeBula-Autonomy/LOCUS.git
cd ..
catkin build
```

### Run

```bash
roslaunch snowboard snowboard_locus.launch
```

```bash
rosbag play ...
```
