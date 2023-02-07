# SERPENT

Maintainer: William Talbot 347J

## Dependencies

### cv_bridge

If you installed OpenCV from source (see OpenCV section), you will have to install `cv_bridge` from source to avoid conflicting opencv libraries. One way to build cv_bridge from source:
```bash
cd ~/src
git clone git@github.com:ros-perception/vision_opencv.git # Check the branch matches your ROS version
cd ~/catkin_ws/src
ln -s ~/src/vision_opencv/cv_bridge cv_bridge
cd ~/catkin_ws
catkin build cv_bridge
```

`cv_bridge` should now be linked to your installed version of OpenCV.

### Eigen

Tested with 3.3.7

Ubuntu 20.04:
```bash
sudo apt install libeigen3-dev
```

### fast_gicp

Follow the ROS build instructions at https://github.com/SMRT-AIST/fast_gicp

### GTSAM

Tested with 4.0.3, the latest stable release of GTSAM 4.

Ubuntu 20.04 (instructions at https://gtsam.org/get_started/):
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev
```

(`add-apt-repository` is installable with `sudo apt-get install software-properties-common` if not already installed)

**WARNING:** Installing GTSAM from source (on default branch at time of writing) causes SERPENT to crash on an 

### OpenCV

SERPENT uses SIFT as one of its stereo feature types, however SIFT lost its patent only in March 2020, so is only part of core OpenCV starting from version 4.4.0. Ubuntu 20.04's OpenCV `apt` version is 4.2.0, so is not sufficient. Ubuntu 22.04's `apt` version is 4.5.4, so you should be able to install via `apt` (not tested):
```bash
sudo apt install libopencv-dev
```

For Ubuntu 20.04 or earlier, OpenCV >=4.4.0 must be installed from source **if you want to use SIFT features** (replace `<version>`):
```bash
git clone git@github.com:opencv/opencv.git
git clone git@github.com:opencv/opencv_contrib.git
cd opencv
git checkout <version>
cd ../opencv_contrib
git checkout <version>
cd ../opencv
mkdir build && cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.x/modules ..
cmake --build . -j
sudo cmake --build . --target install
```

If you do not need SIFT features, then 4.2.0 is sufficient:
```bash
sudo apt install libopencv-dev
```

If it is desirable to keep an older version of OpenCV installed on your system, you can do so, making sure to turn on the OPENCV_ENABLE_NONFREE flag while building.

**Note that if you install OpenCV from source, you must also install `cv_bridge` from source.**

### PCL

Tested with 1.10

### pcl_conversions

Tested with `ros-noetic-pcl-conversions`

## Build

Create a symbolic link (`ln -s`) from your `catkin_ws/src` to this directory. It is highly encouraged to build in RELEASE with optimisation.

```bash
catkin build serpent
```

## Usage

To run everything together:
```bash
roslaunch serpent serpent.launch <arg>:=<value> ...
```

To run the frontend and backend as separate processes:
```bash
roslaunch serpent serpent_frontend.launch <arg>:=<value> ...
```
```bash
roslaunch serpent serpent_backend.launch <arg>:=<value> ...
```

This latter option may be useful if, for example, you want to run SERPENT on a remote machine and only want to run the frontend on the robot, where the pointcloud can be deskewed and downsampled before it is sent over the network.

### Frames

SERPENT uses the `eigen_ros::BodyFrames` abstraction layer in its handling of robot reference frames, which is compatible with the ROS TF tree architecture. It is important to keep in mind that frame and frame_id are two separate concepts, where `eigen_ros::BodyFrames` defines frames, with which frame_ids can be associated, e.g. for when messages are published to ROS or transforms must be looked up via the TF2 api. In order for SERPENT to operate correctly, it requires the following configuration parameters:

* `map_frame_id` which resides in the tuning config file (e.g. `tuning_default.yaml`), and defaults to `"map"` if not set explicitly.
* A `body_frames` block which defines the robot body frames in a way that is consistent with the documentation in `eigen_ros/body_frames.hpp`. The top-level frame (`body_frame_name` in the documentation) defined in this block is used by SERPENT as the body frame reference of all sensors during optimisation. There must also be a `base_link` frame somewhere in the hierarchy, or as a dynamic link. A typical use case will have the top-level frame `body_frame_name` set to `base_link` (note the `frame_id` can be set by the user to be different from `base_link`), but the `base_link` frame can in fact exist anywhere in the hierarchy, or as a `dynamic` link. SERPENT will produce map_frame -> base_link transforms and poses (with covariance), which may differ from the poses of the underlying optimisation (map_frame -> body_frame). Additionally the `body_frames` block must have an `imu` and `lidar` frame (with their `frame_id` each set to the frame_id of the sensor for your robot).
* If stereo factors are enabled, the frame of the left camera (note the frame_id) has to be specified in the `stereo_factors` block in the `left_cam_frame` parameter.

## Debugging

### gprof

Compile with:
```bash
catkin build serpent --verbose --cmake-args -DENABLE_GPROF=ON
```
Note that this option is cached, so needs to be turned `OFF` in a subsequent compilation.

After running there will be a `serpent.<pid>` file in `~/.ros/`.

### gdb

```bash
roslaunch serpent serpent.launch use_gdb:=true
```

### callgrind

The following should be sufficient to run the program with callgrind.
```bash
roslaunch serpent serpent.launch use_callgrind:=true
```

However despite the launch exitting cleanly when interupting with CTRL+C without callgrind, when running with callgrind it does not.

Using the `--sigint-timeout=SIGINT_TIMEOUT` and `--sigterm-timeout=SIGTERM_TIMEOUT` options (http://wiki.ros.org/roslaunch/Commandline%20Tools) was tried to give the program more time to shutdown cleanly, however this only very occasionally worked. Terminating the node through `rosnode kill /serpent` was also only occasionally successful.

The only reliable method found so far of generating a valid output file is to use the `callgrind_control --dump` command in a separate terminal.

The result can be visualised with:
```bash
kcachegrind ~/.ros/callgrind.serpent.<pid>.1
```
The `.1` is present if callgrind was dumped.

## Testing

```bash
catkin test serpent
```
