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

```bash
roslaunch serpent serpent.launch <arg>:=<value> ...
```

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
