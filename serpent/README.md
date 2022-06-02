# SERPENT

Maintainer: William Talbot 347J

## Dependencies

### Eigen

Tested with 3.3.7

Ubuntu 20.04:
```bash
sudo apt install libeigen3-dev
```

### fast_gicp

Follow the ROS build instructions at https://github.com/SMRT-AIST/fast_gicp

### GTSAM

Tested with 4.0.3

Ubuntu 20.04:
```bash
sudo apt install libgtsam-dev
```

### PCL

Tested with 1.10

### pcl_conversions

Tested with `ros-noetic-pcl-conversions`

## Build

Create a symbolic link (`ln -s`) from your `catkin_ws/src` to this directory.

```
catkin build serpent
```

## Usage

```
roslaunch serpent serpent.launch
```

## Debugging

### gprof

Compile with:
```
catkin build serpent --verbose --cmake-args -DENABLE_GPROF=ON
```
Note that this option is cached, so needs to be turned `OFF` in a subsequent compilation.

After running there will be a `serpent.<pid>` file in `~/.ros/`.

### gdb

```
roslaunch serpent serpent.launch use_gdb:=true
```

### callgrind

The following should be sufficient to run the program with callgrind.
```
roslaunch serpent serpent.launch use_callgrind:=true
```

However despite the launch exitting cleanly when interupting with CTRL+C without callgrind, when running with callgrind it does not.

Using the `--sigint-timeout=SIGINT_TIMEOUT` and `--sigterm-timeout=SIGTERM_TIMEOUT` options (http://wiki.ros.org/roslaunch/Commandline%20Tools) was tried to give the program more time to shutdown cleanly, however this only very occasionally worked. Terminating the node through `rosnode kill /serpent` was also only occasionally successful.

The only reliable method found so far of generating a valid output file is to use the `callgrind_control --dump` command in a separate terminal.

The result can be visualised with:
```
kcachegrind ~/.ros/callgrind.serpent.<pid>.1
```
The `.1` is present if callgrind was dumped.

## Testing

```
catkin test serpent
```
