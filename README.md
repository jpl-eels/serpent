# SERPENT, State Estimation through Robust Perception in Extreme and Novel Terrains

This repository houses a package suite for lidar/visual/inertial/plus SLAM for the EELS mission.

## Build

The following build instructions assumes that a catkin workspace has been set up at `~/catkin_ws`. Adapt the following instructions for your system accordingly.

First clone the repository to your `~/catkin_ws/src` or clone elsewhere (e.g. `~/src`) and create a symlink:
```bash
cd ~/catkin_ws/src
git clone git@fornat1.jpl.nasa.gov:eels/modules/c3_serpent.git
```
OR
```bash
cd ~/src
git clone git@fornat1.jpl.nasa.gov:eels/modules/c3_serpent.git
cd ~/catkin_ws/src
ln -s ~/src/c3_serpent c3_serpent
```

Build:
```bash
cd ~/catkin_ws
catkin build
```

## Usage

See `serpent/README.md`.

## Plotting Results

To generate plots for trials and compare to ground truth reference, first record the output odometry to file.

Then run `main.m` from its directory with MATLAB after configuring the config and plot options to your datatset.

To run from the command line:
```bash
matlab -nodisplay -nosplash -nodesktop -r "run('main.m'); exit;"
```

The following plots will be generated:

 - position vs ground truth, position APE, position RPE
 - orientation vs ground truth in angle-axis form, orientation APE, orientation RPE
 - linear velocity vs ground truth, linear velocity AE, linear velocity RE
 - angular velocity vs ground truth, angular velocity AE, angular velocity RE

## Authors

Initial Author: William Talbot 347J (william.talbot@jpl.nasa.gov)

Current Maintainer: William Talbot 347J (william.talbot@jpl.nasa.gov)
