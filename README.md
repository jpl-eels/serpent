# SERPENT, State Estimation through Robust Perception in Extreme and Novel Terrains

This repository houses a package suite for ldiar/visual/inertial/plus SLAM for the EELS mission.

# Build

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

# Authors

Initial Author: William Talbot 347J (william.talbot@jpl.nasa.gov)
Current Maintainer: William Talbot 347J (william.talbot@jpl.nasa.gov)
