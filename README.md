# CladPlus

Laser Metal Deposition (LMD) is a complex process, which is governed by a large
number of parameters. Among these parameters, powder flow rate, laser power,
and travel speed are typically used to control the process properties, such as
melt pool geometry, temperature, etc. Heat input control in LMD is realized by
adjusting laser power using an infrared image sensing camera as feedback. The
controller helps to overcome the effects of thermal variations and reduces
cladding geometric variations.


## Contents

This meta-package contains two packages:
- cladplus_data: contains the working cell description files and the representation files.
- cladplus_control: contains the user interface and control files.

## Quick start

![User interface](./cladplus/media/width_graph.png)

To record a bag file with the camera adquisition:

```shell
roslaunch cladplus_data cladvid.launch
rosrun rosbag record -O control.bag /tachyon/image
```

To play a bag file with the camera registration:

```shell
roscore
rosrun rosbag play control.bag --clock
roslaunch cladplus_data cladvid.launch sim:=true
```
