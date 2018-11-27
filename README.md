# Object_Collection_Robotic_Arm
[![Build Status](https://travis-ci.org/Ghost1995/Object_Collection_Robotic_Arm.svg?branch=trial)](https://travis-ci.org/Ghost1995/Object_Collection_Robotic_Arm)
[![Coverage Status](https://coveralls.io/repos/github/Ghost1995/Object_Collection_Robotic_Arm/badge.svg)](https://coveralls.io/github/Ghost1995/Object_Collection_Robotic_Arm)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)
---

It is the initial directory for Final Project of ENPM808X.

## Make Sure to Install

iiwa stack

```
git clone https://github.com/IFL-CAMP/iiwa_stack.git
```

## Run Instruction 

```
source devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch 
```
In other terminal

```
source devel/setup.bash
rosrun kuka_arm detect 
```

