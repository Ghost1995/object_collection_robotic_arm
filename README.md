# Object Collection Robotic Arm

[![Build Status](https://travis-ci.org/Ghost1995/object_collection_robotic_arm.svg?branch=master)](https://travis-ci.org/Ghost1995/object_collection_robotic_arm)
[![Coverage Status](https://coveralls.io/repos/github/Ghost1995/object_collection_robotic_arm/badge.svg?branch=master)](https://coveralls.io/github/Ghost1995/object_collection_robotic_arm?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

In this project we develop an autonomous robotic segregation system for Acme Robotics. It is an implementation of a robotic manipulator capable of segregating objects based on their color. This project has been implemented on the Kuka robotic manipulator using Gazebo, Rviz and ROS (and its various packages and libraries).

The developed system is capable of:
* detecting the objects placed on a table,
* picking up the objects using vacuum gripper and placing them at a desired position, and
* segregating the objects based on their color.

Few applications of the project include:
* Object segregation in shipping companies, like Amazon, Alibaba and Ikea.
* Transporting objects from one location to another.
* Kit building operation in assembly lines. 

This project has been programmed in the C++ programming language and uses C++ 11/14 features with emphasis given to Object Oriented Programming principles. The code follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with cpplint validation. Cppcheck is also used for static code analysis. This project is done by implementing pair-programming and follows Test-driven Development to guide implementation and uses unit tests to test the code coverage written using Google Test framework. The code follows doxygen-formatted comments to aid doxygen documentation.


The project consists of 4 sprints spread across 4 weeks. The first week included the creation of a product backlog using the requirements and sorting them in the order of their priority. The second and third week consisted of class development and the final week was for refactoring and presentation video. In detail SIP planning can be found in the [SIP](#solo-iterative-process) sub heading.

## Demonstration of the Project

Here we show the object segregation of two differently colored disc configurations. The notion here is that the disc should be segregated onto a similar colored tables. For our project we have considered three colors, namely red, blue and green. The green colored disc is regarded as faulty and therefore is not picked up. (The reason is, that the table can very well be replaced by a moving conveyor and if the disc is faulty, the disc can be guided into a faulty part bin at the end of the conveyor)

The Blue Red Disc configuration can be seen below:

<p align="center">
<img src="https://github.com/Ghost1995/object_collection_robotic_arm/blob/master/additional_files/BR.gif">
</p>

If both the discs are of same color, then the second disc will go to a different position. This can be seen in the Red Red Disc configuration below:

<p align="center">
<img src="https://github.com/Ghost1995/object_collection_robotic_arm/blob/master/additional_files/RR.gif">
</p>

The Green Blue Configuration:

<p align="center">
<img src="https://github.com/Ghost1995/object_collection_robotic_arm/blob/master/additional_files/GB.gif">
</p>

We can see that the green disc is being detected as **Faulty** and therefore not picking it up.

Note: The robot seems to be moving very fast, this is because we are publishing on joint coordinated and therefore it is planning in joint coordinate space. This is much faster than the Cartesian planner. Also GIFs are playing with a 1.5X speed.

## About the Developers

This contributors for this project are:

* Anirudh Topiwala 

I am currently pursing my masters in Robotics at the University of Maryland,College Park. I hold a Bachelors degree in Mechanical Engineering from Institute of Technology, Nirma University, India. I wish to pursue a career in Robotics with a focus in Computer Vision. My resume and more about my projects can be found on [GitHub](https://github.com/anirudhtopiwala) and [LinkedIn](https://www.linkedin.com/in/anirudhtopiwala/).

* Ashwin Goyal

I am currently pursing my masters in Robotics at University of Maryland, College Park. I hold a Bachelors degree in Mechanical Engineering from Indian Institute of Technology Patna, India. I have worked for two automobile companies and a Robotics centered company in India. I have worked as a Technical Assistant for two courses at UMD. I wish to pursue a career in Robotics with a focus in Artificial Intelligence and Computer Vision. More about my projects can be found on [GitHub](https://github.com/Ghost1995/) and [LinkedIn](https://www.linkedin.com/in/ashwin-goyal/).

## Solo Iterative Process

Link to SIP Planning: [SIP Logs](https://docs.google.com/spreadsheets/d/1xqPFYUN3OQtDBTkjzTHtdPEkzV7B79HENrkyVhmQdCs/edit#gid=0)

Link to Sprint Planning Notes: [Sprint Notes](https://docs.google.com/document/d/1bKvMWFakLjcAiFi2MnL_bBGUG79L-Hk_sQdlqZ0YWP8/edit?usp=sharing)

## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu 16.04
* ROS Kinetic Kame
* Gazebo 7.x (part of ros-kinetic-desktop-full package)
* openCV

#### ROS Kinetic

* To install ROS Kinetic in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu). Make sure to install the recommended version (Desktop-Full).

* To install catkin, follow the installation steps for ROS kinetic in this [link](http://wiki.ros.org/catkin).

#### Ros Control Dependencies (REQUIRED)

Please install the required dependencies by running:
```
sudo apt-get install ros-kinetic-velocity-controllers ros-kinetic-ros-control
sudo apt-get install ros-kinetic-position-controllers ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-joint-trajectory-controller ros-kinetic-moveit
sudo apt install ros-kinetic-gazebo-ros-control
```

## Installation of Additional Packages (REQUIRED)

If you do not have a catkin workspace directory, to create a new one, run the following commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
Now, in your catkin workspace directory, install the required additional package by running the following commands:
```
cd ~/catkin_ws/src/
git clone https://github.com/anirudhtopiwala/iiwa_stack.git
```

## Build Instructions

To build this code in your catkin workspace (or a new one created using the [above instructions](#installation-of-additional-packages)), run the following commands:
```
cd ~/catkin_ws/src/
git clone https://github.com/Ghost1995/object_collection_robotic_arm.git
cd ..
catkin_make
source devel/setup.bash
```

## Running the Demo using Launch File

To run the demo, a launch file has been created. This launch file loads the Gazebo environment and runs the objSeg node to detect the objects on the table and segregate them onto the matching colored tables.
 
After following the [build instructions](#build-instructions), to run the demo, launch the code using the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch object_collection_robotic_arm kuka.launch
```
To change the colored disc configuration, we can pass an argument **Color** to the launch file. The argument will take the first initial of the color in capitals as its input. Therefore the possible color configurations would be:
```
roslaunch object_collection_robotic_arm kuka.launch Color:=RB
roslaunch object_collection_robotic_arm kuka.launch Color:=BR
roslaunch object_collection_robotic_arm kuka.launch Color:=RR
roslaunch object_collection_robotic_arm kuka.launch Color:=BB
roslaunch object_collection_robotic_arm kuka.launch Color:=GB
```
Note the default configuration is RB and if no Color argument is given it will open in the default Red Blue disc configuration.

We can pass a couple of more arguments to the launch file. The first argument is **rviz** which states whether to open Rviz when running the program or not. The second argument is **gui** which states whether to open Gazebo while running the program or not. In both cases, if the respective softwares are not opened, the software is still publishing their respective topics. By not opening these softwares in the foreground you save processing speed and memory. A sample of these arguments would be:
```
roslaunch object_collection_robotic_arm kuka.launch rviz:=true
roslaunch object_collection_robotic_arm kuka.launch gui:=false
```
Note that, by default, Rviz does not open (rviz is set as false) and Gazebo opens to show the movement of the Kuka robot (gui is set as true).

Also, note that, these arguments need not necessarily be given one at a time. Multiple arguments can be given when launching the program.

## Video Demo

Link to the video presentation uploaded on YouTube will be here shortly.

## Record bag File

A ros bag file records all the topic and messages being published in the terminal. After following the [build instructions](#build-instructions), to record the bag file, run the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch object_collection_robotic_arm kuka.launch record:=enable
```
The bag file, by default, is saved in the results directory as "[kuka.bag](https://github.com/Ghost1995/object_collection_robotic_arm/blob/master/results/kuka.bag)". Note that, the bag file does not contain any data published on the topic /camera/. This has been done to ensure that the bag file generated is not too big in size. Also, when playing the bag file, to observe the robot motion, the data published on the /camera/ topic is not required.

#### Inspecting the bag File Generated

To get more information about the generated rosbag file, run the following commands:
```
cd ~/catkin_ws/src/object_collection_robotic_arm/results/
rosbag info kuka.bag
```

#### Playing the bag File Generated

To play the bag file, you need ROS master to be running. So, in a new terminal, run the following command:
```
roscore
```
Then, in a new terminal, run the following commands:
```
cd ~/catkin_ws/src/object_collection_robotic_arm/results/
rosbag play kuka.bag
```
To check if the topic are live, in a new terminal, run the following commands:
```
source ~/catkin_ws/devel/setup.bash
rostopic list
```
To see the data available on any of the topics, in the same terminal, run the following command:
```
rostopic echo <topic_name>
```

#### Playing the bag File Generated to Observe Robot Motion

As stated [above](#playing-the-bag-file-generated), you can simply play the bag file. But, to get the intuition of what is actually happening you might need to see the motion being generated by the robot. To do so, follow the steps below:

After following the [build instructions](#build-instructions), launch the Gazebo world by running the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch
```
To only run Gazebo, and not Rviz, run
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch rviz:=false
```
You can also launch the worlds with different configurations of the discs as shown in "[Running the Demo using Launch File](#running-the-demo-using-launch-file)" section. Just remember that you would still be launching the launch file stated in this section.

Now, once Gazebo and Rviz (if opened) are running, in a new terminal, run
```
cd ~/catkin_ws/src/object_collection_robotic_arm/results/
rosbag play kuka.bag
```
Note that, this will only move the robot. As the services are not recorded in the bag file, the robot will not actually segregate the discs (interact with the discs). To do so, you need to activate the services on your own. To learn how to do that refer to "[Services](#services)" section.

## Services

The vacuum gripper used in the program has two services. These services are used to switch ON and switch OFF the gripper.

To switch ON the gripper, call the service by running the following commands:
```
source ~/catkin_ws/devel/setup.bash
rosservice call /robot/left_vacuum_gripper/on
```
Similarly, to switch OFF the gripper, call the service by running the following command:
```
source ~/catkin_ws/devel/setup.bash
rosservice call /robot/left_vacuum_gripper/off
```
Note that, both these services are part of the iiwa_stack package. So, before calling either of the services, you need to build the package by following the [build instructions](#build-instructions) and launch the package by running the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch
```
You can launch the package with different inputs as shown in "[Playing the bag File](#playing-the-bag-file-generated-to-observe-robot-motion)" section.

## Run Tests

There are 4 integration tests written for the objSeg node and added in the test directory. The first test checks class Detection, the second test checks class KukaKinematics and the remaining two tests check class KukaGripper. There are no level 1 tests as all the methods of all the classes are dependent on ROS nodes generated by the third-party package. Note that, these tests have been created using rostest.

### 1) Run the Tests while Compiling the Code

You can run the tests while building the code by running the following command:
```
catkin_make run_tests_object_collection_robotic_arm
```

### 2) Run the Tests using Launch File

After compiling the code by following the [above instructions](#run-the-tests-while-compiling-the-code) to generate the allTests node, to run the tests independently using the launch file, use the following commands:
```
source ~/catkin_ws/devel/setup.bash
rostest object_collection_robotic_arm allTests.test
```

### 3) Run the Tests using Test Node

After compiling the code by following the [aforementioned instructions](#run-the-tests-while-compiling-the-code) to generate the allTests node, you can run the tests using this node. But, you first need to start all the topics. To do so, use the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch
```
You can still give the arguments to this launch file as stated in the "[Playing the bag File](#playing-the-bag-file-generated-to-observe-robot-motion)" section.

Now, in a new terminal, run the following commands:
```
source ~/catkin_ws/devel/setup.bash
rosrun object_collection_robotic_arm allTests
```

## Code Coverage
![Code coverage](https://img.shields.io/badge/coverage-71.7%25-red.svg)
![Code coverage](https://img.shields.io/badge/coverage-100%25-green.svg)

(NOTE: THESE ARE SELF GENERATED TAGS, not to be confused with the automatic generated tag from coveralls. The first tag is with the Detection Class whereas the second one is without the Detection Class. We have put in two tags as the test for class Detection are currently not running.)

Since there is currently some issue with coveralls picking up the build from travis and checking for code coverage, provided below is a screen shot of the coverage report generated using lcov, locally.
For the First Coverage Tag:
![code_coverage](additional_files/lcov_coverage1.png)
For the Second Coverage Tag:
![code_coverage](additional_files/lcov_coverage2.png)

To run code coverage, you need lcov package. If this package is not installed, then to install the package, run the following command:
```
sudo apt-get install lcov
```
Now, code coverage report is generated by running the test node independently. So, first you need to generate the allTests node by compiling the code by following the [aforementioned instructions](#run-the-tests-while-compiling-the-code). As stated in "[Run the Tests using Test Node](#run-the-tests-using-test-node)" section, before running the node, you need to start all the topics. To do so, run the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch
```
You can still give the arguments to this launch file as stated in the "[Playing the bag File](#playing-the-bag-file-generated-to-observe-robot-motion)" section.

Now, in a new terminal, to generate code coverage, run the following command:
```
catkin_make code_coverage
```
To view the code coverage report generated, run the following commands:
```
cd ~/catkin_ws/build/coverage/
firefox index.html
```

## Plug-ins

##### CppChEclipse

If cppcheck package is not installed, then to install the package, run the following command:
```
sudo apt-get install cppcheck
```
To run cppcheck in Terminal, run the following commands:
```
cd ~/catkin_ws/src/object_collection_robotic_arm/
cppcheck --std=c++11 -I include/ --suppress=missingIncludeSystem $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./docs/" -e "^./launch/" -e "^./results/" -e "^./UML/" -e "./world/")
```

##### Google C++ Style

If cpplint package is not installed, then to install the package, run the following command:
```
sudo apt-get install python-pip
pip install cpplint
```
To check Google C++ Style formatting in Terminal, run the following commands:
```
cd ~/catkin_ws/src/object_collection_robotic_arm/
cpplint $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./docs/" -e "^./launch/" -e "^./results/" -e "^./UML/" -e "./world/")
```

<!-- ## Known Issues/Bugs

All the issues and bugs can be seen in the project report on GitHub at this [link](https://github.com/Ghost1995/object_collection_robotic_arm/projects/1).
 -->

## Generating Doxygen Documentation

To install doxygen run the following command:
```
sudo apt install doxygen
```
<!-- ```
cd ~/catkin_ws/src/object_collection_robotic_arm/
mkdir docs
doxygen -g config
```
Open the Doxygen configuration file "config" and update the following parameters:

* PROJECT_NAME = "object_collection_robotic_arm"

* INPUT = ./src ./include/ ./test

* OUTPUT_DIRECTORY = docs

Then, rename the "config" file to "doxconfig".
 -->
Now, to generate doxygen documentation, run the following commands:
```
cd ~/catkin_ws/src/object_collection_robotic_arm/
doxygen doxconfig
```
Doxygen files will be generated to /docs folder. To view them in a browser, run the following commands:
```
cd docs/html
firefox index.html
```

## License 

* OpenCV:  Copyright (C) 2015-2016, OpenCV Foundation, all rights reserved.
* Doxygen license: Copyright © 1997-2016 by Dimitri van Heesch.
* Google Test license: Copyright 2008, Google Inc.
* iiwa_stack license: Copyright (c) 2016-2017, Salvatore Virga - salvo.virga@tum.de

So, this software is released under the BSD 3-clause License.
```
 BSD 3-Clause License

Copyright (c) 2018, Ashwin Goyal
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Disclaimer

```
Past Performance may not be indicative of future results!

This program is dependent on several third-party packages. Therefore, you should
not assume that the future performance of the program will be equal to corresponding
past performance levels. The usage decision you make should be determined with
reference to the specific information available regarding the third-party packages,
and not based upon the past success of the program.

The contributors assumes no responsibility or liability for any errors or omissions
in the content of this code. The information contained on this site is provided on an
"as is" basis with no guarantee of completeness, accuracy, usefulness or timeliness and
without any warranties of any kind whatsoever, express or implied.

None of the authors or contributors, or anyone else connected with GitHub, in any way
whatsoever, can be responsible for your use of the information contained in or linked
from this web page.
```
