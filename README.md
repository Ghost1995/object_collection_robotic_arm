# Object Collection Robotic Arm

[![Build Status](https://travis-ci.org/Ghost1995/Object_Collection_Robotic_Arm.svg?branch=master)](https://travis-ci.org/Ghost1995/Object_Collection_Robotic_Arm)
[![Coverage Status](https://coveralls.io/repos/github/Ghost1995/Object_Collection_Robotic_Arm/badge.svg?branch=master)](https://coveralls.io/github/Ghost1995/Object_Collection_Robotic_Arm?branch=master)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)
---

## Overview

This project is an implementation of a simulated robotic system capable of segregating objects based on their color. This project has been implemented on the Kuka robot using Gazebo, Rviz and ROS (and its various packages and libraries).

The developed system is capable of:
* detecting the objects placed on a table,
* picking up the objects using vacuum gripper and placing them into bins, and
* segregating the objects based on their color.

Few applications of the projects include:
* Object segregation in shipping companies, like Amazon, and Alibaba, and
* Transporting objects from one location to another.

This project has been programmed in the C++ programming language and uses C++ 11/14 features with emphasis given to Object Oriented Programming principles. The code follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with cpplint validation. Cppcheck was also used for static code analysis. This project followed Test-driven Development to guide implementation and uses unit tests to test the code coverage written using Google Test framework. The code follows doxygen-formatted comments to aid doxygen documentation.

The project was completed in about three weeks time. This project was implemented by following the Solo Iterative Process(SIP). This included the creation of a product backlog using the requirements and ordering them in the order of their priority. Finally, the software was developed over three sprints which were one week long in duration. The first week focused on the planning and design of the project inculding the initial UML diagrams. The second week involved the programming of the actual module. In the third week, the remainder of programming aspect of the project was completed and a video presentation was created.

## About the Developers

This project has been developed by two people.

- Anirudh Topiwala
I am Anirudh Topiwala. I am currently pursing my masters in Robotics at University of Maryland - College Park. I hold a Bachelors degree in Mechanical Engineering from Institute of Technology, Nirma University, India. I wish to pursue a career in Robotics with a focus in Computer Vision.
Few modest projects I was a part of:
Mechanical Engineer, Coding Enthusiast, Robotics Graduate Student at UMD. Research interests: object detection,deep learning,ML,etc. (Python,C++, Pytorch,Keras)

- Ashwin Goyal
I am Ashwin Goyal. I am currently pursing my masters in Robotics at University of Maryland - College Park. I hold a Bachelors degree in Mechanical Engineering from Indian Institute of Technology Patna, India. I have worked for two automobile companies, namely Honda Motorcycle and Scooter India Pvt. Ltd. and Maruti Suzuki India Ltd. I worked as summer trainee at both these companies. I also worked at a Robotics centered company, namely Robotech India Pvt. Ltd. I worked as a Technical Associate and worked on microcontrollers and their programmers for a period of 1 month. I am currently working as a Technical Assisstant for the Rehabilitation robotics course. I wish to pursue a career in Robotics with a focus in Artificial Intelligence and Computer Vision.

Few modest projects I was a part of:
* Visual Odometry in MATLAB with Computer Vision Toolbox.
* Lane Marker Detection in MATLAB with Computer Vision Toolbox.
* A-star Path Planning Algorithm using MATLAB.
* Autonomous Pole Balancing using Q-Learning in MATLAB.
* Crowd Estimation using Unmanned Aerial Vehicles using ROS and C++.

## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu (Xenial)
* ROS Kinetic
* Gazebo 7.x
* iiwa-stack
* openCV

#### ROS Kinetic

To install ROS Kinetic in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

#### Gazebo 7.x



#### iiwa-stack



#### openCV

Install OpenCV 3.3.0 using the following commands:

Install OpenCV Dependencies
```
sudo apt-get install build-essential checkinstall cmake pkg-config yasm gfortran git
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
## If you are using Ubuntu 14.04
sudo apt-get install libtiff4-dev
## If you are using Ubuntu 16.04
sudo apt-get install libtiff5-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install libqt4-dev libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
```
Download and Compile OpenCV
```
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.0 
cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.0
cd ..
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
## find out number of CPU cores in your machine
nproc
## substitute 4 by output of nproc
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```

## Solo Iterative Process

Link to SIP Planning: [SIP Logs]()

Link to Sprint Planning Notes: [Sprint Notes]()

## Video Demo

Link to the video presentation uploaded on YouTube: [Presentation Video]()

## Build Instructions

To build this code in a catkin workspace:
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Ghost1995/object_collection_robotic_arm.git
cd ..
catkin_make
```
Note, that if you do not have a catkin workspace, then to build this code use the following commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Ghost1995/object_collection_robotic_arm.git
cd ..
catkin_make
```

## Running the Demo using Launch File

To run the demo, a launch file has been created. This launch file loads the Gazebo environment and runs the objSeg node to detect the objects on the table and segregate them into bins based on their color.

After following the build instructions, to run the demo, launch the code using the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch object_collection_robotic_arm kuka_fwd.launch
```

## Record bag File

A ros bag file records all the topic and messages being published in the terminal. To record a bag file, just enable recording in the launch file using the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch object_collection_robotic_arm kuka_fwd.launch record:=enable
```
Note that, the bag file is saved in the results folder by default.

#### Inspecting the bag File Generated

To get more information about the generated rosbag file, such as the data being recorded, the time duration, the size of the bag file, and so on, use the following commands:
```
cd <path to directory>/results
rosbag info kuka.bag
```

#### Playing the bag File Generated

The rosbag file records all the messages being published on to the terminal. To check if the messages were recorded properly, you can playback the recorded rosbag file.

In a terminal, run "roscore" by using the following command:
```
roscore
```
Now, open a new terminal and use the following commands:
```
cd <path to directory>/results
source ~/catkin_ws/devel/setup.bash
rosbag play kuka.bag
```
Note that, while playing the rosbag file, make sure that Gazebo is not running.

## Run Tests

### 1) Run the Tests while Compiling the Code

You can run the tests while building the code by running the following commands:
```
cd ~/catkin_ws
catkin_make run_tests_object_collection_robotic_arm
```

### 2) Run the Tests after Compiling the Code

After compiling the code by following the build instructions, to run the tests independently, use the following commands:
```
cd ~/catkin_ws
source devel/setup.bash
rostest object_collection_robotic_arm kukaTests.test
```

### 3) Run the Tests after Running the Code

After running the code by following the run instructions, open a new terminal and use the following commands to run the tests:
```
cd ~/catkin_ws
source devel/setup.bash
rosrun object_collection_robotic_arm kukaTests
```

## Plugins

##### CppChEclipse

To run cppcheck in Terminal
```
cd <path to directory>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./launch/" -e "^./results/" -e "./world/")
```
##### Google C++ Sytle

To check Google C++ Style formatting in Terminal
```
cd <path to directory>
cpplint $(find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./launch/" -e "^./world/" -e "^./results")
```

## Known Issues/Bugs

All the issues and bugs can be seen in the project report on GitHub at this [link]().

## Generating Doxygen Documentation

To install doxygen run the following command: 
```
sudo apt-get install doxygen
```
Now, to generate doxygen documentation, run the following commands:
```
cd ~/catkin_ws/src/object_collection_robotic_arm
doxygen doxconfig
```
Doxygen files will be generated to /docs folder. To view them in a browser, run the following commands:
```
cd docs/html
google-chrome index.html
```

## Disclaimer

This software is released under the GNU Lesser General Public License v3.0.
```
                   GNU LESSER GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.


  This version of the GNU Lesser General Public License incorporates
the terms and conditions of version 3 of the GNU General Public
License, supplemented by the additional permissions listed below.

  0. Additional Definitions.

  As used herein, "this License" refers to version 3 of the GNU Lesser
General Public License, and the "GNU GPL" refers to version 3 of the GNU
General Public License.

  "The Library" refers to a covered work governed by this License,
other than an Application or a Combined Work as defined below.

  An "Application" is any work that makes use of an interface provided
by the Library, but which is not otherwise based on the Library.
Defining a subclass of a class defined by the Library is deemed a mode
of using an interface provided by the Library.

  A "Combined Work" is a work produced by combining or linking an
Application with the Library.  The particular version of the Library
with which the Combined Work was made is also called the "Linked
Version".

  The "Minimal Corresponding Source" for a Combined Work means the
Corresponding Source for the Combined Work, excluding any source code
for portions of the Combined Work that, considered in isolation, are
based on the Application, and not on the Linked Version.

  The "Corresponding Application Code" for a Combined Work means the
object code and/or source code for the Application, including any data
and utility programs needed for reproducing the Combined Work from the
Application, but excluding the System Libraries of the Combined Work.

  1. Exception to Section 3 of the GNU GPL.

  You may convey a covered work under sections 3 and 4 of this License
without being bound by section 3 of the GNU GPL.

  2. Conveying Modified Versions.

  If you modify a copy of the Library, and, in your modifications, a
facility refers to a function or data to be supplied by an Application
that uses the facility (other than as an argument passed when the
facility is invoked), then you may convey a copy of the modified
version:

   a) under this License, provided that you make a good faith effort to
   ensure that, in the event an Application does not supply the
   function or data, the facility still operates, and performs
   whatever part of its purpose remains meaningful, or

   b) under the GNU GPL, with none of the additional permissions of
   this License applicable to that copy.

  3. Object Code Incorporating Material from Library Header Files.

  The object code form of an Application may incorporate material from
a header file that is part of the Library.  You may convey such object
code under terms of your choice, provided that, if the incorporated
material is not limited to numerical parameters, data structure
layouts and accessors, or small macros, inline functions and templates
(ten or fewer lines in length), you do both of the following:

   a) Give prominent notice with each copy of the object code that the
   Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the object code with a copy of the GNU GPL and this license
   document.

  4. Combined Works.

  You may convey a Combined Work under terms of your choice that,
taken together, effectively do not restrict modification of the
portions of the Library contained in the Combined Work and reverse
engineering for debugging such modifications, if you also do each of
the following:

   a) Give prominent notice with each copy of the Combined Work that
   the Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the Combined Work with a copy of the GNU GPL and this license
   document.

   c) For a Combined Work that displays copyright notices during
   execution, include the copyright notice for the Library among
   these notices, as well as a reference directing the user to the
   copies of the GNU GPL and this license document.

   d) Do one of the following:

       0) Convey the Minimal Corresponding Source under the terms of this
       License, and the Corresponding Application Code in a form
       suitable for, and under terms that permit, the user to
       recombine or relink the Application with a modified version of
       the Linked Version to produce a modified Combined Work, in the
       manner specified by section 6 of the GNU GPL for conveying
       Corresponding Source.

       1) Use a suitable shared library mechanism for linking with the
       Library.  A suitable mechanism is one that (a) uses at run time
       a copy of the Library already present on the user's computer
       system, and (b) will operate properly with a modified version
       of the Library that is interface-compatible with the Linked
       Version.

   e) Provide Installation Information, but only if you would otherwise
   be required to provide such information under section 6 of the
   GNU GPL, and only to the extent that such information is
   necessary to install and execute a modified version of the
   Combined Work produced by recombining or relinking the
   Application with a modified version of the Linked Version. (If
   you use option 4d0, the Installation Information must accompany
   the Minimal Corresponding Source and Corresponding Application
   Code. If you use option 4d1, you must provide the Installation
   Information in the manner specified by section 6 of the GNU GPL
   for conveying Corresponding Source.)

  5. Combined Libraries.

  You may place library facilities that are a work based on the
Library side by side in a single library together with other library
facilities that are not Applications and are not covered by this
License, and convey such a combined library under terms of your
choice, if you do both of the following:

   a) Accompany the combined library with a copy of the same work based
   on the Library, uncombined with any other library facilities,
   conveyed under the terms of this License.

   b) Give prominent notice with the combined library that part of it
   is a work based on the Library, and explaining where to find the
   accompanying uncombined form of the same work.

  6. Revised Versions of the GNU Lesser General Public License.

  The Free Software Foundation may publish revised and/or new versions
of the GNU Lesser General Public License from time to time. Such new
versions will be similar in spirit to the present version, but may
differ in detail to address new problems or concerns.

  Each version is given a distinguishing version number. If the
Library as you received it specifies that a certain numbered version
of the GNU Lesser General Public License "or any later version"
applies to it, you have the option of following the terms and
conditions either of that published version or of any later version
published by the Free Software Foundation. If the Library as you
received it does not specify a version number of the GNU Lesser
General Public License, you may choose any version of the GNU Lesser
General Public License ever published by the Free Software Foundation.

  If the Library as you received it specifies that a proxy can decide
whether future versions of the GNU Lesser General Public License shall
apply, that proxy's public statement of acceptance of any version is
permanent authorization for you to choose that version for the
Library.
```

