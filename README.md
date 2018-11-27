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










To install ROS, use this [link](http://wiki.ros.org/kinetic/Installation)

To install turtlebot simulation stack. In a terminal:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
source /opt/ros/kinetic/setup.bash
```
To install gmapping, In a terminal:
```
sudo apt-get install ros-kinetic-slam-gmapping
```
To install Map_server, In a terminal:
```
 sudo apt-get install ros-kinetic-map-server
```



## SIP & Sprint Logs

Link to SIP Planning: [SIP Logs](https://docs.google.com/spreadsheets/d/1yglRR3HuQ96tQThB4AsiW9a2gjR8kYN-1Wcj_BuyqH0/edit?usp=sharing)

Link to Sprint Planning Notes: [Sprint Notes](https://docs.google.com/document/d/1rXK6foPKe-qIE33yUQAk5DwvEb2_a-keuWt45c4f9LM/edit)

Link to the video presentation uploaded on youtube: [Presentation Video](https://youtu.be/ouJtWGgyxr0)



## Build Steps
If you do not have a catkin workspace, in a new terminal:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/terrapinavigator.git
cd ..
catkin_make
```
If you wish to run this code in an existing catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/terrapinavigator.git
cd ..
catkin_make
```
## Running the Demo Using Launch File
To run the demo we need to run two launch files. First launch file loads the Gazebo environment and runs the terrapinavigator node to explore and map the environment. The seconds lauch file loads rviz (for visualization) and gmapping (for SLAM and Mapping).

After following the build instructions:

To run the demo, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch terrapinavigator terrapinavigator.launch 
```
This will load the turtlebot in the gazebo world and wait for 15 seconds. Now to run gmapping and Rviz, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch terrapinavigator demo.launch 
```

This will start the gmapping package and load rviz for visualization.

Note: You may close the gazebo window if your rig can't handle the load and continue to use rviz for visualization.

### Saving the Map
Once you are happy with the map created. To save a map, in a new terminal:
```
rosrun map_server map_saver -f <map_name>
```
To view the saved map. In a new terminal
```
eog <map_name>.pgm
```
### Running Picture Service From Command Line
The demo implementation is such that a picture is taken every 40 seconds and it gets stored in the ros folder with file name "termpImage" followed by the current time stamp at the time of takin the picture.

If you wish to manually click a picture while the demo is running at any desired time. In a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /pictureService "reqImg: true" 
```
If the image was taken succesfully you will see the following output:
```
clickImg: True
```
The files will get saved in ./ros folder. To view the files. In a terminal:
```
gnome-open ~/.ros
```
This will open the ./ros folder using the GUI and you can see all the pictures taken manually as well as those autmatically taken every 40 seconds in the demo.

Note: If you do not have gnome-open installed. In a terminal:
```
sudo apt install libgnome2-bin
```

### Recording, Inspecting and Playing back bag files
To enable bag file recording of all topics except camera. While launching the demo, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch terrapinavigator terrpainavigator.launch rec:=1
```
This will save a bag file in the results folder of the package.

To inspect the bag file, In a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/terrapinavigator/results
rosbag info recording.bag
```


To replay the bag file, first run rosmaster from a terminal:
```
roscore
```
Now, from the results folder run the following command in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/terrapinavigator/results
rosbag play recording.bag
```

## Running Rostest
To run rostest, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
cd build/
make run_tests
```
### Code Coverage Output Using lcov
![Code Coverage](https://user-images.githubusercontent.com/31521157/34026729-fee76d8e-e125-11e7-8342-f9dcf03d3111.png)
## Known Issues/Bugs 
* Unable to show code coverage using coveralls. Added lcov output to the readme instead
* When gazebo initializes it  throws some errors of missing plugins and thi followin error:
  SpawnModel: Failure - model name mobile_base already exist.
  However, these make no difference to the demo. The latter error maybe resolved once ros answers is back online.

## API and other developer documentation

### Generating Doxygen Documentation

To install doxygen run the following command: 
```
sudo apt-get install doxygen
```

Now from your cloned directory, run the following command:

```
doxygen terpDocs
```

Doxygen files will be generated to /docs folder

To view them in a browser:
```
cd docs
cd html
firefox index.html
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

