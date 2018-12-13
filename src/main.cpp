/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Ashwin Goyal
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * @file main.cpp
 * @brief This is the implementation of the KukaKinematics clas
 * @author Anirudh Topiwala [anirudhtopiwala]
 * @author Ashwin Goyal [Ghost1995] 
 * @date Nov 27, 2018
 */

#include <iostream>
#include "KukaKinematics.hpp"
#include "KukaGripper.hpp"
#include "Detection.hpp"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "objSeg");
    ros::Time::init();

    // Initialize class objects
    KukaKinematics kuka;
    KukaGripper gripper;
    Detection detect(kuka);

    // Initialize the node handle
    ros::NodeHandle n;

    // Make a vector of colors
    std::vector<std::string> colors = {"blue", "red", "green"};
    ROS_INFO_STREAM("Waiting for all Windows to Open");
    ros::Duration(10).sleep();

    // Start the main function
    while (ros::ok()) {
        // Start from the Home Position
        ROS_INFO_STREAM("Going to Home Position");
        kuka.sendRobotToPos(kuka.HOME);

        // Read all colored object positions
        auto blue = detect.colorThresholder(colors.at(0));
        auto red = detect.colorThresholder(colors.at(1));
        auto green = detect.colorThresholder(colors.at(2));

        // Move the blue disks, if any
        if (blue.empty()) {
            ROS_INFO_STREAM("There is no Blue Disk in the Workspace");
        } else if (blue.size() == 1) {
            ROS_INFO_STREAM("There is only one Blue Disk in the Workspace");
            ROS_INFO_STREAM("Going to the Blue Disc");
            kuka.sendRobotToPos(blue.at(0));
            gripper.gripperToggle(true);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to the Blue Table");
            kuka.sendRobotToPos(kuka.RIGHT_TABLE_POS_1);
            gripper.gripperToggle(false);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("All Blue Disks have been Successfully Placed");
        } else if (blue.size() == 2) {
            ROS_INFO_STREAM("There are two Blue Disks in the Workspace");
            ROS_INFO_STREAM("Going to Blue Disc 1");
            kuka.sendRobotToPos(blue.at(0));
            gripper.gripperToggle(true);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to the Blue Table");
            kuka.sendRobotToPos(kuka.RIGHT_TABLE_POS_1);
            gripper.gripperToggle(false);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to Blue Disc 2");
            kuka.sendRobotToPos(blue.at(1));
            gripper.gripperToggle(true);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to the Blue Table");
            kuka.sendRobotToPos(kuka.RIGHT_TABLE_POS_2);
            gripper.gripperToggle(false);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("All Blue Disks have been Successfully Placed");
        } else {
            ROS_INFO_STREAM("More than two disks have been identified as " <<
                            "Blue Disks. However, there are only two disks " <<
                            "in the workspace. Check the code.");
            return 0;
        }

        // Move the red disks, if any
        if (red.empty()) {
            ROS_INFO_STREAM("There is no Red Disk in the Workspace");
        } else if (red.size() == 1) {
            ROS_INFO_STREAM("There is only one Red Disk in the Workspace");
            ROS_INFO_STREAM("Going to the Red Disc");
            kuka.sendRobotToPos(red.at(0));
            gripper.gripperToggle(true);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to the Red Table");
            kuka.sendRobotToPos(kuka.LEFT_TABLE_POS_1);
            gripper.gripperToggle(false);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("All Red Disks have been Successfully Placed");
        } else if (red.size() == 2) {
            ROS_INFO_STREAM("There are two Red Disks in the Workspace");
            ROS_INFO_STREAM("Going to Red Disc 1");
            kuka.sendRobotToPos(red.at(0));
            gripper.gripperToggle(true);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to the Red Table");
            kuka.sendRobotToPos(kuka.LEFT_TABLE_POS_1);
            gripper.gripperToggle(false);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to Red Disc 2");
            kuka.sendRobotToPos(red.at(1));
            gripper.gripperToggle(true);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("Going to the Red Table");
            kuka.sendRobotToPos(kuka.LEFT_TABLE_POS_2);
            gripper.gripperToggle(false);
            ROS_INFO_STREAM("Going to Home Position");
            kuka.sendRobotToPos(kuka.HOME);
            ROS_INFO_STREAM("All Red Disks have been Successfully Placed");
        } else {
            ROS_INFO_STREAM("More than two disks have been identified as " <<
                            "Red Disks. However, there are only two disks " <<
                            "in the workspace. Check the code.");
            return 0;
        }

        // Identify the green disks, if any
        if (green.empty()) {
            ROS_INFO_STREAM("There is no Faulty Disk in the Workspace");
        } else if (green.size() == 1) {
            ROS_INFO_STREAM("There is only one Faulty Disk in the Workspace");
        } else if (green.size() == 2) {
            ROS_INFO_STREAM("There are two Faulty Disks in the Workspace");
        } else {
            ROS_INFO_STREAM("More than two disks have been identified as " <<
                            "Faulty Disks. However, there are only two " <<
                            "disks in the workspace. Check the code.");
            return 0;
        }

        // Refresh all the topics before shutting down
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::shutdown();
    }

    return 0;
}
