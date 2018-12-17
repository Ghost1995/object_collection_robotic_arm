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
    Detection detect(kuka, true);

    // Initialize the node handle
    ros::NodeHandle n;

    // Create a flag for positions available on the table
    int tablePos[] = {2, 2};  // Left (red), Right (blue)

    // Wait for all windows to open
    ROS_INFO_STREAM("Waiting for all Windows to Open");
    ros::Duration(5).sleep();

    // Start the main function
    if (ros::ok()) {
        // Start from the Home Position
        ROS_INFO_STREAM("Going to Home Position");
        kuka.sendRobotToPos(kuka.HOME);

        // Read the color of the disks
        std::vector<std::string> color;
        color.push_back(detect.colorThresholder(kuka.LEFT_DISK));
        color.push_back(detect.colorThresholder(kuka.RIGHT_DISK));
        for (auto i = 0; i < 2; i++) {
            // Pick up the disc
            if ((color.at(i) == "red") || (color.at(i) == "blue")) {
                if (i == 0) {
                    ROS_INFO_STREAM("Going to the Left Disc");
                    kuka.sendRobotToPos(kuka.LEFT_DISK);
                } else if (i == 1) {
                    ROS_INFO_STREAM("Going to the Right Disc");
                    kuka.sendRobotToPos(kuka.RIGHT_DISK);
                }
                gripper.gripperToggle(true);
                ROS_INFO_STREAM("Going to Home Position");
                kuka.sendRobotToPos(kuka.HOME);

                // Place the disc appropriately
                if ((color.at(i) == "red") && (tablePos[0] > 0)) {
                    ROS_INFO_STREAM("Going to the Red Table");
                    if (tablePos[0] == 2) {
                        kuka.sendRobotToPos(kuka.LEFT_TABLE_POS_1);
                    } else if (tablePos[0] == 1) {
                        kuka.sendRobotToPos(kuka.LEFT_TABLE_POS_2);
                    }
                    tablePos[0] -= 1;
                } else if ((color.at(i) == "blue") && (tablePos[1] > 0)) {
                    ROS_INFO_STREAM("Going to the Blue Table");
                    if (tablePos[1] == 2) {
                        kuka.sendRobotToPos(kuka.RIGHT_TABLE_POS_1);
                    } else if (tablePos[1] == 1) {
                        kuka.sendRobotToPos(kuka.RIGHT_TABLE_POS_2);
                    }
                    tablePos[1] -= 1;
                } else {
                    ROS_ERROR_STREAM("At this instant, the code should have" <<
                                     " read the color of the disc as either" <<
                                     " 'red' or 'blue'. But, instead, the " <<
                                     "color of the disk is read as " <<
                                     color.at(i) << ". Check the algorithm " <<
                                     "for the mistake. [Line 113 - main.cpp]");
                    break;
                }
                gripper.gripperToggle(false);
                ROS_INFO_STREAM("Going to Home Position");
                kuka.sendRobotToPos(kuka.HOME);
            } else if (color.at(i) == "green") {
                if (i == 0) {
                    ROS_INFO_STREAM("Left Disc is a Faulty Disc");
                } else if (i == 1) {
                    ROS_INFO_STREAM("Right Disc is a Faulty Disc");
                }
            } else {
                if (i == 0) {
                    ROS_WARN_STREAM("The Color of the Left Disc is Unknown");
                } else if (i == 1) {
                    ROS_WARN_STREAM("The Color of the Right Disc is Unknown");
                }
            }
        }

        // Refresh all the topics before shutting down
        ros::spinOnce();
        ROS_WARN_STREAM("Closing all running systems");
        ros::Duration(5).sleep();
        ros::shutdown();
    } else {
        ROS_WARN_STREAM("ROS not running. Closing all systems.");
    }
    system("killall roscore & killall gzserver & killall gzclient");
    system("killall rosmaster");

    return 0;
}
