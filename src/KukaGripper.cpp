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
 * @file KukaGripper.cpp
 * @brief This is the implementation of the KukaGripper class
 * @author Ashwin Goyal [Ghost1995] - driver
 * @author Anirudh Topiwala [anirudhtopiwala] - navigator
 * @date Dec 07, 2018
 */

#include "KukaGripper.hpp"

// This is the constructor for the class
KukaGripper::KukaGripper() {
    // Initialize the subscriber
    gripperSubscriber = n.subscribe("/robot/left_vacuum_gripper/grasping", 10,
                                        &KukaGripper::gripperCallback, this);
    // Initialize the service client for switching ON the gripper
    gripperOn = n.serviceClient<std_srvs::Empty>(
                                            "/robot/left_vacuum_gripper/on");
    // Initialize the service client for switching OFF the gripper
    gripperOff = n.serviceClient<std_srvs::Empty>(
                                            "/robot/left_vacuum_gripper/off");
}

// This is the first method of the class. It toggles the state of the gripper.
void KukaGripper::gripperToggle(const bool & state) {
    // Call the gripper service
    std_srvs::Empty empty;
    if (state) {
        gripperOn.call(empty);
        // Check if the gripper was activated
        while (!gripperState) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ros::Duration(2).sleep();
        ROS_INFO_STREAM("Gripper has been Switched ON");
    } else {
        gripperOff.call(empty);
        // Check if the gripper was deactivated
        while (gripperState) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ros::Duration(2).sleep();
        ROS_INFO_STREAM("Gripper has been Switched OFF");
    }
}

// This is the second method of the class. It gives the state of the gripper.
bool KukaGripper::getGripperState() {
    return gripperState;
}

// This is a private method of this class. It is the gripper callback function
// which checks the current state of the gripper.
void KukaGripper::gripperCallback(const std_msgs::Bool & state) {
    gripperState = state.data;
}

// This is the destructor for the class
KukaGripper::~KukaGripper() {
    ROS_WARN_STREAM("Gripper State Control Module has been Shut Down");
}
