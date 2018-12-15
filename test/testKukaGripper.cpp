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
 * @file testKukaGripper.cpp
 * Copyright [2018] anirudhtopiwala [Anirudh Topiwala] - driver
 * Ghost1995 [Ashwin Goyal] - navigator
 * @date Dec 07, 2018
 * @brief It defines rostest and gtest for KukaGripper class.
 */

#include <gtest/gtest.h>
#include "KukaKinematics.hpp"
#include "KukaGripper.hpp"

// This is the google test for the second method of the class.
TEST(KukaGripperTest, testGetGripperState) {
    // Initialize the KukaGripper object
    KukaGripper test;

    // Check if the gripper state is initialized to false
    ASSERT_FALSE(test.getGripperState());
}

// This is the google test for the first method of the class.
TEST(KukaGripperTest, testGripperToggle) {
    // Initialize the KukaGripper and KukaKinematics object
    KukaGripper test;
    KukaKinematics robot;

    // Move the robot to the center disk
    robot.sendRobotToPos(robot.HOME);
    robot.sendRobotToPos(robot.RIGHT_DISK);

    // Change the gripper state
    test.gripperToggle(true);

    // Check if the state of the gripper was changed
    ASSERT_TRUE(test.getGripperState());

    // Change the gripper state
    test.gripperToggle(false);

    // Move the robot to the home position
    robot.sendRobotToPos(robot.HOME);
}
