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
 * @file KukaGripper.hpp
 * @brief This is the declaration of the KukaGripper class
 * @author Anirudh Topiwala [anirudhtopiwala] - driver
 * @author Ashwin Goyal [Ghost1995] - navigator
 * @date Dec 07, 2018
 */

#ifndef INCLUDE_OBJECT_COLLECTION_ROBOTIC_ARM_KUKAGRIPPER_HPP_
#define INCLUDE_OBJECT_COLLECTION_ROBOTIC_ARM_KUKAGRIPPER_HPP_

#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <iostream>

// KukaGripper is a class used for working with the Kuka robot vacuum gripper.
// It basically controls the state of the gripper.
class KukaGripper {
 private:
    // ROS node handle
    ros::NodeHandle n;
    // Create service clients for switching ON and OFF the gripper
    ros::ServiceClient gripperOn, gripperOff;
    // Create a subscriber to check for the status of the gripper
    ros::Subscriber gripperSubscriber;
    // Check the status of the gripper
    bool gripperState = false;

    /*
     * @brief This is a private method of this class. It is the gripper
     *        callback function which checks the current state of the gripper.
     *
     * @param This method takes the message being published to the 'grasping'
     *        topic as input.
     *
     * @return This method does not return any argument. It simply updates the
     *         gripper state according to the message being read.
     */
    void gripperCallback(const std_msgs::Bool &);

 public:
    /*
     * @brief This is the constructor for the class.
     *
     * @param The constructor does not take any inputs. It creates a subscriber
     *        and two service clients.
     *
     * @return The constructor does not return anything. All initializations
     *         are done for the private variables.
     */
    KukaGripper();

    /*
     * @brief This is the first method of the class. It toggles the state of
     *        the gripper.
     *
     * @param This method takes state of the gripper that need to be achieved.
     *
     * @result This method does not return anything. It toggles the state by
     *         calling the gripper service using gripper client.
     */
    void gripperToggle(const bool &);

    /*
     * @brief This is the second method of the class. It gives the state of the
     *        gripper.
     *
     * @param This method does not take any input. It directly accesses the
     *        private variable defining the gripper state.
     *
     * @result This method returns the state of the gripper.
     */
    bool getGripperState();

    /*
     * @brief This is the destructor for the class.
     *
     * @param The desctructor does not take any inputs.
     *
     * @return The destructor does not return anything. It simply prints a
     *         string stating that this class is now closed.
     */
    ~KukaGripper();
};

#endif  // INCLUDE_OBJECT_COLLECTION_ROBOTIC_ARM_KUKAGRIPPER_HPP_
