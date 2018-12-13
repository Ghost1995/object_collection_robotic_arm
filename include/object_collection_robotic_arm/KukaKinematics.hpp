/*
 * BSD 3-Clause LICENSE
 *
 * Copyright (c) 2018, Anirudh Topiwala
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file KukaKinematics.hpp
 * @brief This is the declaration of the KukaKinematics class
 * @author Ashwin Goyal [Ghost1995] - driver
 * @author Anirudh Topiwala [anirudhtopiwala] - navigator
 * @date Nov 27, 2018
 */

#ifndef INCLUDE_OBJECT_COLLECTION_ROBOTIC_ARM_KUKAKINEMATICS_HPP_
#define INCLUDE_OBJECT_COLLECTION_ROBOTIC_ARM_KUKAKINEMATICS_HPP_

#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>
#include <iostream>

/*
 * @brief KukaKinematics is a class used for working with the Kuka robot
 *        manipulation
 */
class KukaKinematics {
 private:
    // Final motion commads sent to the robot
    trajectory_msgs::JointTrajectory jointCommands;
    // Number of kinematic joints in the robot
    const unsigned int numJoints = 7;
    // Total points that need to be traversed
    const unsigned int totalPoints = 7;
    // Joint values to achieve each point
    double posJoints[7][7];
    // Publish robot joints to move the robot
    ros::Publisher jointPublisher;
    // ROS node handle
    ros::NodeHandle n;

    /**
     * @brief This is a private method of this class. It initializes all the
     *        attributes of the trajectory message.
     *
     * @param This method does not take any inputs. It simply initializes all
     * the attributes to pre-defined values.
     *
     * @return This method does not return any argument. All the
     *         initializations are done for a private variable.
     */
    void initializeTrajectoryPoint();

 public:
    // Define the various states of the robot
    enum States {HOME, LEFT_DISK, RIGHT_DISK, LEFT_TABLE_POS_1,
                 LEFT_TABLE_POS_2, RIGHT_TABLE_POS_1, RIGHT_TABLE_POS_2};

    /*
     * @brief This is the constructor for the class
     */
    KukaKinematics();

    /*
     * @brief This is the first method of the class. It simply moves the robot
     *        from its current position to the desired position.
     *
     * @param This method takes the position that need to be reached as input.
     *
     * @result This method does not return anything. It plans the robot motion
     *         from one point to another and then publishes the joint
     *         trajectory to the robot.
     */
    void sendRobotToPos(const States);

    /*
     * @brief This is the destructor for the class
     */
    ~KukaKinematics();
};

#endif  // INCLUDE_OBJECT_COLLECTION_ROBOTIC_ARM_KUKAKINEMATICS_HPP_
