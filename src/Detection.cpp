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
 * @file Detection.cpp
 * @brief This is the declaration of the Detection class
 * @author Anirudh Topiwala [anirudhtopiwala] - driver
 * @author Ashwin Goyal [Ghost1995] - navigator
 * @date Dec 07, 2018
 */

#include "Detection.hpp"

// This is the constructor for the class
Detection::Detection(KukaKinematics & ku) : imgT(n), kuka(ku) {
    // Subscribe to input video feed
    imageSubscriber = imgT.subscribe("/camera/image_raw", 1,
                                                    &Detection::readImg, this);
    // Display the image
    cv::namedWindow(OPENCV_WINDOW);
}

// This is the first method of the class. It detects the position of a
// particularly colored object.
std::vector<KukaKinematics::States> Detection::colorThresholder(
                                                const std::string & color) {
    std::vector<KukaKinematics::States> value;
    auto rightDisc = cv_ptr->image.at<cv::Vec3b>(51, 190);
    auto leftDisc = cv_ptr->image.at<cv::Vec3b>(199, 188);

    // Detect red colored discs
    if (color == "red" ||color =="r" || color == "Red" || color == "R") {
        if (rightDisc.val[2] >= 125) {
            value.push_back(kuka.RIGHT_DISK);
        }
        if (leftDisc.val[2] >= 125) {
            value.push_back(kuka.LEFT_DISK);
        }
    }
    // Detect green colored discs
    if (color == "green" ||color =="g" || color == "Green" || color == "G") {
        if (rightDisc.val[1] >= 125) {
            value.push_back(kuka.RIGHT_DISK);
        }
        if (leftDisc.val[1] >= 125) {
            value.push_back(kuka.LEFT_DISK);
        }
    }
    // Detect blue colored discs
    if (color == "blue" ||color =="b" || color == "Blue" || color == "B") {
        if (rightDisc.val[0] >= 125) {
            value.push_back(kuka.RIGHT_DISK);
        }
        if (leftDisc.val[0] >= 125) {
            value.push_back(kuka.LEFT_DISK);
        }
    }

    return value;
}

// This is a private method of this class. It is the image callback function
// which reads the image captured by the camera sensor.
void Detection::readImg(const sensor_msgs::ImageConstPtr & msg) {
    // Read the image
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
}

// This is the destructor for the class
Detection::~Detection() {
    cv::destroyWindow(OPENCV_WINDOW);
    ROS_WARN_STREAM("Image Capture Module has been Shut Down");
}
