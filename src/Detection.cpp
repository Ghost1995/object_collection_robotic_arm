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
 * @file Detection.cpp
 * @brief This is the declaration of the Detection class
 * @author Anirudh Topiwala [anirudhtopiwala] - driver
 * @author Ashwin Goyal [Ghost1995] - navigator
 * @date Dec 07, 2018
 */

#include "Detection.hpp"

// This is the constructor for the class
Detection::Detection(KukaKinematics & ku, const bool & display) : imgT(n),
                                                kuka(ku), dispImg(display) {
    // Subscribe to input video feed
    imageSubscriber = imgT.subscribe("/camera/image_raw", 1,
                                                    &Detection::readImg, this);
    // Display the image, if asked
    if (dispImg)
        cv::namedWindow(OPENCV_WINDOW);
}

// This is the first method of the class. It detects the position of a
// particularly colored object.
std::string Detection::colorThresholder(const KukaKinematics::States & pos) {
    auto posInd = static_cast<int>(pos);
    cv::Vec3b disc;

    // Define pixel for the corresponding disk
    if (posInd == 1) {
        disc = cv_ptr->image.at<cv::Vec3b>(199, 188);
    } else if (posInd == 2) {
        disc = cv_ptr->image.at<cv::Vec3b>(51, 190);
    } else {
        ROS_WARN_STREAM("The input to this method is incorrect. Instead of " <<
                        "position of the disc, the position of '" <<
                        kuka.statesStr.at(posInd) << "'' has been provided.");
        return "";
    }

    // Detect the color of the disk
    if ((disc.val[0] >= 125) && (disc.val[1] < 125) && (disc.val[2] < 125)) {
        return "blue";
    } else if ((disc.val[0] < 125) && (disc.val[1] >= 125) &&
                                                        (disc.val[2] < 125)) {
        return "green";
    } else if ((disc.val[0] < 125) && (disc.val[1] < 125) &&
                                                        (disc.val[2] >= 125)) {
        return "red";
    } else {
        ROS_WARN_STREAM("The color of disc cannot be uniquely identified.");
        return "";
    }
}

// This is the second method of this class. It is the image callback function
// which reads the image captured by the camera sensor.
void Detection::readImg(const sensor_msgs::ImageConstPtr & msg) {
    // Read the image
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window, if it exists
    if (dispImg)
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
}

// This is the destructor for the class
Detection::~Detection() {
    if (dispImg)
        cv::destroyWindow(OPENCV_WINDOW);
    ROS_WARN_STREAM("Image Capture Module has been Shut Down");
}
