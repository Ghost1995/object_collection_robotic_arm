#include<iostream>
#include "KukaKinematics.hpp"
int main(int argc, char **argv) {
 ros::init(argc, argv, "objSeg");
  ros::Time::init();
  KukaKinematics ku;

 ros::NodeHandle n;
 auto joints_sub = n.subscribe("/iiwa/joint_states",10,  &KukaKinematics::getJoints, &ku);
KDL::Frame cartpos;
KDL::JntArray inv;
// ros::Duration(5).sleep();

while (ros::ok()) {
ros::Duration(0.0011).sleep();

ROS_INFO_STREAM("hi");
cartpos = ku.evalKinematicsFK();
inv = ku.evalKinematicsIK(cartpos);
ROS_INFO_STREAM("hi");
ROS_INFO_STREAM("Hi"<< inv(1));
ros::spinOnce();
}

  return 0;
}
