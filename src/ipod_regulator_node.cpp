/* Quadcopter Control                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: September 2020                        */
/* File: mav_regulator_node.cpp                */

#include <ipod_regulator/ipod_regulator.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "ipod_regulator_node");
  ros::NodeHandle nh("~");

  IPodRegulator positionController(nh);
  ros::spin();

  return 0;
}