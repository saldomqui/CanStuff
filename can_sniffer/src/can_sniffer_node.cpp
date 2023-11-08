/************************************************************************
 * by Salvador Dominguez 
 ************************************************************************/

/**
  \file can_sniffer.cpp
  \brief ROS node to detect can msgs that change
  \author Salvador Dominguez
  \date 23/2/2016
  */

//ROS
#include "ros/ros.h"

//The header of the class CarControl
#include "can_sniffer.h"

//Namespaces
using namespace std;

int main (int argc, char** argv)
{
  //Connect to ROS
  ros::init(argc, argv, "can_sniffer");
  ROS_INFO("Node can_sniffer Connected to roscore");

  //The controller object
  CanSniffer car_data;

  ros::spin();

  ROS_INFO("ROS-Node Terminated\n");
}


