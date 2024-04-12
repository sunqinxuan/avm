/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2021-10-09 15:14
#
# Filename:		pub_euler_angle.cpp
#
# Description: 
#
************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <time.h>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pub_euler_angle", ros::init_options::AnonymousName);

	if(!ros::ok())
	{
		return 0;
	}

	if(argc!=8) 
	{
		std::cout<<"usage: pub_euler_angle camera angle_x angle_y angle_z"<<std::endl;
		return 0;
	}

	std::string topic = "/offset_"+std::string(argv[1]);

	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<std_msgs::Float32MultiArray>(topic,7);

	std_msgs::Float32MultiArray msg;
	msg.data.push_back(atof(argv[2]));
	msg.data.push_back(atof(argv[3]));
	msg.data.push_back(atof(argv[4]));
//	msg.data.push_back(atof(argv[5]));
//	msg.data.push_back(atof(argv[6]));
//	msg.data.push_back(atof(argv[7]));

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		pub.publish(msg);
		std::cout<<"publish: "<<topic<<" "<<msg.data.at(0)<<" "<<msg.data.at(1)<<" "<<msg.data.at(2)<<std::endl;
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::shutdown();
	return 0;
}

