/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2021-09-29 15:18
#
# Filename:		bag_to_image.cpp
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

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
//#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <cv_bridge/cv_bridge.h>

class Bag2Image
{
private:
	ros::NodeHandle nh_;
	std::string folder_;

public:
	Bag2Image(const std::string &folder)
	{
		folder_=folder;
		std::cout<<"folder_name="<<folder_.c_str()<<std::endl;
		std::string mkdir_img="mkdir -p "+folder_+"/img";
		system(mkdir_img.c_str());
	}

	~Bag2Image(){}

	void run()
	{
		std::string topic="/avm_image";
		ros::Subscriber sub_cam = nh_.subscribe(topic, 1, &Bag2Image::callbackImage, this);

		ros::Rate loop_rate(3); // 10Hz
		while(ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

private:

	void callbackImage(const sensor_msgs::Image::ConstPtr& msgImg)
	{
		std::string file_name=folder_+"/img.txt";
		std::ofstream fp;
		fp.open(file_name.c_str(),std::ios::app);

		cv::Mat img;
		cv_bridge::CvImageConstPtr pCvImage = cv_bridge::toCvShare(msgImg, msgImg->encoding);
		pCvImage->image.copyTo(img);

		std::ostringstream name;
		name.setf(std::ios::fixed);
		double sec=msgImg->header.stamp.sec;
		double nsec=msgImg->header.stamp.nsec;
		double time_stamp=sec+nsec*1e-9;
		// fp<<std::fixed<<time_stamp<<" img/"<<std::fixed<<time_stamp<<".png"<<std::endl;
		fp<<std::fixed<<time_stamp<<".png"<<std::endl;
		name<<folder_<<"/img/"<<time_stamp<<".png";
		cv::imwrite(name.str().c_str(),img);
		std::cout<<name.str().c_str()<<std::endl;

		fp.close();
	}
}; 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bag_to_image", ros::init_options::AnonymousName);

	if(!ros::ok())
	{
		return 0;
	}

	std::string folder_name;

	time_t tt = time(NULL);
	struct tm* t= localtime(&tt);
	std::ostringstream ostr;
	ostr<<t->tm_year + 1900<<"-"<<t->tm_mon + 1<<"-" <<t->tm_mday<<"-"
		<<t->tm_hour<<"-" <<t->tm_min<<"-" <<t->tm_sec;
	folder_name="./"+ostr.str();
	std::cout<<"save images to "<<folder_name<<std::endl;

	Bag2Image bag_reader(folder_name);

	std::cout<<"awaiting rosbag play ..."<<std::endl;
	bag_reader.run();

	ros::shutdown();
	return 0;
}

