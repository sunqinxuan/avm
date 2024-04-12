/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2021-10-11 15:15
#
# Filename:		image_to_msg.cpp
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
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

class ImgPublisher
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher img_pub_;

	std::string folder_name;
	std::vector<std::string> image_filenames;

public:
	ImgPublisher(const std::string &folder): it_(nh_)
	{
		folder_name=folder;

		img_pub_=it_.advertise("/mipi_four_cameras",7);
//		img_pub_=nh_.advertise<sensor_msgs::CompressedImage>("/mipi_four_cameras/compressed",7);
	}

//	void setFolderName(const std::string &name) { folder_name=name; }

	~ImgPublisher()
	{
	}

	void run()
	{
		loadImage(folder_name);
		ros::Rate loop_rate(10);
		while(ros::ok())
		{
			for(int i=0;i<image_filenames.size();i++)
			{
				std::cout<<image_filenames[i].c_str()<<std::endl;
				publishImage(image_filenames[i].c_str());

				ros::spinOnce();
				loop_rate.sleep();
			}
		}
	}

private:

	void publishImage(const std::string &file_name)
	{
		std_msgs::Header header;
		header.stamp=ros::Time::now();
		header.frame_id="";
		std::string file_right=folder_name+file_name;

		cv::Mat img=cv::imread(file_right.c_str(),1);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header,"bgr8",img).toImageMsg();

		img_pub_.publish(msg);
	}

	void loadImage(const std::string &folder)
	{
		std::string file_name=folder+"img.txt";
		std::cout<<file_name<<std::endl;
		
		std::ifstream fp;
		fp.open(file_name.c_str());
		image_filenames.clear();
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				image_filenames.push_back(s);
			}
		}
		fp.close();
	}

}; 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imgPublisher", ros::init_options::AnonymousName);

	if(!ros::ok())
	{
		return 0;
	}

	std::string folder_name="./data/img/";

	ImgPublisher img_publisher(folder_name);

	img_publisher.run();

	ros::shutdown();
	return 0;
}

