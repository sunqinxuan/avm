/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2021-10-11 09:06
#
# Filename:		avm.cpp
#
# Description: 
#
************************************************/
#include "avm/avm.h"

namespace RESMAL 
{
	using namespace std;
	AVM::AVM() : //avm_width_(16.0), avm_resolution_(600), 
				 lookup_table_(new LookupTable()),
				 dist_backwheel_(1.5), height_backwheel_(0.32), offset_(1.3), 
//				 dist_backwheel_(1.51), height_backwheel_(0.25), offset_(1.0), 
				 cx_avm_(avm_resolution_/2.0+offset_*avm_resolution_/avm_width_), 
				 cy_avm_(avm_resolution_/2.0),
				 it_(nh_),
				 filter_wd_(7)
	{
		// set trans_wheel_vehicle_;
		// T_wv = [ 1 0 0 |  0  ]
		//        [ 0 1 0 | -d/2]
		//        [ 0 0 1 |  h  ]
		//        [ 0 0 0 |  1  ]
		trans_wheel_vehicle_.setIdentity();
		trans_wheel_vehicle_.translation()(1)=-0.5*dist_backwheel_;
		trans_wheel_vehicle_.translation()(2)=height_backwheel_;
		trans_ground_vehicle_=trans_wheel_vehicle_;

		YAML::Node yaml_extrinsic=YAML::LoadFile("data/calib_extrinsic.yaml");
		YAML::Node yaml_intrinsic_right=YAML::LoadFile("data/calib_right.yaml");
		YAML::Node yaml_intrinsic_front=YAML::LoadFile("data/calib_front.yaml");
		YAML::Node yaml_intrinsic_left=YAML::LoadFile("data/calib_left.yaml");
		YAML::Node yaml_intrinsic_rear=YAML::LoadFile("data/calib_rear.yaml");

		std::vector<double> tmp;
		Eigen::Quaterniond q;

		// [x,y,z,qw,qx,qy,qz]
		tmp=yaml_extrinsic["pose_wheel_camera_right"].as<std::vector<double>>();
		trans_wheel_camera_right_calib_.setIdentity();
		trans_wheel_camera_right_calib_.translation()(0)=tmp[0];
		trans_wheel_camera_right_calib_.translation()(1)=tmp[1];
		trans_wheel_camera_right_calib_.translation()(2)=tmp[2];
		q.w()=tmp[3]; 
		q.x()=tmp[4]; 
		q.y()=tmp[5];
		q.z()=tmp[6]; 
		q.normalize();
		trans_wheel_camera_right_calib_.linear()=q.toRotationMatrix();
		cout<<"trans_wheel_camera_right_calib_:"<<endl<<trans_wheel_camera_right_calib_.matrix()<<endl<<endl;
		trans_wheel_camera_right_=trans_wheel_camera_right_calib_;

		tmp=yaml_extrinsic["pose_wheel_camera_front"].as<std::vector<double>>();
		trans_wheel_camera_front_calib_.setIdentity();
		trans_wheel_camera_front_calib_.translation()(0)=tmp[0];
		trans_wheel_camera_front_calib_.translation()(1)=tmp[1];
		trans_wheel_camera_front_calib_.translation()(2)=tmp[2];
		q.w()=tmp[3]; 
		q.x()=tmp[4]; 
		q.y()=tmp[5]; 
		q.z()=tmp[6]; 
		q.normalize();
		trans_wheel_camera_front_calib_.linear()=q.toRotationMatrix();
		cout<<"trans_wheel_camera_front_calib_:"<<endl<<trans_wheel_camera_front_calib_.matrix()<<endl<<endl;
		trans_wheel_camera_front_=trans_wheel_camera_front_calib_;

		tmp=yaml_extrinsic["pose_wheel_camera_left"].as<std::vector<double>>();
		trans_wheel_camera_left_calib_.setIdentity();
		trans_wheel_camera_left_calib_.translation()(0)=tmp[0];
		trans_wheel_camera_left_calib_.translation()(1)=tmp[1];
		trans_wheel_camera_left_calib_.translation()(2)=tmp[2];
		q.w()=tmp[3]; 
		q.x()=tmp[4]; 
		q.y()=tmp[5]; 
		q.z()=tmp[6]; 
		q.normalize();
		trans_wheel_camera_left_calib_.linear()=q.toRotationMatrix();
		cout<<"trans_wheel_camera_left_calib_:"<<endl<<trans_wheel_camera_left_calib_.matrix()<<endl<<endl;
		trans_wheel_camera_left_=trans_wheel_camera_left_calib_;

		tmp=yaml_extrinsic["pose_wheel_camera_rear"].as<std::vector<double>>();
		trans_wheel_camera_rear_calib_.setIdentity();
		trans_wheel_camera_rear_calib_.translation()(0)=tmp[0];
		trans_wheel_camera_rear_calib_.translation()(1)=tmp[1];
		trans_wheel_camera_rear_calib_.translation()(2)=tmp[2];
		q.w()=tmp[3]; 
		q.x()=tmp[4];
		q.y()=tmp[5];
		q.z()=tmp[6];
		q.normalize();
		trans_wheel_camera_rear_calib_.linear()=q.toRotationMatrix();
		cout<<"trans_wheel_camera_rear_calib_:"<<endl<<trans_wheel_camera_rear_calib_.matrix()<<endl<<endl;
		trans_wheel_camera_rear_=trans_wheel_camera_rear_calib_;

		tmp=yaml_intrinsic_right["camera_matrix"]["data"].as<std::vector<double>>();
		camera_intrinsic_right_[0]=tmp[0];
		camera_intrinsic_right_[1]=tmp[4];
		camera_intrinsic_right_[2]=tmp[2];
		camera_intrinsic_right_[3]=tmp[5];
		tmp=yaml_intrinsic_right["distortion_coefficients"]["data"].as<std::vector<double>>();
		camera_intrinsic_right_[4]=tmp[0];
		camera_intrinsic_right_[5]=tmp[1];
		camera_intrinsic_right_[6]=tmp[2];
		camera_intrinsic_right_[7]=tmp[3];
		camera_intrinsic_right_[8]=yaml_intrinsic_right["image_width"].as<double>();
		camera_intrinsic_right_[9]=yaml_intrinsic_right["image_height"].as<double>();
		cout<<"camera_intrinsic_right_: "<<endl;
		for(int i=0;i<10;i++) cout<<camera_intrinsic_right_[i]<<" ";
		cout<<endl;

		tmp=yaml_intrinsic_front["camera_matrix"]["data"].as<std::vector<double>>();
		camera_intrinsic_front_[0]=tmp[0];
		camera_intrinsic_front_[1]=tmp[4];
		camera_intrinsic_front_[2]=tmp[2];
		camera_intrinsic_front_[3]=tmp[5];
		tmp=yaml_intrinsic_front["distortion_coefficients"]["data"].as<std::vector<double>>();
		camera_intrinsic_front_[4]=tmp[0];
		camera_intrinsic_front_[5]=tmp[1];
		camera_intrinsic_front_[6]=tmp[2];
		camera_intrinsic_front_[7]=tmp[3];
		camera_intrinsic_front_[8]=yaml_intrinsic_front["image_width"].as<double>();
		camera_intrinsic_front_[9]=yaml_intrinsic_front["image_height"].as<double>();
		cout<<"camera_intrinsic_front_: "<<endl;
		for(int i=0;i<10;i++) cout<<camera_intrinsic_front_[i]<<" ";
		cout<<endl;

		tmp=yaml_intrinsic_left["camera_matrix"]["data"].as<std::vector<double>>();
		camera_intrinsic_left_[0]=tmp[0];
		camera_intrinsic_left_[1]=tmp[4];
		camera_intrinsic_left_[2]=tmp[2];
		camera_intrinsic_left_[3]=tmp[5];
		tmp=yaml_intrinsic_left["distortion_coefficients"]["data"].as<std::vector<double>>();
		camera_intrinsic_left_[4]=tmp[0];
		camera_intrinsic_left_[5]=tmp[1];
		camera_intrinsic_left_[6]=tmp[2];
		camera_intrinsic_left_[7]=tmp[3];
		camera_intrinsic_left_[8]=yaml_intrinsic_left["image_width"].as<double>();
		camera_intrinsic_left_[9]=yaml_intrinsic_left["image_height"].as<double>();
		cout<<"camera_intrinsic_left_: "<<endl;
		for(int i=0;i<10;i++) cout<<camera_intrinsic_left_[i]<<" ";
		cout<<endl;

		tmp=yaml_intrinsic_rear["camera_matrix"]["data"].as<std::vector<double>>();
		camera_intrinsic_rear_[0]=tmp[0];
		camera_intrinsic_rear_[1]=tmp[4];
		camera_intrinsic_rear_[2]=tmp[2];
		camera_intrinsic_rear_[3]=tmp[5];
		tmp=yaml_intrinsic_rear["distortion_coefficients"]["data"].as<std::vector<double>>();
		camera_intrinsic_rear_[4]=tmp[0];
		camera_intrinsic_rear_[5]=tmp[1];
		camera_intrinsic_rear_[6]=tmp[2];
		camera_intrinsic_rear_[7]=tmp[3];
		camera_intrinsic_rear_[8]=yaml_intrinsic_rear["image_width"].as<double>();
		camera_intrinsic_rear_[9]=yaml_intrinsic_rear["image_height"].as<double>();
		cout<<"camera_intrinsic_rear_: "<<endl;
		for(int i=0;i<10;i++) cout<<camera_intrinsic_rear_[i]<<" ";
		cout<<endl;

		remove("debug.txt");
		fp.open("debug.txt",std::ios::app);

		pub_avm_=it_.advertise("/avm_image",7);
		pub_avm_adap_=it_.advertise("/avm_image_adaptive",7);

//		Eigen::AngleAxisd rot_x,rot_y,rot_z;
//		offset_right_[0]=0.2*M_PI/180.0; offset_right_[1]=0.0; offset_right_[2]=0.0;
//		rot_x=Eigen::AngleAxisd(offset_right_[0],Eigen::Vector3d::UnitX());
//		rot_y=Eigen::AngleAxisd(offset_right_[1],Eigen::Vector3d::UnitY());
//		rot_z=Eigen::AngleAxisd(offset_right_[2],Eigen::Vector3d::UnitZ());
//		trans_wheel_camera_right_.linear()=trans_wheel_camera_right_calib_.linear()
//			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();
//
//		offset_front_[0]=0.0; offset_front_[1]=-0.1*M_PI/180.0; offset_front_[2]=0.0;
//		rot_x=Eigen::AngleAxisd(offset_front_[0],Eigen::Vector3d::UnitX());
//		rot_y=Eigen::AngleAxisd(offset_front_[1],Eigen::Vector3d::UnitY());
//		rot_z=Eigen::AngleAxisd(offset_front_[2],Eigen::Vector3d::UnitZ());
//		trans_wheel_camera_front_.linear()=trans_wheel_camera_front_calib_.linear()
//			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();
//
//		offset_left_[0]=-0.3*M_PI/180.0; offset_left_[1]=0.0; offset_left_[2]=-0.3*M_PI/180.0;
//		rot_x=Eigen::AngleAxisd(offset_left_[0],Eigen::Vector3d::UnitX());
//		rot_y=Eigen::AngleAxisd(offset_left_[1],Eigen::Vector3d::UnitY());
//		rot_z=Eigen::AngleAxisd(offset_left_[2],Eigen::Vector3d::UnitZ());
//		trans_wheel_camera_left_.linear()=trans_wheel_camera_left_calib_.linear()
//			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();
//
//		offset_rear_[0]=0.0; offset_rear_[1]=0.0; offset_rear_[2]=0.0;
//		rot_x=Eigen::AngleAxisd(offset_rear_[0],Eigen::Vector3d::UnitX());
//		rot_y=Eigen::AngleAxisd(offset_rear_[1],Eigen::Vector3d::UnitY());
//		rot_z=Eigen::AngleAxisd(offset_rear_[2],Eigen::Vector3d::UnitZ());
//		trans_wheel_camera_rear_.linear()=trans_wheel_camera_rear_calib_.linear()
//			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();

		pitch_=0;
		roll_=0;
	}

	AVM::~AVM()
	{
		fp.close();
	}

	bool AVM::run()
	{
		std::string topic="/mipi_four_cameras/compressed";
		ros::Subscriber sub_image=nh_.subscribe(topic,7,&AVM::callbackFisheyeImage,this);
		topic="/INS/cicv_location";
		ros::Subscriber sub_location=nh_.subscribe(topic,7,&AVM::callbackPerceptionLocalization,this);

		topic="/offset_right";
		ros::Subscriber sub_offset_right=nh_.subscribe(topic,7,&AVM::callbackEulerRight,this);
		topic="/offset_front";
		ros::Subscriber sub_offset_front=nh_.subscribe(topic,7,&AVM::callbackEulerFront,this);
		topic="/offset_left";
		ros::Subscriber sub_offset_left=nh_.subscribe(topic,7,&AVM::callbackEulerLeft,this);
		topic="/offset_rear";
		ros::Subscriber sub_offset_rear=nh_.subscribe(topic,7,&AVM::callbackEulerRear,this);

		GenerateLookupTable();

		ros::Rate loop_rate(10); // 10Hz
		while(ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		return true;
	}

	void AVM::callbackEulerRight(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		offset_right_[0]=msg->data.at(0)*M_PI/180.0;
		offset_right_[1]=msg->data.at(1)*M_PI/180.0;
		offset_right_[2]=msg->data.at(2)*M_PI/180.0;
		Eigen::AngleAxisd rot_x(offset_right_[0],Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd rot_y(offset_right_[1],Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z(offset_right_[2],Eigen::Vector3d::UnitZ());
		trans_wheel_camera_right_.linear()=trans_wheel_camera_right_calib_.linear()
			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();
		std::cout<<"------------------------------------------"<<std::endl;
		std::cout<<"subscribed: right "<<offset_right_[0]<<" "<<offset_right_[1]<<" "<<offset_right_[2]<<std::endl;
		std::cout<<"------------------------------------------"<<std::endl;
	}

	void AVM::callbackEulerFront(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		offset_front_[0]=msg->data.at(0)*M_PI/180.0;
		offset_front_[1]=msg->data.at(1)*M_PI/180.0;
		offset_front_[2]=msg->data.at(2)*M_PI/180.0;
		Eigen::AngleAxisd rot_x(offset_front_[0],Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd rot_y(offset_front_[1],Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z(offset_front_[2],Eigen::Vector3d::UnitZ());
		trans_wheel_camera_front_.linear()=trans_wheel_camera_front_calib_.linear()
			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();
		std::cout<<"------------------------------------------"<<std::endl;
		std::cout<<"subscribed: front "<<offset_front_[0]<<" "<<offset_front_[1]<<" "<<offset_front_[2]<<std::endl;
		std::cout<<"------------------------------------------"<<std::endl;
	}

	void AVM::callbackEulerLeft(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		offset_left_[0]=msg->data.at(0)*M_PI/180.0;
		offset_left_[1]=msg->data.at(1)*M_PI/180.0;
		offset_left_[2]=msg->data.at(2)*M_PI/180.0;
//		offset_left_[3]=msg->data.at(3);
//		offset_left_[4]=msg->data.at(4);
//		offset_left_[5]=msg->data.at(5);
		Eigen::AngleAxisd rot_x(offset_left_[0],Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd rot_y(offset_left_[1],Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z(offset_left_[2],Eigen::Vector3d::UnitZ());
//		Eigen::Vector3d trns(offset_left_[3],offset_left_[4],offset_left_[5]);
		trans_wheel_camera_left_.linear()=trans_wheel_camera_left_calib_.linear()
			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();
//		trans_wheel_camera_left_.translation()=trans_wheel_camera_left_calib_.translation()+trns;
		std::cout<<"------------------------------------------"<<std::endl;
		std::cout<<"subscribed: left "<<offset_left_[0]<<" "<<offset_left_[1]<<" "<<offset_left_[2]<<std::endl;
		std::cout<<"------------------------------------------"<<std::endl;
	}

	void AVM::callbackEulerRear(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		offset_rear_[0]=msg->data.at(0)*M_PI/180.0;
		offset_rear_[1]=msg->data.at(1)*M_PI/180.0;
		offset_rear_[2]=msg->data.at(2)*M_PI/180.0;
		Eigen::AngleAxisd rot_x(offset_rear_[0],Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd rot_y(offset_rear_[1],Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z(offset_rear_[2],Eigen::Vector3d::UnitZ());
		trans_wheel_camera_rear_.linear()=trans_wheel_camera_rear_calib_.linear()
			  *rot_x.toRotationMatrix()*rot_y.toRotationMatrix()*rot_z.toRotationMatrix();
		std::cout<<"------------------------------------------"<<std::endl;
		std::cout<<"subscribed: rear "<<offset_rear_[0]<<" "<<offset_rear_[1]<<" "<<offset_rear_[2]<<std::endl;
		std::cout<<"------------------------------------------"<<std::endl;
	}


	void AVM::callbackFisheyeImage(const sensor_msgs::CompressedImage::ConstPtr& msg)
	{
//		double time=msg->header.stamp.sec+msg->header.stamp.nsec*1e-9;
//		while(true)
//		{
//			if(ins_location_filtered_.size()==0) break;
//			double delta=time-ins_location_filtered_.front().time_stamp_;
//			if(delta<=0 || fabs(delta)<0.01)
//			{
//				pitch_=(ins_location_filtered_.front().pitch_angle_-8.3e-4)*M_PI/180.0;
//				roll_=(ins_location_filtered_.front().roll_angle_+0.15)*M_PI/180.0;
//				break;
//			}
//			else 
//			{
//				ins_location_filtered_.pop();
//			}
//		}
//
//		cout<<std::fixed<<time<<"\t"<<pitch_*180.0/M_PI<<"\t"<<roll_*180.0/M_PI<<endl;
//		Eigen::AngleAxisd pitch_rot(-pitch_,Eigen::Vector3d::UnitY());
//		Eigen::AngleAxisd roll_rot(-roll_,Eigen::Vector3d::UnitX());
//		trans_ground_vehicle_.linear()=trans_wheel_vehicle_.linear()*roll_rot.toRotationMatrix()*pitch_rot.toRotationMatrix();

		cv_bridge::CvImageConstPtr ptr_img_4=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		cv::Mat img_4=ptr_img_4->image;

		img_4(cv::Rect(0,0,1280,720)).copyTo(img_right_);
		img_4(cv::Rect(1280,0,1280,720)).copyTo(img_front_);
		img_4(cv::Rect(2560,0,1280,720)).copyTo(img_left_);
		img_4(cv::Rect(3840,0,1280,720)).copyTo(img_rear_);

		// gettimeofday(&runtime_start,NULL);
		// GenerateAVM(img_right_,img_front_,img_left_,img_rear_,img_avm_);
		GenerateAVM_noFusion(img_right_,img_front_,img_left_,img_rear_,img_avm_adap_);
		// gettimeofday(&runtime_end,NULL);
		// runtime=(1000000*(runtime_end.tv_sec-runtime_start.tv_sec)+runtime_end.tv_usec-runtime_start.tv_usec);
		// cout<<"AVM runtime: "<<runtime/1000.0<<" ms"<<endl;r

		gettimeofday(&runtime_start,NULL);
		GenerateAVM_lookup(img_right_,img_front_,img_left_,img_rear_,img_avm_);
		gettimeofday(&runtime_end,NULL);
		runtime=(1000000*(runtime_end.tv_sec-runtime_start.tv_sec)+runtime_end.tv_usec-runtime_start.tv_usec);
		cout<<"AVM_lookup runtime: "<<runtime/1000.0<<" ms"<<endl;
		fp<<runtime/1000.0<<endl;

		sensor_msgs::ImagePtr msg_avm=cv_bridge::CvImage(msg->header,"bgr8",img_avm_).toImageMsg();
		sensor_msgs::ImagePtr msg_avm_adap=cv_bridge::CvImage(msg->header,"bgr8",img_avm_adap_).toImageMsg();
		pub_avm_.publish(msg_avm);
		pub_avm_adap_.publish(msg_avm_adap);
	}

	void AVM::callbackPerceptionLocalization(const perception_msgs::PerceptionLocalization::ConstPtr& msg)
	{
		double time=msg->header.stamp.sec+msg->header.stamp.nsec*1e-9;
		ins_location_.push_back(INSLocation(time,msg->pitch,msg->roll));
		ins_location_filtered_.push(INSLocation(time,msg->pitch,msg->roll));
//		if(ins_location_.size()>=filter_wd_)
//		{
//			std::list<INSLocation>::iterator it=ins_location_.end();
//			double pitch=0,roll=0;
//			for(int i=0;i<filter_wd_;i++)
//			{
//				it--;
//				pitch+=it->pitch_angle_;
//				roll+=it->roll_angle_;
//			}
//			pitch/=filter_wd_;
//			roll/=filter_wd_;
//			ins_location_filtered_.push(INSLocation(time,pitch,roll));
//			fp<<msg->pitch<<" "<<pitch<<std::endl;
//		}
	}

	bool AVM::GenerateLookupTable()
	{
		Eigen::Vector3d pc;
		Eigen::Vector2d uc;

		// (cx,cy): approximate projection center of the front camera;
		double cx=cx_avm_-2.5*avm_resolution_/avm_width_, cy=cy_avm_;
		
		for(int u=0;u<avm_resolution_;u++)
		{
			for(int v=0;v<avm_resolution_;v++)
			{
				double v_line_right_front=(u-cx)*(cy-avm_resolution_)/cx+cy;
				double v_line_right_rear=(u-cx_avm_)*(cy_avm_-avm_resolution_)/(cx_avm_-avm_resolution_)+cy_avm_;
				double v_line_left_front=(u-cx)*cy/cx+cy;
				double v_line_left_rear=(u-cx_avm_)*cy_avm_/(cx_avm_-avm_resolution_)+cy_avm_;

				if(v>cy_avm_ && v>v_line_right_front && v>v_line_right_rear)
				if(Vehicle2Camera(trans_wheel_camera_right_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_right_,pc,uc))
					{
						lookup_table_->insert(u,v,LookupValue(RIGHT,uc));
					}
				}

				if(u<cx_avm_ && v>v_line_left_front && v<v_line_right_front)
				if(Vehicle2Camera(trans_wheel_camera_front_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_front_,pc,uc))
					{
						lookup_table_->insert(u,v,LookupValue(FRONT,uc));
					}
				}

				if(v<cy_avm_ && v<v_line_left_front && v<v_line_left_rear)
				if(Vehicle2Camera(trans_wheel_camera_left_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_left_,pc,uc))
					{
						lookup_table_->insert(u,v,LookupValue(LEFT,uc));
					}
				}

				if(u>cx_avm_ && v>v_line_left_rear && v<v_line_right_rear)
				if(Vehicle2Camera(trans_wheel_camera_rear_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_rear_,pc,uc))
					{
						lookup_table_->insert(u,v,LookupValue(REAR,uc));
					}
				}
			}
		}
		return true;
	}

	bool AVM::GenerateAVM_lookup(const cv::Mat& img_right, const cv::Mat& img_front,
								 const cv::Mat& img_left,  const cv::Mat& img_rear, cv::Mat& img_avm)
	{
		Eigen::Vector3d rgb;
		Eigen::Vector2d uc;
		img_avm=cv::Mat::zeros(avm_resolution_,avm_resolution_,CV_8UC3);
		for(int u=0;u<avm_resolution_;u++)
		{
			for(int v=0;v<avm_resolution_;v++)
			{
				rgb.setZero();
				uc=lookup_table_->Pixel(u,v);
				switch(lookup_table_->FoV(u,v))
				{
				case RIGHT:
//					BilinearInterp(img_right,lookup_table_->Pixel(u,v),rgb);
					img_avm.at<cv::Vec3b>(u,v)=img_right.at<cv::Vec3b>((int)uc(1),(int)uc(0));
					break;
				case FRONT:
//					BilinearInterp(img_front,lookup_table_->Pixel(u,v),rgb);
					img_avm.at<cv::Vec3b>(u,v)=img_front.at<cv::Vec3b>((int)uc(1),(int)uc(0));
					break;
				case LEFT:
//					BilinearInterp(img_left,lookup_table_->Pixel(u,v),rgb);
					img_avm.at<cv::Vec3b>(u,v)=img_left.at<cv::Vec3b>((int)uc(1),(int)uc(0));
					break;
				case REAR:
//					BilinearInterp(img_rear,lookup_table_->Pixel(u,v),rgb);
					img_avm.at<cv::Vec3b>(u,v)=img_rear.at<cv::Vec3b>((int)uc(1),(int)uc(0));
					break;
				default:
					break;
				}
//				img_avm.at<cv::Vec3b>(u,v)[0]=(unsigned char)rgb(0);
//				img_avm.at<cv::Vec3b>(u,v)[1]=(unsigned char)rgb(1);
//				img_avm.at<cv::Vec3b>(u,v)[2]=(unsigned char)rgb(2);
			}
		}
		return true;
	}

	bool AVM::GenerateAVM_noFusion(const cv::Mat& img_right, const cv::Mat& img_front,
								   const cv::Mat& img_left,  const cv::Mat& img_rear, cv::Mat& img_avm)
	{
		img_avm=cv::Mat::zeros(avm_resolution_,avm_resolution_,CV_8UC3);
		Eigen::Vector3d pc;
		Eigen::Vector2d uc;
		Eigen::Vector3d rgb;

		// (cx,cy): approximate projection center of the front camera;
		double cx=cx_avm_-2.5*avm_resolution_/avm_width_, cy=cy_avm_;
		
		for(int u=0;u<avm_resolution_;u++)
		{
			for(int v=0;v<avm_resolution_;v++)
			{
				rgb.setZero();
				double v_line_right_front=(u-cx)*(cy-avm_resolution_)/cx+cy;
				double v_line_right_rear=(u-cx_avm_)*(cy_avm_-avm_resolution_)/(cx_avm_-avm_resolution_)+cy_avm_;
				double v_line_left_front=(u-cx)*cy/cx+cy;
				double v_line_left_rear=(u-cx_avm_)*cy_avm_/(cx_avm_-avm_resolution_)+cy_avm_;

				if(v>cy_avm_ && v>v_line_right_front && v>v_line_right_rear)
				if(Vehicle2Camera(trans_wheel_camera_right_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_right_,pc,uc))
					{
						BilinearInterp(img_right,uc,rgb);
					}
				}

				if(u<cx_avm_ && v>v_line_left_front && v<v_line_right_front)
				if(Vehicle2Camera(trans_wheel_camera_front_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_front_,pc,uc))
					{
						BilinearInterp(img_front,uc,rgb);
					}
				}

				if(v<cy_avm_ && v<v_line_left_front && v<v_line_left_rear)
				if(Vehicle2Camera(trans_wheel_camera_left_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_left_,pc,uc))
					{
						BilinearInterp(img_left,uc,rgb);
					}
				}

				if(u>cx_avm_ && v>v_line_left_rear && v<v_line_right_rear)
				if(Vehicle2Camera(trans_wheel_camera_rear_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_rear_,pc,uc))
					{
						BilinearInterp(img_rear,uc,rgb);
					}
				}

				img_avm.at<cv::Vec3b>(u,v)[0]=(unsigned char)rgb(0);
				img_avm.at<cv::Vec3b>(u,v)[1]=(unsigned char)rgb(1);
				img_avm.at<cv::Vec3b>(u,v)[2]=(unsigned char)rgb(2);
			}
		}
		return true;
	}

	bool AVM::GenerateAVM(const cv::Mat& img_right, const cv::Mat& img_front,
						  const cv::Mat& img_left,  const cv::Mat& img_rear, cv::Mat& img_avm)
	{
		img_avm=cv::Mat::zeros(avm_resolution_,avm_resolution_,CV_8UC3);
		Eigen::Vector3d pc;
		Eigen::Vector2d uc;
		boost::array<Eigen::Vector3d,4> rgb;
		boost::array<double,4> rho={0.0,0.0,0.0,0.0};
		unsigned char fov;
		
		for(int u=0;u<avm_resolution_;u++)
		{
			for(int v=0;v<avm_resolution_;v++)
			{
				fov=NONE;
				if(v>cy_avm_)
				if(Vehicle2Camera(trans_wheel_camera_right_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_right_,pc,uc))
					{
						fov=fov|RIGHT;
						BilinearInterp(img_right,uc,rgb[0]);
						rho[0]=cameraWeight(uc,camera_intrinsic_right_);
					}
				}
				if(u<cx_avm_)
				if(Vehicle2Camera(trans_wheel_camera_front_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_front_,pc,uc))
					{
						fov=fov|FRONT;
						BilinearInterp(img_front,uc,rgb[1]);
						rho[1]=cameraWeight(uc,camera_intrinsic_front_);
					}
				}
				if(v<cy_avm_)
				if(Vehicle2Camera(trans_wheel_camera_left_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_left_,pc,uc))
					{
						fov=fov|LEFT;
						BilinearInterp(img_left,uc,rgb[2]);
						rho[2]=cameraWeight(uc,camera_intrinsic_left_);
					}
				}
				if(u>cx_avm_)
//				if(fov==REAR)
				if(Vehicle2Camera(trans_wheel_camera_rear_,Eigen::Vector2d(u,v),pc))
				{
					if(Camera2Image(camera_intrinsic_rear_,pc,uc))
					{
						fov=fov|REAR;
						BilinearInterp(img_rear,uc,rgb[3]);
						rho[3]=cameraWeight(uc,camera_intrinsic_rear_);
					}
				}

				// overlap fusion;
				unsigned char cam=RIGHT;
				Eigen::Vector3d rgb_fuse(0,0,0);
				double den=0.0;
				for(int i=0;i<4;i++)
				{
					if(fov&cam)
					{
						int idx=log2((double)cam);
						rgb_fuse+=rho[idx]*rgb[idx];
						den+=rho[idx];
					}
					cam=cam<<1;
				}
				rgb_fuse/=den;

				img_avm.at<cv::Vec3b>(u,v)[0]=(unsigned char)rgb_fuse(0);
				img_avm.at<cv::Vec3b>(u,v)[1]=(unsigned char)rgb_fuse(1);
				img_avm.at<cv::Vec3b>(u,v)[2]=(unsigned char)rgb_fuse(2);
			}
		}
		return true;
	}

	bool AVM::Camera2Image(const boost::array<double,10>& cam, const Eigen::Vector3d& pc, Eigen::Vector2d& u)
	{
//		gettimeofday(&runtime_start,NULL);

		const double fx=cam[0];
		const double fy=cam[1];
		const double cx=cam[2];
		const double cy=cam[3];
		const double k1=cam[4];
		const double k2=cam[5];
		const double k3=cam[6];
		const double k4=cam[7];
		const double width=cam[8];
		const double height=cam[9];

		const double x=pc(0), y=pc(1), z=pc(2);
		if(fabs(z)<1e-3) { return false; }

		const double a=x/z, b=y/z;
		const double r=sqrt(a*a+b*b);
		const double theta=atan(r);
		double theta_sq=theta*theta;
		double theta_d=theta*(1+k1*theta_sq+k2*theta_sq*theta_sq
							   +k3*theta_sq*theta_sq*theta_sq
							   +k4*theta_sq*theta_sq*theta_sq*theta_sq);
		double x1=a*theta_d/r;
		double y1=b*theta_d/r;

		// TODO:
		// u(0)=fx*(x1+alpha*y1)+cx;
		u(0)=fx*(x1)+cx;
		u(1)=fy*y1+cy;

//		gettimeofday(&runtime_end,NULL);
//		runtime=(1000000*(runtime_end.tv_sec-runtime_start.tv_sec)+runtime_end.tv_usec-runtime_start.tv_usec);
//		fp<<"Camera2Image: "<<runtime/1000.0<<" ms"<<endl;

		if(u(0)>0 && u(0)<width-1 && u(1)>0 && u(1)<height-1)
		{
			return true;
		}
		else 
		{
			return false;
		}
	}

	bool AVM::Vehicle2Camera(const Eigen::Isometry3d& T_wc, const Eigen::Vector2d& u, Eigen::Vector3d& pc)
	{
//		gettimeofday(&runtime_start,NULL);

		const double cell_width=avm_width_/avm_resolution_;

		// T_cv: transformation from the vehicle to camera frame;
		const Eigen::Isometry3d T_cv=T_wc.inverse()*trans_ground_vehicle_;
		// Eigen::Isometry3d T_cw, T_wv;
		// T_cw.linear()=T_wc.linear().transpose();
		// T_cw.translation()=-T_cw.linear()*T_wc.translation();
		// T_wv.linear()=trans_vehicle_wheel_.linear().transpose();
		// T_wv.translation()=-T_wv.linear()*trans_vehicle_wheel_.translation();
		// const Eigen::Isometry3d T_cv=T_cw*T_wv;

		// 3D coordinates w.r.t. the vehicle frame, corresponding to the avm pixel (u,v);
		Eigen::Vector3d pv(cell_width*(cx_avm_-u(0)),cell_width*(cy_avm_-u(1)),-height_backwheel_);
		// 3D coordinates w.r.t. the camera frame;
		pc=T_cv.linear()*pv+T_cv.translation();

//		gettimeofday(&runtime_end,NULL);
//		runtime=(1000000*(runtime_end.tv_sec-runtime_start.tv_sec)+runtime_end.tv_usec-runtime_start.tv_usec);
//		fp<<"Vehicle2Camera: "<<runtime/1000.0<<" ms"<<endl;

		if(pc(2)>0) return true;
		else return false;
	}

	bool AVM::BilinearInterp(const cv::Mat& img, const Eigen::Vector2d& u, Eigen::Vector3d& rgb)
	{
//		gettimeofday(&runtime_start,NULL);

		int u_floor=floor(u(1)), u_ceil=ceil(u(1));
		int v_floor=floor(u(0)), v_ceil=ceil(u(0));

		double ku=(u(1)-u_floor)/(u_ceil-u_floor);
		double kv=(u(0)-v_floor)/(v_ceil-v_floor);
		rgb[0]=BilinearInterp_entry(img.at<cv::Vec3b>(u_floor,v_floor)[0], img.at<cv::Vec3b>(u_ceil,v_floor)[0],
									img.at<cv::Vec3b>(u_floor,v_ceil )[0], img.at<cv::Vec3b>(u_ceil,v_ceil )[0], ku,kv);
		rgb[1]=BilinearInterp_entry(img.at<cv::Vec3b>(u_floor,v_floor)[1], img.at<cv::Vec3b>(u_ceil,v_floor)[1],
									img.at<cv::Vec3b>(u_floor,v_ceil )[1], img.at<cv::Vec3b>(u_ceil,v_ceil )[1], ku,kv);
		rgb[2]=BilinearInterp_entry(img.at<cv::Vec3b>(u_floor,v_floor)[2], img.at<cv::Vec3b>(u_ceil,v_floor)[2],
									img.at<cv::Vec3b>(u_floor,v_ceil )[2], img.at<cv::Vec3b>(u_ceil,v_ceil )[2], ku,kv);

//		gettimeofday(&runtime_end,NULL);
//		runtime=(1000000*(runtime_end.tv_sec-runtime_start.tv_sec)+runtime_end.tv_usec-runtime_start.tv_usec);
//		fp<<"BilinearInterp: "<<runtime/1000.0<<" ms"<<endl;

		return true;
	}

	double AVM::BilinearInterp_entry(int f_u1v1, int f_u1v2, int f_u2v1, int f_u2v2, double ku, double kv)
	{
		Eigen::Vector2d matu;
		Eigen::Vector2d matv;
		Eigen::Matrix2d matf;
		matu<<1-ku,ku;
		matv<<1-kv,kv;
		matf<<f_u1v1,f_u1v2,f_u2v1,f_u2v2;
		double f = matu.transpose()*matf*matv;
		return f;
	}

	Eigen::Vector2d AVM::avmPixel(const Eigen::Vector3d& pv)
	{
		const double x=pv(0), y=pv(1);
		const double delta=avm_width_/avm_resolution_;
		
		Eigen::Vector2d u;
		u(0)=cx_avm_-x/delta;
		u(1)=cy_avm_-y/delta;
		return u;
	}

	double AVM::inAngle(const Eigen::Vector2d& u, const Eigen::Isometry3d& T_wc)
	{
		const Eigen::Isometry3d T_vc(trans_ground_vehicle_.inverse()*T_wc);

		// o_vc: the vertical projection on AVM w.r.t. the origin of the camera frame;
		Eigen::Vector2d o_vc=avmPixel(T_vc.translation());
		// z_vc: the vertical projection on AVM w.r.t. the (0,0,1) coordinates of the camera frame;
		Eigen::Vector2d z_vc=avmPixel(T_vc.translation()+T_vc.linear().block<3,1>(0,2));

		// ray: projection of incoming ray corresponding to the pixel u on AVM;
		// opt_ax: projection of optical axis on AVM;
		Eigen::Vector2d ray=u-o_vc, opt_ax=z_vc-o_vc;
		ray.normalize();
		opt_ax.normalize();

		double dot=ray.dot(opt_ax);
		if(dot>=1.0) dot=0.99999;
		else if(dot<=-1.0) dot=-0.99999;
		return acos(dot);
	}

	unsigned char AVM::inFoV(const Eigen::Vector2d& u)
	{
		unsigned char fov=NONE;

		if(inAngle(u,trans_wheel_camera_right_)<=M_PI/3.0)
		{
			fov=fov|RIGHT;
		}
		if(inAngle(u,trans_wheel_camera_front_)<=M_PI/3.0)
		{
			fov=fov|FRONT;
		}
		if(inAngle(u,trans_wheel_camera_left_)<=M_PI/3.0)
		{
			fov=fov|LEFT;
		}
		if(inAngle(u,trans_wheel_camera_rear_)<=M_PI/3.0)
		{
			fov=fov|REAR;
		}

		return fov;
	}

	double AVM::cameraWeight(const Eigen::Vector2d& uc, const boost::array<double,10>& cam)
	{
		const double width=cam[8];
		const double cx=cam[2];
		return fabs(width/2.0-fabs(uc(0)-cx));
	}


}
