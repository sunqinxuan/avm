/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2021-10-11 09:06
#
# Filename:		avm.h
#
# Description: 
#
************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <map>
#include <queue>
#include <list>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <perception_msgs/PerceptionLocalization.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <sys/time.h>

namespace RESMAL
{
	class AVM
	{
	private:
		enum FOV 
		{
			NONE=0x00,
			RIGHT=0x01,
			FRONT=0x02,
			LEFT=0x04,
			REAR=0x08
		};

		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Publisher pub_avm_, pub_avm_adap_;
		cv::Mat img_right_, img_front_, img_left_, img_rear_, img_avm_, img_avm_adap_;

		struct INSLocation
		{
			INSLocation() {}
			INSLocation(double time,double pitch,double roll)
				:time_stamp_(time),pitch_angle_(pitch),roll_angle_(roll) {}
			double time_stamp_;
			double pitch_angle_, roll_angle_;
		};
		const int filter_wd_;
		std::list<INSLocation> ins_location_;
		std::queue<INSLocation> ins_location_filtered_;

		void callbackFisheyeImage(const sensor_msgs::CompressedImage::ConstPtr& msg);
		void callbackPerceptionLocalization(const perception_msgs::PerceptionLocalization::ConstPtr& msg);

		boost::array<double,3> offset_right_, offset_front_, offset_left_, offset_rear_; 
		void callbackEulerRight(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void callbackEulerFront(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void callbackEulerLeft(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void callbackEulerRear(const std_msgs::Float32MultiArray::ConstPtr& msg);

		double pitch_,roll_;

	public:
		AVM();
		~AVM();

		bool run();

		// for debugging;
		timeval runtime_start,runtime_end;
		double runtime;
		std::ofstream fp;

	private:

		// width of the avm image;
		constexpr static double avm_width_=16.0; // [m]
		// AVM image: avm_resolution_*avm_resolution_;
		constexpr static int avm_resolution_=600;
		// (cx_avm_,cy_avm): vertical projection of the origin of the vehicle frame onto the avm image;
		// offset [m]: physical distance between (cx_avm_,cy_avm) and (avm_resolution_/2,avm_resolution_/2) along u axis;
		const double offset_, cx_avm_, cy_avm_;

		// distance between two back-wheels;
		const double dist_backwheel_; // [m]
		// height of the back-wheel center;
		const double height_backwheel_; // [m]

		// transformation from wheel to vehicle coordinate system;
//		Eigen::Isometry3d trans_vehicle_wheel_;
		Eigen::Isometry3d trans_wheel_vehicle_;
		Eigen::Isometry3d trans_ground_vehicle_;

		// extrinsic wheel-camera;
		Eigen::Isometry3d trans_wheel_camera_right_;
		Eigen::Isometry3d trans_wheel_camera_front_;
		Eigen::Isometry3d trans_wheel_camera_left_;
		Eigen::Isometry3d trans_wheel_camera_rear_;

		Eigen::Isometry3d trans_wheel_camera_right_calib_;
		Eigen::Isometry3d trans_wheel_camera_front_calib_;
		Eigen::Isometry3d trans_wheel_camera_left_calib_;
		Eigen::Isometry3d trans_wheel_camera_rear_calib_;

		// intrinsic camera;
		// [fx,fy,cx,cy,k1,k2,k3,k4,width,height];
		boost::array<double,10> camera_intrinsic_right_;
		boost::array<double,10> camera_intrinsic_front_;
		boost::array<double,10> camera_intrinsic_left_;
		boost::array<double,10> camera_intrinsic_rear_;

		// GenerateAVM
		// generate the avm image by inverse projection;
		bool GenerateAVM(const cv::Mat& img_right, const cv::Mat& img_front,
						 const cv::Mat& img_left,  const cv::Mat& img_rear, cv::Mat& img_avm);
		bool GenerateAVM_noFusion(const cv::Mat& img_right, const cv::Mat& img_front,
								  const cv::Mat& img_left,  const cv::Mat& img_rear, cv::Mat& img_avm);

		// Camera2Image 
		// project the point in the camera frame onto the image plane;
		// input:  cam - camera intrinsics [fx,fy,cx,cy,k1,k2,k3,k4,width,height];
		//         pc  - 3D point in the camera frame;
		// output: u   - pixel coordinates on the image plane;
		bool Camera2Image(const boost::array<double,10>& cam, const Eigen::Vector3d& pc, Eigen::Vector2d& u);

		// Vehicle2Camera 
		// transform the point in the vehicle frame to the camera frame,
		// given the camera-vehicle extrinsics;
		// input:  T_cv - transfromation from the camera frame to left-back wheel frame;
		//         u    - pixel coordinates in the AVM image;
		// output: pc   - 3D coordinates in the camera frame;
		bool Vehicle2Camera(const Eigen::Isometry3d& T_wc, const Eigen::Vector2d& u, Eigen::Vector3d& pc);

		// BilinearInterp 
		// get the rgb value of a specific pixel 
		// via bilinear interpolation;
		// input:  img - color image captured by the camera;
		// 		   u   - pixel position where the rgb values are required;
		// output: rgb - the rgb value correponding to the pixel u;
		bool BilinearInterp(const cv::Mat& img, const Eigen::Vector2d& u, Eigen::Vector3d& rgb);
		double BilinearInterp_entry(int f_u1v1, int f_u1v2, int f_u2v1, int f_u2v2, double ku, double kv);

		// avmPixel
		// compute the pixel coordinates 
		// corresponding to the vertical projection of point pv;
		// pv is described w.r.t. the vehicle frame;
		Eigen::Vector2d avmPixel(const Eigen::Vector3d& pv);

		// inAngle 
		// incident angle between the incoming ray corresponding to u (avm pixel)
		// and the optical axis of the camera;
		double inAngle(const Eigen::Vector2d& u, const Eigen::Isometry3d& T_wc);

		// inFOV
		// for an avm pixel u,
		// return the region of FoV where u locates;
		unsigned char inFoV(const Eigen::Vector2d& u);
		
		// cameraWeight
		// compute the weight of the camera on pixel uc for the overlap fusion;
		double cameraWeight(const Eigen::Vector2d& uc, const boost::array<double,10>& cam);

	private:

		struct LookupValue
		{
			LookupValue():fov_(NONE),uc_(Eigen::Vector2d(-1,-1)){}
			LookupValue(const enum FOV& fov, const Eigen::Vector2d& u):fov_(fov),uc_(u){}
			enum FOV fov_;
			Eigen::Vector2d uc_;
		};

		class LookupTable 
		{
		public:
			LookupTable()
			{
//				for(int i=0;i<table_.size();i++)
//				{
//					table_[i]=LookupValue(NONE,Eigen::Vector2d(-1,-1));
//				}
			}

			~LookupTable(){}

			void insert(const int& u, const int& v, const LookupValue& val)
			{
				int idx=u*avm_resolution_+v;
				table_[idx]=val;
			}

			enum FOV FoV(const int& u, const int& v) const 
			{
				int idx=u*avm_resolution_+v;
				return table_[idx].fov_;
			}

			Eigen::Vector2d Pixel(const int& u, const int& v) const 
			{
				int idx=u*avm_resolution_+v;
				return table_[idx].uc_;
			}

		private:
			boost::array<LookupValue,avm_resolution_*avm_resolution_> table_;
		};

		// lookup_table_ 
		// generated in function GenerateAVM() or GenerateAVM_noFusion();
		boost::shared_ptr<LookupTable> lookup_table_;
		bool GenerateLookupTable();
		bool GenerateAVM_lookup(const cv::Mat& img_right, const cv::Mat& img_front,
								const cv::Mat& img_left,  const cv::Mat& img_rear, cv::Mat& img_avm);
	};
}
