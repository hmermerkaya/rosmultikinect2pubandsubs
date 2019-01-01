#include <chrono>
#include "k2g.h"
#include <pcl/pcl_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class K2GRos 
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	K2GRos(Processor freenect_processor = CPU, bool mirror = false, std::string serial = std::string()): k2g_(freenect_processor, mirror, serial),
			cloud_(new pcl::PointCloud<pcl::PointXYZRGB>(512, 424)),
			size_color_(1920, 1080),
		  	size_depth_(512, 424)
	{
		header_color_.frame_id = "kinect2_rgb_optical_frame_"+serial;
		header_depth_.frame_id = "kinect2_ir_optical_frame_"+serial;
		header_cloud_.frame_id = "kinect2_rgb_optical_frame_"+serial;
		
		point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect2_" + serial + "/hd/points", 10);
		color_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2_" + serial + "/hd/image_color", 1);
		color_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/kinect2_" + serial + "/hd/camera_info", 10);
		depth_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2_" + serial + "/sd/image_depth", 1);
		depth_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/kinect2_" + serial + "/sd/camera_info", 10);

		depth_pubCompressed_ = nh_.advertise<sensor_msgs::CompressedImage>("/kinect2_" + serial + "/sd/image_depth/compressed", 10);
		color_pubCompressed_ = nh_.advertise<sensor_msgs::CompressedImage>("/kinect2_" + serial + "/hd/image_color/compressed", 10);

		libfreenect2::Freenect2Device::IrCameraParams ir = k2g_.getIrParameters();
		libfreenect2::Freenect2Device::ColorCameraParams rgb = k2g_.getRgbParameters();

		createCameraInfoColor(rgb);
		createCameraInfoDepth(ir);
	}

	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishDepth(){
		cv::Mat tmp_depth;
		
		k2g_.getDepth(tmp_depth);
		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);
	}

	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishColor(){

		cv::Mat tmp_color;
		k2g_.getColor(tmp_color);

		header_color_.stamp = ros::Time::now();
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);
	}

	// Depth and color are aligned and registered 
	void publishDepthColor(const bool full_hd = true, const bool remove_points = false){
		
		cv::Mat tmp_depth, tmp_color;
		k2g_.get(tmp_depth, tmp_color, full_hd, remove_points);
		
		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);

		header_color_.stamp = ros::Time::now();
		
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);

	}

	void publishDepthColorCompressedAndCameraInfos(const ros::Time & tm = ros::Time::now(), const bool full_hd = true, const bool remove_points = false){

		initCompression(jpeg_quality, png_level, use_png);

		cv::Mat tmp_depth, tmp_color;
		k2g_.get(tmp_color, tmp_depth, full_hd, remove_points);
                //std::cout<<"tmp_depth "<<tmp_depth.rows<<std::endl;
		header_depth_.stamp = tm; //ros::Time::now();

        createCompressed(tmp_depth, header_depth_,DEPTH_SD, depth_imageCompressed);
              //  std::cout<<"after tmp_depth "<<tmp_depth.rows<<std::endl;

		depth_pubCompressed_.publish(depth_imageCompressed);

		header_color_.stamp = tm; //ros::Time::now();
		createCompressed(tmp_color, header_color_,COLOR_HD, color_imageCompressed);
		color_pubCompressed_.publish(color_imageCompressed);

		camera_info_depth_.header=header_color_;
		camera_info_color_.header=header_color_;
		depth_info_pub_.publish(camera_info_depth_);
		color_info_pub_.publish(camera_info_color_);


//		header_color_.stamp = tm;//ros::Time::now();
//		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
//		color_pub_.publish(color_image_);

	}


	// All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
	void publishAll(const bool full_hd = true, const bool remove_points = false){

		cv::Mat tmp_depth, tmp_color;

		k2g_.get(tmp_color, tmp_depth, cloud_, full_hd, remove_points);

		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);

		header_color_.stamp = ros::Time::now();
			
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);
		pcl::toROSMsg(*cloud_, point_cloud_2_);

		point_cloud_2_.header.frame_id = "world";

		point_cloud_2_.header.stamp = ros::Time::now();
		point_cloud_pub_.publish(point_cloud_2_);
	}

	void shutDown(){
		k2g_.shutDown();
	}

	bool terminate(){
		return stop;
	}

	void setShutdown(){
		stop = true;
	}
	
	void publishCameraInfoColor(){
		color_info_pub_.publish(camera_info_color_);
	}

	void publishCameraInfoDepth(){
		depth_info_pub_.publish(camera_info_depth_);
	}

	void mirror(){
		k2g_.mirror();
	}
   
private:
	  enum Image
	  {
	    IR_SD = 0,
	    IR_SD_RECT,

	    DEPTH_SD,
	    DEPTH_SD_RECT,
	    DEPTH_HD,
	    DEPTH_QHD,

	    COLOR_SD_RECT,
	    COLOR_HD,
	    COLOR_HD_RECT,
	    COLOR_QHD,
	    COLOR_QHD_RECT,

	    MONO_HD,
	    MONO_HD_RECT,
	    MONO_QHD,
	    MONO_QHD_RECT,

	    COUNT
	  };

	void initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png)
		  {
			compressionParams.resize(7, 0);
			compressionParams[0] = CV_IMWRITE_JPEG_QUALITY;
			compressionParams[1] = jpegQuality;
			compressionParams[2] = CV_IMWRITE_PNG_COMPRESSION;
			compressionParams[3] = pngLevel;
			compressionParams[4] = CV_IMWRITE_PNG_STRATEGY;
			compressionParams[5] = CV_IMWRITE_PNG_STRATEGY_RLE;
			compressionParams[6] = 0;

			if(use_png)
			{
			  compression16BitExt = ".png";
			  compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; png compressed";
			}
			else
			{
			  compression16BitExt = ".tif";
			  compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; tiff compressed";
			}
		}

	void createCompressed(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::CompressedImage &msgImage) const
	  {
	    msgImage.header = header;

	    switch(type)
	    {
	    case IR_SD:
	    case IR_SD_RECT:
	    case DEPTH_SD:
	    case DEPTH_SD_RECT:
	    case DEPTH_HD:
	    case DEPTH_QHD:
	      msgImage.format = compression16BitString;
	      cv::imencode(compression16BitExt, image, msgImage.data, compressionParams);
	      break;
	    case COLOR_SD_RECT:
	    case COLOR_HD:
	    case COLOR_HD_RECT:
	    case COLOR_QHD:
	    case COLOR_QHD_RECT:
	      msgImage.format = sensor_msgs::image_encodings::BGR8 + "; jpeg compressed bgr8";
	      cv::imencode(".jpg", image, msgImage.data, compressionParams);
	      break;
	    case MONO_HD:
	    case MONO_HD_RECT:
	    case MONO_QHD:
	    case MONO_QHD_RECT:
	      msgImage.format = sensor_msgs::image_encodings::TYPE_8UC1 + "; jpeg compressed ";
	      cv::imencode(".jpg", image, msgImage.data, compressionParams);
	      break;
	    case COUNT:
	      return;
	    }
	  }



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		return k2g_.getCloud();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
		return k2g_.updateCloud(cloud);
	}

	void createImage(cv::Mat & image, std_msgs::Header & header, sensor_msgs::Image & msgImage, const bool color) const
	{	
		size_t step, size;
		step = image.cols * image.elemSize();
		size = image.rows * step;
		if(color)
			msgImage.encoding = sensor_msgs::image_encodings::BGRA8;
		else
			msgImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

		msgImage.header = header;
		msgImage.height = image.rows;
		msgImage.width = image.cols;
		msgImage.is_bigendian = false;
		msgImage.step = step;
		msgImage.data.resize(size);

		memcpy(msgImage.data.data(), image.data, size);
	}


	void createCameraInfoColor(libfreenect2::Freenect2Device::ColorCameraParams color_params)
	{
	  cv::Mat proj_matrix_color = cv::Mat::zeros(3, 4, CV_64F);
	  cv::Mat camera_matrix_color = cv::Mat::eye(3, 3, CV_64F);
	  cv::Mat distortion_matrix_color = cv::Mat::zeros(1, 5, CV_64F);

	  camera_matrix_color.at<double>(0, 0) = color_params.fx;
	  camera_matrix_color.at<double>(1, 1) = color_params.fy;
	  camera_matrix_color.at<double>(0, 2) = color_params.cx;
	  camera_matrix_color.at<double>(1, 2) = color_params.cy;
	  camera_matrix_color.at<double>(2, 2) = 1;
	  camera_matrix_color.copyTo(proj_matrix_color(cv::Rect(0, 0, 3, 3)));
	  
	  createCameraInfo(size_color_, camera_matrix_color, distortion_matrix_color, cv::Mat::eye(3, 3, CV_64F), 
	  																   proj_matrix_color, camera_info_color_, true);
	}

	void createCameraInfoDepth(libfreenect2::Freenect2Device::IrCameraParams ir_params)
	{
		cv::Mat proj_matrix_depth = cv::Mat::zeros(3, 4, CV_64F);
		cv::Mat camera_matrix_depth = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat distortion_matrix_depth = cv::Mat::zeros(1, 5, CV_64F);

		camera_matrix_depth.at<double>(0, 0) = ir_params.fx;
		camera_matrix_depth.at<double>(1, 1) = ir_params.fy;
		camera_matrix_depth.at<double>(0, 2) = ir_params.cx;
		camera_matrix_depth.at<double>(1, 2) = ir_params.cy;
		camera_matrix_depth.at<double>(2, 2) = 1;
		camera_matrix_depth.copyTo(proj_matrix_depth(cv::Rect(0, 0, 3, 3)));
		
		createCameraInfo(size_depth_, camera_matrix_depth, distortion_matrix_depth, cv::Mat::eye(3, 3, CV_64F), 
																		   proj_matrix_depth, camera_info_depth_, false);
	}

	void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, 
						  const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo, const bool color ) const
	{

		if (color)
		{
			cameraInfo.header.frame_id = "kinect2_rgb_optical_frame";	
		}
		else
		{
			cameraInfo.header.frame_id = "kinect2_ir_optical_frame";	
		}
		cameraInfo.height = size.height;
		cameraInfo.width = size.width;

		const double *itC = cameraMatrix.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itC)
		{
			cameraInfo.K[i] = *itC;
		}

		const double *itR = rotation.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itR)
		{
			cameraInfo.R[i] = *itR;
		}

		const double *itP = projection.ptr<double>(0, 0);
		for(size_t i = 0; i < 12; ++i, ++itP)
		{
			cameraInfo.P[i] = *itP;
		}

		cameraInfo.distortion_model = "plumb_bob";
		cameraInfo.D.resize(distortion.cols);
		const double *itD = distortion.ptr<double>(0, 0);
		
		for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
		{
			cameraInfo.D[i] = *itD;
		}
	}

	ros::NodeHandle nh_;
	ros::Publisher point_cloud_pub_, color_pub_, color_pubCompressed_, color_info_pub_, depth_pub_, depth_pubCompressed_, depth_info_pub_;
    sensor_msgs::PointCloud2 point_cloud_2_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
    cv::Mat color_, depth_;
    cv::Size size_color_, size_depth_;
    std_msgs::Header header_depth_, header_color_, header_cloud_;
    sensor_msgs::Image depth_image_;
    sensor_msgs::CompressedImage depth_imageCompressed;
    sensor_msgs::Image::Ptr color_image_;
    sensor_msgs::CompressedImage color_imageCompressed;
    int32_t jpeg_quality=90, png_level=1;
    bool use_png=false;
    K2G k2g_;
    libfreenect2::SyncMultiFrameListener * listener_;
    sensor_msgs::CameraInfo  camera_info_color_, camera_info_depth_;
    std::vector<int> compressionParams;
    std::string compression16BitExt, compression16BitString, baseNameTF;



};
