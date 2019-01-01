/*
 * rosPairKinectv2Viewer.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: hamit
 */

#include "k2g_ros.h"

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <boost/range/algorithm.hpp>
#include <boost/program_options.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


class Receiver
{
public:
  enum Mode
  {
	IMAGE = 0,
	CLOUD,
	BOTH
  };

private:
	std::mutex lock;
	std::vector<std::tuple<std::string, std::string, std::string, std::string >> topicsDepthAndColorCamInfos_;

	const bool useExact, useCompressed;

	bool updateImage, updateCloud;
	bool save;
	bool running;
	size_t frame;
	const size_t queueSize;
	std::string frame_id1, frame_id2;
	cv::Mat color1, depth1, color2, depth2;
	std::vector<cv::Mat >  cameraSMatrixColor, cameraSMatrixDepth;
	cv::Mat lookupX, lookupY;
	std::map<std::string, std::pair<cv::Mat,cv::Mat>> mapSerialDepthLookups;
	std::map<std::string, std::pair<cv::Mat,cv::Mat>> mapSerialColorLookups;

//	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image > ExactSyncPolicy;
//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image > ApproximateSyncPolicy;

//	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

   typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproximateSyncPolicy;

	ros::NodeHandle nh;
	ros::AsyncSpinner spinner;
	// std::vector< image_transport::ImageTransport> it;

	image_transport::ImageTransport it;
	std::vector<boost::shared_ptr<image_transport::SubscriberFilter > > subDepthImages;
	std::vector<boost::shared_ptr<image_transport::SubscriberFilter> > subColorImages;
	std::vector<boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> > > syncsExact;
	std::vector< boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >> syncsApproximate;
//	std::vector< boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>>  subCameraSInfoColor, subCameraSInfoDepth;
	std::vector< sensor_msgs::CameraInfo::ConstPtr> subCamerasInfoColor, subCamerasInfoDepth;
//	std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo > >> subCameraSInfoColor, subCameraSInfoDepth;



//
//  image_transport::ImageTransport it;
//  boost::shared_ptr<image_transport::SubscriberFilter >  subDepthImages1;
//  boost::shared_ptr<image_transport::SubscriberFilter >  subDepthImages2;
//
//  boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> >  syncSExact;
//  boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > syncSApproximate;

  //std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::vector<std::string>  &serials, const bool useExact, const bool useCompressed) // @suppress("Class members should be properly initialized")
    : useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(10),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {

	    createTopicDepthandImagesCamInfosTuple(serials);
	    cameraSMatrixColor.resize(topicsDepthAndColorCamInfos_.size());
		cameraSMatrixDepth.resize(topicsDepthAndColorCamInfos_.size());


		params.push_back(cv::IMWRITE_JPEG_QUALITY);
		params.push_back(100);
		params.push_back(cv::IMWRITE_PNG_COMPRESSION);
		params.push_back(9);
		params.push_back(cv::IMWRITE_PNG_STRATEGY);
		params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
		params.push_back(0);


  }

  ~Receiver()
  {
  }

  void run()
  {
    start();
    stop();
  }





  void start()
  {

	  running = true;



    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");


    for (const auto & t:topicsDepthAndColorCamInfos_) {
    	subDepthImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<0>(t), queueSize, hints) );
    	subColorImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<1>(t), queueSize, hints) );

    //	subCameraSInfoDepth.push_back( boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, std::get<2>(t) , queueSize));
    //	subCameraSInfoColor.push_back( boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, std::get<3>(t), queueSize));
    	subCamerasInfoDepth.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<2>(t),nh) );
    	subCamerasInfoColor.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<3>(t),nh) );

    }




	std::transform (subCamerasInfoDepth.begin(), subCamerasInfoDepth.end(), std::inserter(mapSerialDepthLookups, mapSerialDepthLookups.end()),

  							[this](sensor_msgs::CameraInfo::ConstPtr camInfo )  {


  			  			    	cv::Mat tmpCamInfo;
  			  			    	readCameraInfo(camInfo, tmpCamInfo);

  			  			    	cv::Mat tmplookupX, tmplookupY;
  			  			    	createLookup(static_cast<size_t>(camInfo->width), static_cast<size_t>(camInfo->height), tmpCamInfo,
  			  			    			tmplookupX, tmplookupY );
  			  			    	std::string serial= stripStrTillEnd(camInfo->header.frame_id, "_");
  			  			    	std::cout<<"serial "<<serial<<std::endl;
  			  			    	return std::make_pair(serial, std::make_pair(tmplookupX, tmplookupY ));


  			});






    for (size_t i = 1; i < subDepthImages.size() ; ++i) {

    	 if(useExact)
    	  	    {
    	  	      syncsExact.push_back(boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
    	  	    		  *subDepthImages[0], *subDepthImages[i], *subColorImages[0], *subColorImages[i]
    	  	      ));
    	  	      syncsExact.back()->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    	  	    }
    	  	    else
    	  	    {
    		      syncsApproximate.push_back(boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
    		    		  *subDepthImages[0], *subDepthImages[i], *subColorImages[0], *subColorImages[i]
    		      ));
    	  	      syncsApproximate.back()->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    	  	    }


    }

//
//    for (size_t i = 0; i < subDepthImages.size() ; ++i) {
//
//    	if(useExact)
//    	    	  	    {
//    	    	  	      syncsExact.push_back(boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
//    	    	  	    		  *subDepthImages[i],  *subColorImages[i]
//
//    	    	  	      ));
//    	    	  	      syncsExact.back()->registerCallback(boost::bind(&Receiver::callback2, this, _1, _2));
//    	    	  	    }
//    	    	  	    else
//    	    	  	    {
//    	    		      syncsApproximate.push_back(boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
//    	    		    		  *subDepthImages[i],  *subColorImages[i]
//
//    	    		      ));
//    	    	  	      syncsApproximate.back()->registerCallback(boost::bind(&Receiver::callback2, this, _1, _2));
//    	    	  	    }
//
//
//
//
//    }




    spinner.start();

    std::chrono::milliseconds duration(1);

    while(!updateImage)
    {
      if(!ros::ok())
      {
        return;
      }

	//  std::cout<<" ros ok"<<std::endl;



      std::this_thread::sleep_for(duration);
    }

    imageViewer();
  }

  void stop()
  {
    spinner.stop();
    running = false;

  }

private:

  void callback2(const sensor_msgs::Image::ConstPtr imageDepth1, const sensor_msgs::Image::ConstPtr imageColor1  )
    {

	  cv::Mat depth1 , color1;

	  readImage(imageDepth1, depth1);
	  readImage(imageColor1, color1);
   //   ros::Duration(1).sleep();
	 std::cout<<"time "<<ros::Time::now()<<std::endl;
	 std::cout<<"header depth: "<<imageDepth1->header.frame_id<<std::endl;
	 std::cout<<"header color: "<<imageColor1->header.frame_id<<std::endl;

	 std::string serial= stripStrTillEnd(imageDepth1->header.frame_id, "_");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->height = depth1.rows;
	cloud->width = depth1.cols;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);


	lock.lock();
	updateImage = true;
	createCloud(depth1, mapSerialDepthLookups[serial].first, mapSerialDepthLookups[serial].second, cloud);
	writer.writeBinary(serial+"_"+std::to_string(imageDepth1->header.stamp.toSec()) +".pcd", *cloud);
	lock.unlock();




    }


  inline std::string stripStrTillEnd(const std::string & str, const std::string &delim ) {

      	  std::size_t found = str.find_last_of(delim);
      	  return str.substr(found+1);
   }


  void callback(const sensor_msgs::Image::ConstPtr imageDepth1, const sensor_msgs::Image::ConstPtr imageDepth2 ,
		  const sensor_msgs::Image::ConstPtr imageColor1, const sensor_msgs::Image::ConstPtr imageColor2 )
  {
    cv::Mat depth1, depth2 , color1, color2;


    readImage(imageDepth1, depth1);
    readImage(imageDepth2, depth2);
    readImage(imageColor1, color1);
    readImage(imageColor2, color2);


    lock.lock();
    frame_id1=stripStrTillEnd(imageDepth1->header.frame_id, "_");//std::to_string(imageDepth1->header.stamp.toNSec());;
    frame_id2=stripStrTillEnd(imageDepth2->header.frame_id, "_");
    this->color1 = color1;
	this->depth1 = depth1;
	this->color2 = color2;
	this->depth2 = depth2;
	updateImage = true;

	lock.unlock();




  }


//
  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }


  void createCloud(const cv::Mat &depth, const cv::Mat & lookupX, const cv::Mat & lookupY, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
     {
       const float badPoint = std::numeric_limits<float>::quiet_NaN();

       #pragma omp parallel for
       for(int r = 0; r < depth.rows; ++r)
       {
         pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
         const uint16_t *itD = depth.ptr<uint16_t>(r);

      //   const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
         const float y = lookupY.at<float>(0, r);
         const float *itX = lookupX.ptr<float>();

         for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itX)
         {
           register const float depthValue = *itD / 1000.0f;
          // std::cout<<" depth "<<depthValue<<std::endl;
           // Check for invalid measurements
           if(*itD == 0)
           {
             // not valid
             itP->x = itP->y = itP->z = badPoint;
            // itP->rgba = 0;
             continue;
           }
           itP->z = depthValue;
           itP->x = *itX * depthValue;
           itP->y = y * depthValue;
 //          itP->b = itC->val[0];
 //          itP->g = itC->val[1];
 //          itP->r = itC->val[2];
 //          itP->a = 255;
         }
       }
     }


  void createCloud(const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
    {
      const float badPoint = std::numeric_limits<float>::quiet_NaN();

      #pragma omp parallel for
      for(int r = 0; r < depth.rows; ++r)
      {
        pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);

     //   const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itX)
        {
          register const float depthValue = *itD / 1000.0f;
         // std::cout<<" depth "<<depthValue<<std::endl;
          // Check for invalid measurements
          if(*itD == 0)
          {
            // not valid
            itP->x = itP->y = itP->z = badPoint;
           // itP->rgba = 0;
            continue;
          }
          itP->z = depthValue;
          itP->x = *itX * depthValue;
          itP->y = y * depthValue;
//          itP->b = itC->val[0];
//          itP->g = itC->val[1];
//          itP->r = itC->val[2];
//          itP->a = 255;
        }
      }
    }



  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
	cameraMatrix  = cv::Mat::zeros(3, 3, CV_64F);
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }

  //  std::cout<<cameraMatrix<<std::endl;
  }

  cv::Mat readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo) const
  {
	 cv::Mat  cameraMatrix= cv::Mat::zeros(3, 3, CV_64F);
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }

 //   std::cout<<cameraMatrix<<std::endl;
    return cameraMatrix;
  }




  void createLookup(size_t width, size_t height, const cv::Mat & cameraMat, cv::Mat &lookupX, cv::Mat &lookupY)
  {
    const float fx = 1.0f / cameraMat.at<double>(0, 0);
    const float fy = 1.0f / cameraMat.at<double>(1, 1);
    const float cx = cameraMat.at<double>(0, 2);
    const float cy = cameraMat.at<double>(1, 2);
  //  std::cout<<"cam params: "<<fx<<" "<<fy<<" "<<cx<<" "<<cy<<std::endl;


    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }

  void createLookup(size_t width, size_t height, const cv::Mat & cameraMat)
  {
    const float fx = 1.0f / cameraMat.at<double>(0, 0);
    const float fy = 1.0f / cameraMat.at<double>(1, 1);
    const float cx = cameraMat.at<double>(0, 2);
    const float cy = cameraMat.at<double>(1, 2);
  //  std::cout<<"cam params: "<<fx<<" "<<fy<<" "<<cx<<" "<<cy<<std::endl;


    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }

 void createTopicDepthandImagesCamInfosTuple(const std::vector<std::string>  &serials) {

  	for(auto & s: serials ) {
  	  topicsDepthAndColorCamInfos_.push_back(std::make_tuple("/kinect2_" + s + "/sd/image_depth", "/kinect2_" + s + "/hd/image_color",
  			  "/kinect2_" + s + "/sd/camera_info", "/kinect2_" + s + "/hd/camera_info"));

  	}


  }


 void imageViewer()
  {
    cv::Mat color1, depth1, color2, depth2;
   cv::namedWindow("Color Image 1");
   cv::moveWindow("Color Image 1", 250,0);
   cv::namedWindow("Color Image 2");
   cv::moveWindow("Color Image 2", 800,0);
   // cv::namedWindow("Image Viewer");
   // oss << "starting...";

    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color1 = this->color1;
        depth1 = this->depth1;
        color2 = this->color2;
        depth2 = this->depth2;


        updateImage = false;
        lock.unlock();



        cv::imshow("Color Image 1", color1);
      //  cv::moveWindow("Color Image 1", 250,0);
        cv::imshow("Color Image 2", color2);
      //  cv::moveWindow("Color Image 2", 800,0);


//        cv::Mat matDst(cv::Size(color1.cols*2,color1.rows),color1.type(),cv::Scalar::all(0));
//        cv::Mat matRoi = matDst(cv::Rect(0,0,color1.cols,color1.rows));
//        color1.copyTo(matRoi);
//        matRoi = matDst(cv::Rect(color1.cols,0,color1.cols,color1.rows));
//        color2.copyTo(matRoi);
//
//        imshow("",color1);

      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
    	  printf("saving colors  \n");
    	  std::cout<<"size of dpth : "<<color1.elemSize()<<" "<< color1.elemSize1()<<std::endl;
    	  cv::imwrite(frame_id1+"_0.png", color1, params);
    	  cv::imwrite(frame_id2+"_1.png", color2, params);

    	  printf("saving depths \n");

    	  std::cout<<"size of dpth : "<<depth1.elemSize()<<" "<< depth1.elemSize1()<<std::endl;
    	  cv::imwrite(frame_id1+"_0_depth.png", depth1, params);
    	  cv::imwrite(frame_id2+"_1_depth.png", depth2, params);
          break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }





};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "kinect2_viewer");

	namespace po = boost::program_options;
	std::vector<std::string> serials;
	std::string serials_file;
	int postime;
	try {
		po::options_description desc("");
		desc.add_options()
		 ("serials,s", po::value<std::vector<std::string> >()->multitoken(), "kinectv2 serials")
		// ("postime", po::value<int>()->default_value(3), "the time to start grabber");
		 ("file_serial", po::value<std::string>(),"file containing serials");
		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		if (vm.count("file_serial") && vm.count("serials") ) throw po::error("both argument 'serials' and 'file_serials' are provided! Not possible!"); //std::cout<<"hellole "<<std::endl;
		else  if (vm.count("file_serial") && !vm.count("serials") )	serials_file = vm["file_serial"].as<std::string >();
		else  if (!vm.count("file_serial")   && vm.count("serials") ) serials = vm["serials"].as<std::vector<std::string> >();
		else throw po::error("Argument 'serials' or 'file_serials' must be provided!" );

	}
	catch (const po::error &ex)
	  {
	    std::cerr << ex.what() << '\n';
	 //   PCL_ERROR ("Any kinectv2 serial is not provided! Please enter serial to continue. Syntax is %s --serials serial1 serial2 ...\n", argv[0]);
	    exit(1);

	 }




	std::ifstream infile(serials_file);

	std::string serial;
	while (infile >> serial)
	{
	    serials.push_back(serial);
	 //   std::cout<<"serial "<<serial<<std::endl;

	}


	if(!ros::ok())
	{
	return 0;
	}

	bool useExact=false;
	bool useCompressed=true;


	Receiver receiver(serials , useExact, useCompressed);

	receiver.start();

	receiver.stop();



	ros::shutdown();
	return 0;
}
