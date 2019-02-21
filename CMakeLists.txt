

cmake_minimum_required(VERSION 2.8)
Project(kinect2grabber CXX)

set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE  Debug)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(INCLUDE           "${CMAKE_SOURCE_DIR}/include")


option(WITH_OPENCL "adds opencl support for freenect2" OFF)
if(${WITH_OPENCL})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_OPENCL")
endif()

option(WITH_CUDA "adds cuda support for freenect2" ON)
if(${WITH_CUDA})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_CUDA")
endif()

option(WITH_SERIALIZATION "adds support for boost serialization" OFF)
if(${WITH_SERIALIZATION})
	find_package(Boost REQUIRED)
	include_directories(${INCLUDE})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_SERIALIZATION")
	set(BOOST_LIBS "boost_serialization")
endif()

option(WITH_ROS "adds support for ROS" ON)
if(${WITH_ROS})
	find_package(catkin REQUIRED COMPONENTS   roscpp std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						message_generation )
	catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						cmake_modules
	)
	include_directories(${catkin_INCLUDE_DIRS} include)
endif()

option(WITH_PCL "adds pcl cloud support" ON)
if(${WITH_PCL})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_PCL")
	find_package(PCL REQUIRED)
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	include_directories(${PCL_INCLUDE_DIRS})
endif()

find_package(OpenCV     REQUIRED)
message("opencv ${OpenCV_INCLUDE_DIRS}")
#find_package(Freenect2  REQUIRE)
find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${INCLUDE})
include_directories(  ${freenect2_INCLUDE_DIRS})


#set(CMAKE_CXX_FLAGS "-D__STDC_CONSTANT_MACROS")
#
#
#
##include_directories("/home/hamit/ffmpeg_build/include/")
#find_package (FFmpeg REQUIRED)
#
## FFMPEG_FOUND - system has ffmpeg or libav
## FFMPEG_INCLUDE_DIR - the	 ffmpeg include directory
## FFMPEG_LIBRARIES
## FFMPEG_LIBAVCODEC
## FFMPEG_LIBAVFORMAT
## FFMPEG_LIBAVUTIL
#if(FFMPEG_FOUND)
#  message("Found FFMPEG/LibAV libraries ${FFMPEG_INCLUDE_DIR}")
#  include_directories(${FFMPEG_INCLUDE_DIR})
#  
#else()
#  message("Can't find libavcodec, libavformat or libavutil. Add them!")
#endif()

include_directories(/home/hamit/ffmpeg_build/include)
#include_directories(/home/hamit/NetBeansProjects/ThreeDContainer)
#link_directories(/home/hamit/NetBeansProjects/ThreeDContainer/dist/Release/GNU-Linux)

link_directories(/usr/lib/x86_64-linux-gnu/)
link_directories(/home/hamit/ffmpeg_build/lib)

#find_library(ffmpeg
#    NAMES avformat 
#    HINTS "/home/hamit/ffmpeg_build/lib"
#)
#
#message("ffmpeg ${ffmpeg}")

set(ffmpeg_lib_path /home/hamit/ffmpeg_build/lib)
#add_library(libavcodec STATIC IMPORTED) # or STATIC instead of SHARED
#set_target_properties(ThreeDContainer PROPERTIES
#  IMPORTED_LOCATION "/home/hamit/ffmpeg_build/lib/libavcodec.a"
#  INTERFACE_INCLUDE_DIRECTORIES "/home/hamit/ffmpeg_build/include"
#)

#find_package(PkgConfig REQUIRED)



add_executable(Kinect2Grabber      test.cpp)
add_executable(MultiKinect2Grabber multi_test.cpp)

target_link_libraries(Kinect2Grabber      ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
target_link_libraries(MultiKinect2Grabber ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

add_executable(retransformpcds   retransformpcds.cpp)
target_link_libraries(retransformpcds ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

add_executable(meshlab_cams   meshlab_cams.cpp)
target_link_libraries(meshlab_cams ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})




if(${WITH_ROS})
		add_executable(rosKinect2Grabber test_ros.cpp)
        target_link_libraries(rosKinect2Grabber ${catkin_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
        add_executable(rosMultiKinect2Publisher rosMultiKinect2Publisher.cpp)
        target_link_libraries(rosMultiKinect2Publisher   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS} )
        add_executable(rosMultiKinect2Subscriber    rosMultiKinect2Subscriber.cpp)
        target_link_libraries(rosMultiKinect2Subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS}   )
		add_executable(rosMultiKinect2Subscriber2    rosMultiKinect2Subscriber2.cpp)
        #target_link_libraries(rosMultiKinect2Subscriber2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS} draco ${ffmpeg_lib_path}/libavformat.a ${ffmpeg_lib_path}/libavcodec.a  ${ffmpeg_lib_path}/libswscale.a ${ffmpeg_lib_path}/libavutil.a ${ffmpeg_lib_path}/libavfilter.a  ${ffmpeg_lib_path}/libswresample.a ${ffmpeg_lib_path}/libavdevice.a ${ffmpeg_lib_path}/libpostproc.a pthread fdk-aac mp3lame  ass theora ogg  x264 x265 vpx vorbis vorbisfile vorbisenc theoraenc theoradec opus  z lzma bz2 rt)
        target_link_libraries(rosMultiKinect2Subscriber2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS} draco avformat avcodec swscale avutil avfilter swresample avdevice postproc pthread fdk-aac mp3lame  ass theora ogg  x264 x265 vpx vorbis vorbisfile vorbisenc theoraenc theoradec opus  z lzma bz2 rt )
		
		add_executable(rosPairKinectv2Viewer    rosPairKinectv2Viewer.cpp)	
		target_link_libraries(rosPairKinectv2Viewer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
		add_executable(rosPairKinectv2Viewer2    rosPairKinectv2Viewer2.cpp)	
		target_link_libraries(rosPairKinectv2Viewer2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
		
endif()

