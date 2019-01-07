/*
 *  retransformpcds.cpp
 *
 *  Created on: Jan 6, 2019
 *      Author: hamit
 */


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>

#include <algorithm>

#include <boost/range/algorithm.hpp>
#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/conditional_removal.h>


typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;


using namespace boost::filesystem;
using namespace std;

typedef std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> >  vecMatrix4f;




namespace fs = boost::filesystem;

typedef std::vector<fs::path> path_vec;

void sort_numeric(path_vec& v)
{
    std::sort(v.begin(), v.end()
        , [](fs::path const& a, fs::path const& b) {
        return std::stoi(a.filename().string()) < std::stoi(b.filename().string());
    });
}

path_vec sort_root_dir(fs::path const& p)
{
    path_vec dirs;

    for (fs::directory_iterator it(p); it != fs::directory_iterator(); ++it) {
        if (is_directory(*it)) {
            dirs.emplace_back(*it);
        }
    }

    sort_numeric(dirs);

    path_vec files;
    for (path_vec::const_iterator it(dirs.begin()), it_end(dirs.end()); it != it_end; ++it) {
        path_vec dir_files;
        std::copy(fs::directory_iterator(*it), fs::directory_iterator(), back_inserter(dir_files));
        sort_numeric(dir_files);
        files.insert(files.end(), dir_files.begin(), dir_files.end());
    }

    return files;
}


void readTransformFromText(const std::string &file, Eigen::Matrix<float, 4, 4, Eigen::RowMajor> &transform )
	 {




	        std::ifstream infile(file.c_str());
	        std::stringstream buffer;

	        buffer << infile.rdbuf();
	        float temp;
	        std::vector<float> matrixElements;
	        while (buffer >> temp ) {
	        matrixElements.push_back(temp);
	      //  std::cout<<"temp: "<<temp<<"\n";

	        }

	       transform= Eigen::Map<Eigen::Matrix<float,4,4,Eigen::RowMajor> >(matrixElements.data());
         // transform=Eigen::Map<Eigen::Matrix4f >(matrixElements.data());



	      infile.close();



	}

void produceTranformMatrices(const std::string &folder, vecMatrix4f & transformMatrices) {

	path p(folder);
	std::vector<std::string> filenames;

	try
	{
		if (exists(p))    // does p actually exist?
		{
			if (is_regular_file(p))        // is p a regular file?
			cout << p << " size is " << file_size(p) << '\n';

			else if (is_directory(p))      // is p a directory?
			{
			cout << p << " is a directory containing:\n";

			typedef std::vector<path> vec;             // store paths,
			vec v;                                // so we can sort them later

			copy(directory_iterator(p), directory_iterator(), back_inserter(v));

			sort(v.begin(), v.end());             // sort, since directory iteration
			vec::const_iterator it(v.begin()) ;

			for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
			{
						  if ((*it).string().find(".out") != std::string::npos) {
							  filenames.push_back((*it).string());
						  	  cout << "   " << *it << '\n';
						  }

			}



			}
			else
			  cout << p << " exists, but is neither a regular file nor a directory\n";
		}
		else
		cout << p << " does not exist\n";
	}

	catch (const filesystem_error& ex)
	{
	cout << ex.what() << '\n';
	}


	if (filenames.size() < 1 ) {
		std::cout<<"At least 1 files containing transformation matrix shoud exists \n";
		return ;
	}

	for (const auto & file:filenames) {
		Eigen::Matrix<float,4,4,Eigen::RowMajor> tmpMat;
		readTransformFromText(file, tmpMat );
		transformMatrices.push_back(tmpMat);
		std::cout<<"transform: \n"<<tmpMat<<std::endl;

	}




}
bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}



int main(int argc, char **argv) {
	pcl::PCDWriter writer_;
	pcl::PLYWriter plyWriter;

	  std::string input_filenames, output_dir;
	  if (pcl::console::parse_argument (argc, argv, "-input_filenames", input_filenames) != -1)
	  {
	    PCL_INFO ("Input filenames given as %s. Batch process mode on.\n", input_filenames.c_str ());
	    if (pcl::console::parse_argument (argc, argv, "-output_dir", output_dir) == -1)
	    {
	      PCL_ERROR ("Need an output directory! Please use -output_dir to continue.\n");
	      return (-1);
	    }

	    // Both input dir and output dir given, switch into batch processing mode

	  } else {

		  PCL_ERROR ("Need an input filenames ! Please use -input_filenames to continue.\n");
	      return (-1);
	  }


//		path p(input_dir);
//		std::vector<std::string> filenames;
//
//		try
//		{
//			if (exists(p))    // does p actually exist?
//			{
//				if (is_regular_file(p))        // is p a regular file?
//				cout << p << " size is " << file_size(p) << '\n';
//
//				else if (is_directory(p))      // is p a directory?
//				{
//				cout << p << " is a directory containing:\n";
//
////				typedef std::vector<path> vec;             // store paths,
////				vec v;                                // so we can sort them later
////
////				copy(directory_iterator(p), directory_iterator(), back_inserter(v));
////
////				sort(v.begin(), v.end(),mysort());             // sort, since directory iteration
////				vec::const_iterator it(v.begin()) ;
////
////				for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
////				{
////							  if ((*it).string().find(".pcd") != std::string::npos) {
////								  filenames.push_back((*it).string());
////							  	  cout << "   " << *it << '\n';
////							  }
////
////				}
//
//				path_vec files = sort_root_dir(input_dir);
//
//
//			    for (auto const& f : files) {
//			        std::cout << f << "\n";
//			    }
//
//				}
//				else
//				  cout << p << " exists, but is neither a regular file nor a directory\n";
//			}
//			else
//			cout << p << " does not exist\n";
//		}
//
//		catch (const filesystem_error& ex)
//		{
//		cout << ex.what() << '\n';
//		}

	    vecMatrix4f transformMatrices;

		produceTranformMatrices("./", transformMatrices);


		std::ifstream infile(input_filenames);

		std::string filename;
		while (infile >> filename)
		{
			for (int i=0; i<6; ++i) {
				std::string tmp=filename+"_"+std::to_string(i)+".pcd";
				//std::cout<<"filename: "<<tmp<<std::endl;
				if (is_file_exist(tmp.c_str())) {
					std::cout<<"file exists: "<<tmp<<std::endl;

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

					if (pcl::io::loadPCDFile<pcl::PointXYZ> (tmp, *cloud) == -1) //* load the file
					{
					PCL_ERROR ("Couldn't read file\n");
					return (-1);
					}


					try {
						  if (i!=0) pcl::transformPointCloud ( *cloud, *cloud, transformMatrices.at(i-1).inverse() );
					   //   std::cout<<"transform : \n"<<transMatrices_.at(i);
					  }
					  catch (const std::out_of_range& e) {
						  cout << "Out of Range error.";
					  }


					  float radius=2.5;
					  bool inside =true;
					  bool keep_organized=true;
					  pcl::ConditionOr<pcl::PointXYZ>::Ptr cond (new pcl::ConditionOr<pcl::PointXYZ> ());
					  cond->addComparison (pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::ConstPtr (new pcl::TfQuadraticXYZComparison<pcl::PointXYZ> (inside ? pcl::ComparisonOps::LT : pcl::ComparisonOps::GT, Eigen::Matrix3f::Identity (),
					                                                                                                                   Eigen::Vector3f::Zero (), - radius * radius)));

					  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
					  condrem.setCondition (cond);
					  condrem.setInputCloud (cloud);
					  condrem.setKeepOrganized (keep_organized);
					  condrem.filter (*cloud);




					// std::string command="system(/home/hamit/pcl-sw/pcl-trunk/build/bin/pcl_radius_filter -radius 2.5 " ;
					 //command = command + output_dir+"/"+tmp + " " + output_dir+"/"+tmp ;

//					 if (pcl::io::loadPCDFile<pcl::PointXYZ> (tmp, *cloud) == -1) //* load the file
//					 {
//					 		PCL_ERROR ("Couldn't read file\n");
//					 		return (-1);
//					 }
//
					try {
					  if (i!=0) pcl::transformPointCloud ( *cloud, *cloud, transformMatrices.at(i-1) );
					//   std::cout<<"transform : \n"<<transMatrices_.at(i);
					}
					catch (const std::out_of_range& e) {
					  cout << "Out of Range error.";
					}

					  writer_.writeBinaryCompressed (output_dir+"/"+tmp , *cloud);

					  plyWriter.write (output_dir+"/"+filename+"_"+std::to_string(i)+".ply", *cloud, true, false);




				}



			}






		}








}


