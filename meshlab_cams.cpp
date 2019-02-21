#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/vtk_io.h>

#include <pcl/visualization/cloud_viewer.h>

#include <tuple>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>


bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

void readTransform(const std::string &file,Eigen::Matrix4f &transform ){



  fstream binary_file(file.c_str(),ios::binary|ios::in);
  //while (binary_file.good()) {

  binary_file.read(reinterpret_cast<char *>(&transform),sizeof(Eigen::Matrix4f));
  //}

  binary_file.close();


  std::cout<<"tranform read \n "<<transform<<std::endl;

}

void readTransformFromText(const std::string &file, Eigen::Matrix4f &transform ){



  //fstream infile(file.c_str(), ios::in );
  //while (binary_file.good()) {
   
   std::ifstream infile(file.c_str());
    std::stringstream buffer;

    buffer << infile.rdbuf();
    float temp;
    std::vector<float> matrixElements;
    while (buffer >> temp) {
    matrixElements.push_back(temp);
  //  std::cout<<"temp: "<<temp<<"\n";

    }

     transform= Eigen::Map<Eigen::Matrix<float,4,4,Eigen::RowMajor> >(matrixElements.data());



  /*while (std::getline(infile, line)){
    
    float temp;
    std::istringstream iss(line);
    while (iss >> temp) {
     transform<<temp;
    }
     

  }*/
  //}

  infile.close();


 // std::cout<<"tranform read \n "<<transform<<std::endl;

}

void readTransformsBinary(const std::string &file,  std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &transforms) {

        size_t sz ;
        fstream FILE(file,ios::binary|ios::in);
        //while (binary_file.good()) {
        FILE.read(reinterpret_cast<char*>(&sz), sizeof(size_t));
       // std::cout<<"sz "<<sz<<std::endl;
        transforms.resize(sz);
        FILE.read(reinterpret_cast<char*>(&transforms[0]), sz * sizeof(transforms[0]));
        FILE.close();

}



void
  saveTransform (const std::string &file, const Eigen::Matrix4f &transform)
  {
    ofstream ofs;
    ofs.open (file.c_str (), ios::trunc | ios::binary);
    //for (int i = 0; i < 4; ++i)
    //  for (int j = 0; j < 4; ++j)
      //  ofs.write (reinterpret_cast<const char*>(&transform (i, j)), sizeof (double));  
              ofs.write (reinterpret_cast<const char*>(&transform), sizeof (Eigen::Matrix4f ));  

    ofs.close ();
  }
int
main (int argc, char** argv)
{
 
//std::vector<Eigen::Matrix4f> trans, calibs;
//
//
//Eigen::Matrix4f transform0, transform00;
//readTransformFromText("output_0.transtext", transform0);
//trans.push_back(transform0);
//saveTransform("output_0.transform",transform0);
//
//Eigen::Matrix4f transform1;
//readTransformFromText("output_1.transtext", transform1);
//transform1=transform0*transform1;
//trans.push_back(transform1);
//
//saveTransform("output_1.transform",transform1);

Eigen::Matrix4f trans1;
readTransformFromText("1.out", trans1);
Eigen::Matrix4f trans2;
readTransformFromText("2.out", trans2);
Eigen::Matrix4f trans3;
readTransformFromText("3.out", trans3);
Eigen::Matrix4f trans4;
readTransformFromText("4.out", trans4);
Eigen::Matrix4f trans5;
readTransformFromText("5.out", trans5);

Eigen::Matrix4f calib0;
readTransformFromText("cam0.calib", calib0);
//calibs.push_back(calib0);

Eigen::Matrix4f calib1;
readTransformFromText("cam1.calib", calib1);
//calibs.push_back(calib1);

Eigen::Matrix4f calib2;
readTransformFromText("cam2.calib", calib2);
//calibs.push_back(calib2);



Eigen::Matrix4f calib3;
readTransformFromText("cam3.calib", calib3);
//calibs.push_back(calib3);

Eigen::Matrix4f calib4;
readTransformFromText("cam4.calib", calib4);
//calibs.push_back(calib4);

Eigen::Matrix4f calib5;
readTransformFromText("cam5.calib", calib5);
//calibs.push_back(calib5);


//std::cout<<"transfom  \n "<<transform0<<std::endl;

//std::cout<<"calib2  \n "<<calib2<<std::endl;
//
//std::cout<<"transfom calib \n "<<transform0*calib2<<std::endl;

/*std::vector<Eigen::Matrix4f> GlobalTransforms;//(trans.size());
GlobalTransforms.push_back(trans.at(0));
GlobalTransforms.push_back(trans.at(1));*/

std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> >  GlobalCalibratedTransforms;//(calibs.size());
//std::vector<Eigen::Matrix4f> GlobalCalibratedTransforms;//(calibs.size());
GlobalCalibratedTransforms.push_back(calib0);
GlobalCalibratedTransforms.push_back(trans1*calib1);
GlobalCalibratedTransforms.push_back(trans2*calib2);
GlobalCalibratedTransforms.push_back(trans3*calib3);
GlobalCalibratedTransforms.push_back(trans4*calib4);
GlobalCalibratedTransforms.push_back(trans5*calib5);



for(auto &v:GlobalCalibratedTransforms) { 
  v.col(1)*=-1;
  v.col(2)*=-1;

}
//transform00=transform0*calib0;


 std::cout<<"GlobalCalibTrans  \n "<<GlobalCalibratedTransforms[1]<<std::endl;
 




//size_t sz = calibs.size();
//if (sz>0) {
//  std::ofstream FILE("./output.calibs", std::ios::out | std::ofstream::binary);
//  FILE.write(reinterpret_cast<const char*>(&sz), sizeof(sz));
//  FILE.write(reinterpret_cast<const char*>(&calibs[0]), sz * sizeof(calibs[0]));
//  FILE.close ();
//}




size_t sz = GlobalCalibratedTransforms.size();
if (sz>0) {
  std::ofstream FILE("./output.globalcalibtrans", std::ios::out | std::ofstream::binary);
  FILE.write(reinterpret_cast<const char*>(&sz), sizeof(sz));
  FILE.write(reinterpret_cast<const char*>(&GlobalCalibratedTransforms[0]), sz * sizeof(GlobalCalibratedTransforms[0]));
  FILE.close ();
}






  

std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > GlobalCalibTransforms;



    if (is_file_exist("output.globalcalibtrans")){


      readTransformsBinary("output.globalcalibtrans",GlobalCalibTransforms);




   }  else return 0;


    using boost::property_tree::ptree;
    ptree pt;

    read_xml("cameras_templates.xml", pt, boost::property_tree::xml_parser::trim_whitespace );

   

    //  std::cout<<"Transform "<<i<< "\n"<<Transforms.at(i)<<std::endl;

      for( auto &v : pt.get_child("document.chunk.cameras"))
        {

           for (int i=0;i<GlobalCalibTransforms.size();i++)  {
            


          // std::string name;
         //  ptree sub_pt;
          // std::tie(name, sub_pt) = v;
           std::string temp = v.second.get<std::string>("<xmlattr>.id");
           if (std::stoi(temp)!=i) continue; 

           std::cout << v.second.get<std::string>("transform")<< "\n temp " <<temp<< std::endl;
           std::stringstream iss;
           iss<<GlobalCalibTransforms.at(i);

           temp=iss.str();
          // temp.erase(std::remove(temp.begin(), temp.end(), '\n'), temp.end());
           boost::replace_all(temp, "\n", " ");
           v.second.put<std::string>("transform",temp);

         }

          //  std::cout <<v.first.data() <<" "<< v.second.data() << std::endl;
        // }
       }  
 std::locale newlocale1("en_GB.UTF-8");
 auto settings = boost::property_tree::xml_writer_make_settings<std::string>('\t', 1);
//bpt::xml_parser::xml_writer_settings<char> xmlstyle(' ',4);

  write_xml("cameras.xml", pt,  newlocale1, settings);

     
     

/*std::vector<Eigen::Matrix4f> Transforms;

size_t sz ;
        fstream FILE("output.transforms",ios::binary|ios::in);
        //while (binary_file.good()) {
        FILE.read(reinterpret_cast<char*>(&sz), sizeof(size_t));
        std::cout<<"sz "<<sz<<std::endl;
        Transforms.resize(sz);
        FILE.read(reinterpret_cast<char*>(&Transforms[0]), sz * sizeof(Transforms[0]));
        FILE.close();
*/



  // Finish
  return (0);
}
