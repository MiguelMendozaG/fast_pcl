#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>

using namespace std;
//pcl::PCLPointCloud2::Ptr z (new pcl::PCLPointCloud2 ());
vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> all_z;
vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> all_bg;
int user_data;
 
bool readDouble(std::string file_name, std::vector< double >& data)
{
  double value;
  
  std::ifstream file(file_name.c_str());
  
  if(file.is_open()){
    
//     while(file.good()){
//       file >> value;
//       data.push_back(value);
//     }
    while(file>>value)
      data.push_back(value);
    
   // std::cout << data.size() << "points" <<std::endl;
    
    file.close();
    return true;
  } else {
    std::cout << "Unable to open file: "  << file_name.c_str() << std::endl;
    return false;
  }
}

bool readDoubleCoordinates(std::string file_name, std::vector< std::vector< double > >& data)
{
  data.clear();
  std::vector<double> lineal;
  std::cout << "Reading double coordinates from " << file_name.c_str() << std::endl;
  if(!readDouble(file_name, lineal)){
    std::cout << "Error" << std::endl;
    return false;
  }
    
  std::vector<double> point(3);
  std::vector<double>::iterator it;
  it = lineal.begin();
  
  while(it != lineal.end()){
    point[0] = *it;
    it ++;
    point[1] = *it;
    it ++;
    point[2] = *it;
    it ++;
    data.push_back(point);
  }
  
  std::cout << data.size() << " readed points." << std::endl;
  return true;
}

void read_all_z(std::string input_folder, bool cloud_select){
  std::vector< std::vector<double> > pcl_raw;
  std::vector< std::vector<double> > pcl_copy;
  std::vector< std::vector< std::vector <double> > > cloud_vector;
   pcl::PCDReader reader_pcl;
   pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_file_pcd (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ> cloud_c;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
   vector < vector <double> > new_cloud;
   
  

  for(int i = 0; i<1312; i+=5){
    std::stringstream indice_img;
    indice_img << i;
    std::cout  << "\n Imagen " << i << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (input_folder + "imagen-" +  indice_img.str() + ".pcd", *cloud);
    if (cloud_select==0)
      all_z.push_back(cloud); //almacena en una estructura tipo pcl
    else
      all_bg.push_back(cloud);
      
    /*
    readDoubleCoordinates(input_folder + "imagen-" +  indice_img.str() + ".xyz", pcl_raw);
    cloud_vector.push_back(pcl_raw); //almacena en una estructura tipo vector
    std::cout << "\n pcl raw" << pcl_raw[0][0] << endl;*/
   }
   //double val = cloud_vector[0][0][0];
   /*
  std::cout << "\n cloud_vector(0): " << cloud_vector[2][0][1] << endl;
  std::cout << "\n pcl raw " << pcl_raw.size() << endl;
  //cloud_vector[2].swap(pcl_copy);
  pcl_copy = cloud_vector[2];
  std::cout << "\n plc_copy (0): " << pcl_copy[0][1] << endl;
  std::cout << "\n cloud_vector (0): " << cloud_vector[2][0][1] << endl;*/
}

int
main (int argc, char** argv)
{
  std::string input_folder ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/absolute/model/");
  std::string input_folder2 ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/absolute/background_filtered/");
  read_all_z(input_folder, 0);
  read_all_z(input_folder2, 1);
  std::cin.get();

  return (0);
}