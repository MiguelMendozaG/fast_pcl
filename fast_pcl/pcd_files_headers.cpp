#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
using namespace std;
//pcl::PCLPointCloud2::Ptr z (new pcl::PCLPointCloud2 ());
vector < pcl::PCLPointCloud2::Ptr > all_z;

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

void read_all_z(std::string input_folder){
  vector< vector<double> > pcl_raw;
   pcl::PCDReader reader_pcl;
   pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_file_pcd (new pcl::PointCloud<pcl::PointXYZ>);
  // Replace the path below with the path where you saved your file
   vector < pcl::PCLPointCloud2::Ptr >::iterator ite;
   //ite = all_z.begin();
  for(int i = 0; i<1312; i+=5){
    std::stringstream indice_img;
    indice_img << i;
    readDoubleCoordinates(input_folder + "imagen-" + indice_img.str() + ".dat", pcl_raw);
    pcl_file_pcd->width = pcl_raw.size();
    pcl_file_pcd->height = 1;
    pcl_file_pcd->is_dense = false;
    pcl_file_pcd->resize(pcl_file_pcd ->width * pcl_file_pcd ->height);
    long int m = 0;
  for (size_t i = 0; i < pcl_raw.size(); i++){
    pcl_file_pcd->points[i].x = pcl_raw[m][0];
    pcl_file_pcd->points[i].y = pcl_raw[m][1];
    pcl_file_pcd->points[i].z = pcl_raw[m][2];
    m++;
  }
  pcl::io::savePCDFile( input_folder + "imagen-" + indice_img.str() + ".pcd" , *pcl_file_pcd);  // se guarda una nube de puntos de posible_z
    //*ite = (new pcl::PCLPointCloud2 ());
    //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    //reader_pcl.read (input_folder + indice_img.str(), *(*ite)); 
   }
  
}

int
main (int argc, char** argv)
{
  std::string input_folder ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/modelo/");
  read_all_z(input_folder);
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("imagen-nube_de_puntos.xyz", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::PCDWriter writer;
  writer.write ("image_0_downs.xyz", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}