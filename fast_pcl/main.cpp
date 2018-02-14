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
int user_data;
 

void read_all_z(std::string input_folder){
  vector< vector<double> > pcl_raw;
   pcl::PCDReader reader_pcl;
   pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_file_pcd (new pcl::PointCloud<pcl::PointXYZ>);
  for(int i = 0; i<1312; i+=5){
    std::stringstream indice_img;
    indice_img << i;
    std::cout  << "\n Imagen " << i << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (input_folder + "imagen-" +  indice_img.str() + ".pcd", *cloud);
    all_z.push_back(cloud);
   }
   pcl::visualization::CloudViewer viewer("Cloud Viewer");
   
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(all_z[250]);

    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
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