#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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

bool saveDoubleCoordinates(std::string file_name, std::vector< std::vector< double > >& data)
{
  std::ofstream file;
  std::vector< std::vector< double > >::iterator it_points;
  
  //cout << "Saving data..." <<std::endl;
  file.open(file_name.c_str());
  
  if(file.is_open()){
    for(it_points = data.begin(); it_points!= data.end(); it_points++){
      file << (*it_points)[0] << "\t" << (*it_points)[1] << "\t" << (*it_points)[2] << "\n";
    }
    file.close();
  }
  else {
    std::cout << "ERROR: unable to save file "  << file_name << std::endl;
    return false;
  }
  
  //std::cout << file_name << " Saved" << std::endl;
  return true;
}

bool readLineCoordinatesfromPCD(std::string file_name, std::vector< double >& data, std::string& header)
{
  double value;
  
  std::ifstream file(file_name.c_str());
  char line[100];
  int n = 100;
  
  if(file.is_open()){
    // Read the header
    
    //# .PCD v0.7 - Point Cloud Data file format
    file.getline(line,100);
    header.append(line);
    //VERSION 0.7
    file.getline(line,100);
    header.append(line);
    //FIELDS x y z
    file.getline(line,n);
    header.append(line);
    //SIZE 4 4 4
    file.getline(line,n);
    header.append(line);
    //TYPE F F F
    file.getline(line,n);
    header.append(line);
    //COUNT 1 1 1
    file.getline(line,n);
    header.append(line);
    //WIDTH 212879
    file.getline(line,n);
    header.append(line);
    //HEIGHT 1
    file.getline(line,n);
    header.append(line);
    //VIEWPOINT 0 0 0 1 0 0 0
    file.getline(line,n);
    header.append(line);
    //POINTS 212879
    file.getline(line,n);
    header.append(line);
    //DATA ascii
    file.getline(line,n);
    header.append(line);
    
    std::cout << "File readed info:" << std::endl << header.c_str() << std::endl;

    while(file.good()){
	//cout << value <<std::endl;
	file >> value;
	data.push_back(value);
    }
    
    file.close();
    
    std::cout << "closed file" << std::endl;
    return true;
  } else {
    std::cout << "Unable to open file: "  << file_name.c_str() << std::endl;
    return false;
  }
}


bool readDoubleCoordFromPCD(std::string file_name, std::vector< std::vector< double > >& data, std::string& header)
{
  //cout << " Reading double  ---" <<std::endl;
  data.clear();
  std::vector<double> lineal;
  if(!readLineCoordinatesfromPCD(file_name, lineal, header))
    return false;
  
  std::vector<double> point(3);
  std::vector<double>::iterator it;
  it = lineal.begin();
  
  //cout << "Readed values: " << lineal.size() <<std::endl;
  std::cout << "Readed " << lineal.size()/3 << "coordinates from PCD" <<  std::endl;	
  //getchar();
  
  long int i=0;
  while(it != lineal.end()){
    point[0] = *it;
    it ++;
    if(it == lineal.end())
      break;
    
    point[1] = *it;
    it ++;
    if(it == lineal.end())
      break;
    
    point[2] = *it;
    it ++;
    if(it == lineal.end())
      break;
    
    //cout << i << " " << point[0] << " "  <<  point[1] << " "  << point[2] <<std::endl;
    i++;
    data.push_back(point);
  }
  
  //getchar();
  
  //cout << "--- Done rcf pcd ---" <<std::endl;
  return true;
}

void filter_all(std::string input_folder, std::string output_foder)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  std::vector< std::vector< double > > pcl_aux;

  // Fill in the cloud data
  pcl::PCDReader reader;
  for (int i = 0; i < 1312; i+=5){
    std::stringstream indice_img;
    indice_img << i;
    std::string input_pcl (input_folder + "imagen-" + indice_img.str() + ".pcd");
    std::string output_pcl (output_foder + "imagen-" + indice_img.str() + ".pcd");
    std::string output_pcl_xyz (output_foder + "imagen-" + indice_img.str() + ".xyz");
  // Replace the path below with the path where you saved your file
  reader.read ( input_pcl, *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::PCDWriter writer;
  writer.write (output_pcl, *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  readDoubleCoordFromPCD(output_pcl, pcl_aux, output_pcl);
  saveDoubleCoordinates(output_pcl_xyz, pcl_aux);
  pcl_aux.clear();
  //saveDoubleCoordinates(output_pcl, cloud_filtered);
  }

}

int main ()
{
  std::string input_folder ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/absolute/background/");
  std::string output_folder ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/absolute/background_filtered/");
  filter_all(input_folder, output_folder);

  return (0);
}