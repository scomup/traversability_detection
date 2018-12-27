
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include <utility>
#include <boost/functional/hash.hpp>
#include <liblas/liblas.hpp>
#include <time.h>
#include "src/playable_bag.h"
#include "sensor_msgs/PointCloud2.h"
#include "viewer/viewer.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include "rrt/BiRRT.h"
#include "rrt/CloudStateSpace.h"
#include "cloud_analyzer.h"
#include "viewer/cloud_analyzer_handle.h"
#include "viewer/planner_handle.h"



using namespace sample_carto;


void ReadLas (const std::string &file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::ifstream ifs(file_name.c_str(), std::ios::in | std::ios::binary);

    if(ifs.fail()) 
	{
        std::cerr << "ERROR : Impossible to open the file : " << file_name <<std::endl;
        abort();
    }


    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();
    std::cout<<"Cloud size:"<<nbPoints<<std::endl;

	// Fill in the cloud data
	cloud->width    = nbPoints;				// This means that the point cloud is "unorganized"
	cloud->height   = 1;						// (i.e. not a depth map)
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);


	int i=0;				// counter

    double scale = 10;
	//double bias_x = (reader.GetPoint().GetX())/scale;
	//double bias_y = (reader.GetPoint().GetY())/scale;
	//double bias_z = (reader.GetPoint().GetZ())/scale;					

    Eigen::Vector3d xyz_centroid(0,0,0);
	while(reader.ReadNextPoint()) 
	{
		// get XYZ information
		cloud->points[i].x = (reader.GetPoint().GetX())/scale;
	    cloud->points[i].y = (reader.GetPoint().GetY())/scale;
	    cloud->points[i].z = (reader.GetPoint().GetZ())/scale;
        xyz_centroid.x() += cloud->points[i].x;
        xyz_centroid.y() += cloud->points[i].y;	
        xyz_centroid.z() += cloud->points[i].z;	



		i++; // ...moving on
	}
    xyz_centroid = xyz_centroid/cloud->points.size();

    //Eigen::Vector4f xyz_centroid;
    //pcl::compute3DCentroid(*cloud, xyz_centroid); //重心を計算

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = cloud->points[i].x - xyz_centroid[0];
        cloud->points[i].y = cloud->points[i].y - xyz_centroid[1];
        cloud->points[i].z = cloud->points[i].z - xyz_centroid[2];
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr gcloud;

void 
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *local_cloud);  
  //pcl::io::savePCDFileASCII("/home/liu/workspace/traversability_detection/build/f.pcd", *local_cloud);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::io::loadPCDFile("/home/liu/workspace/traversability_detection/build/f.pcd", *cloud_in);


  gcloud = local_cloud;
}


int main(int argc, char **argv)
{
    
 ros::init (argc, argv, "test_traversability");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/map3d", 1, cloud_cb);

  std::cout<<"wait map3d topic\n";
  while (gcloud == nullptr && ros::ok())
  {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      ros::spinOnce();
  }
  std::cout<<"map3d loaded.\n";


  pcl::io::savePCDFileASCII("/home/liu/workspace/traversability_detection/build/f.pcd", *gcloud);
  pcl::io::loadPCDFile("/home/liu/workspace/traversability_detection/build/f.pcd", *gcloud);

  std::vector< int > idx;
  std::cout<<gcloud->size()<<std::endl;
  pcl::removeNaNFromPointCloud(*gcloud, *gcloud, idx); 
  std::cout<<gcloud->size()<<std::endl;
  

  auto viewer = new Viewer();
  auto viewer_thread = std::thread(&Viewer::Run, viewer);


    

  auto cloud_analyzer = std::make_shared<CloudAnalyzer<pcl::PointXYZ>>(gcloud);
  auto cloud_analyzer_handle = std::make_shared<CloudAnalyzerHandle>(cloud_analyzer);
  viewer->SetCloudDrawer(cloud_analyzer_handle);
  auto state_space = std::make_shared<RRT::CloudStateSpace>(cloud_analyzer);
  auto planner_handle = std::make_shared<PlannerHandle>(state_space);

  viewer->SetRRTHandler(planner_handle);

  viewer_thread.join();
    
}
