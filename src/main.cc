
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
#include "viewer/handler_for_cloud_analyzer.h"
#include "viewer/handler_for_BiRRT.h"



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


int main(int argc, char **argv)
{

    std::string filename;
    if (argc >= 2)
    {
        filename = argv[1];
    }
    else
    {
        std::cout<<"No input file!"<<std::endl;
        //return 0;
        filename = std::string("/home/liu/bag/lx35/lx35_7f_aisle.bag");
    }

    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ReadLas(filename, cloud);

      // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setFilterFieldName("x");
  filter.setFilterLimits(-100, 0);
  filter.filter (*cloud);
  filter.setFilterFieldName("y");
  filter.setFilterLimits(-100, 0);
  filter.filter (*cloud);


    auto cloud_analyzer = std::make_shared<CloudAnalyzer<pcl::PointXYZ>>(cloud);
    auto handler_for_cloud_analyzer = std::make_shared<HandlerForCloudAnalyzer>(cloud_analyzer);
    viewer->SetCloudDrawer(handler_for_cloud_analyzer);
    auto state_space = std::make_shared<RRT::CloudStateSpace>(cloud_analyzer);
    auto bi_rrt = std::make_shared<RRT::BiRRT<Eigen::Vector3d>>(
        state_space, 
        [](Eigen::Vector3d state) { size_t seed = 0;
            boost::hash_combine(seed, state.x());
            boost::hash_combine(seed, state.y());
            boost::hash_combine(seed, state.z());
            return seed; },
        3);
    bi_rrt->setGoalMaxDist(0.5);
    bi_rrt->setStepSize(0.5);
    bi_rrt->setMaxIterations(1e6);
    auto handler_for_rrt = std::make_shared<HandlerForBiRRT>(bi_rrt);

    viewer->SetRRTHandler(handler_for_rrt);

    viewer_thread.join();
    
}
