
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
#include "rrt/BiRRT.h"
#include "rrt/CloudStateSpace.h"
#include "cloud_analyzer.h"
#include "drawer_for_cloud_analyzer.h"



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

	// Fill in the cloud data
	cloud->width    = nbPoints;				// This means that the point cloud is "unorganized"
	cloud->height   = 1;						// (i.e. not a depth map)
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);


	int i=0;				// counter
	uint16_t r1, g1, b1;	// RGB variables for .las (16-bit coded)
	int r2, g2, b2;			// RGB variables for converted values (see below)
	uint32_t rgb;			// "packed" RGB value for .pcd

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

    ::ros::init(argc, argv, "sample_carto_3d");
    ::ros::start();

/*
    std::string filename;
    if (argc >= 2)
    {
        filename = argv[1];
    }
    else
    {
        std::cout<<"No input file!"<<std::endl;
        //return 0;
        filename = std::string("/home/liu/Downloads/short_test.bag");
    }

    top::PlayableBagMultiplexer playable_bag_multiplexer;

    playable_bag_multiplexer.AddPlayableBag(top::PlayableBag(
        filename, 0, ros::TIME_MIN, ros::TIME_MAX, ::ros::Duration(1.0),
        [](const rosbag::MessageInstance &msg) { return true; }));

    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    while (playable_bag_multiplexer.IsMessageAvailable())
    {
        if (!::ros::ok()){
            return 0;
        }

        const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
        const rosbag::MessageInstance &msg = std::get<0>(next_msg_tuple);
        //const int bag_index = std::get<1>(next_msg_tuple);
        //const bool is_last_message_in_bag = std::get<2>(next_msg_tuple);

        if (msg.isType<sensor_msgs::PointCloud2>()){
            std::string topic = std::string(msg.getTopic());
            if (topic != "/mapcloud")
                continue;

            pcl::fromROSMsg(*msg.instantiate<sensor_msgs::PointCloud2>(), *cloud);

            break;
        }
    }
*/
    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ReadLas("/home/liu/points.las", cloud);

    auto cloud_analyzer = std::make_shared<CloudAnalyzer<pcl::PointXYZ>>(cloud);
    auto drawer_for_cloud_analyzer = std::make_shared<DrawerForCloudAnalyzer>(cloud_analyzer);
    viewer->SetCloudDrawer(drawer_for_cloud_analyzer);
    auto state_space = std::make_shared<RRT::CloudStateSpace>(cloud_analyzer);
    auto bi_rrt = std::make_shared<RRT::BiRRT<Eigen::Vector3d>>(
        state_space, 
        [](Eigen::Vector3d state) { size_t seed = 0;
            boost::hash_combine(seed, state.x());
            boost::hash_combine(seed, state.y());
            boost::hash_combine(seed, state.z());
            return seed; },
        3);

    bi_rrt->setStartState(Eigen::Vector3d(5, -19, -0.2));
    //bi_rrt->setGoalState(Eigen::Vector3d(42, -65, 2.3));
    bi_rrt->setGoalState(Eigen::Vector3d(-98, 0, -2.3));
    bi_rrt->setMaxStepSize(30);
    bi_rrt->setGoalMaxDist(0.5);
    bi_rrt->setStepSize(0.5);
    bi_rrt->setMaxIterations(100000);

    //RRT::VoxeltateSpace<Eigen::Vector3d> state_space(bounds);
    //auto _biRRT = std::make_unique<BiRRT<Eigen::Vector3d>>(_stateSpace, RRT::hash, dimensions);
    viewer->SetRRT(bi_rrt);
    //bi_rrt->run();


    ros::Rate loop_rate(100);
    while (::ros::ok())
    {
        loop_rate.sleep();
        if (viewer->isFinished())
        {   
            break;
        }
    }
    viewer_thread.join();
    
}
