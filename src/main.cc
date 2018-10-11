
#include <chrono>
#include <thread>
#include <memory>

#include "src/playable_bag.h"
#include "sensor_msgs/PointCloud2.h"
#include "viewer/viewer.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "rrt/tree.h"


using namespace sample_carto;

int main(int argc, char **argv)
{

    RRT::Tree<Eigen::Vector3d, pcl::PointNormal>(nullptr); 
    ::ros::init(argc, argv, "sample_carto_3d");
    ::ros::start();

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

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::fromROSMsg(*msg.instantiate<sensor_msgs::PointCloud2>(), *cloud);
            // Create the normal estimation class, and pass the input dataset to it
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(cloud);

            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            ne.setSearchMethod(tree);

            // Output datasets
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

            // Use all neighbors in a sphere of radius 3cm
            ne.setRadiusSearch(0.2);

            // Compute the features
            ne.compute(*normals);

            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);// (new pcl::PointCloud<pcl::PointXYZNormal>);
            pcl::copyPointCloud(*cloud, *cloud_normals);
            pcl::copyPointCloud(*normals, *cloud_normals);
            //for(auto p: cloud_normals){
            //    std::cout<<p<<std::endl;
            //}
            // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
            viewer->SetCloud(cloud);
            viewer->SetCloudNormals(cloud_normals);
        }
    }
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
