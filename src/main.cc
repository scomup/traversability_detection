
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include <utility>

#include "src/playable_bag.h"
#include "sensor_msgs/PointCloud2.h"
#include "viewer/viewer.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "rrt/Tree.h"
#include "rrt/CloudStateSpace.h"

#include "cloud_analyzer.h"
#include "drawer_for_cloud_analyzer.h"



using namespace sample_carto;

int main(int argc, char **argv)
{
        //auto rrt = std::make_shared<RRT::Tree<Vector3d>>(stateSpace, hash, 3);
        auto a = std::
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
    
    Eigen::AlignedBox3d bounds(Eigen::Vector3d(0,0,0));

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

            auto cloud_analyzer = std::make_shared<CloudAnalyzer<pcl::PointXYZ>>(cloud);
            auto drawer_for_cloud_analyzer = std::make_shared<DrawerForCloudAnalyzer>(cloud_analyzer);
            viewer->SetCloudDrawer(drawer_for_cloud_analyzer);

            for(auto &point : *cloud){
                Eigen::Vector3d p(point.x,point.y,point.z);
                bounds.extend(p);
            }
            break;
        }
    }

    //RRT::VoxeltateSpace<Eigen::Vector3d> state_space(bounds);
    //auto _biRRT = std::make_unique<BiRRT<Eigen::Vector3d>>(_stateSpace, RRT::hash, dimensions);

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
