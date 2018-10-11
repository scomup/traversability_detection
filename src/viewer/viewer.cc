/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "viewer.h"
#include <pangolin/pangolin.h>
#include "myhandler.h"

#include "draw_object_helper.h"
#include "frustum_culling.h"

#include "Eigen/Core"
#include "Eigen/Geometry"


#include <iostream>
#include <chrono>
#include <thread>




Viewer::Viewer()
{
    t_ = 1e3/30;
    image_width_ = 640;
    image_height_ = 480;
    view_point_x_ = 20;
    view_point_y_ = 20;
    view_point_z_ = 10;
    view_point_f_ = 2000;

    pose_ = Eigen::Matrix4d::Identity();
}

void Viewer::Run()
{
    finish_ = false;

    pangolin::CreateWindowAndBind("imu viewer",1024,768);

    //pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    //pangolin::Var<bool> menuShowPath("menu.show path",true,true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,view_point_f_,view_point_f_,512,389,0.1,1000),
                pangolin::ModelViewLookAt(view_point_x_,view_point_y_,view_point_z_, 0,0,0, 0.0 ,0.0,1.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    auto handler = new pangolin::MyHandler3D(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0)
            .SetHandler(handler);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    while( !pangolin::ShouldQuit() && !finish_)
    {


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        d_cam.Activate(s_cam);


        pangolin::glDrawAxis(1);
        DrawGrid(200,1);
        //DrawPoints(pcl_point_cloud_);
        if(pcl_point_cloud_normals_ == nullptr){
            continue;
        }
        DrawCloudNormals(*pcl_point_cloud_normals_);

        
        CloudEstimator<pcl::PointNormal> finder(pcl_point_cloud_normals_, kdtree_);
        double r = 0.2;
        Eigen::Matrix4d trans;
        if(handler->getPose(trans)){
            pose_ =  pose_ * trans;
            Eigen::Matrix4d pose_out;
            Eigen::Matrix4d in_pose = pose_;
            finder.FindPoseLieOnTheSurface(pose_, pose_out, r);
            pose_ = pose_out;
        }
        
        DrawIMU(pose_.data());

        pcl::PointNormal searchPoint;
        searchPoint.x = pose_(0,3);
        searchPoint.y = pose_(1,3);
        searchPoint.z = pose_(2,3);
        std::vector<int> idx = finder.FindNearest(searchPoint, r);
        finder.EstimateTraversability(searchPoint, r);
        for (auto i : idx)
        {
            double x = pcl_point_cloud_normals_->points[i].x;
            double y = pcl_point_cloud_normals_->points[i].y;
            double z = pcl_point_cloud_normals_->points[i].z;
            glColor3f(0, 0, 1);
            glPointSize(10);
            glBegin(GL_POINTS);
            glVertex3f(x, y, z);
            glEnd();
        }
        
        Eigen::Vector4f plane_parameters_f;
        Eigen::Vector4d plane_parameters;
        float curvature;
        pcl::computePointNormal(*pcl_point_cloud_, idx, plane_parameters_f, curvature);
        plane_parameters = plane_parameters_f.cast<double>();
        //finder.GetNormalVector(searchPoint, idx, normal_vector);
        //auto plane = finder.FindPlane(searchPoint, normal_vector);
        auto plane_on_points = finder.FindPlaneOnCloud(idx, plane_parameters);
        auto dist = finder.GetDistToPlane(searchPoint, plane_on_points);
        Eigen::Vector3d projected_point = pose_.block(0,3,3,1) - dist * plane_parameters.head<3>();
        searchPoint.x = projected_point.x();
        searchPoint.y = projected_point.y();
        searchPoint.z = projected_point.z();

        auto dist2 = finder.GetDistToPlane(searchPoint, plane_on_points);
        glColor3f(0, 0, 1);
        glPointSize(20);
        glBegin(GL_POINTS);
        glVertex3f(searchPoint.x, searchPoint.y, searchPoint.z);
        glEnd();

        DrawPlane(projected_point,r,plane_parameters.head<3>());
        glColor3f(0,0,1);
        pangolin::glDrawLine(projected_point.x(),projected_point.y(),projected_point.z(), projected_point.x() + plane_parameters.x(),projected_point.y()+plane_parameters.y(),projected_point.z()+plane_parameters.z());


            pangolin::FinishFrame();
    }
    SetFinish();
}



void Viewer::SetFinish()
{
    finish_ = true;
}

void Viewer::SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud){
    pcl_point_cloud_ = pcl_point_cloud;
    
}

void Viewer::SetCloudNormals(pcl::PointCloud<pcl::PointNormal>::Ptr pcl_point_cloud){
    pcl_point_cloud_normals_ = pcl_point_cloud;
    kdtree_.setInputCloud(pcl_point_cloud_normals_);

}


