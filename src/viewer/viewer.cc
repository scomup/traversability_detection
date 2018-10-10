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

        FindPoseOnSurface finder(pcl_point_cloud_normals_, kdtree_);
        Eigen::Matrix4d trans;
        if(handler->getPose(trans)){
            pose_ =  pose_ * trans;
            Eigen::Matrix4d pose_out;
            finder.FindPoseLieOnTheSurface(pose_, pose_out);
            pose_ = pose_out;

        }
        
        DrawIMU(pose_.data());
        //Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
        //mat4.block(0, 0, 3, 3) = RollPitchYaw(0.3,0.3,0.5).toRotationMatrix();
        //mat4.block(0, 3, 3, 1) = Eigen::Vector3d(-1.,0.,1.);

        /*
        DrawIMU(pose_.data());
        int stat = Cov(pcl_point_cloud_normals_, kdtree_, pose_.block(0, 3, 3, 1));
        while(true){
            if(stat == 0 || stat == -2)
                break;
            else if(stat == -1){
                pose_(2, 3) -= 0.05;
            }
            else{
                pose_(2, 3) += 0.05;
            }
            stat = Cov(pcl_point_cloud_normals_, kdtree_, pose_.block(0, 3, 3, 1));

        }
        Cov(pcl_point_cloud_normals_, kdtree_, pose_.block(0, 3, 3, 1),true);
        DrawIMU(pose_.data());
        */
        


        /*
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        
        int nn = kdtree_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

        if (nn > 0)
        {
            double a_z = 0;
            for (size_t i = 0; i < nn; ++i)
            {
                double z = pcl_point_cloud_normals_->points[pointIdxNKNSearch[i]].z;
                a_z += z;
            }
            
            a_z /= nn;
            searchPoint.z = a_z;
            mat4(2, 3) = a_z + 0.1;

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            float radius = 0.2;

            glColor3f(0,0,1);
            glPointSize(20);
            glBegin(GL_POINTS);
            glVertex3f(searchPoint.x, searchPoint.y, searchPoint.z);
            glEnd();

            if (kdtree_.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                Eigen::Vector3d normal(0,0,0);
                double a_z = 0;
                for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                {
                    double x = pcl_point_cloud_normals_->points[pointIdxRadiusSearch[i]].x;
                    double y = pcl_point_cloud_normals_->points[pointIdxRadiusSearch[i]].y;
                    double z = pcl_point_cloud_normals_->points[pointIdxRadiusSearch[i]].z;
                    a_z += z;
                    normal[0] += pcl_point_cloud_normals_->points[pointIdxRadiusSearch[i]].normal[0];
                    normal[1] += pcl_point_cloud_normals_->points[pointIdxRadiusSearch[i]].normal[1];
                    normal[2] += pcl_point_cloud_normals_->points[pointIdxRadiusSearch[i]].normal[2];
                    glColor3f(1,0,0);
                    glPointSize(10);
                    glBegin(GL_POINTS);
                    glVertex3f(x, y, z);
                    glEnd();
                }
                
                normal /= pointIdxRadiusSearch.size();
                a_z /= pointIdxRadiusSearch.size();
                Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(mat4.block(0, 2, 3, 1), normal);
                mat4.block(0, 0, 3, 3) = q.toRotationMatrix() * mat4.block(0, 0, 3, 3);
                DrawIMU(mat4.data());


            }
            
        }
        */
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


