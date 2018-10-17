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

        glColor3f(0.0f, 0.0f, 0.0f);
        glPointSize(10);
        glBegin(GL_POINTS);
        glVertex3f(5, -19, -0.2);
        glVertex3f(42, -65, 2.3);

        glEnd();
        if(drawer_for_cloud_analyzer_ != nullptr){

            Eigen::Matrix4d trans;
            if (handler->getPose(trans))
            {
                pose_ = pose_ * trans;
                Eigen::Matrix4d pose_out;
                Eigen::Matrix4d in_pose = pose_;
                drawer_for_cloud_analyzer_->GetCloudAnalyzer()->FindPoseLieOnTheSurface(pose_, pose_out);
                pose_ = pose_out;
            }
            //DrawIMU(pose_.data());
            drawer_for_cloud_analyzer_->DrawPoint();
            //drawer_for_cloud_analyzer_->DrawObj(pose_);
        }

        if (rrt_ != nullptr)
        {
            auto& start_tree = rrt_->startTree();
            auto& goal_tree = rrt_->goalTree();
            auto start_node = rrt_->startSolutionNode(); 
            //auto goal_node = rrt_->goalSolutionNode();

            auto path = rrt_->getPath();
            glColor3f(0.0f, 0.0f, 0.0f);
            glLineWidth(10);
            glBegin(GL_LINE_STRIP);
            for (auto it = path.begin(); it != path.end(); it++)
            {
                glVertex3f(it->x(), it->y(), it->z());
            }
            glEnd();

            glLineWidth(5);
            glColor3f(0.0f, 1.0f, 0.0f);
            for (const auto &node : start_tree.allNodes())
            {
                if (node.parent())
                {
                    auto &p = node.parent()->state();
                    auto &c = node.state();
                    pangolin::glDrawLine(p.x(), p.y(), p.z(),c.x(), c.y(), c.z()); 
                }
            }
            glColor3f(0.0f, 0.0f, 1.0f);
            for (const auto &node : goal_tree.allNodes())
            {

                if (node.parent())
                {
                    auto &p = node.parent()->state();
                    auto &c = node.state();
                    pangolin::glDrawLine(p.x(), p.y(), p.z(),c.x(), c.y(), c.z()); 
                }
            }


        }

        pangolin::FinishFrame();
    }
    SetFinish();
}



void Viewer::SetFinish()
{
    finish_ = true;
}


void Viewer::SetCloudDrawer(std::shared_ptr<DrawerForCloudAnalyzer> drawer_for_cloud_analyzer){
    drawer_for_cloud_analyzer_ = drawer_for_cloud_analyzer;
}