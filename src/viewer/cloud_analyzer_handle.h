#ifndef HANDLER_FOR_CLOUD_ANALYZER_H_
#define HANDLER_FOR_CLOUD_ANALYZER_H_

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

#include "../cloud_analyzer.h"

class CloudAnalyzerHandle
{
public:
  CloudAnalyzerHandle(std::shared_ptr<CloudAnalyzer<pcl::PointXYZ>> cloud_analyzer)
      : cloud_analyzer_(cloud_analyzer){};

  void DrawPoint(bool mode = 0)
  {
    double scale = 0.05;
    if (cloud_analyzer_->pcl_point_cloud_ == nullptr)
      return;
    glPointSize(3);

    glBegin(GL_POINTS);
    for (size_t i = 0; i < cloud_analyzer_->pcl_point_cloud_->size(); i++)
    {
      if (cloud_analyzer_->traversability_[i] == CloudAnalyzer<pcl::PointXYZ>::TRAVERSABILITY::IGNORE)
        continue;
      pcl::PointXYZ &point = cloud_analyzer_->pcl_point_cloud_->points[i];
      if (mode == 0)
      {
        pangolin::glColorHSV(std::abs(point.z * 100));
      }
      else if (mode == 1)
      {
        pangolin::glColorHSV(cloud_analyzer_->traversability_[i]);
      }
      glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
    /*
    glBegin(GL_LINES);
    for (size_t i = 0; i < cloud_analyzer_->pcl_point_cloud_->size(); i++)
    {
      if(cloud_analyzer_->traversability_[i] == CloudAnalyzer<pcl::PointXYZ>::TRAVERSABILITY::IGNORE)
        continue;
      pcl::PointXYZ &point = cloud_analyzer_->pcl_point_cloud_->points[i];
      pcl::Normal &normal = cloud_analyzer_->normal_->points[i];
      if (mode == 0)
      {
        pangolin::glColorHSV(std::abs(point.z * 100));
      }
      else if (mode == 1)
      {
        //bool t = cloud_analyzer_->traversability_[i];
        pangolin::glColorHSV(cloud_analyzer_->traversability_[i]);
      }
      glVertex3f(point.x, point.y, point.z);
      glVertex3f(
          point.x + normal.normal[0] * scale,
          point.y + normal.normal[1] * scale,
          point.z + normal.normal[2] * scale);
    }
    glEnd();
*/
/*
    glBegin(GL_LINES);
    for (size_t i = 0; i < cloud_analyzer_->pcl_point_cloud_->size(); i++)
    {
      if (cloud_analyzer_->traversability_[i] !=
          CloudAnalyzer<pcl::PointXYZ>::TRAVERSABILITY::UNTRAVERSABLE)
        continue;
      for (auto ii : cloud_analyzer_->idxs_[i])
      {
        if (cloud_analyzer_->traversability_[ii] == CloudAnalyzer<pcl::PointXYZ>::TRAVERSABILITY::ADJACENTED)
        {
          pcl::PointXYZ &a = cloud_analyzer_->pcl_point_cloud_->points[i];
          pcl::PointXYZ &b = cloud_analyzer_->pcl_point_cloud_->points[ii];
          if (mode == 0)
          {
            pangolin::glColorHSV(std::abs(a.z * 100));
          }
          else if (mode == 1)
          {
            //bool t = cloud_analyzer_->traversability_[i];
            pangolin::glColorHSV(cloud_analyzer_->traversability_[i]);
          }
          
          glVertex3f(a.x, a.y, a.z);
          glVertex3f(b.x, b.y, b.z);
        }
      }
    }
    glEnd();*/
  }

  void DrawPlane(const Eigen::Vector3d point, const GLfloat radius, const Eigen::Vector3d normal)
  {
    GLfloat N = 20;
    GLfloat step = (2 * M_PI / N);
    float angle = 0.0;
    GLfloat x, y, z;

    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = q.toRotationMatrix();
    T.block(0, 3, 3, 1) = point;
    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
    glPushMatrix();
    glMultMatrixd(T.data());
    glBegin(GL_TRIANGLES);
    for (int j = 0; j < N; j++)
    {
      angle = j * step;
      glVertex3f(0, 0, 0);
      x = radius * cos(angle);
      y = radius * sin(angle);
      z = 0;
      glVertex3f(x, y, z);
      x = radius * cos(angle + step);
      y = radius * sin(angle + step);
      z = 0;
      glVertex3f(x, y, z);
    }
    glEnd();
    glPopMatrix();
  }

  void DrawObj(const Eigen::Matrix4d pose)
  {
    if (cloud_analyzer_->pcl_point_cloud_ == nullptr)
      return;

    auto &pcl_point_cloud = cloud_analyzer_->pcl_point_cloud_;
    pcl::PointXYZ search_point;
    search_point.x = pose(0, 3);
    search_point.y = pose(1, 3);
    search_point.z = pose(2, 3);
    std::vector<int> idx = cloud_analyzer_->FindPointsInRadius(search_point, cloud_analyzer_->radius_);
    cloud_analyzer_->EstimateTraversability(search_point);
    for (auto i : idx)
    {
      double x = pcl_point_cloud->points[i].x;
      double y = pcl_point_cloud->points[i].y;
      double z = pcl_point_cloud->points[i].z;
      glColor3f(0, 0, 1);
      glPointSize(10);
      glBegin(GL_POINTS);
      glVertex3f(x, y, z);
      glEnd();
    }

    Eigen::Vector4f plane_parameters;
    float curvature;
    pcl::computePointNormal(*pcl_point_cloud, idx, plane_parameters, curvature);

    auto plane_on_points = cloud_analyzer_->FindPlaneOnCloud(idx, plane_parameters.cast<double>());
    auto dist = cloud_analyzer_->GetDistToPlane(search_point, plane_on_points);
    Eigen::Vector3d projected_point = pose.block(0, 3, 3, 1) - dist * plane_parameters.head<3>().cast<double>();
    search_point.x = projected_point.x();
    search_point.y = projected_point.y();
    search_point.z = projected_point.z();

    glColor3f(0, 0, 1);
    glPointSize(20);
    glBegin(GL_POINTS);
    glVertex3f(search_point.x, search_point.y, search_point.z);
    glEnd();

    DrawPlane(projected_point, cloud_analyzer_->radius_, plane_parameters.head<3>().cast<double>());
    glColor3f(0, 0, 1);
    pangolin::glDrawLine(projected_point.x(),
                         projected_point.y(),
                         projected_point.z(),
                         projected_point.x() + plane_parameters.x(),
                         projected_point.y() + plane_parameters.y(),
                         projected_point.z() + plane_parameters.z());
  }

  std::shared_ptr<CloudAnalyzer<pcl::PointXYZ>> GetCloudAnalyzer() { return cloud_analyzer_; };

private:
  std::shared_ptr<CloudAnalyzer<pcl::PointXYZ>> cloud_analyzer_;
};

#endif // HANDLER_FOR_CLOUD_ANALYZER_H_
