#ifndef SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_
#define SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_


#include <pangolin/pangolin.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

/*
Eigen::Affine3f RollPitchYaw(const double roll, const double pitch,const double yaw) {
  const Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(roll, Eigen::Vector3f(1, 0, 0)));
  const Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(pitch, Eigen::Vector3f(0, 1, 0)));
  const Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(yaw, Eigen::Vector3f(0, 0, 1)));
  return rz * ry * rx;
}*/


inline void DrawIMU(const double* twc)
{
    glPushMatrix();
    glMultMatrixd(twc);
    GLdouble r = 0.1;
    GLdouble l[3] = {- r/ 2, -r / 2, -r / 2};
    GLdouble h[3] = {+ r/ 2, +r / 2, +r / 2};
    //Front
    glBegin(GL_QUADS);
    glColor3f(0.5f,0.5f,0.5f);
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);
    glVertex3f(l[0], l[1], l[2]);
    //Back
    glColor3f(0.0f,1.0f,0.0f);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(h[0], h[1], l[2]);
    //Top
    glColor3f(0.0f,0.0f,1.0f);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);
    //Bottom
    glColor3f(0.5f,0.5f,0.5f);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], h[1], l[2]);
    //Left
    glColor3f(0.5f,0.5f,0.5f);
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], l[1], h[2]);
    //Right
    glColor3f(1.0f,0.0f,0.0f);
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], h[1], h[2]);
    glEnd();
    glPopMatrix();
}

inline void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M=10, GLfloat N=10)
{
    float step_z = M_PI / M;
    float step_xy = 2 * M_PI / N;
    float x[4], y[4], z[4];

    float angle_z = 0.0;
    float angle_xy = 0.0;
    int i = 0, j = 0;
    glBegin(GL_QUADS);
    for (i = 0; i < M; i++)
    {
        angle_z = i * step_z;

        for (j = 0; j < N; j++)
        {
            angle_xy = j * step_xy;

            x[0] = radius * sin(angle_z) * cos(angle_xy);
            y[0] = radius * sin(angle_z) * sin(angle_xy);
            z[0] = radius * cos(angle_z);

            x[1] = radius * sin(angle_z + step_z) * cos(angle_xy);
            y[1] = radius * sin(angle_z + step_z) * sin(angle_xy);
            z[1] = radius * cos(angle_z + step_z);

            x[2] = radius * sin(angle_z + step_z) * cos(angle_xy + step_xy);
            y[2] = radius * sin(angle_z + step_z) * sin(angle_xy + step_xy);
            z[2] = radius * cos(angle_z + step_z);

            x[3] = radius * sin(angle_z) * cos(angle_xy + step_xy);
            y[3] = radius * sin(angle_z) * sin(angle_xy + step_xy);
            z[3] = radius * cos(angle_z);

            for (int k = 0; k < 4; k++)
            {
                glVertex3f(xx + x[k], yy + y[k], zz + z[k]);
            }
        }
    }
    glEnd();
}


inline void DrawPoints(pcl::PointCloud<pcl::PointXYZ> &pcl_point_cloud)
{
    glPushMatrix();
    //glMultMatrixd(twc);
    glColor3f(1.0f,0.0f,0.0f);
    glPointSize(3);

    for (const auto &point : pcl_point_cloud)
    {   
        auto color = point.z > 0 ? point.z * 40 : -point.z * 40;
        pangolin::glColorHSV( color);
        glBegin(GL_POINTS);
        glVertex3f(point.x, point.y, point.z);
        glEnd();
    }
    //glPopMatrix();
}

inline void DrawCloudNormals(pcl::PointCloud<pcl::PointNormal> &pcl_point_cloud_normals)
{
    //glPushMatrix();
    //glMultMatrixd(twc);
    glColor3f(1.0f,0.0f,0.0f);
    //glPointSize(3);
    //glBegin(GL_POINTS);

    for (const auto &point : pcl_point_cloud_normals)
    {
        //glVertex3f(point.x, point.y, point.z);
        double n_x = point.normal[0];
        double n_y = point.normal[1];
        double n_z = point.normal[2];
        if (n_z < 0.)
        {
            n_x = -n_x;
            n_y = -n_y;
            n_z = -n_z;
        }
        Eigen::Vector3d normal(n_x, n_y, n_z);

        
        /*
        Eigen::Vector3d a(n_x,n_y,n_z);
        Eigen::Vector3d b(0,0,1.);
        double dot = a.dot(b);
        dot = ( dot < -1.0 ? -1.0 : ( dot > 1.0 ? 1.0 : dot ) );
        double angle = acos( dot );
        if(angle > M_PI/2.){
            angle = angle - M_PI/2. - angle;
        }
        */
        Eigen::AngleAxisd angle_axis(Eigen::Quaterniond::FromTwoVectors(
            normal, Eigen::Vector3d::UnitZ()));
        pangolin::glColorHSV(angle_axis.angle()*100);
        //std::cout<<angle<<"  "<<angle_axis.angle()<<std::endl;
        pangolin::glDrawLine(point.x, point.y, point.z, 
            point.x + n_x/10., point.y + n_y/10., point.z + n_z/10.);
        glPointSize(5);
        glBegin(GL_POINTS);
        glVertex3f(point.x, point.y, point.z);
        glEnd();
    }

}




GLint DrawGrid(GLdouble w, GLdouble s)
{
    glLineWidth(1);
    glColor4f(0.0f,0.0f,0.0f,0.3f);
    for (double i = -w / 2; i <= w / 2; i+=s)
    {
        pangolin::glDrawLine(i, -w / 2, 0, i, w / 2, 0);
        pangolin::glDrawLine(-w / 2, i, 0, w / 2, i, 0);
    };
    return (0);
}




GLint DrawPlane(Eigen::Vector3d point, GLfloat radius, Eigen::Vector3d normal)
{
    GLfloat N = 20;
    GLfloat step = (2 * M_PI / N);
    float angle = 0.0;
    GLfloat x, y, z;
    
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = q.toRotationMatrix();
    T.block(0, 3, 3, 1) = point;
    glColor4f(0.0f,0.0f,0.0f,0.5f);
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
    return 0;
}

#endif //SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_
