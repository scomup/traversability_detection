#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <memory>
#include <mutex>

namespace pangolin
{
struct MyHandler3D : Handler3D
{
  std::mutex mutex_;
  Eigen::Matrix4d trans_pose_;
  bool done_;
  MyHandler3D(OpenGlRenderState &cam_state,
              AxisDirection enforce_up = AxisZ,
              //AxisDirection enforce_up = AxisNone,
              float trans_scale = 0.01f,
              float zoom_fraction = PANGO_DFLT_HANDLER3D_ZF): 
      Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction)
      {
          done_ = true;
          trans_pose_ = Eigen::Matrix4d::Identity();
      };

void rotationMatrixToEulerAngles(GLprecision M[16], GLprecision angle[3])
{
 
     
    float sy = sqrt(M[0] * M[0] +  M[1] * M[1] );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(M[6] , M[10]);
        y = atan2(M[2], sy);
        z = atan2(M[1], M[0]);
    }
    else
    {
        x = atan2(-M[9], M[5]);
        y = atan2(-M[2], sy);
        z = 0;
    }
    angle[0] = x;
    angle[1] = y;
    angle[2] = z;
}

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw)
{
    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    return yaw_angle * pitch_angle * roll_angle;
}

void Keyboard(View &, unsigned char key, int x, int y, bool pressed)
{
    std::unique_lock<std::mutex> lock(mutex_);
    // TODO: hooks for reset / changing mode (perspective / ortho etc)
    if(!pressed)
    return;
    trans_pose_ = Eigen::Matrix4d::Identity();
    done_ = false;

    if (key == 229 )//up
    {
    trans_pose_.block(0, 3, 3, 1) = Eigen::Vector3d(0.05,0,0);

    }
    else if (key == 231)//back
    {
           trans_pose_.block(0, 3, 3, 1) = Eigen::Vector3d(-0.05,0,0);
    }
    else if (key == 230)//right
    {
           trans_pose_.block(0, 0, 3, 3) = RollPitchYaw(0,0,-0.1).toRotationMatrix();
    }
    else if (key == 228)//left
    {
        trans_pose_.block(0, 0, 3, 3) = RollPitchYaw(0,0,0.1).toRotationMatrix();
    }
  }

  bool getPose(Eigen::Matrix4d& pose)
  {
      std::unique_lock<std::mutex> lock(mutex_);
      if (done_){
          return false;
      }
      else{
          done_ = true;
          pose = trans_pose_;
          return true;

      }
  }
};


} // namespace pangolin