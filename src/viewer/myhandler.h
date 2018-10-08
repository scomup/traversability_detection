#include <pangolin/pangolin.h>
#include <Eigen/Dense>

namespace pangolin
{
struct MyHandler3D : Handler3D
{
  MyHandler3D(OpenGlRenderState &cam_state,
              AxisDirection enforce_up = AxisZ,
              //AxisDirection enforce_up = AxisNone,
              float trans_scale = 0.01f,
              float zoom_fraction = PANGO_DFLT_HANDLER3D_ZF)
      : Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction){};

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

  void Keyboard(View &, unsigned char key, int x, int y, bool pressed)
  {
    // TODO: hooks for reset / changing mode (perspective / ortho etc)
    if(!pressed)
    return;

    GLprecision aboutx = 0;
    GLprecision aboutz = 0;


    if (key == 229 )
    {
    aboutz = 0.1;

    }
    else if (key == 231)
    {
           aboutz = -0.1;
    }
    else if (key == 230)
    {
           aboutx = -0.1;
    }
    else if (key == 228)
    {
           aboutx = 0.1;
    }

    OpenGlMatrix &mv = cam_state->GetModelViewMatrix();
    mv.m[12] += aboutx;
    //mv.m[13] -= abouty / tan(euler_angles[0]);
    mv.m[14] += aboutz ;
    
    //printf("[% 6.2f % 6.2f % 6.2f]\n", angle[0], angle[1], angle[2]);

/*
//Eigen::Matrix4d Twc(mv.Inverse().m);
    Eigen::Matrix4d Twc;//mv.Inverse().m
    GLprecision *m = mv.Inverse().m;

    Twc <<m[0],m[4],m[8],m[12],
          m[1],m[5],m[9],m[13],
          m[2],m[6],m[10],m[14],
          m[3],m[7],m[11],m[15];
            
            
    Eigen::Vector3f euler_angles = Twc.cast<float>().block<3,3>(0,0).eulerAngles(0, 1, 2); 
    std::cout<<Twc.cast<float>().block<3,3>(0,0)<<std::endl;
    printf("[% 6.2f % 6.2f % 6.2f ]\n", mv.Inverse().m[0], mv.Inverse().m[4], mv.Inverse().m[8]);
    printf("[% 6.2f % 6.2f % 6.2f ]\n", mv.Inverse().m[1], mv.Inverse().m[5], mv.Inverse().m[9] );
    printf("[% 6.2f % 6.2f % 6.2f ]\n", mv.Inverse().m[2], mv.Inverse().m[6], mv.Inverse().m[10]);



    //if(euler_angles[0] > M_PI);
    printf("1[% 6.2f % 6.2f % 6.2f]\n", angle[0], angle[1], angle[2]);
    printf("2[% 6.2f % 6.2f % 6.2f]\n", euler_angles[0], euler_angles[1], euler_angles[2]);
    */

    /*
    printf("[% 6.2f % 6.2f % 6.2f]\n", angle[0], angle[1], angle[2]);
    std::cout << "Tcw" << std::endl;
    printf("[% 6.2f % 6.2f % 6.2f % 6.2f]\n", mv.m[0], mv.m[4], mv.m[8], mv.m[12]);
    printf("[% 6.2f % 6.2f % 6.2f % 6.2f]\n", mv.m[1], mv.m[5], mv.m[9], mv.m[13]);
    printf("[% 6.2f % 6.2f % 6.2f % 6.2f]\n", mv.m[2], mv.m[6], mv.m[10], mv.m[14]);
    printf("[% 6.2f % 6.2f % 6.2f % 6.2f]\n", mv.m[3], mv.m[7], mv.m[11], mv.m[15]);

    GLprecision T_wc[4 * 4];
    LieSetIdentity(T_wc);
    LieSE3from4x4(T_wc, mv.Inverse().m);
    std::cout << "Twc" << std::endl;
    printf("[% 6.2f % 6.2f % 6.2f % 6.2f]\n", T_wc[0], T_wc[3], T_wc[6], T_wc[9]);
    printf("[% 6.2f % 6.2f % 6.2f % 6.2f]\n", T_wc[1], T_wc[4], T_wc[7], T_wc[10]);
    printf("[% 6.2f % 6.2f % 6.2f % 6.2f]\n", T_wc[2], T_wc[5], T_wc[8], T_wc[11]);
    */
  }
};


} // namespace pangolin