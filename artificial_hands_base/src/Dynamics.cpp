#include <artificial_hands_base/Dynamics.hpp> 

namespace atk
{
  Dynamics::Dynamics(int filter_length, const double controller_rate, const char* target_frame, const char* planning_group,
    const char* robot_description, const char* model_frame):
  Kinematics(filter_length,controller_rate, target_frame, planning_group, robot_description, model_frame)
  {
    Sensor::SetFilter(filter_length);

    gravity_.x = 0;
    gravity_.y = 0;
    gravity_.z = -9.81;

    x_.resize(6*10*1000);
    y_.resize(6*1000);

  }

  void Dynamics::Update(sensor_msgs::JointState js, geometry_msgs::Wrench ft) 
  {
    Kinematics::Update(js); 
    Sensor::Update(ft);

    Eigen::Vector3d g;
    tf::vectorMsgToEigen(gravity_,g);
    tf::vectorEigenToMsg(rotation_*g,gravity);
  }

  void Dynamics::GetCoefficients()
  {

    double wx = -twist_velocity.angular.x;
    double wy = -twist_velocity.angular.y;
    double wz = -twist_velocity.angular.z;

    double wx2 = pow(wx,2);
    double wy2 = pow(wy,2);
    double wz2 = pow(wz,2);

    double ax = -twist_acceleration.linear.x;
    double ay = -twist_acceleration.linear.y;
    double az = -twist_acceleration.linear.z;

    double ex = -twist_acceleration.angular.x;
    double ey = -twist_acceleration.angular.y;
    double ez = -twist_acceleration.angular.z;

    double gx = -gravity.x;
    double gy = -gravity.y;
    double gz = -gravity.z;

    double x[60] = {ax-gx,    -wy2-wy2,     wx*wy-ez,     wx*wz+ey,   .0, .0, .0, .0, .0, .0,
                    ay-gy,    wx*wy+ez,     -wy2-wy2,     wy*wz-ex,   .0, .0, .0, .0, .0, .0,
                    az-gz,    wx*wz-ey,     wy*wz+ex,     -wy2-wy2,   .0, .0, .0, .0, .0, .0,

                    .0,       .0,           az-gz,        gy-ay,      ex,         ey-wx*wz,     ez+wx*wy,    -wy*wz,      wy2-wy2,      wy*wz,
                    .0,       gz-az,        .0,           ax-gx,      wx*wz,      ex+wy*wz,     wy2-wy2,     ey,          ez-wx*wy,     -wx*wz,
                    .0,       ay-gy,        gx-ax,        .0,         -wx*wy,     wy2-wy2,      ex-wy*wz,    wx*wy,       ey+wx*wz,     ez      };
    
    copyArray(&V_,x,60);
   
  }

  void Dynamics::AddEquation()
  {
    GetCoefficients();
    double y[6] = {force.x, force.y, force.z, torque.x, torque.y, torque.z};
    addArray(&x_,V_.data(),60);
    addArray(&y_,y,6);
  }

  bool Dynamics::Solve()
  {
    double l[10] = {0, -inf, -inf, -inf, 0, 0, 0, 0, 0, 0};
    double u[10];
    for(int i = 0; i < 10; i++)u[i] = inf;

    double* c = leastSquareFit(y_.size(),10,x_.data(),y_.data(),l,u);
    copyArray(&phi_,c,10);
    phi_str.str("");
    for(int i = 0; i < 10; i++)phi_str << phi_[i] << " ";
    return true;
  }

  void Dynamics::Estimate()
  {
    resetVector3(&force_hat);
    resetVector3(&torque_hat);
    
    GetCoefficients();
    
    for(int i = 0; i < 10; i++)
    {
      force_hat.x += V_[i]*phi_[i];
      force_hat.y += V_[i+10]*phi_[i];
      force_hat.z += V_[i+20]*phi_[i];
      torque_hat.x += V_[i+30]*phi_[i];
      torque_hat.y += V_[i+40]*phi_[i];
      torque_hat.z += V_[i+50]*phi_[i];
    }
  };

}