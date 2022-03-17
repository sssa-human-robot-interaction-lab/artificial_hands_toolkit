#ifndef ATK_FRAME_DYNAMICS
#define ATK_FRAME_DYNAMICS

#include <artificial_hands_base/BaseFTSensor.hpp>
#include <artificial_hands_base/FrameKinematics.hpp>

namespace atk
{

  class FrameDynamics: public atk::FrameKinematics, public atk::BaseFTSensor<BaseFilter>
  {
    
    public:  
      /**
       * @brief Construct a new FrameDynamics object
       * @param filter_length length in samples of FIR (simple moving average) filter on joint angles
       * @param controller_rate rate in Hz of the joint state controller
       * @param target_frame frame to compute for absolute kinematics
       * @param srdf_group name of the planning group for the target frame (according to robot SRDF)
       * @param robot_description name of XML robot description (robot URDF)
       * @param model_frame absolute frame for the planning group
       */
      FrameDynamics(int filter_length, const double controller_rate, const char* target_frame, const char* srdf_group="manipulator", 
      const char* robot_description="robot_description", const char* model_frame="world"):
        FrameKinematics(filter_length,controller_rate, target_frame, srdf_group, robot_description, model_frame)
      {
        BaseFTSensor::SetFilter(filter_length);

        gravity_.x = 0;
        gravity_.y = 0;
        gravity_.z = -9.81;

        x_.reserve(6*10*1000);
        y_.reserve(6*1000);
      }

      /**
       * @brief Initialize dynamics state.
       * @param js message from joint_state controller
       * @param ft message from sensor driver
       * @return True on success
       */
      bool Init(sensor_msgs::JointState js, geometry_msgs::Wrench ft) 
      {
        x_.clear();
        y_.clear();
        V_.clear();
        phi_.clear();
        return FrameKinematics::Init(js) & BaseFTSensor::Init(ft);
      };

      /**
       * @brief Update dynamics state.
       * @param js message from joint_state controller
       * @param ft message from sensor driver
       */
      void Update(sensor_msgs::JointState js, geometry_msgs::Wrench ft)
      {
        FrameKinematics::Update(js); 
        BaseFTSensor::Update(ft);

        Eigen::Vector3d g;
        tf::vectorMsgToEigen(gravity_,g);
        tf::vectorEigenToMsg(rotation_*g,gravity);
      }

      /**
       * @brief Get updated target frame dynamics.
       * @return True on success
       */
      bool Get() 
      {
        return FrameKinematics::Get() & BaseFTSensor::Get();
      };

      /**
       * @brief Add dynamics state to the set of equations.
       */
      void AddEquation(){
        GetCoefficients();
        double y[6] = {force.x, force.y, force.z, torque.x, torque.y, torque.z};
        addArray(&x_,V_.data(),60);
        addArray(&y_,y,6);
      }

      /**
       * @brief Get updated vector of coefficients
       */
      void GetCoefficients()
      {

        double wx = twist_velocity.angular.x;
        double wy = twist_velocity.angular.y;
        double wz = twist_velocity.angular.z;

        double wx2 = -pow(wx,2);
        double wy2 = -pow(wy,2);
        double wz2 = -pow(wz,2);

        double ax = -twist_acceleration.linear.x;
        double ay = -twist_acceleration.linear.y;
        double az = -twist_acceleration.linear.z;

        double ex = -twist_acceleration.angular.x;
        double ey = -twist_acceleration.angular.y;
        double ez = -twist_acceleration.angular.z;

        double gx = -gravity.x;
        double gy = -gravity.y;
        double gz = -gravity.z;

        double x[60] = {ax-gx,    -wy2-wz2,     wx*wy-ez,     wx*wz+ey,   .0, .0, .0, .0, .0, .0,
                        ay-gy,    wx*wy+ez,     -wx2-wz2,     wy*wz-ex,   .0, .0, .0, .0, .0, .0,
                        az-gz,    wx*wz-ey,     wy*wz+ex,     -wy2-wx2,   .0, .0, .0, .0, .0, .0,

                        .0,       .0,           az-gz,        gy-ay,      ex,         ey-wx*wz,     ez+wx*wy,    -wy*wz,      wy2-wz2,      wy*wz,
                        .0,       gz-az,        .0,           ax-gx,      wx*wz,      ex+wy*wz,     wz2-wx2,     ey,          ez-wx*wy,     -wx*wz,
                        .0,       ay-gy,        gx-ax,        .0,         -wx*wy,     wx2-wy2,      ex-wy*wz,    wx*wy,       ey+wx*wz,     ez      };
        
        copyArray(&V_,x,60);
      
      }

      /**
       * @brief Solve dynamics equation to retrieve the inertial model
       * @return True on Succcess
       */
      bool Solve()
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

      /**
       * @brief Estimate forces acting on the dynamic system.
       */
      void Estimate()
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
      }

      geometry_msgs::Vector3 force_hat;
      geometry_msgs::Vector3 torque_hat;
      geometry_msgs::Vector3 gravity;
      std::stringstream phi_str;
      
    private:

      geometry_msgs::Vector3 gravity_;
      std::vector<double> x_;
      std::vector<double> y_;
      std::vector<double> V_;
      std::vector<double> phi_;

  };
}

#endif