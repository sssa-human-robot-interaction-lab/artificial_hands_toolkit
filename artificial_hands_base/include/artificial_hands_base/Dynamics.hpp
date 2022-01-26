#ifndef ATK_DYNAMICS
#define ATK_DYNAMICS

#include <artificial_hands_base/Kinematics.hpp>
#include <artificial_hands_base/Sensor.hpp>

namespace atk
{

  class Dynamics: public atk::Kinematics, public atk::Sensor<Filter>
  {
    
    public:  
      /**
       * @brief Construct a new Dynamics object
       * @param filter_length length in samples of FIR (simple moving average) filter on joint angles
       * @param controller_rate rate in Hz of the joint state controller
       * @param target_frame frame to compute for absolute kinematics
       * @param planning_group name of the moveit planning group of target frame
       * @param robot_description name of XML robot description
       * @param model_frame absolute frame for the planning group
       */
      Dynamics(int filter_length, const double controller_rate, const char* target_frame, const char* planning_group="manipulator", 
      const char* robot_description="robot_description", const char* model_frame="world");

      /**
       * @brief Initialize dynamics state.
       * @param js message from joint_state controller
       * @param ft message from sensor driver
       * @return True on success
       */
      bool Init(sensor_msgs::JointState js, geometry_msgs::Wrench ft) 
      {
        return Kinematics::Init(js) & Sensor::Init(ft);
      };

      /**
       * @brief Update dynamics state.
       * @param js message from joint_state controller
       * @param ft message from sensor driver
       */
      void Update(sensor_msgs::JointState js, geometry_msgs::Wrench ft);

      /**
       * @brief Get updated target frame dynamics.
       * @return True on success
       */
      bool Get() 
      {
        return Kinematics::Get() & Sensor::Get();
      };

      /**
       * @brief Add dynamics state to the set of equations.
       */
      void AddEquation();

      /**
       * @brief Get updated vector of coefficients
       */
      void GetCoefficients();

      /**
       * @brief Solve dynamics equation to retrieve the inertial model
       * @return True on Succcess
       */
      bool Solve();

      /**
       * @brief Estimate forces acting on the dynamic system.
       */
      void Estimate();

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