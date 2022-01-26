#ifndef ATK_INTERACTION
#define ATK_INTERACTION

#include <artificial_hands_base/Dynamics.hpp>
#include <artificial_hands_wrist/Detection.hpp>

namespace atk
{

  using FIR = atk::Sensor<atk::Filter>;
  using IIR = atk::Sensor<atk::IRFilter>;

  class Interaction: public atk::Dynamics, public atk::Detection
  {

    public:
      /**
       * @brief Construct a new Interaction object
       * @param filter_length length in samples of FIR (simple moving average) filter on joint angles
       * @param controller_rate rate in Hz of the joint state controller
       * @param target_frame frame to compute for absolute kinematics
       * @param planning_group name of the moveit planning group of target frame
       * @param robot_description name of XML robot description
       * @param model_frame absolute frame for the planning group
       */
      Interaction(int filter_length, const double controller_rate, const char* target_frame, const char* planning_group="manipulator", 
      const char* robot_description="robot_description", const char* model_frame="world"):
      Dynamics(filter_length,controller_rate,target_frame)
      {
        resetVector3(&th_dyn_.force);
        resetVector3(&th_dyn_.torque);
      };

      /**
       * @brief Initialize interaction state.
       * @param js message from joint_state controller
       * @param ft message from sensor driver
       * @return True on success
       */
      bool Init(sensor_msgs::JointState js, geometry_msgs::Wrench ft) 
      {
        resetVector3(&force_e);
        resetVector3(&torque_e);
        return Dynamics::Init(js,ft) & Detection::Init(ft);
      };

      /**
       * @brief Get updated interaction state.
       * @return True on success
       */
      bool Get() 
      {
        Dynamics::Get();
        Detection::Get();
        force_fir = FIR::force;
        torque_fir = FIR::torque;
        force_iir = IIR::force;
        torque_iir = IIR::torque;
        force_raw = IIR::force_raw;
        torque_raw = IIR::torque_raw;
        return true;
      };

      /**
       * @brief Estimate interaction (external) forces.
       */
      void Estimate()
      {
        Dynamics::Estimate();
        force_e = subtractVector3(force_fir,force_hat);
        torque_e = subtractVector3(torque_fir,torque_hat);
        absoluteVector3(&force_e);
        absoluteVector3(&torque_e); 
      };

      /**
       * @brief Save maximum of interaction forces for further thresholding.
       */
      void SaveInteraction()
      {
        Estimate();
        compareVector3(&th_dyn_.force,force_e);
        compareVector3(&th_dyn_.torque,torque_e);
      };

      /**
       * @brief Trigger interaction forces for dynamic handover
       * @param factor multiplication factor for the saved threshold vector
       */
      void TriggerDynamics(double factor = 1.0)
      {
        Estimate();
        th_dyn_.torque.z = torque_e.z; //removed too noisy torque on z axis from trigger
        trigger = triggerOrVector3(th_dyn_.force,force_e,factor) | triggerOrVector3(th_dyn_.torque,torque_e,factor);
      };

      /**
       * @brief Do zero of sensor readings
       */
      void DoZero()
      {
        Dynamics::Sensor::SetZero(true);
        Detection::Sensor::SetZero(true);
      }

      geometry_msgs::Vector3 force_fir;
      geometry_msgs::Vector3 torque_fir;
      geometry_msgs::Vector3 force_iir;
      geometry_msgs::Vector3 torque_iir;
      geometry_msgs::Vector3 force_raw;
      geometry_msgs::Vector3 torque_raw;
      geometry_msgs::Vector3 force_e;
      geometry_msgs::Vector3 torque_e;

    private:
      
      int order_ = 3;
      int chann_ = 6;
      double num_[3] = {0.0099537574599431831, -0.019594794464524581, 0.0099537574599431831};
      double den_[3] = {1.0, -1.9749591089928671, 0.97527182944822877};
      double factor_ = 2.0;
      geometry_msgs::Wrench th_det_;
      geometry_msgs::Wrench th_dyn_;
  };
}

#endif