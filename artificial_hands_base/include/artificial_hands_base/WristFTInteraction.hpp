#ifndef ATK_WRIST_FT_INTERACTION
#define ATK_WRIST_FT_INTERACTION

#include <artificial_hands_base/FrameDynamics.hpp>
#include <artificial_hands_base/WristFTDetection.hpp>

namespace atk
{

  using FIR = atk::BaseFTSensor<atk::BaseFilter>;
  using IIR = atk::BaseFTSensor<atk::BaseIRFilter>;

  class WristFTInteraction: public atk::WristFTDetection, public atk::FrameDynamics
  {

    public:
      /**
       * @brief Construct a new WristFTInteraction object
       * @param filter_length length in samples of FIR (simple moving average) filter on joint angles
       * @param controller_rate rate in Hz of the joint state controller
       * @param target_frame frame to compute for absolute kinematics
       * @param planning_group name of the moveit planning group of target frame
       * @param robot_description name of XML robot description
       * @param model_frame absolute frame for the planning group
       */
      WristFTInteraction(int filter_length, const double controller_rate, const char* target_frame, const char* srdf_group="manipulator", 
      const char* robot_description="robot_description", const char* model_frame="world"):
      FrameDynamics(filter_length,controller_rate,target_frame,srdf_group,robot_description,model_frame)
      {
        sensor_e_ = new atk::BaseFTSensor<atk::BaseFilter>();
        sensor_e_->SetFilter(10);
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
        resetVector3(&th_dyn_.force);
        resetVector3(&th_dyn_.torque);
        wrench_e_.force = force_e;
        wrench_e_.torque = torque_e;
        sensor_e_->Init(wrench_e_);
        WristFTDetection::SetZero(false);
        return WristFTDetection::Init(ft) & FrameDynamics::Init(js,ft);
      };

      /**
       * @brief Get updated interaction state.
       * @return True on success
       */
      bool Get() 
      {
        WristFTDetection::Get();
        FrameDynamics::Get();
        force_fir = FIR::force;
        torque_fir = FIR::torque;
        force_iir = IIR::force;
        torque_iir = IIR::torque;
        force_raw = IIR::force_raw;
        torque_raw = IIR::torque_raw;
        force_e = sensor_e_->force;
        torque_e = sensor_e_->torque;
        return true;
      };

      /**
       * @brief Estimate interaction (external) forces.
       */
      void Estimate()
      {
        FrameDynamics::Estimate();
        wrench_e_.force = subtractVector3(force_fir,force_hat);
        wrench_e_.torque = subtractVector3(torque_fir,torque_hat);
        sensor_e_->Update(wrench_e_);
        sensor_e_->Get();
        absoluteVector3(&sensor_e_->force); //TO DO remove absolute value from here
        absoluteVector3(&sensor_e_->torque); 
      };

      /**
       * @brief Save maximum of interaction forces for further thresholding.
       */
      void SaveInteraction()
      {
        Estimate();
        compareVector3(&th_dyn_.force,sensor_e_->force);
        compareVector3(&th_dyn_.torque,sensor_e_->torque);
      };

      /**
       * @brief Trigger interaction forces for dynamic handover
       * @param factor multiplication factor for the saved threshold vector
       */
      void TriggerDynamics(double factor = 1.0)
      {
        Estimate();
        th_dyn_.torque.z = torque_e.z; //removed too sensible torque vector
        trigger = triggerOrVector3(th_dyn_.force,sensor_e_->force,factor) | triggerOrVector3(th_dyn_.torque,torque_e,factor);
      };

      geometry_msgs::Vector3 force_fir;
      geometry_msgs::Vector3 torque_fir;
      geometry_msgs::Vector3 force_iir;
      geometry_msgs::Vector3 torque_iir;
      geometry_msgs::Vector3 force_raw;
      geometry_msgs::Vector3 torque_raw;
      geometry_msgs::Vector3 force_e;
      geometry_msgs::Vector3 torque_e;
      geometry_msgs::Wrench th_dyn_; // only for debug

    private:
      
      int order_ = 3;
      int chann_ = 6;
      double num_[3] = {0.0099537574599431831, -0.019594794464524581, 0.0099537574599431831};
      double den_[3] = {1.0, -1.9749591089928671, 0.97527182944822877};
      double factor_ = 2.0;
      geometry_msgs::Wrench th_det_;
      //geometry_msgs::Wrench th_dyn_;
      geometry_msgs::Wrench wrench_e_;
      atk::BaseFTSensor<atk::BaseFilter>* sensor_e_;
  };
}

#endif