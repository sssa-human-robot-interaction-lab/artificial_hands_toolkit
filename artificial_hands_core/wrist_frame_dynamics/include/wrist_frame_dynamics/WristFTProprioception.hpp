#ifndef ATK_WRIST_FT_INTERACTION
#define ATK_WRIST_FT_INTERACTION

#include <artificial_hands_base/FrameDynamics.hpp>
#include <wrist_frame_dynamics/WristFTContactDetection.hpp>

namespace atk
{

  class WristFTProprioception: public atk::WristFTContactDetection, public atk::FrameDynamics
  {

    public:
    
      /**
       * @brief Construct a new WristFTProprioception object
       * @param controller_rate rate in Hz of the joint state controller
       * @param target_frame frame to compute for absolute kinematics
       * @param planning_group name of the moveit planning group of target frame
       * @param robot_description name of XML robot description
       */
      WristFTProprioception(const double controller_rate, const char* target_frame, const char* srdf_group="manipulator", const char* robot_description="robot_description"):
        FrameDynamics(controller_rate,target_frame,srdf_group,robot_description)
      {
        sensor_pr_ = new atk::BaseFTSensor<atk::BaseFilter>();
        sensor_pr_->SetFilter(3);
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
        resetVector3(&force_pr);
        resetVector3(&torque_pr);
        resetVector3(&th_dyn_.force);
        resetVector3(&th_dyn_.torque);
        wrench_pr_.force = force_pr;
        wrench_pr_.torque = torque_pr;
        sensor_pr_->Init(wrench_pr_);
        WristFTContactDetection::SetZero(false);
        return WristFTContactDetection::Init(ft) & FrameDynamics::Init(js,ft);
      };

      /**
       * @brief Get updated interaction state.
       * @return True on success
       */
      bool Get() 
      {
        WristFTContactDetection::Get();
        FrameDynamics::Get();
        force_dyn = FrameDynamics::force;
        torque_dyn = FrameDynamics::torque;
        force_lp = WristFTContactDetection::force;
        torque_lp = WristFTContactDetection::torque;
        force_raw = WristFTContactDetection::force_raw;
        torque_raw = WristFTContactDetection::torque_raw;
        force_pr = sensor_pr_->force;
        torque_pr = sensor_pr_->torque;
        return true;
      };

      /**
       * @brief Estimate interaction (external) forces.
       */
      void Estimate()
      {
        FrameDynamics::Estimate();
        wrench_pr_.force = subtractVector3(force_dyn,force_hat);
        wrench_pr_.torque = subtractVector3(torque_dyn,torque_hat);
        sensor_pr_->Update(wrench_pr_);
        sensor_pr_->Get();
        absoluteVector3(&sensor_pr_->force); //TO DO remove absolute value from here
        absoluteVector3(&sensor_pr_->torque); 
      };

      /**
       * @brief Save maximum of proprioceptive forces for further thresholding.
       */
      void SaveInteraction()
      {
        Estimate();
        compareVector3(&th_dyn_.force,sensor_pr_->force);
        compareVector3(&th_dyn_.torque,sensor_pr_->torque);
      };

      /**
       * @brief Trigger interaction forces for dynamic handover
       * @param factor multiplication factor for the saved threshold vector
       */
      void TriggerDynamics(double factor = 1.0)
      {
        Estimate();
        trigger = triggerOrVector3(th_dyn_.force,sensor_pr_->force,factor); // not using torque
      };

      geometry_msgs::Vector3 force_dyn;
      geometry_msgs::Vector3 torque_dyn;
      geometry_msgs::Vector3 force_lp;
      geometry_msgs::Vector3 torque_lp;
      geometry_msgs::Vector3 force_raw;
      geometry_msgs::Vector3 torque_raw;
      geometry_msgs::Vector3 force_pr;
      geometry_msgs::Vector3 torque_pr;
      geometry_msgs::Wrench th_dyn_; // only for debug

    private:
      
      double factor_ = 2.0;
      geometry_msgs::Wrench th_det_;
      //geometry_msgs::Wrench th_dyn_;
      geometry_msgs::Wrench wrench_pr_;
      atk::BaseFTSensor<atk::BaseFilter>* sensor_pr_;
  };
}

#endif