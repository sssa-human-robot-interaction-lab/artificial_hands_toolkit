#ifndef ATK_KINEMATICS
#define ATK_KINEMATICS

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <artificial_hands_base/Filters.hpp>

namespace atk
{
  class Kinematics: private atk::Filter, public atk::kinematics6D_t
  {

    public:
      /**
       * @brief Construct a new Kinematics object
       * @param filter_length length in samples of FIR (simple moving average) filter on joint angles
       * @param controller_rate rate in Hz of the joint state controller
       * @param target_frame frame to compute for absolute kinematics
       * @param planning_group name of the moveit planning group of target frame
       * @param robot_description name of XML robot description
       * @param model_frame absolute frame for the planning group
       */
      Kinematics(int filter_length, const double loop_rate, const char* target_frame, const char* planning_group="manipulator", 
      const char* robot_description="robot_description", const char* model_frame="world");

      /**
       * @brief Initialize kinematics state.
       * @param js message from joint_state controller
       * @return True on success
       */
      bool Init(sensor_msgs::JointState js) 
      {
        Filter::Init(js.position); 
        Update(js);
        Update(js); //Double update ensure acceleration starting from zero
        return true;
      };

      /**
       * @brief Update kinematics state.
       * @param js message from joint_state controller
       */
      void Update(sensor_msgs::JointState js);

      /**
       * @brief Get updated target frame kinematics.
       * @return True on success
       */
      bool Get();

      /**
       * @brief Transform from 6 dof kinematics to target frame twist
       * @param in 6 dof kinematics expressed in the reference frame
       * @param out twist expressed in the target frame
       * @param rot rotation matrix from kinematics reference frame to target frame
       */
      void Transform6d(Eigen::VectorXd in, geometry_msgs::Twist *out, Eigen::Matrix3d rot);

      /**
       * @brief Transform 3D vector from reference frame to target frame
       * @param in 3D vector expressed in the reference frame
       * @param out 3D vector expressed in the target frame
       * @param rot rotation matrix from vector reference frame to target frame
       */
      void Transform3d(Eigen::Vector3d in, geometry_msgs::Vector3 *out, Eigen::Matrix3d rot);

      Eigen::Affine3d transform_;
      Eigen::MatrixXd rotation_;

    private:

      const char* robot_description_;
      const char* srdf_group_;
      const char* target_frame_;
      const char* model_frame_;
      const double loop_rate_;
      double j_pos_[20] = {0};
      double j_vel_[20] = {0};
      robot_state::JointModelGroup *joint_model_group_;
      moveit::core::RobotState *kinematic_state_;
      KDL::ChainJntToJacSolver *jac_solver_;
      KDL::ChainJntToJacDotSolver *dot_solver_;
      KDL::Chain robot_chain_;    
  };
}

#endif