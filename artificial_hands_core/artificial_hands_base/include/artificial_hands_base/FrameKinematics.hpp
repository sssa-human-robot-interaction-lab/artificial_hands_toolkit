#ifndef ATK_FRAME_KINEMATICS
#define ATK_FRAME_KINEMATICS

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <artificial_hands_base/BaseFilters.hpp>

#define USE_KALMAN false
#define USE_SG false

#if USE_KALMAN
#include <kalmancpp/kalman.hpp>
#endif

namespace atk
{
  class FrameKinematics: public atk::kinematics6D_t
  {

    public:
      /**
       * @brief Construct a new FrameKinematics object
       * @param update_rate rate in Hz of the joint state controller
       * @param target_frame frame to compute for absolute kinematics
       * @param srdf_group name of the planning group for the target frame (according to robot SRDF)
       * @param robot_description name of XML robot description (robot URDF)
       */
      FrameKinematics(const double update_rate, const char* target_frame, const char* srdf_group="manipulator", const char* robot_description="robot_description"):
        update_rate_(update_rate),
        target_frame_(target_frame),
        robot_description_(robot_description),
        srdf_group_(srdf_group)
      {

        robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

        joint_model_group_ = kinematic_model->getJointModelGroup(srdf_group_);
        kinematic_state_ = new moveit::core::RobotState(kinematic_model);
        
        KDL::Tree robot_kin;
        kdl_parser::treeFromUrdfModel(*robot_model_loader.getURDF(),robot_kin);
        robot_kin.getChain(kinematic_model->getRootLinkName(),target_frame_,robot_chain_);

        chann_ = robot_chain_.getNrOfJoints();

        jac_solver_ = new KDL::ChainJntToJacSolver(robot_chain_);
        dot_solver_ = new KDL::ChainJntToJacDotSolver(robot_chain_);

        j_pos_filter_ = new atk::BaseIRFilter();
        j_vel_filter_ = new atk::BaseIRFilter();
        j_acc_filter_ = new atk::BaseSGFilter(update_rate);

#if USE_KALMAN
        F_.resize(2*chann_,j_kal_init_obs_);
#endif
        j_meas_.resize(2*chann_);
        j_state_.resize(3*chann_);

        transform_ = kinematic_state_->getGlobalLinkTransform(target_frame_);
        rotation_ = transform_.rotation().inverse();

        std::cout << "USE_KALMAN: " << USE_KALMAN << std::endl;
        std::cout << "USE_SG: " << USE_SG << std::endl;
      }

      /**
       * @brief Set filter parameters
       * @param filter struct of filtrt_t parameters
       */
      void SetFilter(filter_t filter)
      {
        j_pos_filter_->SetFilter(filter,chann_);
        j_vel_filter_->SetFilter(filter,chann_);
      };

      /**
       * @brief Initialize kinematics state.
       * @param js message from joint_state controller
       * @return True on success
       */
      bool Init(sensor_msgs::JointState js) 
      {
#if USE_KALMAN
        j_kal_init_ = false;
        j_kal_init_counter_ = 0;
#endif
        j_pos_filter_->Init(js.position); 
        j_vel_filter_->Init(js.velocity);
        j_acc_filter_->Init(js.velocity);
        j_name_ = js.name;
        Update(js);
        Update(js); //Double update ensure acceleration starting from zero
        return true;
      };

      /**
       * @brief Initialize kinematics state.
       * @param js message from joint_state controller
       * @param q process noise variance
       * @return True on success
       */
      bool Init(sensor_msgs::JointState js, double q) 
      {
#if USE_KALMAN
        j_kal_filter_->pause();
        j_kal_filter_->variance(q);
#endif
        Init(js);
        return true;
      };

      /**
       * @brief Update kinematics state.
       * @param js message from joint_state controller
       */
      void Update(sensor_msgs::JointState js)
      {
        j_pos_filter_->Update(js.position);
        j_vel_filter_->Update(js.velocity);
        j_acc_filter_->Update(js.velocity);
             
        std::vector<double> j_position = j_pos_filter_->Get();
        std::vector<double> j_velocity = j_vel_filter_->Get();
        for(int i = 0; i <j_name_.size(); i++)
        {
          kinematic_state_->setJointPositions(kinematic_state_->getJointModel(j_name_[i].c_str()), &j_position[i]);
          kinematic_state_->setJointVelocities(kinematic_state_->getJointModel(j_name_[i].c_str()), &j_velocity[i]);
        }

      #if USE_KALMAN

        if(!j_kal_init_)
        {
          observeKalman(j_position, j_velocity);

          if(j_kal_init_counter_ == j_kal_init_obs_ - 1)
          {
            initKalman();
          }

          j_kal_init_counter_ += 1;
        }
        else
        {
          updateKalman(j_position, j_velocity);

          j_state_ = j_kal_filter_->state();

          std::vector<std::string> j_group_names = joint_model_group_->getJointModelNames();
          std::vector<double> j_acc_new(chann_,0.0);

          for(int i = 0; i <j_name_.size(); i++)
          {
            kinematic_state_->setJointPositions(kinematic_state_->getJointModel(j_name_[i].c_str()), &j_state_(3*i));
            kinematic_state_->setJointVelocities(kinematic_state_->getJointModel(j_name_[i].c_str()), &j_state_(3*i+1));
            j_acc_new[std::find(j_group_names.begin(), j_group_names.end(), j_name_[i].c_str()) - j_group_names.begin() - 1] = j_state_(3*i+2);
          }
          
        }

      #elif USE_SG

        std::vector<double> j_acceleration = j_acc_filter_->Get();
        
        std::vector<std::string> j_group_names = joint_model_group_->getJointModelNames();
        std::vector<double> j_acc_new(chann_,0.0);

        for(int i = 0; i <j_name_.size(); i++)
        {
          j_acc_new[std::find(j_group_names.begin(), j_group_names.end(), j_name_[i].c_str()) - j_group_names.begin() - 1] = j_acceleration[i];
        }
          
      #else

        std::vector<double> j_vel_new;
        kinematic_state_->copyJointGroupVelocities(joint_model_group_, j_vel_new);

        std::vector<double> j_acc_new;
        for(int i = 0; i <j_name_.size(); i++)
        {
          j_acc_new.push_back((j_vel_new[i] - j_vel_[i])*update_rate_);
          j_vel_[i] = j_vel_new[i];
        }
        
      #endif

        kinematic_state_->setJointGroupAccelerations(joint_model_group_, j_acc_new);
      
      };

      /**
       * @brief Get updated target frame kinematics.
       * @return True on success
       */
      bool Get()
      { 
        transform_ = kinematic_state_->getGlobalLinkTransform(target_frame_);
        rotation_ = transform_.rotation().inverse();
      
        tf::vectorEigenToMsg(transform_.translation(),position.linear);
        tf::vectorEigenToMsg(transform_.rotation().eulerAngles(0,1,2),position.angular);

        KDL::JntArrayAcc j_arr_acc;
        kinematic_state_->copyJointGroupPositions(joint_model_group_,j_arr_acc.q.data);
        kinematic_state_->copyJointGroupVelocities(joint_model_group_,j_arr_acc.qdot.data);
        kinematic_state_->copyJointGroupAccelerations(joint_model_group_,j_arr_acc.qdotdot.data);

        KDL::JntArrayVel j_arr_vel;
        j_arr_vel.q.data = j_arr_acc.q.data;
        j_arr_vel.qdot.data = j_arr_acc.qdot.data;

        KDL::JntArray j_arr;
        j_arr.data = j_arr_vel.q.data;

        KDL::Jacobian jac(6);
        if(jac_solver_->JntToJac(j_arr,jac))
        {
          // str_message = "KDL::ChainToJacSolver >> %s",jac_solver_->strError(jac_solver_->getError());
          return false;
        }

        Eigen::VectorXd vel_vec = jac.data*j_arr_vel.qdot.data;
        tf::twistEigenToMsg(vel_vec,velocity);
        Transform6d(vel_vec,&twist_velocity,rotation_);

        KDL::Jacobian jac_dot(6);
        if(dot_solver_->JntToJacDot(j_arr_vel,jac_dot))
        {
          // str_message = "KDL::ChainToJacDotSolver >> %s",dot_solver_->strError(dot_solver_->getError());
          return false;
        }

        Eigen::VectorXd acc_vec = jac.data*j_arr_acc.qdotdot.data + jac_dot.data*j_arr_acc.qdot.data;
        tf::twistEigenToMsg(acc_vec,acceleration);
        Transform6d(acc_vec,&twist_acceleration,rotation_);

        return true;
      }
      
      /**
       * @brief Transform from 6 dof kinematics to target frame twist
       * @param in 6 dof kinematics expressed in the reference frame
       * @param out twist expressed in the target frame
       * @param rot rotation matrix from kinematics reference frame to target frame
       */
      void Transform6d(Eigen::VectorXd in, geometry_msgs::Twist *out, Eigen::Matrix3d rot)
      {
        Eigen::Vector3d lin;
        Eigen::Vector3d ang;

        for(int i = 0; i < 3; i++)
        {
          lin(i) = in(i);
          ang(i) = in(i+3);
        }

        lin = rot*lin;
        ang = rot*ang;

        tf::vectorEigenToMsg(lin,out->linear);
        tf::vectorEigenToMsg(ang,out->angular);
      }

      Eigen::Affine3d transform_;
      Eigen::MatrixXd rotation_;

    private:
#if USE_KALMAN
      void observeKalman(std::vector<double>& j_pos, std::vector<double>& j_vel)
      {
        for(int i = 0; i < chann_; i++)
          {
            F_(2*i,j_kal_init_counter_) = j_pos[i];
            F_(2*i+1,j_kal_init_counter_) = j_vel[i];
          }
      }

      void updateKalman(std::vector<double>& j_pos, std::vector<double>& j_vel)
      {
        for(int i = 0; i < chann_; i++)
        {
          j_meas_(2*i) = j_pos[i];
          j_meas_(2*i+1) = j_vel[i];
        }
        j_kal_filter_->update(j_meas_);
      }

      void initKalman()
      {
        double dt = 1.0/update_rate_;

            Eigen::VectorXd n(j_kal_init_obs_);
            Eigen::MatrixXd A(3*chann_,3*chann_);
            Eigen::MatrixXd C(2*chann_,3*chann_);
            Eigen::MatrixXd Q(3*chann_,3*chann_);
            Eigen::MatrixXd R(2*chann_,2*chann_);
            Eigen::MatrixXd P(3*chann_,3*chann_);

            n.setOnes();
            
            A.setZero();
            for(int i = 0; i < chann_; i++)
            {
              A(3*i,3*i) = 1;
              A(3*i,3*i+1) = dt;
              A(3*i,3*i+2) = dt*dt/2;
              A(3*i+1,3*i+1) = 1;
              A(3*i+1,3*i+2) = dt;
              A(3*i+2,3*i+2) = 1;
            }  
            
            C.setZero();
            int c = 0;
            for(int i = 0; i < chann_; i++)
            {
              C(2*i,2*i+c) = 1;
              C(2*i+1,2*i+c+1) = 1;
              c++;
            }
            
            Q.setZero();
            for(int i = 0; i < chann_; i++)
            {
              Q(3*i,3*i) = pow(dt,4)/4;
              Q(3*i,3*i+1) = pow(dt,3)/2;
              Q(3*i,3*i+2) = dt*dt/2;
              Q(3*i+1,3*i) = Q(3*i,3*i+1);
              Q(3*i+1,3*i+1) = dt*dt;
              Q(3*i+1,3*i+2) = dt;
              Q(3*i+2,3*i) = Q(3*i,3*i+2);
              Q(3*i+2,3*i+1) = dt;
              Q(3*i+2,3*i+2) = 1;
            }
            
            R = 1.0/((double)j_kal_init_obs_-1.0)*(F_-F_.rowwise().mean()*n.transpose())*(F_-F_.rowwise().mean()*n.transpose()).transpose();         
            
            P.setZero();   
            for(int i = 0; i < chann_; i++)
            {
              P(3*i,3*i) = F_.row(2*i).mean();
              P(3*i+1,3*i+1) = F_.row(2*i+1).mean();
            }         

            j_kal_filter_ = new kalmancpp::KalmanFilter(dt,A,C,Q,R,P);
            // j_kal_filter_->variance(1E-6);
            j_kal_filter_->init();
            j_kal_init_ = true;
      }
#endif

      const char* robot_description_;
      const char* srdf_group_;
      const char* target_frame_;
      const char* model_frame_;
      unsigned int chann_;
      const double update_rate_;
      double j_vel_[20] = {0};
      robot_state::JointModelGroup *joint_model_group_;
      moveit::core::RobotState *kinematic_state_;
      KDL::ChainJntToJacSolver *jac_solver_;
      KDL::ChainJntToJacDotSolver *dot_solver_;
      KDL::Chain robot_chain_; 
      Eigen::MatrixXd F_; 
      Eigen::VectorXd j_meas_, j_state_;
      std::vector<std::string> j_name_;

      atk::BaseIRFilter* j_pos_filter_;
      atk::BaseIRFilter* j_vel_filter_;
      atk::BaseSGFilter* j_acc_filter_;  
#if USE_KALMAN
      bool j_kal_init_;
      unsigned int j_kal_init_counter_;
      const unsigned int j_kal_init_obs_ = 500;
      kalmancpp::KalmanFilter* j_kal_filter_;
#endif
  };
}

#endif