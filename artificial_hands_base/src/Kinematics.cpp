#include <artificial_hands_base/Kinematics.hpp>

namespace atk
{
  Kinematics::Kinematics(int filter_length, const double loop_rate, const char* target_frame, const char* planning_group,
    const char* robot_description, const char* model_frame):
    loop_rate_(loop_rate),
    target_frame_(target_frame),
    robot_description_(robot_description),
    planning_group_(planning_group),
    model_frame_(model_frame),
    Filter(filter_length)
  {

    robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    joint_model_group_ = kinematic_model->getJointModelGroup(planning_group_);
    kinematic_state_ = new moveit::core::RobotState(kinematic_model);

    KDL::Tree robot_kin;
    kdl_parser::treeFromUrdfModel(*robot_model_loader.getURDF(),robot_kin);
    robot_kin.getChain(model_frame_,target_frame_,robot_chain_);

    jac_solver_ = new KDL::ChainJntToJacSolver(robot_chain_);
    dot_solver_ = new KDL::ChainJntToJacDotSolver(robot_chain_);

  }

  void Kinematics::Update(sensor_msgs::JointState js)
  {

    Filter::Update(js.position);
    std::vector<double> j_position = Filter::Get();

    for(int i = 0; i <js.name.size(); i++)
    {
      kinematic_state_->setJointPositions(js.name[i].c_str(), &j_position[i]);
    }

    std::vector<double> j_pos_new;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, j_pos_new);

    std::vector<double> j_vel_new;
    std::vector<double> j_acc_new;

    for(int i = 0; i < j_pos_new.size(); i++)
    {
      j_vel_new.push_back((j_pos_new[i] - j_pos_[i])*loop_rate_);
      j_acc_new.push_back((j_vel_new[i] - j_vel_[i])*loop_rate_);

      j_pos_[i] = j_pos_new[i];
      j_vel_[i] = j_vel_new[i];
    }

    kinematic_state_->setJointGroupVelocities(joint_model_group_, j_vel_new);
    kinematic_state_->setJointGroupAccelerations(joint_model_group_, j_acc_new);

    transform_ = kinematic_state_->getGlobalLinkTransform(target_frame_);
    rotation_ = transform_.rotation().inverse();
  }

  bool Kinematics::Get()
  {
    
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

  void Kinematics::Transform6d(Eigen::VectorXd in, geometry_msgs::Twist *out, Eigen::Matrix3d rot)
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

}