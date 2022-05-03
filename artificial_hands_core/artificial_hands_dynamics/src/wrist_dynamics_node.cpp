#include <artificial_hands_base/BaseROSCommon.hpp>
#include <artificial_hands_msgs/DetectionStamped.h>
#include <artificial_hands_msgs/WristDynamicsStamped.h>
#include <artificial_hands_msgs/WristDynamicsCommand.h>
#include <artificial_hands_dynamics/WristFTCalibration.hpp>
#include <artificial_hands_dynamics/WristFTProprioception.hpp>

#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointControllerState.h>

namespace rosatk
{
  using cmd = artificial_hands_msgs::WristDynamicsCommand;

  class WristDynamicsNode: public atk::WristFTProprioception, public atk::WristFTCalibration, public rosatk::ServiceManagerBase<rosatk::WristDynamicsNode,cmd>
  {
    
    public:
      WristDynamicsNode(ros::NodeHandle nh, bool autostart, bool publish, double factor, int filter_length, int rate, const char* sensor, const char* controller, int controller_rate, const char* target_frame):  
        nh_(nh),
        publish_(publish),
        factor_(factor),
        sensor_(sensor),
        controller_(controller),
        WristFTProprioception(controller_rate, target_frame, "manipulator", "robot_description"),
        ServiceManagerBase(nh,&WristDynamicsNode::command)
      {
        rosatk::FilterManagerBase kin_fil(nh,"frame_kinematics",filter_length);
        rosatk::FilterManagerBase dyn_fil(nh,"ft_sensor",filter_length);
        WristFTProprioception::SetKinematicsFilter(dyn_fil.filter);
        WristFTProprioception::SetFTFilter(dyn_fil.filter);

        ROS_INFO("Starting node with loop rate %i Hertz.",rate);
        ROS_INFO("Starting dynamics relative to frame %s.",target_frame);
        if(publish_)loop_pub_ = nh_.advertise<artificial_hands_msgs::WristDynamicsStamped>("wrist_dynamics_data",1000);
        loop_tim_ = nh_.createWallTimer(ros::WallDuration(1.0/(double)rate),&WristDynamicsNode::loopTimerCallback, this, false, false);
        loop_msg_.header.frame_id = target_frame;
        det_pub_ = nh_.advertise<artificial_hands_msgs::DetectionStamped>("wrist_contact_detection",1000);
        det_tim_ = nh_.createWallTimer(ros::WallDuration(0.1),&WristDynamicsNode::externalTimerCallback, this, false, false);
        det_msg_.header.frame_id = target_frame;
        ROS_INFO("Starting node services.");
        addService("wrist_dynamics_command/subscribe", cmd::Request::CMD_SUB, this);
        addService("wrist_dynamics_command/start_loop", cmd::Request::CMD_STA, this);
        addService("wrist_dynamics_command/build_model", cmd::Request::CMD_BUI, this);
        addService("wrist_dynamics_command/stop_loop", cmd::Request::CMD_STO, this);
        addService("wrist_dynamics_command/read_loop_time", cmd::Request::CMD_REA, this);
        addService("wrist_dynamics_command/set_zero", cmd::Request::CMD_ZRO, this);  
        addService("wrist_dynamics_command/estimate_calibration", cmd::Request::CMD_CAL, this); 
        addService("wrist_dynamics_command/set_calibration", cmd::Request::CMD_SET, this); 
        addService("wrist_dynamics_command/check_calibration", cmd::Request::CMD_CHK, this);   
        addService("wrist_dynamics_mode/publish", cmd::Request::MOD_PUB, this);
        addService("wrist_dynamics_mode/trigger_static", cmd::Request::MOD_TST, this);
        addService("wrist_dynamics_mode/estimate_wrench", cmd::Request::MOD_EST, this);
        addService("wrist_dynamics_mode/save_dynamics", cmd::Request::MOD_SDY, this);
        addService("wrist_dynamics_mode/save_interaction", cmd::Request::MOD_SIN, this);
        addService("wrist_dynamics_mode/trigger_dynamics", cmd::Request::MOD_TDY, this);
        addService("wrist_dynamics_mode/save_calibration", cmd::Request::MOD_SCA, this);
        addService("wrist_dynamics_macro/start_node", cmd::Request::MAC_STA, this);
        ROS_INFO("Node ready to take command.");
        if(autostart)
        {
         cmd c;
         command(c.request,c.response,cmd::Request::MAC_STA);
        }
      }

    private:

      void externalTimerCallback(const ros::WallTimerEvent& event)
      {	
        WristFTContactDetection::TriggerStatic();
        ROS_INFO("%s",det_str.str().c_str());
        det_tim_.stop();
      }

      void loopTimerCallback(const ros::WallTimerEvent& event)
      {
        start_ = ros::WallTime::now();

        WristFTProprioception::Get();
        switch(mode_)
        {
          case cmd::Request::MOD_PUB:
            // no action needed here
            break;
          case cmd::Request::MOD_TST:
            if(WristFTContactDetection::PreTriggerStatic() & !det_tim_.hasStarted())
            {
              trigger = false;//to do: find proper way to reset this trigger
              det_tim_.start();
            }
            WristFTContactDetection::BackUpTriggerStatic();
            break;
          case cmd::Request::MOD_EST:
            WristFTProprioception::Estimate(); 
            break;
          case cmd::Request::MOD_SDY:
            FrameDynamics::AddEquation();
            break;
          case cmd::Request::MOD_SIN:
            WristFTProprioception::SaveInteraction();
            break;
          case cmd::Request::MOD_TDY:
            WristFTProprioception::TriggerDynamics(factor_);
            break;
          case cmd::Request::MOD_SCA:
            WristFTCalibration::AddEquation(force_dyn,torque_dyn,gravity,twist_acceleration.linear); //TO DO: if FrameDynamics object can be inherited as virtual, no need to pass arguments here
            break;
        }

        if(publish_)
        {
          loop_msg_.header.seq = (int)cycle_count_;
          loop_msg_.header.stamp = ros::Time::now();
          loop_msg_.wrist_dynamics.frame_kinematics.position = position; 
          loop_msg_.wrist_dynamics.frame_kinematics.velocity = velocity; 
          loop_msg_.wrist_dynamics.frame_kinematics.acceleration = acceleration;
          loop_msg_.wrist_dynamics.frame_kinematics.twist_velocity = twist_velocity; 
          loop_msg_.wrist_dynamics.frame_kinematics.twist_acceleration = twist_acceleration;
          loop_msg_.wrist_dynamics.wrench_dyn.force = force_dyn;
          loop_msg_.wrist_dynamics.wrench_dyn.torque = torque_dyn;
          loop_msg_.wrist_dynamics.wrench_lp.force = force_lp;
          loop_msg_.wrist_dynamics.wrench_lp.torque = torque_lp;
          loop_msg_.wrist_dynamics.wrench_raw.force = force_raw;
          loop_msg_.wrist_dynamics.wrench_raw.torque = torque_raw;
          loop_msg_.wrist_dynamics.wrench_hat.force = force_hat;
          loop_msg_.wrist_dynamics.wrench_hat.torque = torque_hat;
          loop_msg_.wrist_dynamics.wrench_pr.force = force_pr;
          loop_msg_.wrist_dynamics.wrench_pr.torque = torque_pr;
          loop_msg_.wrist_dynamics.wrench_th = th_dyn_; // only for debug
          loop_msg_.wrist_dynamics.gravity = gravity;
          loop_msg_.wrist_dynamics.phi = phi_str.str();
          loop_msg_.wrist_dynamics.cal = cal_str.str();
          loop_msg_.wrist_dynamics.det = det_str.str();
          loop_pub_.publish(loop_msg_);
        }
        
        det_msg_.header = loop_msg_.header;
        det_msg_.detection.pretrig = pretrig;
        det_msg_.detection.trigger = trigger;
        det_msg_.detection.backtrig = backtrig;
        det_pub_.publish(det_msg_);
        cycle_count_ += 1.0;
        cycle_time_ += (ros::WallTime::now() - start_).toSec();
      }

      bool command(cmd::Request& request, cmd::Response& response, int command)
      {
        response.success = 1;
        if(command < 10)
        {
          switch(command)
          {
          case cmd::Request::CMD_SUB:
            ROS_INFO("Subscribing to sensor and joint state topic (%s,%s).",sensor_,controller_);
            ft_sub_ = nh_.subscribe(sensor_, 1000, &WristDynamicsNode::ftDataCallback, this);
            ft_ = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(sensor_);
            j_sub_ = nh_.subscribe(controller_, 1000, &WristDynamicsNode::jDataCallback, this);	
            js_ = ros::topic::waitForMessage<sensor_msgs::JointState>(controller_);
            response.success = WristFTProprioception::Init(*js_,ft_->wrench);
            break; 
          case cmd::Request::CMD_STA:
            ROS_INFO("Starting node loop (computation and publish).");
            cycle_count_ = .0;
            cycle_time_ = .0;
            loop_tim_.start();
            break;
          case cmd::Request::CMD_BUI:
            ROS_INFO("Starting estimate of inertial model.");
            FrameDynamics::Solve();
            ROS_INFO("%s",phi_str.str().c_str());
            break;
          case cmd::Request::CMD_STO:
            ROS_INFO("Pausing subscribers and node loop (going to idle).");
            loop_tim_.stop();
            ft_sub_.shutdown();
            j_sub_.shutdown();
            break;
          case cmd::Request::CMD_REA:
            ROS_INFO("Mean loop time %.6f sec.", cycle_time_/cycle_count_);
            break;
          case cmd::Request::CMD_ZRO:
            ROS_INFO("Setting zero on force/torque sensor IIR filter.");
            WristFTContactDetection::SetZero(true);
            break;
          case cmd::Request::CMD_CAL:
            ROS_INFO("Starting estimate of calibration parameters.");
            WristFTCalibration::Solve();
            ROS_INFO("%s",cal_str.str().c_str());
            break;
          case cmd::Request::CMD_SET:
            ROS_INFO("Setting offset on force/torque sensor.");
            FrameDynamics::SetOffset(WristFTCalibration::Get());
            WristFTContactDetection::SetOffset(WristFTCalibration::Get());
            break;
          case cmd::Request::CMD_CHK:
            ROS_INFO("Checking force/torque sensor calibration (unzeroing sensor).");
            WristFTContactDetection::SetZero(false);
            WristFTContactDetection::SetOffset(WristFTCalibration::Get());
            c_mass_ = 100*(atk::magnitudeVector3(force_dyn)/(WristFTCalibration::GetMass()*9.81) - 1);
            ROS_INFO("Change on mass estimate: %.1f %%",c_mass_);
            (abs(c_mass_) > 6 ? response.success = 0 : response.success = 1); //TO DO: very simple check, more robust approach shuild be considered
            break;            
          }   
          ROS_INFO("Executed command.");
          response.message = "Executed command.";
        }
        else if(command >= 10 & command < 20)
        {
          mode_ = command;
          std::string msg = "Mode changed to ";
          switch(mode_)
          {
          case cmd::Request::MOD_PUB:
            msg += "compute and publish wrist state.";
            break;
          case cmd::Request::MOD_TST:
            msg += "enable trigger for static passer.";
            break;
          case cmd::Request::MOD_EST:
            msg += "produce force/torque estimate.";
            break;
          case cmd::Request::MOD_SDY:
            msg += "save dynamics equations for further estimate of inertial model.";
            break;
          case cmd::Request::MOD_SIN:
            msg += "save maximum of interaction (external) forces for further thresholding.";
            break;
          case cmd::Request::MOD_TDY:
            msg += "enable trigger on interaction (external) forces.";
            break;
          case cmd::Request::MOD_SCA:
            msg += "add force/torque sensor calibration points.";
            break;
          }
          ROS_INFO("%s",msg.c_str());
          response.message = "Mode changed.";
          response.success = 1;
        }
        else
        {
          std::string msg = "Executing macro to ";
          switch(command)
          {
          case cmd::Request::MAC_STA:
            msg += "start node.";
            ROS_INFO("%s",msg.c_str());
            this->command(request,response,cmd::Request::CMD_SUB);
            this->command(request,response,cmd::Request::CMD_STA);
            break;
          }
          ROS_INFO("End of macro.");
          response.message = "End of macro.";
        }
        return true;
      }

      //Store latest force/torque message and update WristFTContactDetection accordingly (filtered and raw data)
      void ftDataCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg){ft_ = msg; WristFTContactDetection::Update(ft_->wrench);}

      //Update FrameDynamics with the latest joint state and force/torque messages (ATTENTION to the sensor rate, it must be higher then publish rate of joint_state_controller)
      void jDataCallback(const sensor_msgs::JointState::ConstPtr& msg){FrameDynamics::Update(*msg,ft_->wrench);}
      
      bool publish_;
      int mode_ = 10;
      double cycle_time_ = 0.0, cycle_count_ = 0.0, factor_, c_mass_;
      const char *sensor_, *controller_;
      artificial_hands_msgs::DetectionStamped det_msg_;
      artificial_hands_msgs::WristDynamicsStamped loop_msg_;
      ros::WallTimer loop_tim_, det_tim_;
      ros::Publisher loop_pub_, det_pub_;
      ros::WallTime start_;
      ros::NodeHandle nh_;
      ros::Subscriber j_sub_, ft_sub_;
      sensor_msgs::JointStateConstPtr js_;
      geometry_msgs::WrenchStampedConstPtr ft_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wrist_node");

  ros::NodeHandle nh;
  
  bool autostart;
  bool publish;
  double factor;
  int filter;
  int rate;
  std::string sensor;
  std::string controller;
  std::string controller_rate;
  std::string target_frame;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "display help")
    ("autostart", po::value<bool>(&autostart)->default_value(1), "set true to autostart the node")
    ("publish", po::value<bool>(&publish)->default_value(1), "set true to advertise a WristStamped publisher")
    ("factor", po::value<double>(&factor)->default_value(2.0), "robustness of interaction trigger for dynamic handover (g.t. 1)")
    ("filter", po::value<int>(&filter)->default_value(20), "length of moving average filters (in samples)")
    ("rate", po::value<int>(&rate)->default_value(100), "rosnode internal rate (in Hertz)")
    ("sensor", po::value<std::string>(&sensor)->default_value("/ft_sensor"), "name of the rostopic published by F/T sensor") 
    ("controller", po::value<std::string>(&controller)->default_value("/joint_states"), "name of the rostopic published by joint_state_controller") 
    ("controller_rate", po::value<std::string>(&controller_rate)->default_value("/joint_state_controller/publish_rate"), "rosparam name of the publish rate in joint_state_controller") 
    ("target_frame", po::value<std::string>(&target_frame)->default_value("hex_sensor_frame"), "wrist F/T sensor frame id") 
    ;
  po::positional_options_description p;
  p.add("filter",  1);
  p.add("node_rate",  1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);
  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    exit(EXIT_SUCCESS);
  }      

  int joint_rate;
  ros::param::get(controller_rate,joint_rate);
  if(joint_rate < rate){rate = joint_rate; ROS_WARN("Required node rate was g.t. joint_state_controller publish rate, node rate lowered to %i Hz",rate);}
  rosatk::WristDynamicsNode hr_per_node = rosatk::WristDynamicsNode(nh,autostart,publish,factor,filter,rate,sensor.c_str(),controller.c_str(),joint_rate,target_frame.c_str());
  ros::spin();

  return true;
}
