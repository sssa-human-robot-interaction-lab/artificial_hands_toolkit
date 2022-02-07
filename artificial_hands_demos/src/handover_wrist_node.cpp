
#include <boost/program_options.hpp>

#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointControllerState.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <artificial_hands_msgs/WristStamped.h>
#include <artificial_hands_msgs/DetectionStamped.h>
#include <artificial_hands_msgs/WristCommand.h>
#include <artificial_hands_wrist/Interaction.hpp>
#include <artificial_hands_wrist/Calibration.hpp>

namespace po = boost::program_options;

namespace rosatk
{

  using cmd = artificial_hands_msgs::WristCommand;
  
  class HandoverWristNode: public ros::NodeHandle, public atk::Interaction, virtual public atk::Calibration
  {
    
    public:
      HandoverWristNode(bool publish, double factor, int filter, int rate, const char* sensor, const char* controller, int controller_rate, const char* target_frame):  
        publish_(publish),
        factor_(factor),
        sensor_(sensor),
        controller_(controller),
        Interaction(filter, controller_rate, target_frame)
      {
        ROS_INFO("Starting FIR filters with length %i samples.",filter); // TO DO cahnge ROS_INFO to ROS_DEBUG where needed
        ROS_INFO("Starting node with loop rate %i Hertz.",rate);
        ROS_INFO("Starting dynamics relative to frame %s.",target_frame);
        if(publish)wr_pub_ = advertise<artificial_hands_msgs::WristStamped>("wrist_data",1000);
        det_pub_ = advertise<artificial_hands_msgs::DetectionStamped>("wrist_detection",1000);
        wr_tim_ = createWallTimer(ros::WallDuration(1.0/(double)rate),&HandoverWristNode::loopTimerCallback, this, false, false);
        det_tim_ = createWallTimer(ros::WallDuration(0.1),&HandoverWristNode::detectionTimerCallback, this, false, false);
        ROS_INFO("Starting node services.");
        cmd_0_ = advertiseService("wrist_command/subscribe", &HandoverWristNode::subscribeCommand, this);
        cmd_1_ = advertiseService("wrist_command/start_loop", &HandoverWristNode::startLoopCommand, this);
        cmd_2_ = advertiseService("wrist_command/build_model", &HandoverWristNode::buildModelCommand, this);
        cmd_3_ = advertiseService("wrist_command/stop_loop", &HandoverWristNode::stopLoopCommand, this);
        cmd_4_ = advertiseService("wrist_command/read_loop_time", &HandoverWristNode::readLoopTimeCommand, this);
        cmd_5_ = advertiseService("wrist_command/set_zero", &HandoverWristNode::setZeroCommand, this);  
        cmd_6_ = advertiseService("wrist_command/set_calibration", &HandoverWristNode::calibrateCommand, this); 
        cmd_7_ = advertiseService("wrist_command/check_calibration", &HandoverWristNode::checkCalibrationCommand, this);   
        
        mod_10_ = advertiseService("wrist_mode/publish", &HandoverWristNode::publishDynamicsMode, this);
        mod_11_ = advertiseService("wrist_mode/trigger_static", &HandoverWristNode::triggerStaticMode, this);
        mod_12_ = advertiseService("wrist_mode/estimate_wrench", &HandoverWristNode::estimateWrenchMode, this);
        mod_13_ = advertiseService("wrist_mode/save_dynamics", &HandoverWristNode::saveDynamicsMode, this);
        mod_14_ = advertiseService("wrist_mode/save_interaction", &HandoverWristNode::saveInteractionMode, this);
        mod_15_ = advertiseService("wrist_mode/trigger_dynamics", &HandoverWristNode::triggerDynamicsMode, this);
        mod_16_ = advertiseService("wrist_mode/save_calibration", &HandoverWristNode::saveCalibrationMode, this);
        ROS_INFO("Node ready to take command.");
      }

    private:

      bool subscribeCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_SUB,response);}
      bool startLoopCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_STA,response);}
      bool buildModelCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_BUI,response);}
      bool stopLoopCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_STO,response);}
      bool readLoopTimeCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_REA,response);}
      bool setZeroCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_ZRO,response);}
      bool calibrateCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_CAL,response);}
      bool checkCalibrationCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_CHK,response);}

      bool publishDynamicsMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_PUB,response);}
      bool triggerStaticMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_TST,response);} 
      bool estimateWrenchMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_EST,response);}
      bool saveDynamicsMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_SDY,response);}
      bool saveInteractionMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_SIN,response);}
      bool triggerDynamicsMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_TDY,response);}
      bool saveCalibrationMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_SCA,response);}

      void detectionTimerCallback(const ros::WallTimerEvent& event)
      {	
        Detection::TriggerStatic();
        ROS_INFO("%s",det_str.str().c_str());
        det_tim_.stop();
      }

      void loopTimerCallback(const ros::WallTimerEvent& event)
      {
        start_ = ros::WallTime::now();
        det_msg_.header.seq = (int)cycle_count_;
        det_msg_.header.stamp.fromNSec(start_.toNSec());

        Interaction::Get();
        switch(mode_)
        {
        case 11:
          if(Detection::PreTriggerStatic() & !det_tim_.hasStarted())
          {
            trigger = false;//to do: find proper way to reset this trigger
            det_tim_.start();
          }
          Detection::BackUpTriggerStatic();
          break;
        case 12:
          Interaction::Estimate(); 
          break;
        case 13:
          Dynamics::AddEquation();
          break;
        case 14:
          Interaction::SaveInteraction();
          break;
        case 15:
          Interaction::TriggerDynamics(factor_);
          break;
        case 16:
          Calibration::AddEquation(force_fir,torque_fir,gravity); //TO DO: if Dynamics object can be inherited as virtual, no need to pass arguments here
          break;
        }

        if(publish_)
        {
          wr_msg_.header.seq = (int)cycle_count_;
          wr_msg_.header.stamp.fromNSec(start_.toNSec());
          wr_msg_.position = position; 
          wr_msg_.velocity = velocity; 
          wr_msg_.acceleration = acceleration;
          wr_msg_.wrench_fir.force = force_fir;
          wr_msg_.wrench_fir.torque = torque_fir;
          wr_msg_.wrench_iir.force = force_iir;
          wr_msg_.wrench_iir.torque = torque_iir;
          wr_msg_.wrench_raw.force = force_raw;
          wr_msg_.wrench_raw.torque = torque_raw;
          wr_msg_.wrench_hat.force = force_hat;
          wr_msg_.wrench_hat.torque = torque_hat;
          wr_msg_.wrench_e.force = force_e;
          wr_msg_.wrench_e.torque = torque_e;
          wr_msg_.wrench_th = th_dyn_; // only for debug
          wr_msg_.gravity = gravity;
          wr_msg_.phi = phi_str.str();
          wr_msg_.cal = cal_str.str();
          wr_msg_.det = det_str.str();
          wr_pub_.publish(wr_msg_);
        }
        
        det_msg_.pretrig = pretrig;
        det_msg_.trigger = trigger;
        det_msg_.backtrig = backtrig;
        det_pub_.publish(det_msg_);
        cycle_count_ += 1.0;
        cycle_time_ += (ros::WallTime::now() - start_).toSec();
      }

      bool command(int command, cmd::Response& response)
      {
        response.success = 1;
        if(command < 10)
        {
          switch(command)
          {
          case 0:
            ROS_INFO("Subscribing to sensor and joint state topic (%s,%s).",sensor_,controller_);
            ft_sub_ = subscribe(sensor_, 1000, &HandoverWristNode::ftDataCallback, this);
            ft_ = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(sensor_);
            j_sub_ = subscribe(controller_, 1000, &HandoverWristNode::jDataCallback, this);	
            js_ = ros::topic::waitForMessage<sensor_msgs::JointState>(controller_);
            response.success = Interaction::Init(*js_,ft_->wrench);
            Dynamics::SetOffset(Calibration::Get());
            Detection::SetOffset(Calibration::Get());
            break; 
          case 1:
            ROS_INFO("Starting node loop (computation and publish).");
            cycle_count_ = .0;
            cycle_time_ = .0;
            wr_tim_.start();
            break;
          case 2:
            ROS_INFO("Starting estimate of inertial model.");
            Dynamics::Solve();
            ROS_INFO("%s",phi_str.str().c_str());
            break;
          case 3:
            ROS_INFO("Pausing subscribers and node loop (going to idle).");
            wr_tim_.stop();
            ft_sub_.shutdown();
            j_sub_.shutdown();
            break;
          case 4:
            ROS_INFO("Mean loop time %.6f sec.", cycle_time_/cycle_count_);
            break;
          case 5:
            ROS_INFO("Setting zero on force/torque sensor IIR filter.");
            Detection::SetZero(true);
            break;
          case 6:
            ROS_INFO("Starting estimate of calibration parameters.");
            Calibration::Solve();
            ROS_INFO("%s",cal_str.str().c_str());
            ROS_INFO("Setting offset on force/torque sensor.");
            Dynamics::SetOffset(Calibration::Get());
            Detection::SetOffset(Calibration::Get());
            break;
          case 7:
            ROS_INFO("Checking force/torque sensor calibration.");
            double m = 100*(atk::magnitudeVector3(force_iir)/(Calibration::GetMass()*9.81) - 1);
            ROS_INFO("Change on mass estimate: %.1f %%",m);
            (abs(m) > 10 ? response.success = 0 : response.success = 1); //TO DO: very simple check, more robust approach shuild be considered
            break;
          }   
          ROS_INFO("Executed command.");
          response.message = "Executed command.";
        }
        else
        {
          mode_ = command;
          std::string msg = "Mode changed to ";
          switch(mode_)
          {
          case 10:
            msg += "compute and publish wrist state.";
            break;
          case 11:
            msg += "enable trigger for static passer.";
            break;
          case 12:
            msg += "produce force/torque estimate.";
            break;
          case 13:
            msg += "save dynamics equations for further estimate of inertial model.";
            break;
          case 14:
            msg += "save maximum of interaction (external) forces for further thresholding.";
            break;
          case 15:
            msg += "enable trigger on interaction (external) forces.";
            break;
          case 16:
            msg += "add force/torque sensor calibration points.";
            break;
          }
          ROS_INFO("%s",msg.c_str());
          response.message = "Mode changed.";
          response.success = 1;
        }
          return true;
      }

      //Store latest force/torque message and update Detection accordingly (filtered and raw data)
      void ftDataCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg){ft_ = msg; Detection::Update(ft_->wrench);}

      //Update Dynamics with the latest joint state and force/torque messages (ATTENTION to the sensor rate, it must be higher then publish rate of joint_state_controller)
      void jDataCallback(const sensor_msgs::JointState::ConstPtr& msg){Dynamics::Update(*msg,ft_->wrench);}
      
      bool publish_;
      int mode_ = 10;
      double cycle_time_ = 0.0, cycle_count_ = 0.0, factor_;
      const char *sensor_, *controller_;
      ros::WallTimer wr_tim_, det_tim_;
      ros::Publisher wr_pub_, det_pub_;
      ros::Subscriber j_sub_, ft_sub_;
      ros::WallTime start_;
      artificial_hands_msgs::WristStamped wr_msg_;
      artificial_hands_msgs::DetectionStamped det_msg_;
      geometry_msgs::WrenchStampedConstPtr ft_;
      sensor_msgs::JointStateConstPtr js_;
      ros::ServiceServer cmd_0_, cmd_1_, cmd_2_, cmd_3_, cmd_4_, cmd_5_, cmd_6_, cmd_7_;
      ros::ServiceServer mod_10_, mod_11_, mod_12_, mod_13_, mod_14_, mod_15_, mod_16_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "handover_wrist_node");
  
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
    ("publish", po::value<bool>(&publish)->default_value(1), "set true for complete publishing of wrist state")
    ("factor", po::value<double>(&factor)->default_value(2.0), "robustness of interaction trigger for dynamic handover (g.t. 1)")
    ("filter", po::value<int>(&filter)->default_value(20), "length of moving average filters (in samples)")
    ("rate", po::value<int>(&rate)->default_value(100), "rosnode internal rate (in Hertz)")
    ("sensor", po::value<std::string>(&sensor)->default_value("/ft_sensor"), "name of the rostopic published by F/T sensor") 
    ("controller", po::value<std::string>(&controller)->default_value("/joint_states"), "name of the rostopic published by joint_state_controller") 
    ("controller_rate", po::value<std::string>(&controller_rate)->default_value("/joint_state_controller/publish_rate"), "rosparam name of the publish rate in joint_state_controller") 
    ("target_frame", po::value<std::string>(&target_frame)->default_value("ft_sensor_frame"), "wrist F/T sensor frame id") 
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
  rosatk::HandoverWristNode hr_per_node = rosatk::HandoverWristNode(publish,factor,filter,rate,sensor.c_str(),controller.c_str(),joint_rate,target_frame.c_str());
  ros::spin();

  return true;
}
