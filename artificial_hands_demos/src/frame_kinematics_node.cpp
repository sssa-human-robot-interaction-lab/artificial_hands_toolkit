
#include <boost/program_options.hpp>

#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointControllerState.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <artificial_hands_msgs/FrameKinematicsStamped.h>
#include <artificial_hands_msgs/FrameKinematicsCommand.h>
#include <artificial_hands_base/Kinematics.hpp>

namespace po = boost::program_options;

namespace rosatk
{

  using cmd = artificial_hands_msgs::FrameKinematicsCommand;
  
  class FrameKinematicsNode: public ros::NodeHandle, atk::Kinematics
  {
    
    public:
      FrameKinematicsNode(bool publish, int filter, int rate, const char* controller, int controller_rate, const char* target_frame, const char* model_frame):  
        publish_(publish),
        controller_(controller),
        Kinematics(filter, controller_rate, target_frame, "manipulator", "robot_description", model_frame)
      {
        ROS_INFO("Starting FIR filters with length %i samples.",filter); // TO DO cahnge ROS_INFO to ROS_DEBUG where needed
        ROS_INFO("Starting node with loop rate %i Hertz.",rate);
        ROS_INFO("Starting dynamics relative to frame %s.",target_frame);
        ROS_INFO("Using reference frame %s.",model_frame);
        if(publish)loop_pub_ = advertise<artificial_hands_msgs::FrameKinematicsStamped>("frame_kinematics_data",1000);
        loop_tim_ = createWallTimer(ros::WallDuration(1.0/(double)rate),&FrameKinematicsNode::loopTimerCallback, this, false, false);
        loop_msg_.header.frame_id = target_frame;
        ROS_INFO("Starting node services.");
        cmd_0_ = advertiseService("frame_kinematics_command/subscribe", &FrameKinematicsNode::subscribeCommand, this);
        cmd_1_ = advertiseService("frame_kinematics_command/start_loop", &FrameKinematicsNode::startLoopCommand, this);
        cmd_2_ = advertiseService("frame_kinematics_command/stop_loop", &FrameKinematicsNode::stopLoopCommand, this);
        cmd_3_ = advertiseService("frame_kinematics_command/read_loop_time", &FrameKinematicsNode::readLoopTimeCommand, this); 
        mod_10_ = advertiseService("frame_kinematics_mode/publish", &FrameKinematicsNode::publisherMode, this);
        mac_20_ = advertiseService("frame_kinematics_macro/start_node", &FrameKinematicsNode::startNodeMacro, this);
        ROS_INFO("Node ready to take command.");
      }

    private:

      bool subscribeCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_SUB,response);}
      bool startLoopCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_STA,response);}
      bool stopLoopCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_STO,response);}
      bool readLoopTimeCommand(cmd::Request& request, cmd::Response& response){return command(cmd::Request::CMD_REA,response);}
      bool publisherMode(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MOD_PUB,response);}
      bool startNodeMacro(cmd::Request& request, cmd::Response& response){return command(cmd::Request::MAC_STA,response);}

      void loopTimerCallback(const ros::WallTimerEvent& event)
      {
        start_ = ros::WallTime::now();

        Kinematics::Get();

        if(publish_)
        {
          loop_msg_.header.seq = (int)cycle_count_;
          loop_msg_.header.stamp.fromNSec(start_.toNSec());      
          loop_msg_.frame_kinematics.position = position; 
          loop_msg_.frame_kinematics.velocity = velocity; 
          loop_msg_.frame_kinematics.acceleration = acceleration;
          loop_msg_.frame_kinematics.twist_velocity = twist_velocity; 
          loop_msg_.frame_kinematics.twist_acceleration = twist_acceleration;
          loop_pub_.publish(loop_msg_);
        }
        
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
            ROS_INFO("Subscribing to joint state topic (%s).",controller_);
            j_sub_ = subscribe(controller_, 1000, &FrameKinematicsNode::jDataCallback, this);	
            js_ = ros::topic::waitForMessage<sensor_msgs::JointState>(controller_);
            response.success = Kinematics::Init(*js_);
            break; 
          case 1:
            ROS_INFO("Starting node loop (computation and publish).");
            cycle_count_ = .0;
            cycle_time_ = .0;
            loop_tim_.start();
            break;
          case 2:
            ROS_INFO("Pausing subscribers and node loop (going to idle).");
            loop_tim_.stop();
            j_sub_.shutdown();
            break;
          case 3:
            ROS_INFO("Mean loop time %.6f sec.", cycle_time_/cycle_count_);
            break;
          }   
          ROS_INFO("Executed command.");
          response.message = "Executed command.";
        }
        else if(command <= 10 & command < 20)
        {
          mode_ = command;
          std::string msg = "Mode changed to ";
          switch(command)
          {
          case 10:
            msg += "compute and publish wrist state.";
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
          case 20:
            msg += "start node.";
            ROS_INFO("%s",msg.c_str());
            this->command(cmd::Request::CMD_SUB,response);
            this->command(cmd::Request::CMD_STA,response);
            break;
          }
          ROS_INFO("End of macro.");
          response.message = "End of macro.";
        }
          return true;
      }

      //Update Kinematics with the latest joint state
      void jDataCallback(const sensor_msgs::JointState::ConstPtr& msg){Kinematics::Update(*msg);}
      
      bool publish_;
      int mode_ = 10;
      double cycle_time_ = 0.0, cycle_count_ = 0.0;
      const char *controller_;
      ros::WallTimer loop_tim_;
      ros::Publisher loop_pub_;
      ros::Subscriber j_sub_;
      ros::WallTime start_;
      artificial_hands_msgs::FrameKinematicsStamped loop_msg_;
      sensor_msgs::JointStateConstPtr js_;
      ros::ServiceServer cmd_0_, cmd_1_, cmd_2_, cmd_3_;
      ros::ServiceServer mod_10_;
      ros::ServiceServer mac_20_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_kinematics_node");
  
  bool publish;
  int filter;
  int rate;
  std::string controller;
  std::string controller_rate;
  std::string target_frame;
  std::string model_frame;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "display help")
    ("publish", po::value<bool>(&publish)->default_value(1), "set true to advertise a FrameKinematicsStamped publisher")
    ("filter", po::value<int>(&filter)->default_value(20), "length of moving average filters (in samples)")
    ("rate", po::value<int>(&rate)->default_value(100), "rosnode internal rate (in Hertz)")
    ("controller", po::value<std::string>(&controller)->default_value("/joint_states"), "name of the rostopic published by joint_state_controller") 
    ("controller_rate", po::value<std::string>(&controller_rate)->default_value("/joint_state_controller/publish_rate"), "rosparam name of the publish rate in joint_state_controller") 
    ("target_frame", po::value<std::string>(&target_frame)->default_value("ft_sensor_frame"), "wrist F/T sensor frame id") 
    ("model_frame", po::value<std::string>(&model_frame)->default_value("world"), "fixed reference frame of the robot model") 
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
  rosatk::FrameKinematicsNode hr_per_node = rosatk::FrameKinematicsNode(publish,filter,rate,controller.c_str(),joint_rate,target_frame.c_str(),model_frame.c_str());
  ros::spin();

  return true;
}
