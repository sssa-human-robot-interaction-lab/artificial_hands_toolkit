#include <artificial_hands_base/BaseROSCommon.hpp>
#include <artificial_hands_base/FrameKinematics.hpp>
#include <artificial_hands_msgs/FrameKinematicsStamped.h>
#include <artificial_hands_msgs/FrameKinematicsCommand.h>
#include <artificial_hands_msgs/ProcessVarianceConfig.h>

#include <dynamic_reconfigure/server.h>

#include <control_msgs/JointControllerState.h>

namespace rosatk
{
  using cmd = artificial_hands_msgs::FrameKinematicsCommand;
  
  class FrameKinematicsNode: public atk::FrameKinematics, public rosatk::ServiceManagerBase<rosatk::FrameKinematicsNode,cmd>, public rosatk::FilterManagerBase
  {
    
    public:
      FrameKinematicsNode(ros::NodeHandle nh, bool autostart, int filter_length, int rate, const char* controller, int controller_rate, const char* target_frame):  
        nh_(nh),
        publish_(true),
        controller_(controller),
        FrameKinematics(controller_rate, target_frame, "manipulator", "robot_description"),
        ServiceManagerBase(nh,&FrameKinematicsNode::command),
        FilterManagerBase(nh,"/frame_kinematics",filter_length)
      {
        dyn_f_ = boost::bind(&FrameKinematicsNode::reconfigure, this, _1, _2);
        dyn_ser_.reset(new dynamic_reconfigure::Server<artificial_hands_msgs::ProcessVarianceConfig>(ros::NodeHandle("frame_kinematics_reconfigure")));
        dyn_ser_->setCallback(dyn_f_);

        FrameKinematics::SetFilter(filter);
        ROS_INFO("Starting node with loop rate %i Hertz.",rate);
        ROS_INFO("Starting kinematics relative to frame %s.",target_frame);
        if(publish_)loop_pub_ = nh_.advertise<artificial_hands_msgs::FrameKinematicsStamped>("frame_kinematics_data",1000);
        loop_tim_ = nh_.createWallTimer(ros::WallDuration(1.0/(double)rate),&FrameKinematicsNode::loopTimerCallback, this, false, false);
        loop_msg_.header.frame_id = target_frame;

        ROS_INFO("Starting node services.");
        addService("frame_kinematics_command/subscribe",cmd::Request::CMD_SUB,this);
        addService("frame_kinematics_command/start_loop",cmd::Request::CMD_STA,this);
        addService("frame_kinematics_command/stop_loop",cmd::Request::CMD_STO,this);
        addService("frame_kinematics_command/read_loop_time",cmd::Request::CMD_REA,this);
        addService("frame_kinematics_mode/publish",cmd::Request::MOD_PUB,this);
        addService("frame_kinematics_macro/start_node",cmd::Request::MAC_STA,this);
        ROS_INFO("Node ready to take command.");

        if(autostart)
        {
         cmd c;
         command(c.request,c.response,cmd::Request::MAC_STA);
        }
      }

    private:

      void reconfigure(artificial_hands_msgs::ProcessVarianceConfig &config, uint32_t level) {
        if(js_ != NULL)
        {
          FrameKinematics::Init(*js_,config.process_variance);
        }
      }

      bool command(cmd::Request& request, cmd::Response& response, int command)
      {
        response.success = 1;
        if(command < 10)
        {
          switch(command)
          {
          case cmd::Request::CMD_SUB:
            ROS_INFO("Subscribing to joint state topic (%s).",controller_);
            js_ = ros::topic::waitForMessage<sensor_msgs::JointState>(controller_);
            response.success = FrameKinematics::Init(*js_);
            j_sub_ = nh_.subscribe(controller_, 1000, &FrameKinematicsNode::jDataCallback, this);	
            break; 
          case cmd::Request::CMD_STA:
            ROS_INFO("Starting node loop (computation and publish).");
            cycle_count_ = .0;
            cycle_time_ = .0;
            loop_tim_.start();
            break;
          case cmd::Request::CMD_STO:
            ROS_INFO("Pausing subscribers and node loop (going to idle).");
            loop_tim_.stop();
            j_sub_.shutdown();
            break;
          case cmd::Request::CMD_REA:
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
          case cmd::Request::MOD_PUB:
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

      void loopTimerCallback(const ros::WallTimerEvent& event)
      {
        wall_start_ = ros::WallTime::now();

        FrameKinematics::Get();
        switch(mode_)
        {
          case cmd::Request::MOD_PUB:
            // no action needed here
            break;
        }

        if(publish_)
        {
          loop_msg_.header.seq = (int)cycle_count_;
          loop_msg_.header.stamp = ros::Time::now();   
          loop_msg_.frame_kinematics.position = position; 
          loop_msg_.frame_kinematics.velocity = velocity; 
          loop_msg_.frame_kinematics.acceleration = acceleration;
          loop_msg_.frame_kinematics.twist_velocity = twist_velocity; 
          loop_msg_.frame_kinematics.twist_acceleration = twist_acceleration;
          loop_pub_.publish(loop_msg_);
        }
        
        cycle_count_ += 1.0;
        cycle_time_ += (ros::WallTime::now() - wall_start_).toSec();
      }      

      void jDataCallback(const sensor_msgs::JointState::ConstPtr& msg){FrameKinematics::Update(*msg);}
      
      bool publish_;
      int mode_ = 10;
      double cycle_time_ = 0.0, cycle_count_ = 0.0;
      const char *controller_;
      artificial_hands_msgs::FrameKinematicsStamped loop_msg_;
      ros::WallTimer loop_tim_;
      ros::Publisher loop_pub_;
      ros::WallTime wall_start_;
      ros::NodeHandle nh_;
      ros::Subscriber j_sub_;
      sensor_msgs::JointStateConstPtr js_;
      boost::shared_ptr<dynamic_reconfigure::Server<artificial_hands_msgs::ProcessVarianceConfig>> dyn_ser_;
      dynamic_reconfigure::Server<artificial_hands_msgs::ProcessVarianceConfig>::CallbackType dyn_f_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_kinematics_node");

  ros::NodeHandle nh;

  bool autostart;
  int filter;
  int rate;
  std::string controller;
  std::string controller_rate;
  std::string target_frame;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "display help")
    ("autostart", po::value<bool>(&autostart)->default_value(1), "set true to autostart the node")
    ("filter", po::value<int>(&filter)->default_value(20), "length of moving average filters (in samples)")
    ("rate", po::value<int>(&rate)->default_value(100), "rosnode internal rate (in Hertz)")
    ("controller", po::value<std::string>(&controller)->default_value("/joint_states"), "name of the rostopic published by joint_state_controller") 
    ("controller_rate", po::value<std::string>(&controller_rate)->default_value("/joint_state_controller/publish_rate"), "rosparam name of the publish rate in joint_state_controller") 
    ("target_frame", po::value<std::string>(&target_frame)->default_value("tool0"), "target frame id") 
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
  rosatk::FrameKinematicsNode hr_per_node = rosatk::FrameKinematicsNode(nh,autostart,filter,rate,controller.c_str(),joint_rate,target_frame.c_str());
  ros::spin();

  return true;
}
