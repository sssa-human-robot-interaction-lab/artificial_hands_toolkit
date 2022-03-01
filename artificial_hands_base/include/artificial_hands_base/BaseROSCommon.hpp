#ifndef ATK_BASE_ROS_COMMON
#define ATK_BASE_ROS_COMMON

#include <ros/ros.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace rosatk
{
  template <class T, class C>
  class ServiceManagerBase
  {
    typedef typename C::Request req_t;
    typedef typename C::Response res_t;

    public:
      ServiceManagerBase(ros::NodeHandle nh, bool (T::*func)(req_t&,res_t&,int)):nh_(nh),func_(func){}

      void addService(const char* name, int id, T *func_obj)
      {
        ser_.push_back(nh_.advertiseService<req_t,res_t>(name,boost::bind(func_, func_obj, _1, _2, id)));
      }
      
    private:
      ros::NodeHandle nh_;
      std::vector<ros::ServiceServer> ser_;
      bool (T::*func_)(req_t&,res_t&,int);
  };
}

#endif