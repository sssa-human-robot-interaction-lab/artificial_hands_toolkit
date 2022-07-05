#ifndef ATK_BASE_ROS_COMMON
#define ATK_BASE_ROS_COMMON

#include <ros/ros.h>
#include <boost/program_options.hpp>

#include <artificial_hands_base/BaseCommon.hpp>

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

  class FilterManagerBase
  {

    public:
      FilterManagerBase(ros::NodeHandle nh, std::string ns, int sma_order)
      {

      filter.order = sma_order;
      
      std::string order_p = ns + "_filter/order"; 
      std::string num_p = ns + "_filter/num"; 
      std::string den_p = ns + "_filter/den"; 

      if(nh_.hasParam(order_p))
      {
        nh_.getParam(order_p, filter.order);
      }

      XmlRpc::XmlRpcValue ir_num_val;
      if(nh_.hasParam(num_p))
      {
        nh_.getParam(num_p, ir_num_val);
        filter.num = new double[filter.order + 1];
        for(int i = 0; i <= filter.order; i++)
        {
          filter.num[i]=(double)ir_num_val[i];
        }

        XmlRpc::XmlRpcValue ir_den_val;
        if(nh_.hasParam(den_p))
        {  
          nh_.getParam(den_p, ir_den_val);
          filter.den = new double[filter.order + 1];
          for(int i = 0; i <= filter.order; i++)
          {
            filter.den[i]=(double)ir_den_val[i];
          }
          ROS_INFO("Starting IIR filters of order %i on %s",filter.order,ns.c_str());
        }
        else
        {
          filter.den = new double[filter.order + 1];
          filter.den[0] = 1.0;
          for(int i = 1; i <= filter.order; i++)filter.den[i]=.0;
          ROS_INFO("Starting FIR filters of order %i on %s",filter.order,ns.c_str());
        }
      }
      else
      {
        filter.num = new double[filter.order + 1];
        filter.den = new double[filter.order + 1];
        filter.den[0] = 1.0;
        for(int i = 1; i <= filter.order; i++)filter.den[i]=.0;
        for(int i = 0; i <= filter.order; i++)filter.num[i]=1.0/(filter.order+1);
        ROS_INFO("Starting SMA filters with length %i samples on %s",filter.order + 1, ns.c_str());
      }

      std::cout << "ORDER: " << filter.order << '\n';
      std::stringstream num_st;
      std::stringstream den_st;
      for(int i = 0; i <= filter.order; i++)
      {
        num_st << filter.num[i] << " ";
        den_st << filter.den[i] << " ";
      }
      std::cout << "NUM: " << num_st.str() << '\n';
      std::cout << "DEN: " << den_st.str() << '\n';
    }

    atk::filter_t filter;
      
  private:
    ros::NodeHandle nh_;  
  };

  class FTCalibManagerBase
  {

    public:
      FTCalibManagerBase(ros::NodeHandle nh, std::string ns)
      {
      
      std::string gain_p = ns + "_calib/gain"; 
      std::string bias_p = ns + "_calib/bias"; 
      std::string offset_p = ns + "_calib/offset"; 
      std::string raw_mass_p = ns + "_calib/raw_mass"; 

      XmlRpc::XmlRpcValue gain_val;
      XmlRpc::XmlRpcValue bias_val;
      if(nh_.hasParam(gain_p) && nh_.hasParam(bias_p))
      {
        nh_.getParam(gain_p, gain_val);
        nh_.getParam(bias_p, bias_val);

        if(gain_val.size() == 6 && bias_val.size() == 6)
        {
          for(int i = 0; i <= 6; i++)
          {
            calib.gain[i]=(double)gain_val[i];
            calib.bias[i]=(double)bias_val[i];
          }
          ROS_INFO("Found coefficients for linear correction of FT sensor");
        }
        else
        {
          ROS_WARN("Inconsistent length of gain and/or bias array for FT correction");
        }
      }
      else
      {
        ROS_INFO("Not found coefficients for linear correction of FT sensor");
      }

      if(nh_.hasParam(raw_mass_p))
      {
        nh_.getParam(raw_mass_p, calib.raw_mass);
        ROS_INFO("Found raw mass to check offsets of FT sensor");
      }
      else
      {
        ROS_INFO("Not found raw mass to check offsets of FT sensor");
      }

      XmlRpc::XmlRpcValue offset_val;
      if(nh_.hasParam(offset_p))
      {
        nh_.getParam(offset_p, offset_val);

        if(offset_val.size() == 6)
        {
          for(int i = 0; i < 6; i++)
          {
            calib.offset[i]=(double)offset_val[i];
          }
          ROS_INFO("Found offsets for FT sensor");
        }
        else
        {
          ROS_WARN("Inconsistent length of offset array for FT correction");
        }
      }
      else
      {
        ROS_INFO("Not found offsets for FT sensor");
      }

      std::cout << "RAW MASS: " << calib.raw_mass << '\n';
      std::stringstream gain_st;
      std::stringstream bias_st;
      std::stringstream offset_st;
      for(int i = 0; i < 6; i++)
      {
        gain_st << calib.gain[i] << " ";
        bias_st << calib.bias[i] << " ";
        offset_st << calib.offset[i] << " ";
      }
      std::cout << "GAIN: " << gain_st.str() << '\n';
      std::cout << "BIAS: " << bias_st.str() << '\n';
      std::cout << "OFFSET: " << offset_st.str() << '\n';
    
    }

    atk::ft_calib_t calib;
      
  private:
    ros::NodeHandle nh_;  
  };

}

#endif