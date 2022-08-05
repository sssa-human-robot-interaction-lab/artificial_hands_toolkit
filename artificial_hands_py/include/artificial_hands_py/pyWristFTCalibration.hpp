#ifndef PY_WRIST_FT_CALIBRATION
#define PY_WRIST_FT_CALIBRATION

#include <artificial_hands_dynamics/WristFTCalibration.hpp>
#include <artificial_hands_py/pyBaseConversions.hpp>

class pyWristFTCalibration: public atk::WristFTCalibration
{
  public:

    pyWristFTCalibration():
      WristFTCalibration(){}

    void AddEquation(py::list ft_list, py::list g_list)
    {
      cArrayFromPyList(ft_array_,ft_list);
      cArrayFromPyList(g_array_,g_list);
      
      f_.x = ft_array_[0];
      f_.y = ft_array_[1];
      f_.z = ft_array_[2];

      t_.x = ft_array_[3];
      t_.y = ft_array_[4];
      t_.z = ft_array_[5];

      g_.x = g_array_[0];
      g_.y = g_array_[1];
      g_.z = g_array_[2];

      atk::WristFTCalibration::AddEquation(f_, t_, g_);
    }

    bool Solve()
    {
      return atk::WristFTCalibration::Solve();
    }

    void Get()
    {
      cArrayToPyList(atk::wrenchToVector(atk::WristFTCalibration::Get()).data(),phi);
    }
  
    py::list phi;

  private:

    double ft_array_[6];
    double g_array_[3];
    geometry_msgs::Vector3 f_;
    geometry_msgs::Vector3 t_;
    geometry_msgs::Vector3 g_;
    
};

#endif
