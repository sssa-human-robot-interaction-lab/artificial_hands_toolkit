#include <artificial_hands_base/WristFTDetection.hpp>
#include <artificial_hands_py/pyBaseConversions.hpp>

class pyWristFTDetection: public atk::WristFTDetection
{
  public:

    pyWristFTDetection(int buf=50, int th=20, double rto=.5, double dev=.4, double factor=2):
      WristFTDetection(buf,th,rto,dev,factor){}

    void Init(py::list ft_list)
    {
      cArrayFromPyList(ft_array_,ft_list);
      atk::copyArray(&ft_,ft_array_,6);
      atk::WristFTDetection::Init(atk::wrenchFromVector(ft_));
    }

    void Update(py::list ft_list)
    {
      cArrayFromPyList(ft_array_,ft_list);
      atk::copyArray(&ft_,ft_array_,6);
      atk::WristFTDetection::Update(atk::wrenchFromVector(ft_));
    }

    double d_fi_max;

  private:

    std::vector<double> ft_;
    double ft_array_[6];
    
};

