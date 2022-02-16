#include <pybind11/pybind11.h>

#include <artificial_hands_wrist/Detection.hpp>

namespace py = pybind11;

class pyDetection: public atk::Detection
{
  public:

    pyDetection(int buf=50, int th=20, double rto=.5, double dev=.4, double factor=2):
      Detection(buf,th,rto,dev,factor){}

    void Init(py::list ft_list)
    {
      cArrayFromPyList(ft_array_,ft_list);
      atk::copyArray(&ft_,ft_array_,6);
      atk::Detection::Init(atk::wrenchFromVector(ft_));
    }

    void Update(py::list ft_list)
    {
      cArrayFromPyList(ft_array_,ft_list);
      atk::copyArray(&ft_,ft_array_,6);
      atk::Detection::Update(atk::wrenchFromVector(ft_));
    }

    double d_fi_max;

  private:

    void cArrayFromPyList(double* c_array, py::list py_list)
    {
      int len = py::len(py_list);
      for(int i = 0; i < len; i++)c_array[i] = py_list[i].cast<double>();
    }

    std::vector<double> ft_;
    double ft_array_[6];
    
};


PYBIND11_MODULE(pyatk, m) {

  py::class_<atk::Detection>(m, "atkDetection");

  py::class_<pyDetection,atk::Detection>(m, "Detection")
  .def(py::init<int,int,double,double,double>())
  .def("init",&pyDetection::Init)
  .def("update",&pyDetection::Update)
  .def("get",&pyDetection::Get)
  .def("set_zero",&pyDetection::SetZero)
  .def("pre_trigger_static",&pyDetection::PreTriggerStatic)
  .def("trigger_static",&pyDetection::TriggerStatic)
  .def("backup_trigger_static",&pyDetection::BackUpTriggerStatic)
  .def("get_d_fi_max",&pyDetection::GetdFiMax)
  .def_readwrite("pretrig",&pyDetection::pretrig)
  .def_readwrite("trigger",&pyDetection::trigger)
  .def_readwrite("backtrig",&pyDetection::backtrig);
}