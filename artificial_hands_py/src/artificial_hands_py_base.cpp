
#include <artificial_hands_py/pyBaseFilter.hpp>
#include <artificial_hands_py/pyWristFTContactDetection.hpp>

PYBIND11_MODULE(pyatk, m) {

  py::class_<atk::BaseFilter>(m, "atkBaseFilter");
  py::class_<pyBaseFilter,atk::BaseFilter>(m, "pyBaseFilter")
  .def(py::init<int>())
  .def("c_init",&pyBaseFilter::Init)
  .def("c_update",&pyBaseFilter::Update)
  .def("c_get",&pyBaseFilter::Get);

  py::class_<atk::WristFTContactDetection>(m, "atkWristFTContactDetection");
  py::class_<pyWristFTContactDetection,atk::WristFTContactDetection>(m, "pyWristFTContactDetection")
  .def(py::init<int,int,double,double,double>())
  .def("c_init",&pyWristFTContactDetection::Init)
  .def("c_update",&pyWristFTContactDetection::Update)
  .def("c_get",&pyWristFTContactDetection::Get)
  .def("c_set_zero",&pyWristFTContactDetection::SetZero)
  .def("c_pre_trigger_static",&pyWristFTContactDetection::PreTriggerStatic)
  .def("c_trigger_static",&pyWristFTContactDetection::TriggerStatic)
  .def("c_backup_trigger_static",&pyWristFTContactDetection::BackUpTriggerStatic)
  .def("c_get_d_fi_max",&pyWristFTContactDetection::GetdFiMax)
  .def_readwrite("c_pretrig",&pyWristFTContactDetection::pretrig)
  .def_readwrite("c_trigger",&pyWristFTContactDetection::trigger)
  .def_readwrite("c_backtrig",&pyWristFTContactDetection::backtrig);

}