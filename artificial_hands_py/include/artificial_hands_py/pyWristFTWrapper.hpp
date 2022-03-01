#include <artificial_hands_py/pyWristFTDetection.hpp>

PYBIND11_MODULE(pyatk, m) {

  py::class_<atk::WristFTDetection>(m, "atkWristFTDetection");
  py::class_<pyWristFTDetection,atk::WristFTDetection>(m, "pyWristFTDetection")
  .def(py::init<int,int,double,double,double>())
  .def("c_init",&pyWristFTDetection::Init)
  .def("c_update",&pyWristFTDetection::Update)
  .def("c_get",&pyWristFTDetection::Get)
  .def("c_set_zero",&pyWristFTDetection::SetZero)
  .def("c_pre_trigger_static",&pyWristFTDetection::PreTriggerStatic)
  .def("c_trigger_static",&pyWristFTDetection::TriggerStatic)
  .def("c_backup_trigger_static",&pyWristFTDetection::BackUpTriggerStatic)
  .def("c_get_d_fi_max",&pyWristFTDetection::GetdFiMax)
  .def_readwrite("c_pretrig",&pyWristFTDetection::pretrig)
  .def_readwrite("c_trigger",&pyWristFTDetection::trigger)
  .def_readwrite("c_backtrig",&pyWristFTDetection::backtrig);

}