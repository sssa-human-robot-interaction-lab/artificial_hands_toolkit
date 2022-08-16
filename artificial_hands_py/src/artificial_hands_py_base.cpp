
#include <artificial_hands_py/pyWristFTCalibration.hpp>
#include <artificial_hands_py/pyWristFTContactDetection.hpp>

PYBIND11_MODULE(pyatk, m) {

  py::class_<atk::WristFTCalibration>(m, "atkWristFTCalibration");
  py::class_<pyWristFTCalibration,atk::WristFTCalibration>(m, "pyWristFTCalibration")
  .def("c_update",&pyWristFTCalibration::AddEquation)
  .def("c_solve",&pyWristFTCalibration::Solve)
  .def("c_get",&pyWristFTCalibration::Get)
  .def_readwrite("c_phi",&pyWristFTCalibration::phi);

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