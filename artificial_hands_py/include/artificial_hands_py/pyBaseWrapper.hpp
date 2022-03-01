#include <artificial_hands_py/pyBaseFilter.hpp>

PYBIND11_MODULE(pyatk, m) {

  py::class_<atk::BaseFilter>(m, "atkBaseFilter");
  py::class_<pyBaseFilter,atk::BaseFilter>(m, "pyBaseFilter")
  .def(py::init<int>())
  .def("c_init",&pyBaseFilter::Init)
  .def("c_update",&pyBaseFilter::Update)
  .def("c_get",&pyBaseFilter::Get);

}