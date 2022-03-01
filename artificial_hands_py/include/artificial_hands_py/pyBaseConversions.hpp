#ifndef PY_BASE_CONVERSIONS
#define PY_BASE_CONVERSIONS

#include <pybind11/pybind11.h>

namespace py = pybind11;

void cArrayFromPyList(double* c_array, py::list py_list)
{
  int len = py::len(py_list);
  for(int i = 0; i < len; i++)c_array[i] = py_list[i].cast<double>();
}

void cArrayToPyList(double* c_array, py::list py_list)
{
  int len = py::len(py_list);
  for(int i = 0; i < len; i++)py_list[i] = c_array[i];
}

#endif