#ifndef PY_BASE_FILTER
#define PY_BASE_FILTER

#include <artificial_hands_base/BaseFilters.hpp>
#include <artificial_hands_py/pyBaseConversions.hpp>

class pyBaseFilter: public atk::BaseFilter
{
  public:

    pyBaseFilter(int len=20): 
      len_(len),
      BaseFilter(len)
    {
      f_array_ = new double[len];
      for(int i = 0; i < len; i++)f_list_.append(.0);
    }

    void Init(py::list f_list)
    {
      cArrayFromPyList(f_array_,f_list);
      atk::copyArray(&f_,f_array_,len_);
      atk::BaseFilter::Init(f_);
    }

    void Update(py::list f_list)
    {
      cArrayFromPyList(f_array_,f_list);
      atk::copyArray(&f_,f_array_,len_);
      atk::BaseFilter::Update(f_);
    }

    py::list Get()
    {
      cArrayToPyList(atk::BaseFilter::Get().data(),f_list_);
      return f_list_;
    }
  
  private:

    py::list f_list_;
    std::vector<double> f_;
    double* f_array_;
    const double len_;
};

#endif