#ifndef ATK_ALGORITHM
#define ATK_ALGORITHM

#include <libalglib/stdafx.h>
#include <libalglib/interpolation.h>

using namespace alglib;

namespace atk
{
  /**
   * @brief Used as objective function (callback) for alglib ls fitting
   * @param c vector of coefficients
   * @param x row with constant terms
   * @param y result of x*c
  */
  static void rowByVector(const real_1d_array &c, const real_1d_array &x, double &y, void *ptr)
  {
    y = 0;
    for(int i = 0; i < c.length(); i++)
    {
      y += x[i]*c[i];
    }
  } 

  /**
   * @brief Alglib ls fitting
   * @param rows number of equations
   * @param cols number of coefficients to fit
   * @param x array of x sets
   * @param y array of y = f(x) values
   * @param l array of lower bounds for coefficients
   * @param u array of upper bounds for coefficients
   * @note The x array should be formed appending blocks of x sets and each set must
   * have its corresponding y = f(x) value added to the y array
  */
  static double* leastSquareFit(int rows, int cols, double* x, double* y, double* l, double* u)
    {
      double c[cols];
      for(int i = 0; i < cols; i++)c[i] = .0;

      real_1d_array c_;
      c_.setcontent(cols,c);
    
      ae_int_t info = 0;
      lsfitstate state;
      lsfitreport rep;
      ae_int_t maxits = 0;
      double epsx = 0;
      double diffstep = 0.0001;

      real_2d_array x_;
      real_1d_array y_;
      x_.attach_to_ptr(rows,cols,x);
      y_.attach_to_ptr(rows,y);

      real_1d_array l_;
      real_1d_array u_;
      l_.attach_to_ptr(cols,l);
      u_.attach_to_ptr(cols,u);
      
      lsfitcreatef(x_, y_, c_, diffstep, state);
      lsfitsetbc(state, l_, u_);
      lsfitsetcond(state, epsx, maxits);
      lsfitfit(state, rowByVector);
      lsfitresults(state, info, c_, rep);

      return c_.getcontent();
    }
}

#endif