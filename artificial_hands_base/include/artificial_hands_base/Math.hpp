#ifndef ATK_MATH
#define ATK_MATH

#include <vector>
#include <numeric>
#include <algorithm>
#include <math.h>

namespace atk
{
  /**
   * @brief Find max of vector
   * @param vec input vector
   * @return Value of max element in vector
   */
  static double max(std::vector<double> vec)
  {
    return *max_element(vec.begin(), vec.end());
  }

  /**
   * @brief Find max of vector from first to n-th element
   * @param vec input vector
   * @param n number of element to search 
   * @return Value of max element in the sub-vector
  */
  static double max(std::vector<double> vec, int n)
  {
    return *max_element(vec.begin(), vec.begin() + n);
  }

  /**
   * @brief Find max of vector absolute values
   * @param vec input vector
   * @return Vector max value
   */
  static double maxval(std::vector<double> vec, int len)
  {
    double a = abs(*max_element(vec.begin(), vec.begin() + len));
    double b = abs(*min_element(vec.begin(), vec.begin() + len));
    return abs(std::max(a,b));
  }

  /**
   * @brief Find min of vector
   * @param vec input vector
   * @return Value of min element in vector
   */
  static double min(std::vector<double> vec)
  {
    return *min_element(vec.begin(), vec.end());
  }

  /**
   * @brief Compute ratio between absolute value of signal peaks
   * @param vec input vector
   * @return Peaks ratio (l.t one)
   */
  static double peakrto(std::vector<double> vec)
  {
    double r;
    double M = abs(max(vec));
    double m = abs(min(vec));
    (M > m ? r = m/M : r = M/m);
    return r;
  }

  /**
   * @brief Compute summation along vector
   * @param vec input vector
   * @return Value of element summation
   */
  static double summ(std::vector<double> vec)
  {
    return accumulate(vec.begin(), vec.end(), 0.0);
  }

  /**
   * @brief Compute mean value of vector
   * @param vec input vector
   * @return Mean value of elements
   */
  static double mean(std::vector<double> vec)
  {
    double n = vec.size();

    return accumulate(vec.begin(), vec.end(), 0.0)/n;
  }

  /**
   * @brief Compute skewness of vector
   * @param vec input vector
   * @return Skewness of elements
   */
  static double skewness(std::vector<double> vec)
  {
    double n = vec.size();

    double m = accumulate(vec.begin(), vec.end(), 0.0)/n;
    
    double m3 = 0;
    for(int i = 0; i < n; i++)
    {
      m3 += pow(vec[i] - m, 3);
    }

    double m2 = 0;
    for(int i = 0; i < n; i++)
    {
      m2 += pow(vec[i] - m, 2);
    }

    return (m3/n)/pow(pow(m2,0.5),3);
  }

  /**
   * @brief Compute root means square value of vector
   * @param vec input vector
   * @return RMS of elements
   */
  static double rms(std::vector<double> vec)
  {
    double n = vec.size();

    double m = accumulate(vec.begin(), vec.end(), 0.0)/n;

    double rms = 0;

      for (int i = 0; i < n; i++)
      {
          rms += pow(vec[i], 2);
    }

    rms = pow(rms/n,0.5);

    return rms;
  }

  /**
   * @brief Compute standard deviation of vector
   * @param vec input vector
   * @return Std of elements
   */
  static double stddev(std::vector<double> vec)
  {
    double n = vec.size();

    double m = accumulate(vec.begin(), vec.end(), 0.0)/n;

    double std = 0;

      for (int i = 0; i < n; i++)
      {
          std += pow(vec[i] - m, 2);
    }

    std = pow(std/n,0.5);

    return std;
  }
}

#endif