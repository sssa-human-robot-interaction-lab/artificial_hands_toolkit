#define inf INFINITY

#ifndef ATK_COMMON
#define ATK_COMMON

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <artificial_hands_base/Algorithm.hpp>
#include <artificial_hands_base/Math.hpp>

namespace atk
{  

  /**
   * @brief Stores 6D kinematics
   */
  struct kinematics6D_t
  {
    geometry_msgs::Twist position;
    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;
    geometry_msgs::Twist twist_velocity;
    geometry_msgs::Twist twist_acceleration;
  };

  /**
   * @brief Add element to vector without exceeding its capacity.
   * @param vec pointer to vector
   * @param el element to insert
   */
  template <typename vec_t = double>
  static void addElement(std::vector<vec_t>* vec, vec_t el)
  {
    if(vec->size() == vec->capacity())vec->erase(vec->begin());
    vec->push_back(el);
  }

  /**
   * @brief Copy a double array to vector and resize it.
   * @param vec pointer to vector
   * @param arr double array
   * @param len length of the array
   */
  static void copyArray(std::vector<double>* vec, double* arr, int len)
  {
    std::vector<double> vec_;
    vec_.insert(vec_.begin(),arr,arr+len);
    vec->swap(vec_);
    vec->resize(len);
  }

  /**
   * @brief Add a double array to vector without exceeding its capacity.
   * @param vec pointer to vector
   * @param arr double array
   * @param len length of the array
   */
  static void addArray(std::vector<double>* vec, double* arr, int len)
  {
    if(vec->size() == vec->capacity())
    {
      for(int i = 0; i < len; i++)vec->erase(vec->begin());
    }

    std::vector<double> vec_(arr,arr+len); 
    vec->insert(vec->end(), vec_.begin(), vec_.end() );
  }

  /**
   * @brief Set vector3 components
   */
  static void assignVector3(geometry_msgs::Vector3* vec, double x, double y, double z)
  {
    vec->x = x;
    vec->y = y; 
    vec->z = z;
  };

  /**
   * @brief Set all the vector3 components equal to d
   * @param vec pointer to vector
   * @param d double to fill the vector with
   */
  static void assignVector3(geometry_msgs::Vector3* vec, double d)
  {
    vec->x = d; 
    vec->y = d; 
    vec->z = d;
  };

  /**
   * @brief Set vector3 components to zero.
   * @param vec pointer to vector
   */
  static void resetVector3(geometry_msgs::Vector3* vec)
  {
    assignVector3(vec,.0);
  };

  /**
   * @brief Change vector components to their absolute values
   */
  static void absoluteVector3 (geometry_msgs::Vector3* vec)
  {
    vec->x = abs(vec->x);
    vec->y = abs(vec->y);
    vec->z = abs(vec->z);
  };

  /**
   * @brief Sum two vector
   * @return vec1 + vec2
   */
  static geometry_msgs::Vector3 sumVector3(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2)
  {
    geometry_msgs::Vector3 vec;
    vec.x = vec1.x + vec2.x;
    vec.y = vec1.y + vec2.y;
    vec.z = vec1.z + vec2.z;

    return vec;
  };

  /**
   * @brief Subtract two vector
   * @return vec1 - vec2
   */
  static geometry_msgs::Vector3 subtractVector3(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2)
  {
    geometry_msgs::Vector3 vec;
    vec.x = vec1.x - vec2.x;
    vec.y = vec1.y - vec2.y;
    vec.z = vec1.z - vec2.z;

    return vec;
  };

  /**
   * @brief Compare components of two vectors copying vec_(i) to vec when vec_(i) g.t. vec(i)
   */
  static void compareVector3(geometry_msgs::Vector3* vec, geometry_msgs::Vector3 vec_)
  {
    if(vec_.x > vec->x)vec->x = vec_.x;
    if(vec_.y > vec->y)vec->y = vec_.y;
    if(vec_.z > vec->z)vec->z = vec_.z;
  };

  /**
   * @brief Compare components of a vector with respect to a threshold vector
   * @param th threshold vector
   * @param vec vector to compare
   * @param fth multiplication factor for the threshold vector
   * @return True if for any i components th(i) l.t. vec(i)
   */
  static bool triggerOrVector3(geometry_msgs::Vector3 th, geometry_msgs::Vector3 vec, double fth = 1.0)
  {
    return fth*th.x < vec.x| fth*th.y < vec.y | fth*th.z < vec.z;
  }

  /**
   * @brief Compute magnitude of vector
   */
  static double magnitudeVector3(geometry_msgs::Vector3 vec)
  {
    return pow(pow(vec.x,2)+pow(vec.y,2)+pow(vec.z,2),0.5);
  }
  
  /**
   * @return 6 elements vector from wrench force/torque components
   */
  static std::vector<double> wrenchToVector(geometry_msgs::Wrench ft)
  {
    std::vector<double> vec={.0, .0, .0, .0, .0, .0};
    vec[0] = ft.force.x;
    vec[1] = ft.force.y;
    vec[2] = ft.force.z;
    vec[3] = ft.torque.x;
    vec[4] = ft.torque.y;
    vec[5] = ft.torque.z;
    return vec;
  }

  /**
   * @return Wrench from a 6 elements vector of force/torque components
   */
  static geometry_msgs::Wrench wrenchFromVector(std::vector<double> vec)
  {
    geometry_msgs::Wrench wrench;
    wrench.force.x = vec[0];
    wrench.force.y = vec[1];
    wrench.force.z = vec[2];
    wrench.torque.x = vec[3];
    wrench.torque.y = vec[4];
    wrench.torque.z = vec[5];
    return wrench;
  }

  /**
   * @brief Subtract element to a vector
   * @param vec input vector
   * @param el element to subtract
   */
  static std::vector<double> subtractVector(std::vector<double> vec, double el)
  {
    std::vector<double> vec_;
    vec_.reserve(vec.size());
    for(int i = 0; i < vec.size(); i ++)vec_.push_back(vec[i] - el);
    return vec_;
  };
}
#endif