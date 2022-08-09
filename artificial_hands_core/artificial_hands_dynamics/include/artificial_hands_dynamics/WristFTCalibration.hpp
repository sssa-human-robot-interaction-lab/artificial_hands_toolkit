#ifndef ATK_WRIST_FT_CALIBRATION
#define ATK_WRIST_FT_CALIBRATION

#include <artificial_hands_base/BaseCommon.hpp>

namespace atk
{

  class WristFTCalibration
  {

    public:
      /**
       * @brief Construct a new WristFTCalibration object
       */
      WristFTCalibration()
      {
        for(int i = 0; i < 6; i++)phi_.push_back(0);
        x_.resize(6*10*1000);
        y_.resize(6*1000);
      };  

      /**
       * @brief Add a calibration point
       * @param f force vector
       * @param t torque vector
       * @param g gravity vector
       */
      void AddEquation(geometry_msgs::Vector3 f, geometry_msgs::Vector3 t, geometry_msgs::Vector3 g)
      {
        
        double gx = -g.x;
        double gy = -g.y;
        double gz = -g.z;

        double x[60] = {1., .0, .0, .0, .0, .0,    -gx,    .0,    .0,    .0,
                        0., 1., .0, .0, .0, .0,    -gy,    .0,    .0,    .0,
                        0., .0, 1., .0, .0, .0,    -gz,    .0,    .0,    .0, 
                        0., .0, .0, 1., .0, .0,     .0,    .0,   -gz,    gy,
                        0., .0, .0, .0, 1., .0,     .0,    gz,    .0,   -gx,
                        0., .0, .0, .0, .0, 1.,     .0,   -gy,    gx,    .0};

        double y[6] = {f.x, f.y, f.z, t.x, t.y, t.z};
        
        addArray(&x_,x,60);
        addArray(&y_,y,6);                
      };

      /**
       * @brief Solve for calibration parameters
       * @return True on success
       */
      bool Solve()
      {
        double l[10];
        double u[10];
        for(int i = 0; i < 10; i++)l[i] = -inf;
        for(int i = 0; i < 10; i++)u[i] = inf;
        l[6] = 0;

        double* c = leastSquareFit(y_.size(),10,x_.data(),y_.data(),l,u);
        copyArray(&phi_,c,10);

        cal_str.str("");
        for(int i = 0; i < 10; i++)cal_str << phi_[i] << " ";
        
        x_.clear();
        y_.clear();    
        return true;
      }

      /**
       * @brief Get calibration parameters
       * @return Wrench with calibration offset
       */
      geometry_msgs::Wrench Get()
      {
        std::vector<double> phi;
        for(int i = 0; i < 6; i++)phi.push_back(phi_[i]);
        return wrenchFromVector(phi);   
      }

      /**
       * @brief Get mass found along with calibration parameters
       * @return Mass attached to the force/torque sensor
       */
      double GetMass()
      {
        return phi_[6];
      }

      /**
       * @brief Set mass from calibration parameters
       */
      void SetCalib(double* offset, double m)
      {
        for(int i = 0; i < 6; i++)
        {
          phi_[i] = offset[i];
        }
        phi_[6] = m;
      }

      std::stringstream cal_str;

    private:

      std::vector<double> x_;
      std::vector<double> y_;
      std::vector<double> phi_;
       
  };
}

#endif