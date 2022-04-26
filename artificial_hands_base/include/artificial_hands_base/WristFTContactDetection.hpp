#ifndef ATK_WRIST_FT_DETECTION
#define ATK_WRIST_FT_DETECTION

#include <artificial_hands_base/BaseFTSensor.hpp>

namespace atk
{

  class WristFTContactDetection: public atk::BaseFTSensor<BaseIRFilter>
  {

    public:
      /**
       * @brief Construct a new WristFTContactDetection object
       * @note Default parameters are calibrated for sensor driver streaming at 500 Hz
       */
      WristFTContactDetection(int buf = 50, int th = 20, double rto = .5, double dev = .4, double factor = 2):
      th_(th),
      rto_(rto),
      dev_(dev),
      factor_(factor)
      {
        t_mag_.reserve(buf);
        t_summ_.reserve(buf);
        t_mag_std_.reserve(buf);
        t_summ_sk_.reserve(buf);
        for(int i = 0; i < 3; i++)v_[i].reserve(buf);
        d_fi_.reserve(buf);
        d_fi_.push_back(.0);
        BaseFTSensor::SetFilter(num_,den_,order_,chann_);
        assignVector3(&th_det_.torque,0.01); //not assigning threshold for force since is not used for trigger (torque is far more sensible)
      };

      /**
       * @brief Update detection state.
       * @param ft message from sensor driver
       */
      bool Update(geometry_msgs::Wrench ft)
      {
        BaseFTSensor::Update(ft);
        BaseFTSensor::UpdateRaw(ft);
        
        addElement(&t_mag_,magnitudeVector3(ft.torque));
        addElement(&t_summ_,t_summ_.back() + t_mag_.back());
        addElement(&t_mag_std_,stddev(t_mag_));
        addElement(&t_summ_sk_,skewness(t_summ_));

        addElement(&d_fi_,magnitudeVector3(force)-f_mag_);
        f_mag_ = magnitudeVector3(force);

        geometry_msgs::Vector3 v = force_raw;
        assignVector3(&xy_,v.x,v.y,.0);
        assignVector3(&xz_,v.x,.0,v.z);
        assignVector3(&yz_,.0,v.y,v.z);
        addElement(&v_[0],acos(v.x/magnitudeVector3(xy_))/M_PI);
        addElement(&v_[1],acos(v.x/magnitudeVector3(xz_))/M_PI);
        addElement(&v_[2],acos(v.y/magnitudeVector3(yz_))/M_PI);
        return true;
      }

      /**
       * @brief Trigger interaction forces from IIR filter for static handover
       */
      void BackUpTriggerStatic()
      {
        backtrig = triggerOrVector3(th_det_.torque,torque);
      };

      /**
       * @brief Check pre-trigger condition (event detection) for static handover
       * @return True if a pre-trigger condition was met
       */
      bool PreTriggerStatic()
      {
        pretrig = false | t_mag_std_.back() > max(t_mag_std_,th_)*factor_;
        pretrig = pretrig & abs(t_summ_sk_.back()) > maxval(t_summ_sk_,th_)*factor_;
        return pretrig;
      };

      /**
       * @brief Check trigger condition (event analysis) for static handover
       */
      void TriggerStatic()
      {
        double d = .0;
        for(int i = 0; i < 3; i++)d = std::max(d, stddev(v_[i]));
        BaseFTSensor::GetZero(&zero_);
        double r = peakrto(subtractVector(t_mag_,magnitudeVector3(zero_.torque)));
        if(r < rto_ && d < dev_)trigger = true; 
        det_str.str("");
        det_str << "Rto: " << r << "/" << rto_ << " Dev: " << d << "/" << dev_;
        det_str << " dFImax: " << maxval(d_fi_) << " Trig: " << (int)(trigger | backtrig);
      };

      /**
       * @brief Get maximum derivative of interaction forces
       * @return Maximum force derivative from IIR filter
       */
      double GetdFiMax()
      {
        return maxval(d_fi_);
      }

      bool trigger = false;
      bool pretrig = false;
      bool backtrig = false;
      std::stringstream det_str;
      
    private:

      int th_, order_ = 2, chann_ = 6;
      double rto_, dev_, factor_, t_dev_th_, t_sk_th_, f_mag_ = .0;
      double num_[3] = {0.0099537574599431831, -0.019594794464524581, 0.0099537574599431831};
      double den_[3] = {1.0, -1.9749591089928671, 0.97527182944822877};
      std::vector<double> t_mag_, t_summ_, t_mag_std_, t_summ_sk_, d_fi_;
      std::vector<std::vector<double>> v_{3};
      geometry_msgs::Vector3 xy_, xz_, yz_;
      geometry_msgs::Wrench zero_, th_det_;
      
  };
}

#endif