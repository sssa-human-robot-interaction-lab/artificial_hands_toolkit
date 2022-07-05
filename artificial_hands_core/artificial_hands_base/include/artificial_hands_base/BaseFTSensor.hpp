#ifndef ATK_BASE_FT_SENSOR
#define ATK_BASE_FT_SENSOR

#include <artificial_hands_base/BaseFilters.hpp>

namespace atk
{
  template <typename BaseFilter_t>
  class BaseFTSensor: public BaseFilter_t
  {

    public:
      /**
       * @brief Construct a new BaseFTSensor object
       */
      BaseFTSensor(){};

      /**
       * @brief Initialize sensor state.
       * @param ft message from sensor driver
       * @return True on success
       */
      bool Init(geometry_msgs::Wrench ft)
      {
        BaseFilter_t::Init(wrenchToVector(ft)); 
        // resetVector3(&o_.force);
        // resetVector3(&o_.torque);
        resetVector3(&z_.force);
        resetVector3(&z_.torque);
        return true;
      };

      /**
       * @brief Update sensor state.
       * @param ft message from sensor driver
       */
      void Update(geometry_msgs::Wrench ft)
      {
        BaseFilter_t::Update(wrenchToVector(ft));
      };

      /**
       * @brief Update sensor state (raw data).
       * @param ft message from sensor driver
       */
      void UpdateRaw(geometry_msgs::Wrench ft)
      {
        force_raw = subtractVector3(subtractVector3(ft.force,o_.force),z_.force);
        torque_raw = subtractVector3(subtractVector3(ft.torque,o_.torque),z_.torque);
      };

      /**
       * @brief Get updated sensor state
       * @return True on success
       */
      bool Get()
      {
        geometry_msgs::Wrench ft = wrenchFromVector(BaseFilter_t::Get());
        
        force = subtractVector3(subtractVector3(ft.force,o_.force),z_.force);
        torque = subtractVector3(subtractVector3(ft.torque,o_.torque),z_.torque);
        
        force.x = force.x*m_[0] + q_[0];
        force.y = force.y*m_[1] + q_[1];
        force.z = force.z*m_[2] + q_[2];

        torque.x = torque.x*m_[3] + q_[3];
        torque.y = torque.y*m_[4] + q_[4];
        torque.z = torque.z*m_[5] + q_[5];
        return true;
      };

      // /**
      //  * @brief Set filter parameters
      //  * @param filter struct of filter_t parameters
      //  */
      // void SetFilter(filter_t filter)
      // {
      //   BaseIRFilter::SetFilter(filter,6);
      // };

      // /**
      //  * @brief Set filter parameters
      //  * @param filter struct of filter_t parameters
      //  */
      // void SetFilter(int filter_length)
      // {
      //   BaseFilter::SetFilter(filter_length);
      // };

      /**
       * @brief Set coefficients for linear correction of force/torque sensor
       * @param ft_calib struct of ft_calib_t parameters
       */
      void SetCorrection(atk::ft_calib_t ft_calib)
      {
        for(int i = 0; i < 6; i++)
        {
          m_[i] = ft_calib.gain[i];
          q_[i] = ft_calib.bias[i];
        }
        o_.force.x = ft_calib.offset[0];
        o_.force.y = ft_calib.offset[1];
        o_.force.z = ft_calib.offset[2];

        o_.torque.x = ft_calib.offset[3];
        o_.torque.y = ft_calib.offset[4];
        o_.torque.z = ft_calib.offset[5];
      }

      /**
       * @brief Set zero of force/torque sensor
       * @param do_zero true/false to do/undo zero
       */
      void SetZero(bool do_zero)
      {
        if(do_zero)
        {
          z_.force = force;
          z_.torque = torque;
        }
        else
        {
          resetVector3(&z_.force);
          resetVector3(&z_.torque);        
        }
      }

      /**
       * @brief Get bias introduced with SetZero
       * @param ft pointer to wrench to return bias
       */
      void GetZero(geometry_msgs::Wrench *ft)
      {
        ft->force = z_.force;
        ft->torque = z_.torque;
      }

      /**
       * @brief Set bias of force/torque sensor
       * @param ft wrench of force/torque sensor offet
       */
      void SetOffset(geometry_msgs::Wrench ft)
      {
        o_.force = ft.force;
        o_.torque = ft.torque;
      }

      geometry_msgs::Vector3 force;
      geometry_msgs::Vector3 torque;
      geometry_msgs::Vector3 force_raw;
      geometry_msgs::Vector3 torque_raw;

    private:

      geometry_msgs::Wrench z_;  
      geometry_msgs::Wrench o_; 
      double m_[6] = {1.0,1.0,1.0,1.0,1.0,1.0};
      double q_[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
  };
}

#endif