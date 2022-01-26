#ifndef ATK_SENSOR
#define ATK_SENSOR

#include <artificial_hands_base/Filters.hpp>

namespace atk
{
  template <typename Filter_t>
  class Sensor: public Filter_t
  {

    public:
      /**
       * @brief Construct a new Sensor object
       */
      Sensor(){};

      /**
       * @brief Initialize sensor state.
       * @param ft message from sensor driver
       * @return True on success
       */
      bool Init(geometry_msgs::Wrench ft)
      {
        Filter_t::Init(wrenchToVector(ft)); 
        resetVector3(&bias_.force);
        resetVector3(&bias_.torque);
        resetVector3(&zero_.force);
        resetVector3(&zero_.torque);
        return true;
      };

      /**
       * @brief Update sensor state.
       * @param ft message from sensor driver
       */
      void Update(geometry_msgs::Wrench ft)
      {
        Filter_t::Update(wrenchToVector(ft));
      };

      /**
       * @brief Update sensor state (raw data).
       * @param ft message from sensor driver
       */
      void UpdateRaw(geometry_msgs::Wrench ft)
      {
        force_raw = subtractVector3(subtractVector3(ft.force,zero_.force),bias_.force);
        torque_raw = subtractVector3(subtractVector3(ft.torque,zero_.torque),bias_.torque);
      };

      /**
       * @brief Get updated sensor state
       * @return True on success
       */
      bool Get()
      {
        geometry_msgs::Wrench ft = wrenchFromVector(Filter_t::Get());
        force = subtractVector3(subtractVector3(ft.force,zero_.force),bias_.force);
        torque = subtractVector3(subtractVector3(ft.torque,zero_.torque),bias_.torque);
        return true;
      };

      /**
       * @brief Set zero of force/torque sensor
       * @param do_zero true/false to do/undo zero
       */
      void SetZero(bool do_zero)
      {
        if(do_zero)
        {
          zero_.force = force;
          zero_.torque = torque;
        }
        else
        {
          resetVector3(&zero_.force);
          resetVector3(&zero_.torque);        
        }
      }

      /**
       * @brief Get bias introduced with SetZero
       * @param ft pointer to wrench to return bias
       */
      void GetZero(geometry_msgs::Wrench *ft)
      {
        ft->force = zero_.force;
        ft->torque = zero_.torque;
      }

      /**
       * @brief Set bias of force/torque sensor
       * @param ft wrench of force/torque sensor offet
       */
      void SetOffset(geometry_msgs::Wrench ft)
      {
        bias_.force = ft.force;
        bias_.torque = ft.torque;
      }

      geometry_msgs::Vector3 force;
      geometry_msgs::Vector3 torque;
      geometry_msgs::Vector3 force_raw;
      geometry_msgs::Vector3 torque_raw;

    private:

      geometry_msgs::Wrench zero_;  
      geometry_msgs::Wrench bias_;    
  };
}

#endif