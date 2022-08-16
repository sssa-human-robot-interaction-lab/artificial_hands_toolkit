#ifndef ATK_BASE_FILTERS
#define ATK_BASE_FILTERS

#include <boost/circular_buffer.hpp>

#include <artificial_hands_base/BaseCommon.hpp>

namespace atk{

  class BaseSGFilter
  {
    public:
      /**
       * @brief Construct a new BaseSGFilter object (8 points Savitzky-Golay)
       * @param rate update rate in Hz of the filter
      */
      BaseSGFilter(double rate):
        rate_(rate)
      {
      };

      /**
       * @brief BaseSGFilter destructor
      */
      ~BaseSGFilter()
      {
        f_.clear();
        f_state_.clear();
      };

      /**
       * @brief Init filter state
       */
      void Init(std::vector<double> f)
      {
        f_ = f;
        f_state_.clear();
        f_state_.push_back(f);
      };

      /**
       * @brief Update filter state
       */
      void Update(std::vector<double> f)
      {
        f_state_.push_back(f);
      };

      /**
       * @brief Get filter state
       */
      std::vector<double> Get() 
      {
        for(int i = 0; i < f_.size(); i++)
        {
          f_[i] = 0.0;
          for(int j = 0; j < f_state_.size(); j++)
          {
            f_[i] += f_state_[j][i] * sg_[j];
          }
          f_[i] *= rate_;
        }
        return f_;
      };

    private:

      double rate_;
      double sg_[8] = {-0.08333,-0.05952,-0.03571,-0.0119,0.0019,0.03571,0.05952,0.08333};
      std::vector<double> f_;
      boost::circular_buffer<std::vector<double>> f_state_{8};

  };

  class BaseIRFilter
  {
    public:

      /**
       * @brief Construct a new BaseIRFilter object (coeffients based)
      */
      BaseIRFilter(){};

      /**
       * @brief Construct a new BaseIRFilter object (coeffients based)
       * @param num array of numerator coefficients
       * @param den array of denominator coefficients
       * @param order filter order
       * @param chann number of channels
       */
      BaseIRFilter(double* num, double* den, int order, int chann):
        num_(num),
        den_(den),
        order_(order),
        chann_(chann)
      {
        set(); //to do: create a setCoefficients method to check num and den
      };

      /**
       * @brief BaseIRFilter destructor
      */
      ~BaseIRFilter()
      {
        f_.clear();
        z_.clear();
      };

      /**
       * @brief Set filter parameters
       * @param num array of numerator coefficients
       * @param den array of denominator coefficients
       * @param order filter order
       * @param chann number of channels
       */
      void SetFilter(double* num, double* den, int order, int chann)
      {
        num_ = num;
        den_ = den;
        order_ = order;
        chann_ = chann;
        set();
      };
      
      /**
       * @brief Set filter parameters
       * @param filter struct of filtrt_t parameters
       */
      void SetFilter(atk::filter_t filter, int chann)
      {
        num_ = filter.num;
        den_ = filter.den;
        order_ = filter.order;
        chann_ = chann;
        set();
      };
      
      /**
       * @brief Init filter state
       */
      void Init(std::vector<double> f)
      {
        for(int i = 0; i < chann_; i++)for(int j = 0; j <= order_; j++)z_[i][j] = .0;
        copyArray(&f_,f.data(),f.size());
      };

      /**
       * @brief Update filter state
       */
      void Update(std::vector<double> f)
      {
        f_new_ = f;
        for(int i = 0; i < chann_; i++)
        {
          z_[i][order_] = 0;
          f_[i] = num_[0] * f_new_[i] + z_[i][0];
          for(int j = 1; j <= order_; j++)
          {
            z_[i][j - 1] = num_[j] * f_new_[i] + z_[i][j] - den_[j] * f_[i];
          }   
        }
      };

      /**
       * @brief Get filter state
       */
      std::vector<double> Get()
      {
        return f_;
      };

    private:

      void set()
      {
        for(int i = 0; i < chann_; i++) //to do: add check on num and den
        {
          std::vector<double> z;
          z_.push_back(z);
          for(int j = 0; j <= order_; j++)z_[i].push_back(0);
        }
        // z_ = new double*[chann_];
        // for(int i = 0; i <= order_; i++)z_[i] = new double[order_];
        for(int i = 0; i <= order_; i++)num_[i] = num_[i]/den_[0];
        for(int i = 0; i <= order_; i++)den_[i] = den_[i]/den_[0];
      };

      int order_, chann_;
      double *num_, *den_; //**z_;
      std::vector<double> f_, f_new_;
      std::vector<std::vector<double>> z_;
  };

}

#endif