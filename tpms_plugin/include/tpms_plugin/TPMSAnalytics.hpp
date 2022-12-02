

#include <tpms_plugin/TPMStype.h>
#include <tpms_plugin/reg.hpp>
#include <carma_v2x_msgs/msg/TirePressureMonitoringSystem.hpp>

namespace fmcsa_ace_tpms_analytics
{
  class TPMSAnalytics : public regression
  {
    private:
  
      vector<TPMS> myData;
      vector<unsigned long> myPredictions;
  
    protected:
    
    public:
  
      void readData(string);
      void takeTireInput(int);
      vector<unsigned long> takeInput();
      double predict(double);
      unsigned long getTime();
      void process(string);
      carma_v2x_msgs::msg::TirePressureMonitoringSystem createMessage();
  };
}
