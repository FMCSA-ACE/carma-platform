#include <tpms_plugin/TPMSAnalytics.hpp>

namespace fmcsa_ace_tpms_analytics
{

  void TPMSAnalytics::readData(string filename)
  {
    TPMS input;
    std::ifstream file;
   
    file.open(filename,std::ios_base::in | std::ios_base::binary);
    
    while (file.eof() == 0)
    {
      file.read((char *)(&input),((int)sizeof(TPMS)));
      myData.push_back(input);
    }
    file.close();
  }

    // Function to take input from the dataset
  void TPMSAnalytics::takeTireInput(int tire)
  {
    double xi, yi;

    tire = 4;
    for (auto data : myData)
    {
      xi = data.time;
      yi = data.pressure[tire];

      sum_xy += xi * yi;
      sum_x += xi;
      sum_y += yi;
      sum_x_square += xi * xi;
      sum_y_square += yi * yi;
      x.push_back(xi);
      y.push_back(yi);
    }
  }

  void TPMSAnalytics::process(string filename)
  {
    int i;
    double future;
    vector<unsigned long> futures;
    
    this -> readData(filename);
    for (i=0;i<18;i++)
    {
      this -> takeTireInput(i);
      this -> PrintBestFittingLine();
      future = this -> predict(this -> getTime());
      myPredictions.push_back(future);
      x.clear();
      y.clear();
    }
  }

  // Function to predict the value
  // corresponding to some input
  double TPMSAnalytics::predict(double x)
  {
    double value;
    unsigned int count;

    count = 0;
    value =  coeff * x + constTerm;
    while (count < 1000000 && value > 50)
    {
      x += 10000;
      value =  coeff * x + constTerm;
      count++;
    }
    if (count == 1000000)
      x = -1.0;
    return(x);
  }
   
   
  unsigned long TPMSAnalytics::getTime()
  {
    TPMS data;
    unsigned long time;

    data = myData.back();
    return(time);
  }

  fmcsa_ace_msgs::msg::TPMSData TPMSAnalytics::createMessage()
  {   
    int index;
    TPMS data;
    carma_v2x_msgs::msg::TirePressure tire;
    carma_v2x_msgs::msg::TirePressureMonitoringSystem message;

    data = myData.back();
    message.time = data.time;
    for (index = 0; index < 18; index++)
    {
      tire.pressure = data.pressure[index];
      tire.condition = 0; 
      if (tire.pressure < 50)
      {
        tire.condition = 1;
      }
      message.tires.push_back(tire);
    }
    return(message);
  }
}
 
