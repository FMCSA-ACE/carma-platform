
/***********************************************************************************/
/* Copyright (C) 2022 eScience and Technologies, Inc.                              */
/*                                                                                 */
/* Licensed under the Apache License, Version 2.0 (the "License"); you may not     */
/* use this file except in compliance with the License. You may obtain a copy of   */
/* the License at                                                                  */
/*                                                                                 */
/* http://www.apache.org/licenses/LICENSE-2.0                                      */
/*                                                                                 */
/* Unless required by applicable law or agreed to in writing, software             */
/* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT       */
/* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the        */
/* License for the specific language governing permissions and limitations under   */
/* the License.                                                                    */
/*                                                                                 */
/* Developer:  Wayne Staats                                                        */
/*                                                                                 */
/* Class:    TPMSPlugin                                                            */
/* Purpose:  Implements the Tire Management Management System function.            */
/*           The plugin uses curlpp (curlpp.org) for pulling data from the cloud   */
/*           using curl and rapidjson (rapidjson.org) for parsing the TPMS data    */
/*           received from the cloud.  This data is the tire pressure for each     */
/*           tire.                                                                 */
/*           After pulling the data from the cloud, it saves it in a file for      */
/*           future processing.  The cloud does not maintain the data once it is   */
/*           pulled.                                                               */
/* Note:     The filename used to store data and the cloud's URL are both          */
/*           configured in the config/parameters.yaml file.                        */
/***********************************************************************************/  
#include "tpms_plugin/TPMSPlugin.hpp"

namespace fmcsa_ace_tpms_plugin
{

  /*********************************************************************************/
  /* Method:    TPMSPlugin                                                         */
  /* Purpose:   This method initializes the plugin. 1)  declares and reads the     */
  /*            parameters (filename and URL) from the parameters.yaml file.       */
  /*            2) create the publishers and subscribers.                          */
  /*********************************************************************************/
  TPMSPlugin::TPMSPlugin():Node("tpms_plugin")
  {
    myMessageSubscription = this->create_subscription<std_msgs::msg::String>(
      "fmcsa_request", 10, std::bind(&TPMSPlugin::handle_request_cb, this, _1));

    myTireMessage =
      this->create_publisher<carma_v2x_msgs::msg::TirePressureMonitoringSystem>("tpms_data",10);
    myTireMessageString =
      this->create_publisher<std_msgs::msg::String>("tpms_data_string",10);

    this -> declare_parameter("tpms_cloud_url");
    this -> declare_parameter("tpms_data_file");

    this -> get_parameter("tpms_cloud_url", myCloudURL);
    this -> get_parameter("tpms_data_file", myDataFile);
  }
  
  /*********************************************************************************/
  /* Method:    handle_request_cb                                                  */
  /* Purpose:   This method is the ROS subscriber callback method for handling     */
  /*            requests for TPMS data.  Upon receiving a requests, pulls the data */
  /*            from the cloud, stores it, runs the analytics, and then publishes  */
  /*            the data on the ROS topic.                                         */
  /*********************************************************************************/
  void TPMSPlugin::handle_request_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    TPMSAnalytics analysis;
    std_msgs::msg::String str;
    carma_v2x_msgs::msg::TirePressureMonitoringSystem message;

    if (msg->data == "tpms_data")
    {
      this->pull_data();
      analysis.process(myDataFile);
      message = analysis.createMessage();
      myTireMessage -> publish(message);
    }
  }
 
  /*********************************************************************************/
  /* Method:    processJson                                                        */
  /* Purpose:   Parses the JSON document received from the cloud.                  */
  /*            The format of the data is found in "TireViewAPI_103020.pdf"        */
  /*            provided by the TPM system manufacturer.                           */
  /*********************************************************************************/
  void TPMSPlugin::processJson(rapidjson::Document &document)
  {
    int index, j;
    const char *str;
    rapidjson::Document document1, document2;

    for (index=0; index<document.Size(); index++)
    {
      document1.CopyFrom(document[index], document.GetAllocator());
      if (document1.HasMember(ACTION) &&
          strcmp(document1[ACTION].GetString(), EVENT) == 0) {
        myTireData.time = (unsigned int)document1[RTC_TIME].GetInt();
        if (document1.HasMember(PRESSURE)) 
        {
          if (document1[PRESSURE].IsArray())
          {
            document1.CopyFrom(document1[PRESSURE], document1.GetAllocator());
            for (j = 0; j < document1.Size(); j++)
            {
              myTireData.pressure[j] = document1[j].GetDouble();
            }
          }
          else
          {
            for(j=0;j<NUMBER_OF_TIRES;j++)
            {
              myTireData.pressure[j] = 0.0;
            }
            document1.CopyFrom(document1[PRESSURE],document1.GetAllocator());
            for (rapidjson::Value::ConstMemberIterator itr = document1.MemberBegin();
                 itr != document1.MemberEnd(); ++itr)
            {
              str = itr->name.GetString();
              j = atoi(str);
              if (j < NUMBER_OF_TIRES)
              {
                myTireData.pressure[j] = document1[str].GetDouble();
              }
            }
          }
          this->storeTireData();
        }
      }
    } 
  }

  /*********************************************************************************/
  /* Method:    storeTireData                                                      */
  /* Purpose:   Append the latest tire pressure data to the local file for future  */
  /*            processing                                                         */
  /*********************************************************************************/
  void TPMSPlugin::storeTireData()
  {
    std::ofstream file;

    file.open(myDataFile, std::ios_base::app | std::ios_base::binary);
    if (file)
    {
      file.write((const char *)(&myTireData),(int)sizeof(myTireData));
    }
    file.close();
  }

  /*********************************************************************************/
  /* Method:    pull_data                                                          */
  /* Purpose:   Uses curlpp (curl) to pull the data from the TPMS cloud environment*/
  /*********************************************************************************/
  void TPMSPlugin::pull_data()
  {
    rapidjson::Document document;

    try
    {
      curlpp::Cleanup cleanup;

      ostringstream os;

      os << curlpp::options::Url(myCloudURL);
      string data = os.str();

      if ((data[0] == 'o' || data[0] == 'O') &&
          (data[1] = 'k' || data[1] == 'K'))
      {
        data.erase(0,2);
        document.Parse(data.c_str());
        if (!document.HasParseError())
        {
          this->processJson(document);
        }
      }
    }
    catch (curlpp::RuntimeError &e)
    {
      cerr << e.what() << endl;
    }
    catch (curlpp::LogicError &e)
    {
      cerr << e.what() << endl;
    }
    
  }
}
