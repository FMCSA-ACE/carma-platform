
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
#pragma once

#include <string.h>
#include <istream>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <carma_v2x_msgs/msg/TirePressureMonitoringSystem.hpp>
#include <tpms_plugin/TPMStype.h>
#include <tpms_plugin/TPMSAnalytics.hpp>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include "../../../rapidjson/document.h"
#include "../../../rapidjson/prettywriter.h"

#define ACTION "action"
#define EVENT "event"
#define PRESSURE "pressure"
#define RTC_TIME "rtc_time"
#define NUMBER_OF_TIRES 18

using namespace std;
using namespace rclcpp;
using namespace fmcsa_ace_tpms_analytics;

using std::placeholders::_1;

namespace fmcsa_ace_tpms_plugin
{

  class TPMSPlugin : public rclcpp::Node
  {
    private:
 
      string myDataFile;
      string myCloudURL;

      Publisher<std_msgs::msg::String>::SharedPtr myTireMessageString;
      Publisher<carma_v2x_msgs::msg::TirePressureMonitoringSystem>::SharedPtr myTireMessage;
      Subscription<std_msgs::msg::String>::SharedPtr myMessageSubscription;

      std::string history_filename;

      TPMS myTireData;

    protected:

      void pull_data();
      void storeTireData();
      void processJson(rapidjson::Document&);
      void handle_request_cb(const std_msgs::msg::String::SharedPtr);

    public:

      TPMSPlugin();
      void initialize();
 
      void run();


  };
}
