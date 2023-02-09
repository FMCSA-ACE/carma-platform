
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
/* Class:    CellularDriver                                                        */
/* Purpose:  Implements the management of the WebSocket connections to the         */
/*           external entities.  The class uses easywsclient David Baird           */
/*           rapidjson (rapidjson.org) for parsing/creating json documents.        */
/***********************************************************************************/  

#pragma once

#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <carma_v2x_msgs/msg/ADSSafety.hpp>
#include <carma_v2x_msgs/msg/FullPositionVector.hpp>
#include <carma_v2x_msgs/msg/TirePressureMonitoringSystem.hpp>

#include <cellular_driver/easywsclient.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

using std::placeholders::_1;

#define PREPASS_URL "ws://192.168.1.71:8080/PrePass/PrePassDecision"
// #define SAFE_SPECT_URL "ws://192.168.1.71:8080/SafeSpect/SafeSpect"

#define POSITION_MESSAGE 1
#define ADS_HEALTH_REQUESTED 2

using namespace std;
using namespace rclcpp;
using namespace rapidjson;
using namespace easywsclient;

namespace cellular_driver
{
  class CellularDriver : public rclcpp::Node
  {
    private:

      WebSocket::pointer myPrepassConnection;
      WebSocket::pointer mySafeSpectConnection;

      vector<WebSocket::pointer> myConnections;
      map<int, WebSocket::pointer> myMessageQueue;
      map<WebSocket::pointer, string> mySocketMapping;

      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr myPrepassPublisher;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr myHealthRequestPublisher;
      rclcpp::Publisher<carma_v2x_msgs::msg::TirePressureMonitoringSystem>::SharedPtr myTPMSPublisher;

      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
      rclcpp::Subscription<carma_v2x_msgs::msg::ADSSafety>::SharedPtr myHealthSubscription;
      rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr myPositionSubscription;

    protected:

      void handle_ads_health(const carma_v2x_msgs::msg::ADSSafety::SharedPtr);
      void handle_position(const gps_msgs::msg::GPSFix::SharedPtr);

      double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d);

      void respond(int, const char * );

    public:

      CellularDriver();
      ~CellularDriver();

      void initialize();
      void processWebsockets();
 
      void updatePrepass(const char *);
      void requestADSHealth(WebSocket *);
  };

/***********************************************************************************/  
/* Developer:  Wayne Staats                                                        */
/*                                                                                 */
/* Class:    WsHandler                                                             */
/* Purpose:  Impelements a C+ Functor, which can be used as a callback function    */
/*           in the easywsclient websocket interface.  A functor was required to   */
/*           provide a link from the callback function back to the cellular driver */
/***********************************************************************************/  

  class WsHandler
  {
    private:
      WebSocket *mySocket; 
      CellularDriver *myDriver;

    public:

      WsHandler(CellularDriver *driver, WebSocket *socket)  
      {
        myDriver = driver; 
        mySocket = socket;
      }
      void operator()(const std::string &);
  };
}
