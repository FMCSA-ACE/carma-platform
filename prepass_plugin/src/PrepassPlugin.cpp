
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
/* Class:    PrepassPlugin                                                         */
/* Purpose:  Implements the plugin for handling the final decision from PrePass.   */
/***********************************************************************************/

#include "prepass_plugin/PrepassPlugin.hpp"

namespace fmcsa_ace_prepass_plugin
{


  /*********************************************************************************/
  /* Method:    initialize                                                         */
  /* Purpose:   This method creates the ROS subscriber to receive the PrePass      */
  /*            final decision.  Upon receiving the message, it will either        */
  /*            instruct the truck to bypass or pull into the station.             */
  /*********************************************************************************/

  void PrepassPlugin::initialize()
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "prepass_decision", 10, std::bind(&PrepassPlugin::handle_request_cb, this, _1));
  }
  
  /*********************************************************************************/
  /* Method:    handle_request_cb                                                  */
  /* Purpose:   ROS callback method for whenever a final decision is received.     */
  /*********************************************************************************/
  void PrepassPlugin::handle_request_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cerr << "PrePass received: " << msg>data << "\n";
  }
 
}
