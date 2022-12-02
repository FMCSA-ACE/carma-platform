
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


#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

namespace fmcsa_ace_prepass_plugin
{

  class PrepassPlugin : public rclcpp::Node
  {
    private:
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    protected:

      void handle_request_cb(const std_msgs::msg::String::SharedPtr);

    public:

      PrepassPlugin():Node("prepass_plugin")  {}
      void initialize();
  };
}
